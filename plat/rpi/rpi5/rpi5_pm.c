/*
 * Copyright (c) 2015-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/console.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <plat/common/platform.h>

#include <rpi_hw.h>

#define MBOX_CHAN_SUSPEND              9

#ifdef RPI_HAVE_GIC
#include <drivers/arm/gicv2.h>
#endif

#define CPUPWRCTLR_EL1		S3_0_c15_c2_7
#define CLUSTERPWRCTLR_EL1	S3_0_c15_c3_5

DEFINE_RENAME_SYSREG_RW_FUNCS(clusterpwrctlr_el1, CLUSTERPWRCTLR_EL1)
DEFINE_RENAME_SYSREG_RW_FUNCS(cpupwrctlr_el1, CPUPWRCTLR_EL1)


/* Make composite power state parameter till power level 0 */
#if PSCI_EXTENDED_STATE_ID

#define rpi5_make_pwrstate_lvl0(lvl0_state, pwr_lvl, type) \
		(((lvl0_state) << PSTATE_ID_SHIFT) | \
		 ((type) << PSTATE_TYPE_SHIFT))

#else

#define rpi5_make_pwrstate_lvl0(lvl0_state, pwr_lvl, type) \
		(((lvl0_state) << PSTATE_ID_SHIFT) | \
		 ((pwr_lvl) << PSTATE_PWR_LVL_SHIFT) | \
		 ((type) << PSTATE_TYPE_SHIFT))

#endif /* PSCI_EXTENDED_STATE_ID */

#define rpi5_make_pwrstate_lvl1(lvl1_state, lvl0_state, pwr_lvl, type) \
		(((lvl1_state) << PLAT_LOCAL_PSTATE_WIDTH) | \
		 rpi5_make_pwrstate_lvl0(lvl0_state, pwr_lvl, type))

/*
 *  The table storing the valid idle power states. Ensure that the
 *  array entries are populated in ascending order of state-id to
 *  enable us to use binary search during power state validation.
 *  The table must be terminated by a NULL entry.
 */
static const unsigned int rpi5_pm_idle_states[] = {
	/* State-id - 0x01 */
	rpi5_make_pwrstate_lvl1(PLAT_LOCAL_STATE_RUN, PLAT_LOCAL_STATE_RET,
				MPIDR_AFFLVL0, PSTATE_TYPE_STANDBY),
	/* State-id - 0x02 */
	rpi5_make_pwrstate_lvl1(PLAT_LOCAL_STATE_RUN, PLAT_LOCAL_STATE_OFF,
				MPIDR_AFFLVL0, PSTATE_TYPE_POWERDOWN),
	/* State-id - 0x22 */
	rpi5_make_pwrstate_lvl1(PLAT_LOCAL_STATE_OFF, PLAT_LOCAL_STATE_OFF,
				MPIDR_AFFLVL1, PSTATE_TYPE_POWERDOWN),
	0,
};

/*******************************************************************************
 * Platform handler called to check the validity of the power state
 * parameter. The power state parameter has to be a composite power state.
 ******************************************************************************/
static int rpi5_validate_power_state(unsigned int power_state,
				     psci_power_state_t *req_state)
{
	unsigned int state_id;
	int i;

	assert(req_state != 0);

	/*
	 *  Currently we are using a linear search for finding the matching
	 *  entry in the idle power state array. This can be made a binary
	 *  search if the number of entries justify the additional complexity.
	 */
	for (i = 0; rpi5_pm_idle_states[i] != 0; i++) {
		if (power_state == rpi5_pm_idle_states[i]) {
			break;
		}
	}

	/* Return error if entry not found in the idle state array */
	if (!rpi5_pm_idle_states[i]) {
		return PSCI_E_INVALID_PARAMS;
	}

	i = 0;
	state_id = psci_get_pstate_id(power_state);

	/* Parse the State ID and populate the state info parameter */
	while (state_id) {
		req_state->pwr_domain_state[i++] = state_id &
						PLAT_LOCAL_PSTATE_MASK;
		state_id >>= PLAT_LOCAL_PSTATE_WIDTH;
	}

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Platform handler called when a CPU is about to enter standby.
 ******************************************************************************/
static void rpi5_cpu_standby(plat_local_state_t cpu_state)
{
	assert(cpu_state == PLAT_LOCAL_STATE_RET);

	/*
	 * Enter standby state.
	 * dsb is good practice before using wfi to enter low power states
	 */
	dsb();
	wfi();
}

static void rpi5_pwr_domain_off(const psci_power_state_t *target_state)
{
#ifdef RPI_HAVE_GIC
	gicv2_cpuif_disable();
#endif
}

#if 0
void __dead2 plat_secondary_cold_boot_setup(void);

static void __dead2
rpi5_pwr_domain_pwr_down_wfi(const psci_power_state_t *target_state)
{
	disable_mmu_el3();
	plat_secondary_cold_boot_setup();
}
#endif

/*******************************************************************************
 * Platform handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
static int rpi5_pwr_domain_on(u_register_t mpidr)
{
	int rc = PSCI_E_SUCCESS;
	unsigned int pos = plat_core_pos_by_mpidr(mpidr);
	uintptr_t hold_base = PLAT_RPI3_TM_HOLD_BASE;

	assert(pos < PLATFORM_CORE_COUNT);

	hold_base += pos * PLAT_RPI3_TM_HOLD_ENTRY_SIZE;

	mmio_write_64(hold_base, PLAT_RPI3_TM_HOLD_STATE_GO);
	/* No cache maintenance here, hold_base is mapped as device memory. */

	/* Make sure that the write has completed */
	dsb();
	isb();

	sev();

	return rc;
}

/*******************************************************************************
 * Platform handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
static void rpi5_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	assert(target_state->pwr_domain_state[MPIDR_AFFLVL0] ==
					PLAT_LOCAL_STATE_OFF);

#ifdef RPI_HAVE_GIC
	gicv2_pcpu_distif_init();
	gicv2_cpuif_enable();
#endif
}

static void __dead2 rpi5_pwr_down_wfi(
		const psci_power_state_t *target_state)
{
	uintptr_t hold_base = PLAT_RPI3_TM_HOLD_BASE;
	unsigned int pos = plat_my_core_pos();

	if (pos == 0) {
		/*
		 * The secondaries will always be in a wait
		 * for warm boot on reset, but the BSP needs
		 * to be able to distinguish between waiting
		 * for warm boot (e.g. after psci_off, waiting
		 * for psci_on) and a cold boot.
		 */
		mmio_write_64(hold_base, PLAT_RPI3_TM_HOLD_STATE_BSP_OFF);
		dcsw_op_all(DCCISW);
		dsb();
		isb();
		write_clusterpwrctlr_el1(0xf1);
		dsb();
		isb();
		write_clusterpwrdn_el1(0x0);
		dsb();
		isb();
	}

	//write_rmr_el3(RMR_EL3_RR_BIT | RMR_EL3_AA64_BIT);
	write_cpupwrctlr_el1(0x1);
	dsb();
	isb();

	while (1)
		wfi();
}

/*******************************************************************************
 * Platform handlers for system suspend.
 ******************************************************************************/

void rpi5_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	uint32_t msg = MBOX_CHAN_SUSPEND |
		(*(uint32_t *)PLAT_RPI3_TM_ENTRYPOINT << 4);

	INFO("rpi5_pwr_domain_suspend - %x\n", msg);
	mmio_write_32(RPI3_MBOX_BASE + RPI3_MBOX1_WRITE_OFFSET, msg);
}

void rpi5_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	INFO("rpi5_pwr_domain_suspend_finish\n");
#if 0
	uint32_t lvl;
	plat_local_state_t lvl_state;
	int ret;

	/* Nothing to be done on waking up from retention from CPU level */
	if (RK_CORE_PWR_STATE(target_state) != PLAT_MAX_OFF_STATE)
		return;

	if (RK_SYSTEM_PWR_STATE(target_state) == PLAT_MAX_OFF_STATE) {
		rockchip_soc_sys_pwr_dm_resume();
		goto comm_finish;
	}

	for (lvl = MPIDR_AFFLVL1; lvl <= PLAT_MAX_PWR_LVL; lvl++) {
		lvl_state = target_state->pwr_domain_state[lvl];
		ret = rockchip_soc_hlvl_pwr_dm_resume(lvl, lvl_state);
		if (ret == PSCI_E_NOT_SUPPORTED)
			break;
	}

	rockchip_soc_cores_pwr_dm_resume();

	/*
	 * Program the gic per-cpu distributor or re-distributor interface.
	 * For sys power domain operation, resuming of the gic needs to operate
	 * in rockchip_soc_sys_pwr_dm_resume(), according to the sys power mode
	 * implements.
	 */
	plat_rockchip_gic_cpuif_enable();

comm_finish:
	/* Perform the common cluster specific operations */
	if (RK_CLUSTER_PWR_STATE(target_state) == PLAT_MAX_OFF_STATE) {
		/* Enable coherency if this cluster was off */
		plat_cci_enable();
	}
#endif
}

void rpi5_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	int i = 0;
	unsigned int state_id, power_state;
	int size = ARRAY_SIZE(rpi5_pm_idle_states);

	/*
	 * Find deepest state.
	 * The arm_pm_idle_states[] array has last element by default 0,
	 * so the real deepest state is second last element of that array.
	 */
	power_state = rpi5_pm_idle_states[size - 2];
	state_id = psci_get_pstate_id(power_state);

	INFO("rpi5_get_sys_suspend_power_state(%x, %x)\n", power_state, state_id);
	/* Parse the State ID and populate the state info parameter */
	while (state_id) {
		req_state->pwr_domain_state[i++] =
		    state_id & PLAT_LOCAL_PSTATE_MASK;
		state_id >>= PLAT_LOCAL_PSTATE_WIDTH;
	}
}

/*******************************************************************************
 * Platform handlers for system reset and system off.
 ******************************************************************************/

/* 10 ticks (Watchdog timer = Timer clock / 16) */
#define RESET_TIMEOUT	U(10)

static void __dead2 rpi5_watchdog_reset(void)
{
	uint32_t rstc;

	console_flush();

	dsbsy();
	isb();

	mmio_write_32(RPI5_PM_BASE + RPI5_PM_WDOG_OFFSET,
		      RPI5_PM_PASSWORD | RESET_TIMEOUT);

	rstc = mmio_read_32(RPI5_PM_BASE + RPI5_PM_RSTC_OFFSET);
	rstc &= ~RPI5_PM_RSTC_WRCFG_MASK;
	rstc |= RPI5_PM_PASSWORD | RPI5_PM_RSTC_WRCFG_FULL_RESET;
	mmio_write_32(RPI5_PM_BASE + RPI5_PM_RSTC_OFFSET, rstc);

	for (;;) {
		wfi();
	}
}

static void __dead2 rpi5_system_reset(void)
{
	INFO("rpi5: PSCI_SYSTEM_RESET: Invoking watchdog reset\n");

	rpi5_watchdog_reset();
}

static int rpi5_system_reset2(int is_vendor, int reset_type, u_register_t cookie)
{
	uint32_t rsts;

	INFO("rpi5: PSCI_SYSTEM_RESET2: Invoking watchdog reset2 (%d,%d,%lx)\n",
	     is_vendor, reset_type, cookie);

	rsts = mmio_read_32(RPI5_PM_BASE + RPI5_PM_RSTS_OFFSET);
	rsts &= ~RPI5_PM_RSTS_WRCFG_HALT;
	rsts |= (cookie & 0x1) | ((cookie & 0x2) << 1) |
		((cookie & 0x4) << 2) | ((cookie & 0x8) << 3) |
		((cookie & 0x10) << 4) | ((cookie & 0x20) << 5);
	rsts |= RPI5_PM_PASSWORD;
	mmio_write_32(RPI5_PM_BASE + RPI5_PM_RSTS_OFFSET, rsts);

	rpi5_watchdog_reset();

	return 0;
}

static void __dead2 rpi5_system_off(void)
{
	uint32_t rsts;

	INFO("rpi5: PSCI_SYSTEM_OFF: Invoking watchdog reset\n");

	/*
	 * This function doesn't actually make the Raspberry Pi turn itself off,
	 * the hardware doesn't allow it. It simply reboots it and the RSTS
	 * value tells the bootcode.bin firmware not to continue the regular
	 * bootflow and to stay in a low power mode.
	 */

	rsts = mmio_read_32(RPI5_PM_BASE + RPI5_PM_RSTS_OFFSET);
	rsts |= RPI5_PM_PASSWORD | RPI5_PM_RSTS_WRCFG_HALT;
	mmio_write_32(RPI5_PM_BASE + RPI5_PM_RSTS_OFFSET, rsts);

	rpi5_watchdog_reset();
}

/*******************************************************************************
 * Platform handlers and setup function.
 ******************************************************************************/
static const plat_psci_ops_t plat_rpi5_psci_pm_ops = {
	.cpu_standby = rpi5_cpu_standby,
	.pwr_domain_off = rpi5_pwr_domain_off,
	.pwr_domain_pwr_down_wfi = 0, //rpi5_pwr_domain_pwr_down_wfi,
	.pwr_domain_on = rpi5_pwr_domain_on,
	.pwr_domain_on_finish = rpi5_pwr_domain_on_finish,
	.pwr_domain_pwr_down_wfi = rpi5_pwr_down_wfi,
	.pwr_domain_suspend = rpi5_pwr_domain_suspend,
	.pwr_domain_suspend_finish = rpi5_pwr_domain_suspend_finish,
	.get_sys_suspend_power_state = rpi5_get_sys_suspend_power_state,
	.system_off = rpi5_system_off,
	.system_reset = rpi5_system_reset,
	.system_reset2 = rpi5_system_reset2,
	.validate_power_state = rpi5_validate_power_state,
};

int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	uintptr_t *entrypoint = (void *) PLAT_RPI3_TM_ENTRYPOINT;

	INFO("Installed rpi5 psci ops (entrypoint %lx)\n", sec_entrypoint);

	if ((uint32_t)(sec_entrypoint << 4) != (sec_entrypoint << 4))
		return PSCI_E_INVALID_PARAMS;

	*entrypoint = sec_entrypoint;
	*psci_ops = &plat_rpi5_psci_pm_ops;

	return 0;
}
