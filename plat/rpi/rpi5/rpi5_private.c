/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <bl31/interrupt_mgmt.h>
#include <drivers/console.h>
#include <drivers/arm/pl011.h>
#include <lib/xlat_tables/xlat_tables_v2.h>

#include <rpi5_private.h>

#define MAP_DEVICE0	MAP_REGION_FLAT(DEVICE0_BASE,			\
					DEVICE0_SIZE,			\
					MT_DEVICE | MT_RW | MT_SECURE)

#ifdef IMAGE_BL31
static const mmap_region_t plat_rpi5_mmap[] = {
#ifdef MAP_SHARED_RAM
	MAP_SHARED_RAM,
#endif
	MAP_DEVICE0,
#ifdef RPI5_PRELOADED_DTB_BASE
	MAP_NS_DTB,
#endif
#ifdef BL32_BASE
	MAP_BL32_MEM,
#endif
	{0}
};
#endif

/*******************************************************************************
 * Gets SPSR for BL33 entry
 ******************************************************************************/
uint32_t rpi5_get_spsr_for_bl33_entry(void)
{
	return SPSR_64(MODE_EL2, MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
}

/*******************************************************************************
 * Function that sets up the console
 ******************************************************************************/
static console_t rpi5_console;

void rpi5_console_init(void)
{
	int console_scope = CONSOLE_FLAG_BOOT;
	int rc;

	if (RPI5_RUNTIME_UART != -1)
		console_scope |= CONSOLE_FLAG_RUNTIME;

	rc = console_pl011_register(PLAT_RPI_PL011_UART_BASE,
				    PLAT_RPI_PL011_UART_CLOCK,
				    PLAT_RPI_UART_BAUDRATE,
				    &rpi5_console);

	if (rc == 0) {
		/*
		 * The crash console doesn't use the multi console API, it uses
		 * the core console functions directly. It is safe to call panic
		 * and let it print debug information.
		 */
		panic();
	}

	console_set_scope(&rpi5_console, console_scope);
}

/*******************************************************************************
 * Function that sets up the translation tables.
 ******************************************************************************/
void rpi5_setup_page_tables(uintptr_t total_base, size_t total_size,
			    uintptr_t code_start, uintptr_t code_limit,
			    uintptr_t rodata_start, uintptr_t rodata_limit
#if USE_COHERENT_MEM
			    , uintptr_t coh_start, uintptr_t coh_limit
#endif
			    )
{
	/*
	 * Map the Trusted SRAM with appropriate memory attributes.
	 * Subsequent mappings will adjust the attributes for specific regions.
	 */
	VERBOSE("Trusted SRAM seen by this BL image: %p - %p\n",
		(void *) total_base, (void *) (total_base + total_size));
	mmap_add_region(total_base, total_base,
			total_size,
			MT_MEMORY | MT_RW | MT_SECURE);

	/* Re-map the code section */
	VERBOSE("Code region: %p - %p\n",
		(void *) code_start, (void *) code_limit);
	mmap_add_region(code_start, code_start,
			code_limit - code_start,
			MT_CODE | MT_SECURE);

	/* Re-map the read-only data section */
	VERBOSE("Read-only data region: %p - %p\n",
		(void *) rodata_start, (void *) rodata_limit);
	mmap_add_region(rodata_start, rodata_start,
			rodata_limit - rodata_start,
			MT_RO_DATA | MT_SECURE);

#if USE_COHERENT_MEM
	/* Re-map the coherent memory region */
	VERBOSE("Coherent region: %p - %p\n",
		(void *) coh_start, (void *) coh_limit);
	mmap_add_region(coh_start, coh_start,
			coh_limit - coh_start,
			MT_DEVICE | MT_RW | MT_SECURE);
#endif

	mmap_add(plat_rpi5_mmap);

	init_xlat_tables();
}

unsigned int plat_get_syscnt_freq2(void)
{
	return SYS_COUNTER_FREQ_IN_TICKS;
}
