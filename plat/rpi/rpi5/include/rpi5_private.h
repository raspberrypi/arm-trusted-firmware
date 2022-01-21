/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RPI5_PRIVATE_H
#define RPI5_PRIVATE_H

#include <stdint.h>

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/

/* Utility functions */
void rpi5_console_init(void);
void rpi5_setup_page_tables(uintptr_t total_base, size_t total_size,
			    uintptr_t code_start, uintptr_t code_limit,
			    uintptr_t rodata_start, uintptr_t rodata_limit
#if USE_COHERENT_MEM
			    , uintptr_t coh_start, uintptr_t coh_limit
#endif
			    );

/* Optional functions required in the Raspberry Pi 5 port */
unsigned int plat_rpi5_calc_core_pos(u_register_t mpidr);

/* BL2 utility functions */
uint32_t rpi5_get_spsr_for_bl33_entry(void);

#endif /* RPI5_PRIVATE_H */
