// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Library Initialization
 *
 * Provides dvbs2x_library_init() which pre-computes all static
 * lookup tables used by the library.  Call once at startup before
 * any other API function to ensure thread-safe operation.
 */

#include "dvbs2x_vlsnr.h"

static const char version_str[] = "1.0.0";

/*
 * External init hooks for static tables in other modules.
 * These are defined in their respective source files.
 */
extern void dvbs2x_ldpc_lut_init(void);
extern void dvbs2x_vlsnr_sync_init(void);

void dvbs2x_library_init(void)
{
	dvbs2x_ldpc_lut_init();
	dvbs2x_vlsnr_sync_init();
}

const char *dvbs2x_version_string(void)
{
	return version_str;
}
