// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Library Initialization
 *
 * Provides dvbs2x_library_init() which pre-computes all static
 * lookup tables used by the library.  Call once at startup before
 * any other API function to ensure thread-safe operation.
 */

#include "dvbs2x_vlsnr.h"

#define DVBS2X_STR_(x) #x
#define DVBS2X_STR(x)  DVBS2X_STR_(x)

static const char version_str[] =
	DVBS2X_STR(DVBS2X_VERSION_MAJOR) "."
	DVBS2X_STR(DVBS2X_VERSION_MINOR) "."
	DVBS2X_STR(DVBS2X_VERSION_PATCH);

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
