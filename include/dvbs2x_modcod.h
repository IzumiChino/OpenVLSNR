/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR MODCOD Table Definitions
 *
 * Reference: ETSI EN 302 307-2 V1.3.1, Table 17a
 */

#ifndef DVBS2X_MODCOD_H
#define DVBS2X_MODCOD_H

#include "dvbs2x_types.h"

/*
 * dvbs2x_vlsnr_get_modcod - Get MODCOD parameters by index
 * @index: MODCOD index (1-9)
 *
 * Returns pointer to static MODCOD descriptor, or NULL if invalid.
 */
const struct dvbs2x_modcod *dvbs2x_vlsnr_get_modcod(unsigned int index);

/*
 * dvbs2x_vlsnr_get_modcod_name - Get the canonical name by index
 * @index: MODCOD index (1-9)
 *
 * Returns a stable, unambiguous name, or NULL if the index is invalid.
 */
const char *dvbs2x_vlsnr_get_modcod_name(unsigned int index);

/*
 * dvbs2x_vlsnr_get_modcod_by_name - Get MODCOD by canonical name
 * @name: canonical name returned by dvbs2x_vlsnr_get_modcod_name()
 *
 * Unambiguous legacy names are accepted.  Ambiguous legacy names such as
 * "BPSK 1/5" and "BPSK 1/3" return NULL.
 */
const struct dvbs2x_modcod *dvbs2x_vlsnr_get_modcod_by_name(const char *name);

/*
 * dvbs2x_vlsnr_get_modcod_table - Get pointer to full MODCOD table
 *
 * Returns array of DVBS2X_VLSNR_NUM_MODCODS entries.
 */
const struct dvbs2x_modcod *dvbs2x_vlsnr_get_modcod_table(void);

#endif /* DVBS2X_MODCOD_H */
