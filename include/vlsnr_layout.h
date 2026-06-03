/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Data-Field Layout
 *
 * The VL-SNR data field interleaves payload symbols with pilot blocks
 * following a fixed, MODCOD-set-specific pattern (ETSI EN 302 307-2
 * clause 5.5.2; identical to GNU Radio gr-dtv physical_cc general_work).
 *
 * This builds, once per MODCOD, the map of which data-field positions
 * are payload and which are pilots.  Both the modulator and the
 * demodulator walk this single map, so the byzantine slot/group/pilot
 * loop is implemented exactly once.
 */

#ifndef DVBS2X_VLSNR_LAYOUT_H
#define DVBS2X_VLSNR_LAYOUT_H

#include "dvbs2x_types.h"

struct dvbs2x_vlsnr_layout {
	unsigned int	field_len;	/* total data-field symbols */
	unsigned int	num_data;	/* payload symbols (after spread) */
	unsigned int	num_pilot;	/* pilot symbols */
	uint8_t		*is_pilot;	/* field_len entries, 1 = pilot */
};

/*
 * dvbs2x_vlsnr_build_layout - Build the data/pilot map for a MODCOD
 * @mc: MODCOD descriptor
 * @lay: output layout (caller frees with dvbs2x_vlsnr_free_layout)
 *
 * Returns 0 on success, negative on error.
 */
int dvbs2x_vlsnr_build_layout(const struct dvbs2x_modcod *mc,
			      struct dvbs2x_vlsnr_layout *lay);

/*
 * dvbs2x_vlsnr_free_layout - Release a layout built by build_layout
 */
void dvbs2x_vlsnr_free_layout(struct dvbs2x_vlsnr_layout *lay);

#endif /* DVBS2X_VLSNR_LAYOUT_H */
