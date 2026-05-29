// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Pilot Block Insertion
 *
 * VL-SNR frames have mandatory pilots. Pilot symbols are
 * (1+j)/sqrt(2) before scrambling (unit power, 45 degree phase).
 *
 * Regular pilot blocks (36 symbols) are inserted every 1440
 * data symbols, consistent with DVB-S2 pilot structure.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2.6
 */

#include "pilot.h"
#include <string.h>

#define M_SQRT1_2 0.70710678118654752440

int dvbs2x_pilot_get_params(const struct dvbs2x_modcod *modcod,
			    struct dvbs2x_pilot_params *params)
{
	unsigned int data_symbols;
	unsigned int num_reg_pilots;

	/* Calculate data symbols based on frame type and modulation */
	if (modcod->modulation == DVBS2X_MOD_QPSK)
		data_symbols = modcod->fec_len / 2;
	else
		data_symbols = modcod->fec_len;

	/* Apply spreading if needed */
	if (modcod->has_spread)
		data_symbols *= 2;

	/* Regular pilot blocks */
	params->reg_blk_len = DVBS2X_PILOT_BLK_LEN;
	num_reg_pilots = (data_symbols - 1) / DVBS2X_PILOT_INTERVAL;
	params->reg_num_blks = num_reg_pilots;

	/* VL-SNR type 1 and type 2 pilots (simplified) */
	params->vlsnr1_blk_len = 36;
	params->vlsnr1_num_blks = 0;
	params->vlsnr2_blk_len = 36;
	params->vlsnr2_num_blks = 0;

	/* Total frame symbols */
	params->frame_symbols = data_symbols +
				num_reg_pilots * DVBS2X_PILOT_BLK_LEN;

	return 0;
}

unsigned int dvbs2x_pilot_insert(const struct dvbs2x_complex *data,
				 unsigned int data_len,
				 struct dvbs2x_complex *output,
				 const struct dvbs2x_modcod *modcod)
{
	unsigned int out_idx = 0;
	unsigned int data_idx = 0;
	unsigned int i;

	(void)modcod;

	while (data_idx < data_len) {
		/* Copy data symbols up to next pilot position */
		unsigned int chunk = DVBS2X_PILOT_INTERVAL;

		if (data_idx + chunk > data_len)
			chunk = data_len - data_idx;

		for (i = 0; i < chunk; i++) {
			output[out_idx] = data[data_idx];
			out_idx++;
			data_idx++;
		}

		/* Insert pilot block (if not at end of frame) */
		if (data_idx < data_len) {
			for (i = 0; i < DVBS2X_PILOT_BLK_LEN; i++) {
				output[out_idx].i = M_SQRT1_2;
				output[out_idx].q = M_SQRT1_2;
				out_idx++;
			}
		}
	}

	return out_idx;
}

unsigned int dvbs2x_pilot_extract(const struct dvbs2x_complex *frame,
				  unsigned int frame_len,
				  struct dvbs2x_complex *data,
				  struct dvbs2x_complex *pilots,
				  const struct dvbs2x_modcod *modcod)
{
	unsigned int frame_idx = 0;
	unsigned int data_idx = 0;
	unsigned int pilot_idx = 0;
	unsigned int i;
	unsigned int chunk;

	(void)modcod;

	while (frame_idx < frame_len) {
		/* Extract data symbols */
		chunk = DVBS2X_PILOT_INTERVAL;
		if (frame_idx + chunk > frame_len)
			chunk = frame_len - frame_idx;

		for (i = 0; i < chunk; i++) {
			data[data_idx] = frame[frame_idx];
			data_idx++;
			frame_idx++;
		}

		/* Extract pilot block if present */
		if (frame_idx + DVBS2X_PILOT_BLK_LEN <= frame_len) {
			for (i = 0; i < DVBS2X_PILOT_BLK_LEN; i++) {
				if (pilots)
					pilots[pilot_idx] = frame[frame_idx];
				pilot_idx++;
				frame_idx++;
			}
		}
	}

	return data_idx;
}
