// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Pilot Block Insertion / Extraction
 *
 * Walks the data-field layout (vlsnr_layout.h): payload symbols fill the
 * data positions and the unmodulated pilot value (1+j)/sqrt(2) fills the
 * pilot positions.  Scrambling is applied separately (see scrambler.c).
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2
 */

#include "pilot.h"

unsigned int dvbs2x_pilot_insert(const struct dvbs2x_complex *data,
				 const struct dvbs2x_vlsnr_layout *lay,
				 struct dvbs2x_complex *output)
{
	unsigned int i;
	unsigned int data_idx = 0;

	for (i = 0; i < lay->field_len; i++) {
		if (lay->is_pilot[i]) {
			output[i].i = DVBS2X_PILOT_I;
			output[i].q = DVBS2X_PILOT_Q;
		} else {
			output[i] = data[data_idx++];
		}
	}
	return lay->field_len;
}

unsigned int dvbs2x_pilot_extract(const struct dvbs2x_complex *field,
				  const struct dvbs2x_vlsnr_layout *lay,
				  struct dvbs2x_complex *data,
				  struct dvbs2x_complex *pilots)
{
	unsigned int i;
	unsigned int data_idx = 0;
	unsigned int pilot_idx = 0;

	for (i = 0; i < lay->field_len; i++) {
		if (lay->is_pilot[i]) {
			if (pilots)
				pilots[pilot_idx] = field[i];
			pilot_idx++;
		} else {
			data[data_idx++] = field[i];
		}
	}
	return data_idx;
}
