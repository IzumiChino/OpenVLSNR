/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Pilot Block Insertion and Extraction
 *
 * VL-SNR frames have mandatory pilots with three block types:
 * - Regular pilot blocks (36 symbols, every 1440 data symbols)
 * - VL-SNR Type 1 pilot blocks
 * - VL-SNR Type 2 pilot blocks
 *
 * Pilot symbols are (1+j)/sqrt(2) before scrambling.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2.6
 */

#ifndef DVBS2X_PILOT_H
#define DVBS2X_PILOT_H

#include "dvbs2x_types.h"

/* Regular pilot block interval (data symbols between pilots) */
#define DVBS2X_PILOT_INTERVAL	1440

/* Pilot block parameters for a given MODCOD */
struct dvbs2x_pilot_params {
	/* Regular pilots */
	unsigned int	reg_blk_len;
	unsigned int	reg_num_blks;

	/* VL-SNR type 1 pilots */
	unsigned int	vlsnr1_blk_len;
	unsigned int	vlsnr1_num_blks;

	/* VL-SNR type 2 pilots */
	unsigned int	vlsnr2_blk_len;
	unsigned int	vlsnr2_num_blks;

	/* Total frame length with pilots */
	unsigned int	frame_symbols;
};

/*
 * dvbs2x_pilot_get_params - Get pilot parameters for a MODCOD
 * @modcod: MODCOD parameters
 * @params: output pilot parameters
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_pilot_get_params(const struct dvbs2x_modcod *modcod,
			    struct dvbs2x_pilot_params *params);

/*
 * dvbs2x_pilot_insert - Insert pilot blocks into data stream
 * @data: input data symbols
 * @data_len: number of data symbols
 * @output: output with pilots inserted
 * @modcod: MODCOD parameters
 *
 * Returns total output length (data + pilots).
 */
unsigned int dvbs2x_pilot_insert(const struct dvbs2x_complex *data,
				 unsigned int data_len,
				 struct dvbs2x_complex *output,
				 const struct dvbs2x_modcod *modcod);

/*
 * dvbs2x_pilot_extract - Extract pilot symbols from received frame
 * @frame: received frame symbols (with pilots)
 * @frame_len: total frame length
 * @data: output data symbols (pilots removed)
 * @pilots: output pilot symbols (may be NULL)
 * @modcod: MODCOD parameters
 *
 * Returns number of data symbols extracted.
 */
unsigned int dvbs2x_pilot_extract(const struct dvbs2x_complex *frame,
				  unsigned int frame_len,
				  struct dvbs2x_complex *data,
				  struct dvbs2x_complex *pilots,
				  const struct dvbs2x_modcod *modcod);

#endif /* DVBS2X_PILOT_H */
