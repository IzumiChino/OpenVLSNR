/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Bit Interleaver
 *
 * Column-row interleaving applied after LDPC encoding.
 * For BPSK: no interleaving (single column).
 * For QPSK: 2 columns, row-column read.
 *
 * Reference: ETSI EN 302 307-1 clause 5.3.3
 */

#ifndef DVBS2X_INTERLEAVER_H
#define DVBS2X_INTERLEAVER_H

#include "dvbs2x_types.h"

/*
 * dvbs2x_interleave - Interleave FECFRAME bits
 * @modcod: MODCOD parameters (determines interleaver config)
 * @input: input bits (n_ldpc bits)
 * @output: interleaved bits (n_ldpc bits)
 *
 * For BPSK (modulation order 2): output = input (no interleaving).
 * For QPSK (modulation order 4): 2-column interleaving.
 */
void dvbs2x_interleave(const struct dvbs2x_modcod *modcod,
		       const uint8_t *input, uint8_t *output);

/*
 * dvbs2x_deinterleave - Deinterleave received soft values
 * @modcod: MODCOD parameters
 * @input: input LLR values (n_ldpc values)
 * @output: deinterleaved LLR values (n_ldpc values)
 */
void dvbs2x_deinterleave(const struct dvbs2x_modcod *modcod,
			 const double *input, double *output);

#endif /* DVBS2X_INTERLEAVER_H */
