// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Bit Interleaver
 *
 * The DVB-S2/S2X bit interleaver only permutes bits for the higher-order
 * constellations (8PSK and above), where each column of the FEC frame
 * maps to one bit position of a symbol.  For BPSK and QPSK there is NO
 * interleaving: the coded bits map directly to symbols in natural order
 * (QPSK groups consecutive bit pairs b0,b1 into one symbol).
 *
 * This matches gr-dtv dvbs2_interleaver_bb_impl: MOD_BPSK copies bits
 * through, and MOD_QPSK packs in[consumed++], in[consumed++] into each
 * symbol with no reordering.
 *
 * Reference: ETSI EN 302 307-1 clause 5.3.3 (bit interleaver applies to
 * 8PSK/16APSK/32APSK only).
 */

#include "interleaver.h"

void dvbs2x_interleave(const struct dvbs2x_modcod *modcod,
		       const uint8_t *input, uint8_t *output)
{
	unsigned int n, i;

	n = dvbs2x_tx_coded_bits(modcod);

	/*
	 * BPSK and QPSK: no bit interleaving.  The coded bits are emitted in
	 * natural order; QPSK simply maps each consecutive pair to one symbol
	 * (handled by the constellation mapper), so the bit stream is
	 * unchanged here.
	 */
	for (i = 0; i < n; i++)
		output[i] = input[i];
}

void dvbs2x_deinterleave(const struct dvbs2x_modcod *modcod,
			 const double *input, double *output)
{
	unsigned int n, i;

	n = dvbs2x_tx_coded_bits(modcod);

	/* BPSK and QPSK: no deinterleaving (see dvbs2x_interleave). */
	for (i = 0; i < n; i++)
		output[i] = input[i];
}
