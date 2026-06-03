/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Modulator
 *
 * Implements pi/2-BPSK and QPSK constellation mapping
 * as specified in ETSI EN 302 307-2.
 *
 * pi/2-BPSK: each symbol rotated by pi/2 relative to previous
 * QPSK: Gray-coded mapping per DVB-S2 clause 5.4.1
 */

#ifndef DVBS2X_MODULATOR_H
#define DVBS2X_MODULATOR_H

#include "dvbs2x_types.h"

/*
 * dvbs2x_mod_pi2bpsk - Map bits to pi/2-BPSK symbols
 * @bits: input bit array
 * @symbols: output complex symbols
 * @len: number of bits (= number of symbols)
 *
 * Period-2 diagonal mapping (gr-dtv m_bpsk[i & 1][b]):
 *   even index: bit 0 -> (+,+)/sqrt2, bit 1 -> (-,-)/sqrt2
 *   odd  index: bit 0 -> (-,+)/sqrt2, bit 1 -> (+,-)/sqrt2
 */
void dvbs2x_mod_pi2bpsk(const uint8_t *bits,
			struct dvbs2x_complex *symbols,
			unsigned int len);

/*
 * dvbs2x_mod_qpsk - Map bit pairs to QPSK symbols
 * @bits: input bit array (length = 2 * num_symbols)
 * @symbols: output complex symbols
 * @num_symbols: number of output symbols
 *
 * Gray-coded QPSK per DVB-S2 Table 9:
 *   00 -> (+1/sqrt2, +1/sqrt2)
 *   01 -> (+1/sqrt2, -1/sqrt2)
 *   10 -> (-1/sqrt2, +1/sqrt2)
 *   11 -> (-1/sqrt2, -1/sqrt2)
 */
void dvbs2x_mod_qpsk(const uint8_t *bits,
		     struct dvbs2x_complex *symbols,
		     unsigned int num_symbols);

/*
 * dvbs2x_mod_spread_bits - Apply spreading factor 2 at the bit level
 * @in: input coded bits
 * @out: output bits (length = 2 * in_len)
 * @in_len: number of input bits
 *
 * Duplicates each coded bit before pi/2-BPSK mapping; the two copies
 * map to the even and odd diagonals (ETSI EN 302 307-2).
 */
void dvbs2x_mod_spread_bits(const uint8_t *in, uint8_t *out,
			    unsigned int in_len);

/*
 * dvbs2x_demod_pi2bpsk - Compute LLRs for pi/2-BPSK symbols
 * @symbols: received complex symbols
 * @llr: output LLR values (one per symbol)
 * @len: number of symbols
 * @noise_var: noise variance (sigma^2)
 */
void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
			   double *llr, unsigned int len,
			   double noise_var);

/*
 * dvbs2x_demod_qpsk - Compute LLRs for QPSK symbols
 * @symbols: received complex symbols
 * @llr: output LLR values (two per symbol)
 * @num_symbols: number of symbols
 * @noise_var: noise variance (sigma^2)
 */
void dvbs2x_demod_qpsk(const struct dvbs2x_complex *symbols,
			double *llr, unsigned int num_symbols,
			double noise_var);

/*
 * dvbs2x_demod_despread_llr - Combine spread LLRs (factor 2)
 * @in: input LLRs (2 * out_len), one per received symbol
 * @out: output combined LLRs
 * @out_len: number of output (coded-bit) LLRs
 *
 * Combines LLRs from the two symbols of each spread pair (same coded
 * bit) by summing them (maximum-likelihood combine).
 */
void dvbs2x_demod_despread_llr(const double *in, double *out,
			       unsigned int out_len);

#endif /* DVBS2X_MODULATOR_H */
