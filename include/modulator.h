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
 * Mapping: bit 0 -> +1, bit 1 -> -1, then rotate by n*pi/2
 * where n is the symbol index.
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
 * dvbs2x_mod_spread - Apply symbol spreading (factor 2)
 * @in: input symbols
 * @out: output symbols (length = 2 * in_len)
 * @in_len: number of input symbols
 *
 * Each input symbol is repeated twice in the output.
 */
void dvbs2x_mod_spread(const struct dvbs2x_complex *in,
		       struct dvbs2x_complex *out,
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
 * dvbs2x_demod_despread - Combine spread symbols (factor 2)
 * @in: input symbols (2 * out_len)
 * @out: output combined symbols
 * @out_len: number of output symbols
 */
void dvbs2x_demod_despread(const struct dvbs2x_complex *in,
			   struct dvbs2x_complex *out,
			   unsigned int out_len);

#endif /* DVBS2X_MODULATOR_H */
