/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR BCH Encoder/Decoder
 *
 * Implements t-error correcting BCH codes as specified in
 * ETSI EN 302 307-1 clause 5.3 and ETSI EN 302 307-2.
 *
 * Normal frame:  GF(2^16), t=12, 192 parity bits
 * Medium frame:  GF(2^15), t=12, 180 parity bits
 * Short frame:   GF(2^14), t=12, 168 parity bits
 */

#ifndef DVBS2X_BCH_H
#define DVBS2X_BCH_H

#include "dvbs2x_types.h"

/* Maximum BCH parity bits across all frame types */
#define DVBS2X_BCH_MAX_PARITY	192

/* BCH encoder context */
struct dvbs2x_bch_encoder {
	unsigned int	t;		/* error correction capability */
	unsigned int	n;		/* codeword length (n_bch) */
	unsigned int	k;		/* message length (k_bch) */
	unsigned int	parity_len;	/* n - k */
	unsigned int	gf_m;		/* GF(2^m) field order */
	/*
	 * Generator polynomial coefficients stored as a bit array.
	 * g(x) = g[0] + g[1]*x + ... + g[parity_len]*x^parity_len
	 */
	uint8_t		gen_poly[DVBS2X_BCH_MAX_PARITY + 1];
};

/* BCH decoder context */
struct dvbs2x_bch_decoder {
	unsigned int	t;
	unsigned int	n;
	unsigned int	k;
	unsigned int	parity_len;
	unsigned int	gf_m;
	uint8_t		gen_poly[DVBS2X_BCH_MAX_PARITY + 1];
};

/*
 * dvbs2x_bch_encoder_init - Initialize BCH encoder for given MODCOD
 * @enc: encoder context to initialize
 * @modcod: MODCOD parameters
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_bch_encoder_init(struct dvbs2x_bch_encoder *enc,
			    const struct dvbs2x_modcod *modcod);

/*
 * dvbs2x_bch_encode - Encode a message block
 * @enc: initialized encoder context
 * @input: message bits (k_bch bits)
 * @output: codeword bits (n_bch bits), includes input + parity
 *
 * Systematic encoding: output[0..k-1] = input, output[k..n-1] = parity.
 * Returns 0 on success.
 */
int dvbs2x_bch_encode(const struct dvbs2x_bch_encoder *enc,
		      const uint8_t *input, uint8_t *output);

/*
 * dvbs2x_bch_decoder_init - Initialize BCH decoder for given MODCOD
 * @dec: decoder context to initialize
 * @modcod: MODCOD parameters
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_bch_decoder_init(struct dvbs2x_bch_decoder *dec,
			    const struct dvbs2x_modcod *modcod);

/*
 * dvbs2x_bch_decode - Decode and correct errors in a codeword
 * @dec: initialized decoder context
 * @codeword: received codeword (n_bch bits), corrected in place
 *
 * Returns number of corrected errors (0 = no errors),
 * or -1 if uncorrectable.
 */
int dvbs2x_bch_decode(const struct dvbs2x_bch_decoder *dec,
		      uint8_t *codeword);

#endif /* DVBS2X_BCH_H */
