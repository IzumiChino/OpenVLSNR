/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR LDPC Encoder/Decoder
 *
 * Implements structured quasi-cyclic LDPC codes as specified in
 * ETSI EN 302 307-2 for VL-SNR code rates.
 *
 * Encoding uses the accumulator-based algorithm exploiting the
 * structured parity check matrix (IRA codes).
 *
 * Decoding uses layered offset min-sum belief propagation.
 */

#ifndef DVBS2X_LDPC_H
#define DVBS2X_LDPC_H

#include "dvbs2x_types.h"

/* Maximum row weight in LDPC parity check matrix */
#define DVBS2X_LDPC_MAX_ROW_WEIGHT	32

/*
 * LDPC address table entry.
 * Each row in the standard's table specifies the positions of 1s
 * in the first column of a group of Q columns.
 */
struct dvbs2x_ldpc_table_entry {
	unsigned int	num_addrs;
	unsigned int	addrs[DVBS2X_LDPC_MAX_ROW_WEIGHT];
};

/* LDPC code descriptor */
struct dvbs2x_ldpc_code {
	unsigned int	n;		/* codeword length */
	unsigned int	k;		/* information length */
	unsigned int	q;		/* circulant size */
	unsigned int	num_groups;	/* number of address table rows */
	const struct dvbs2x_ldpc_table_entry *table;
};

/* LDPC encoder context */
struct dvbs2x_ldpc_encoder {
	struct dvbs2x_ldpc_code	code;
};

/* LDPC decoder context */
struct dvbs2x_ldpc_decoder {
	struct dvbs2x_ldpc_code	code;
	unsigned int		max_iter;
	double			offset;		/* min-sum offset */
};

/*
 * dvbs2x_ldpc_encoder_init - Initialize LDPC encoder
 * @enc: encoder context
 * @modcod: MODCOD parameters
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_ldpc_encoder_init(struct dvbs2x_ldpc_encoder *enc,
			     const struct dvbs2x_modcod *modcod);

/*
 * dvbs2x_ldpc_encode - Encode information bits
 * @enc: initialized encoder
 * @info: information bits (k bits)
 * @codeword: output codeword (n bits), systematic
 *
 * Returns 0 on success.
 */
int dvbs2x_ldpc_encode(const struct dvbs2x_ldpc_encoder *enc,
		       const uint8_t *info, uint8_t *codeword);

/*
 * dvbs2x_ldpc_decoder_init - Initialize LDPC decoder
 * @dec: decoder context
 * @modcod: MODCOD parameters
 * @max_iter: maximum decoding iterations
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_ldpc_decoder_init(struct dvbs2x_ldpc_decoder *dec,
			     const struct dvbs2x_modcod *modcod,
			     unsigned int max_iter);

/*
 * dvbs2x_ldpc_decode - Decode LLR values
 * @dec: initialized decoder
 * @llr: input log-likelihood ratios (n values, positive = bit 0)
 * @output: decoded information bits (k bits)
 * @iter_used: actual iterations used (may be NULL)
 *
 * Returns 0 on success (all parity checks satisfied),
 * -1 if max iterations reached without convergence.
 */
int dvbs2x_ldpc_decode(const struct dvbs2x_ldpc_decoder *dec,
		       const double *llr, uint8_t *output,
		       unsigned int *iter_used);

/*
 * dvbs2x_ldpc_get_table - Select LDPC table for given MODCOD
 * @modcod: MODCOD parameters
 * @num_groups: output number of address table groups
 *
 * Returns pointer to address table, or NULL if unsupported.
 */
const struct dvbs2x_ldpc_table_entry *
dvbs2x_ldpc_get_table(const struct dvbs2x_modcod *modcod,
		      unsigned int *num_groups);

/*
 * LDPC table access functions (defined in tables/ directory)
 */
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n64800_r2_9(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n32400_r1_5(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n32400_r11_45(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n32400_r1_3(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r1_5(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r11_45(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r4_15(void);
const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r1_3(void);

#endif /* DVBS2X_LDPC_H */
