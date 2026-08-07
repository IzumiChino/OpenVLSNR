// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR MODCOD Parameter Table
 *
 * Reference: ETSI EN 302 307-2 V1.3.1, Table 17a
 *
 * BCH parameters derived from ETSI EN 302 307-1 Tables 5a/5b
 * and ETSI EN 302 307-2 for medium frame additions.
 */

#include "dvbs2x_modcod.h"
#include <string.h>

/*
 * VL-SNR MODCOD table
 *
 * BCH parameters:
 *   Normal frame (64800): GF(2^16), t=12, parity=192 bits
 *   Medium frame (32400): GF(2^15), t=12, parity=180 bits
 *   Short frame  (16200): GF(2^14), t=12, parity=168 bits
 *
 * LDPC Q factor = (n_ldpc - k_ldpc) / 360
 *
 * n_bch = k_bch + bch_parity_bits
 * k_ldpc = xs + n_bch: the BCH codeword is prepended with xs shortening
 * zeros (not transmitted) to form the LDPC information word, exactly as
 * in ETSI EN 302 307 and gr-dtv.  For modes with xs = 0, k_ldpc = n_bch.
 */
static const struct dvbs2x_modcod
vlsnr_modcod_table[DVBS2X_VLSNR_NUM_MODCODS] = {
	/* Index 1: QPSK 2/9, normal frame */
	{
		.index		= 1,
		.modulation	= DVBS2X_MOD_QPSK,
		.frame_type	= DVBS2X_FRAME_NORMAL,
		.fec_len	= 64800,
		.code_rate_num	= 2,
		.code_rate_den	= 9,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET1,
		.pls_code	= DVBS2X_PLS_VLSNR_SET1,
		.k_bch		= 14208,
		.n_bch		= 14400,
		.bch_t		= 12,
		.k_ldpc		= 14400,
		.q_ldpc		= 140,
		.xs		= 0,
		.xp		= 3240,
		.p_period	= 15,
		.nbch_eff	= 14400,
	},
	/* Index 2: BPSK 1/5, medium frame */
	{
		.index		= 2,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_MEDIUM,
		.fec_len	= 32400,
		.code_rate_num	= 1,
		.code_rate_den	= 5,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET1,
		.pls_code	= DVBS2X_PLS_VLSNR_SET1,
		.k_bch		= 5660,
		.n_bch		= 5840,
		.bch_t		= 12,
		.k_ldpc		= 6480,
		.q_ldpc		= 72,
		.xs		= 640,
		.xp		= 980,
		.p_period	= 25,
		.nbch_eff	= 5840,
	},
	/* Index 3: BPSK 11/45, medium frame */
	{
		.index		= 3,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_MEDIUM,
		.fec_len	= 32400,
		.code_rate_num	= 11,
		.code_rate_den	= 45,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET1,
		.pls_code	= DVBS2X_PLS_VLSNR_SET1,
		.k_bch		= 7740,
		.n_bch		= 7920,
		.bch_t		= 12,
		.k_ldpc		= 7920,
		.q_ldpc		= 68,
		.xs		= 0,
		.xp		= 1620,
		.p_period	= 15,
		.nbch_eff	= 7920,
	},
	/* Index 4: BPSK 1/3, medium frame */
	{
		.index		= 4,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_MEDIUM,
		.fec_len	= 32400,
		.code_rate_num	= 1,
		.code_rate_den	= 3,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET1,
		.pls_code	= DVBS2X_PLS_VLSNR_SET1,
		.k_bch		= 10620,
		.n_bch		= 10800,
		.bch_t		= 12,
		.k_ldpc		= 10800,
		.q_ldpc		= 60,
		.xs		= 0,
		.xp		= 1620,
		.p_period	= 13,
		.nbch_eff	= 10800,
	},
	/* Index 5: BPSK-S 1/5, short frame (spread) */
	{
		.index		= 5,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_SHORT,
		.fec_len	= 16200,
		.code_rate_num	= 1,
		.code_rate_den	= 5,
		.has_spread	= 1,
		.set		= DVBS2X_VLSNR_SET1,
		.pls_code	= DVBS2X_PLS_VLSNR_SET1,
		.k_bch		= 2512,
		.n_bch		= 2680,
		.bch_t		= 12,
		.k_ldpc		= 3240,
		.q_ldpc		= 36,
		.xs		= 560,
		.xp		= 250,
		.p_period	= 30,
		.nbch_eff	= 2680,
	},
	/* Index 6: BPSK-S 11/45, short frame (spread) */
	{
		.index		= 6,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_SHORT,
		.fec_len	= 16200,
		.code_rate_num	= 11,
		.code_rate_den	= 45,
		.has_spread	= 1,
		.set		= DVBS2X_VLSNR_SET1,
		.pls_code	= DVBS2X_PLS_VLSNR_SET1,
		.k_bch		= 3792,
		.n_bch		= 3960,
		.bch_t		= 12,
		.k_ldpc		= 3960,
		.q_ldpc		= 34,
		.xs		= 0,
		.xp		= 810,
		.p_period	= 15,
		.nbch_eff	= 3960,
	},
	/* Index 7: BPSK 1/5, short frame */
	{
		.index		= 7,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_SHORT,
		.fec_len	= 16200,
		.code_rate_num	= 1,
		.code_rate_den	= 5,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET2,
		.pls_code	= DVBS2X_PLS_VLSNR_SET2,
		.k_bch		= 3072,
		.n_bch		= 3240,
		.bch_t		= 12,
		.k_ldpc		= 3240,
		.q_ldpc		= 36,
		.xs		= 0,
		.xp		= 1224,
		.p_period	= 10,
		.nbch_eff	= 3240,
	},
	/* Index 8: BPSK 4/15, short frame */
	{
		.index		= 8,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_SHORT,
		.fec_len	= 16200,
		.code_rate_num	= 4,
		.code_rate_den	= 15,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET2,
		.pls_code	= DVBS2X_PLS_VLSNR_SET2,
		.k_bch		= 4152,
		.n_bch		= 4320,
		.bch_t		= 12,
		.k_ldpc		= 4320,
		.q_ldpc		= 33,
		.xs		= 0,
		.xp		= 1224,
		.p_period	= 8,
		.nbch_eff	= 4320,
	},
	/* Index 9: BPSK 1/3, short frame */
	{
		.index		= 9,
		.modulation	= DVBS2X_MOD_BPSK,
		.frame_type	= DVBS2X_FRAME_SHORT,
		.fec_len	= 16200,
		.code_rate_num	= 1,
		.code_rate_den	= 3,
		.has_spread	= 0,
		.set		= DVBS2X_VLSNR_SET2,
		.pls_code	= DVBS2X_PLS_VLSNR_SET2,
		.k_bch		= 5232,
		.n_bch		= 5400,
		.bch_t		= 12,
		.k_ldpc		= 5400,
		.q_ldpc		= 30,
		.xs		= 0,
		.xp		= 1224,
		.p_period	= 8,
		.nbch_eff	= 5400,
	},
};

static const char * const vlsnr_modcod_names[DVBS2X_VLSNR_NUM_MODCODS] = {
	"QPSK 2/9 normal set1",
	"BPSK 1/5 medium set1",
	"BPSK 11/45 medium set1",
	"BPSK 1/3 medium set1",
	"BPSK-S 1/5 short set1 SF2",
	"BPSK-S 11/45 short set1 SF2",
	"BPSK 1/5 short set2",
	"BPSK 4/15 short set2",
	"BPSK 1/3 short set2",
};

const struct dvbs2x_modcod *dvbs2x_vlsnr_get_modcod(unsigned int index)
{
	if (index < 1 || index > DVBS2X_VLSNR_NUM_MODCODS)
		return NULL;
	return &vlsnr_modcod_table[index - 1];
}

const char *dvbs2x_vlsnr_get_modcod_name(unsigned int index)
{
	if (index < 1 || index > DVBS2X_VLSNR_NUM_MODCODS)
		return NULL;
	return vlsnr_modcod_names[index - 1];
}

const struct dvbs2x_modcod *dvbs2x_vlsnr_get_modcod_by_name(const char *name)
{
	static const struct {
		const char *name;
		unsigned int index;
	} legacy_names[] = {
		{ "QPSK 2/9", 1 },
		{ "BPSK 11/45", 3 },
		{ "BPSK-S 1/5", 5 },
		{ "BPSK-S 11/45", 6 },
		{ "BPSK 4/15", 8 },
	};
	unsigned int i;

	if (!name)
		return NULL;

	for (i = 0; i < DVBS2X_VLSNR_NUM_MODCODS; i++) {
		if (strcmp(name, vlsnr_modcod_names[i]) == 0)
			return &vlsnr_modcod_table[i];
	}
	for (i = 0; i < sizeof(legacy_names) / sizeof(legacy_names[0]); i++) {
		if (strcmp(name, legacy_names[i].name) == 0)
			return dvbs2x_vlsnr_get_modcod(legacy_names[i].index);
	}
	return NULL;
}

const struct dvbs2x_modcod *dvbs2x_vlsnr_get_modcod_table(void)
{
	return vlsnr_modcod_table;
}
