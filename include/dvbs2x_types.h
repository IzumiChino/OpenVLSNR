/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Common Types and Constants
 *
 * Reference: ETSI EN 302 307-2 V1.3.1
 */

#ifndef DVBS2X_TYPES_H
#define DVBS2X_TYPES_H

#include <stdint.h>
#include <stddef.h>

/* Frame lengths (LDPC codeword bits) */
#define DVBS2X_LDPC_NORMAL	64800
#define DVBS2X_LDPC_MEDIUM	32400
#define DVBS2X_LDPC_SHORT	16200

/* PL Header */
#define DVBS2X_SOF_LEN		26
#define DVBS2X_PLSC_LEN		64
#define DVBS2X_PLHEADER_LEN	90

/* VL-SNR Header (896-symbol Walsh-Hadamard sequence) */
#define DVBS2X_VLSNR_WH_LEN	896
#define DVBS2X_VLSNR_HDR_LEN	DVBS2X_VLSNR_WH_LEN

/* VL-SNR frame symbol lengths */
#define DVBS2X_VLSNR_FRAME_LONG		33282
#define DVBS2X_VLSNR_FRAME_SHORT	16686

/* Pilot block */
#define DVBS2X_PILOT_BLK_LEN	36

/* PLS decimal codes for VL-SNR */
#define DVBS2X_PLS_VLSNR_SET1	129
#define DVBS2X_PLS_VLSNR_SET2	131

/* Maximum LDPC iterations */
#define DVBS2X_LDPC_MAX_ITER	50

/* LDPC encoding parallelism factor */
#define DVBS2X_LDPC_PARALLEL	360

/* Roll-off factor (default) */
#define DVBS2X_ROLLOFF_DEFAULT	0.35

/* Complex sample (IQ) */
struct dvbs2x_complex {
	double i;
	double q;
};

/* Modulation type */
enum dvbs2x_mod {
	DVBS2X_MOD_BPSK = 0,
	DVBS2X_MOD_QPSK,
};

/* Frame type */
enum dvbs2x_frame_type {
	DVBS2X_FRAME_NORMAL = 0,	/* 64800 bits */
	DVBS2X_FRAME_MEDIUM,		/* 32400 bits */
	DVBS2X_FRAME_SHORT,		/* 16200 bits */
};

/* VL-SNR set */
enum dvbs2x_vlsnr_set {
	DVBS2X_VLSNR_SET1 = 0,		/* PLSDecimalCode 129 */
	DVBS2X_VLSNR_SET2,		/* PLSDecimalCode 131 */
};

/* MODCOD index (1-9 for VL-SNR) */
struct dvbs2x_modcod {
	unsigned int		index;
	enum dvbs2x_mod		modulation;
	enum dvbs2x_frame_type	frame_type;
	unsigned int		fec_len;	/* LDPC codeword length */
	unsigned int		code_rate_num;
	unsigned int		code_rate_den;
	int			has_spread;	/* spreading factor 2 */
	enum dvbs2x_vlsnr_set	set;
	unsigned int		pls_code;
	unsigned int		k_bch;		/* BCH input bits */
	unsigned int		n_bch;		/* BCH output bits */
	unsigned int		bch_t;		/* BCH correction capability */
	unsigned int		k_ldpc;		/* LDPC info bits = n_bch */
	unsigned int		q_ldpc;		/* LDPC circulant size */
	/* Puncturing/shortening parameters */
	unsigned int		xs;		/* shortening: zero-prepend */
	unsigned int		xp;		/* puncturing: removed parity */
	unsigned int		p_period;	/* puncturing period */
	unsigned int		nbch_eff;	/* effective BCH (k_ldpc-xs) */
};

/*
 * Transmitted coded bits after shortening and puncturing.
 * The LDPC codeword (fec_len bits) has xs info bits removed
 * (shortened, known zero) and xp parity bits removed (punctured).
 */
static inline unsigned int dvbs2x_tx_coded_bits(const struct dvbs2x_modcod *mc)
{
	return mc->fec_len - mc->xs - mc->xp;
}

/* Modulator configuration */
struct dvbs2x_mod_cfg {
	const struct dvbs2x_modcod	*modcod;
	double				rolloff;
	unsigned int			sps;	/* samples per symbol */
	unsigned int			pl_scrambling_idx;
};

/* BB frame header (per ETSI EN 302 307-1 clause 5.1) */
struct dvbs2x_bb_header {
	uint8_t		matype1;
	uint8_t		matype2;
	uint16_t	upl;		/* user packet length */
	uint16_t	dfl;		/* data field length */
	uint8_t		sync;
	uint16_t	syncd;
	uint8_t		crc8;
};

#define DVBS2X_BB_HEADER_LEN	10	/* bytes */

/* Number of VL-SNR MODCODs */
#define DVBS2X_VLSNR_NUM_MODCODS	9

#endif /* DVBS2X_TYPES_H */
