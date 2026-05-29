/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Top-Level API
 *
 * Provides the complete modulator and demodulator pipelines
 * for DVB-S2X VL-SNR operation.
 */

#ifndef DVBS2X_VLSNR_H
#define DVBS2X_VLSNR_H

#include "dvbs2x_types.h"
#include "dvbs2x_modcod.h"
#include "bch.h"
#include "ldpc.h"
#include "modulator.h"
#include "plheader.h"
#include "vlsnr_header.h"
#include "pilot.h"
#include "scrambler.h"
#include "interleaver.h"
#include "bb_frame.h"
#include "filter.h"
#include "sync.h"

/* Modulator pipeline context */
struct dvbs2x_modulator {
	struct dvbs2x_mod_cfg		cfg;
	struct dvbs2x_bb_frame_ctx	bb_ctx;
	struct dvbs2x_bch_encoder	bch_enc;
	struct dvbs2x_ldpc_encoder	ldpc_enc;
	struct dvbs2x_scrambler		scrambler;
	struct dvbs2x_rrc_filter	tx_filter;
};

/* Demodulator pipeline context */
struct dvbs2x_demodulator {
	const struct dvbs2x_modcod	*modcod;
	struct dvbs2x_rrc_filter	rx_filter;
	struct dvbs2x_timing_sync	timing;
	struct dvbs2x_freq_coarse	freq_coarse;
	struct dvbs2x_freq_fine		freq_fine;
	struct dvbs2x_phase_est		phase;
	struct dvbs2x_scrambler		descrambler;
	struct dvbs2x_ldpc_decoder	ldpc_dec;
	struct dvbs2x_bch_decoder	bch_dec;
	struct dvbs2x_bb_frame_ctx	bb_ctx;
	unsigned int			sync_frames;
	unsigned int			frame_count;
};

/*
 * dvbs2x_modulator_init - Initialize modulator
 * @mod: modulator context
 * @modcod_idx: MODCOD index (1-9)
 * @rolloff: roll-off factor (e.g. 0.35)
 * @sps: samples per symbol (e.g. 2)
 * @pl_scrambling_idx: PL scrambling index (0-7)
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_modulator_init(struct dvbs2x_modulator *mod,
			  unsigned int modcod_idx,
			  double rolloff,
			  unsigned int sps,
			  unsigned int pl_scrambling_idx);

/*
 * dvbs2x_modulate - Modulate user data into baseband IQ samples
 * @mod: initialized modulator
 * @user_data: input user data bits
 * @user_len: user data length in bits
 * @output: output IQ samples
 * @out_len: number of output samples produced
 *
 * Performs the complete chain: BB frame -> BCH -> LDPC -> interleave ->
 * modulate -> PL frame build -> scramble -> pulse shape.
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_modulate(struct dvbs2x_modulator *mod,
		    const uint8_t *user_data,
		    unsigned int user_len,
		    struct dvbs2x_complex *output,
		    unsigned int *out_len);

/*
 * dvbs2x_demodulator_init - Initialize demodulator
 * @demod: demodulator context
 * @rolloff: expected roll-off factor
 * @sps: samples per symbol
 * @pl_scrambling_idx: PL scrambling index
 *
 * Returns 0 on success, -1 on error.
 */
int dvbs2x_demodulator_init(struct dvbs2x_demodulator *demod,
			    double rolloff,
			    unsigned int sps,
			    unsigned int pl_scrambling_idx);

/*
 * dvbs2x_demodulate - Demodulate baseband IQ samples to user data
 * @demod: initialized demodulator
 * @input: input IQ samples
 * @in_len: number of input samples
 * @user_data: output user data bits
 * @user_len: output user data length in bits
 *
 * Performs: matched filter -> sync -> descramble -> demap ->
 * deinterleave -> LDPC decode -> BCH decode -> BB frame parse.
 *
 * Returns 0 on success, -1 on decode failure.
 */
int dvbs2x_demodulate(struct dvbs2x_demodulator *demod,
		      const struct dvbs2x_complex *input,
		      unsigned int in_len,
		      uint8_t *user_data,
		      unsigned int *user_len);

#endif /* DVBS2X_VLSNR_H */
