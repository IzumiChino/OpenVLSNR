/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Top-Level API
 *
 * Provides the complete modulator and demodulator pipelines
 * for DVB-S2X VL-SNR operation per ETSI EN 302 307-2.
 */

#ifndef DVBS2X_VLSNR_H
#define DVBS2X_VLSNR_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* Library version */
#define DVBS2X_VERSION_MAJOR	3
#define DVBS2X_VERSION_MINOR	0
#define DVBS2X_VERSION_PATCH	0

/*
 * dvbs2x_library_init - Initialize library-wide static tables
 *
 * Call once before using any other API function.  Initializes the
 * LDPC box-plus LUT and VL-SNR header reference sequences.  This
 * function is NOT thread-safe; call it from a single thread at
 * program startup (analogous to a module_init in the kernel).
 *
 * Safe to call multiple times (idempotent).
 */
void dvbs2x_library_init(void);

/*
 * dvbs2x_version_string - Return library version as "major.minor.patch"
 */
const char *dvbs2x_version_string(void);

/* Modulator pipeline context */
struct dvbs2x_modulator {
	struct dvbs2x_mod_cfg		cfg;
	struct dvbs2x_bb_frame_ctx	bb_ctx;
	struct dvbs2x_bch_encoder	bch_enc;
	struct dvbs2x_ldpc_encoder	ldpc_enc;
	struct dvbs2x_scrambler		scrambler;
	struct dvbs2x_rrc_filter	tx_filter;
};

/* Diagnostics from the most recent physical-layer decode attempt. */
struct dvbs2x_demod_stats {
	unsigned int	modcod;
	double		sync_confidence;
	double		esn0_db;
	unsigned int	ldpc_iterations;
	int		result;
};

/* Demodulator sync state machine */
enum dvbs2x_demod_state {
	DVBS2X_DEMOD_SEARCH = 0,	/* initial acquisition */
	DVBS2X_DEMOD_ACQUIRE,		/* AFC accumulating */
	DVBS2X_DEMOD_TRACK,		/* locked, predicted boundaries */
};

/* Demodulator pipeline context */
struct dvbs2x_demodulator {
	const struct dvbs2x_modcod	*modcod;
	struct dvbs2x_rrc_filter	rx_filter;
	struct dvbs2x_timing_sync	timing;
	struct dvbs2x_freq_coarse	freq_coarse;
	struct dvbs2x_freq_fine		freq_fine;
	struct dvbs2x_phase_est		phase;
	struct dvbs2x_afc		afc;
	struct dvbs2x_scrambler		descrambler;
	struct dvbs2x_ldpc_decoder	ldpc_dec;
	struct dvbs2x_bch_decoder	bch_dec;
	struct dvbs2x_bb_frame_ctx	bb_ctx;
	/* State machine */
	enum dvbs2x_demod_state		state;
	unsigned int			consecutive_successes;
	unsigned int			consecutive_failures;
	/* Continuous-mode persistent state */
	unsigned int			expected_frame_len;
	struct dvbs2x_complex		*stream_buf;
	unsigned int			stream_len;
	unsigned int			stream_cap;
	struct dvbs2x_demod_stats	last_stats;
	volatile int			cancel_requested;
};

/*
 * dvbs2x_modulator_init - Initialize modulator
 * @mod: modulator context
 * @modcod_idx: MODCOD index (1-9)
 * @rolloff: roll-off factor (e.g. 0.35)
 * @sps: samples per symbol (e.g. 2)
 * @pl_scrambling_idx: PL scrambling index (0-7)
 *
 * Returns 0 on success, a negative DVBS2X_ERR_* code on failure.
 */
int dvbs2x_modulator_init(struct dvbs2x_modulator *mod,
			  unsigned int modcod_idx,
			  double rolloff,
			  unsigned int sps,
			  unsigned int pl_scrambling_idx);

/*
 * dvbs2x_modulator_destroy - Release modulator resources
 * @mod: modulator context previously initialized with _init
 *
 * Currently a no-op (all modulator state is inline), but should
 * be called for forward compatibility.
 */
void dvbs2x_modulator_destroy(struct dvbs2x_modulator *mod);

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
 * Returns 0 on success, a negative DVBS2X_ERR_* code on failure.
 */
int dvbs2x_modulate(struct dvbs2x_modulator *mod,
		    const uint8_t *user_data,
		    unsigned int user_len,
		    struct dvbs2x_complex *output,
		    unsigned int *out_len);

/* Capacity-checked variant of dvbs2x_modulate(). */
int dvbs2x_modulate_ex(struct dvbs2x_modulator *mod,
		       const uint8_t *user_data,
		       unsigned int user_len,
		       struct dvbs2x_complex *output,
		       unsigned int output_capacity,
		       unsigned int *out_len);

/*
 * dvbs2x_modulate_symbols - Build the PL frame at symbol rate
 * @mod: initialized modulator
 * @user_data: input user data bits
 * @user_len: user data length in bits
 * @symbols: output complex symbols (caller-allocated, see below)
 * @sym_len: number of symbols produced
 *
 * Performs everything except RRC pulse shaping, producing the
 * symbol-rate PL frame.  The output buffer must hold at least
 * PLHEADER + VLSNR header + fec_len*2 + fec_len/360 + 64 symbols.
 *
 * Returns 0 on success, a negative DVBS2X_ERR_* code on failure.
 */
int dvbs2x_modulate_symbols(struct dvbs2x_modulator *mod,
			    const uint8_t *user_data,
			    unsigned int user_len,
			    struct dvbs2x_complex *symbols,
			    unsigned int *sym_len);

/* Capacity-checked variant of dvbs2x_modulate_symbols(). */
int dvbs2x_modulate_symbols_ex(struct dvbs2x_modulator *mod,
			       const uint8_t *user_data,
			       unsigned int user_len,
			       struct dvbs2x_complex *symbols,
			       unsigned int symbol_capacity,
			       unsigned int *sym_len);

/* Modulate an already mode-adapted and BB-scrambled k_bch-bit BBFRAME. */
int dvbs2x_modulate_bbframe_symbols_ex(struct dvbs2x_modulator *mod,
				       const uint8_t *bbframe,
				       struct dvbs2x_complex *symbols,
				       unsigned int symbol_capacity,
				       unsigned int *sym_len);

/*
 * dvbs2x_demodulator_init - Initialize demodulator
 * @demod: demodulator context
 * @rolloff: expected roll-off factor
 * @sps: samples per symbol
 * @pl_scrambling_idx: PL scrambling index
 *
 * Returns 0 on success, a negative DVBS2X_ERR_* code on failure.
 */
int dvbs2x_demodulator_init(struct dvbs2x_demodulator *demod,
			    double rolloff,
			    unsigned int sps,
			    unsigned int pl_scrambling_idx);

/* Recover the mode-adapted, BB-scrambled k_bch-bit BBFRAME. */
int dvbs2x_demodulate_bbframe_symbols_ex(struct dvbs2x_demodulator *demod,
					 const struct dvbs2x_complex *input,
					 unsigned int in_len,
					 double noise_var,
					 uint8_t *bbframe,
					 unsigned int frame_capacity,
					 unsigned int *frame_len);

/* Recover a mode-adapted BBFRAME from pulse-shaped baseband samples. */
int dvbs2x_demodulate_bbframe_ex(struct dvbs2x_demodulator *demod,
				 const struct dvbs2x_complex *input,
				 unsigned int in_len,
				 uint8_t *bbframe,
				 unsigned int frame_capacity,
				 unsigned int *frame_len);

/*
 * dvbs2x_demodulator_destroy - Release demodulator resources
 * @demod: demodulator context previously initialized with _init
 *
 * Frees the cached LDPC parity-check matrix and pre-allocated
 * decoder working memory.  Safe to call multiple times.
 */
void dvbs2x_demodulator_destroy(struct dvbs2x_demodulator *demod);

/* Copy diagnostics from the most recent decode attempt. */
int dvbs2x_demodulator_get_stats(const struct dvbs2x_demodulator *demod,
				 struct dvbs2x_demod_stats *stats);

/* Request cancellation of a long-running receive operation. */
void dvbs2x_demodulator_request_cancel(struct dvbs2x_demodulator *demod);

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
 * Returns 0 on success, a negative DVBS2X_ERR_* code on failure.
 */
int dvbs2x_demodulate(struct dvbs2x_demodulator *demod,
		      const struct dvbs2x_complex *input,
		      unsigned int in_len,
		      uint8_t *user_data,
		      unsigned int *user_len);

/* Capacity-checked variant of dvbs2x_demodulate(). */
int dvbs2x_demodulate_ex(struct dvbs2x_demodulator *demod,
			 const struct dvbs2x_complex *input,
			 unsigned int in_len,
			 uint8_t *user_data,
			 unsigned int user_capacity,
			 unsigned int *user_len);

/*
 * dvbs2x_demodulate_symbols - Demodulate a symbol-rate stream
 * @demod: initialized demodulator
 * @input: input complex symbols (symbol rate, post matched filter)
 * @in_len: number of input symbols
 * @noise_var: per-component noise variance estimate (0 = auto)
 * @user_data: output user data bits
 * @user_len: output user data length in bits
 *
 * Performs: frame sync -> WH-aided carrier recovery -> descramble ->
 * pilot-aided phase -> pilot extract -> demap -> deinterleave ->
 * depuncture -> LDPC decode -> BCH decode -> BB frame parse.
 *
 * This is the symbol-level core used by dvbs2x_demodulate() after
 * matched filtering and timing recovery.  Exposed for testing the
 * receiver independently of the RF front-end.
 *
 * Returns 0 on success, a negative DVBS2X_ERR_* code on failure.
 */
int dvbs2x_demodulate_symbols(struct dvbs2x_demodulator *demod,
			      const struct dvbs2x_complex *input,
			      unsigned int in_len,
			      double noise_var,
			      uint8_t *user_data,
			      unsigned int *user_len);

/* Capacity-checked variant of dvbs2x_demodulate_symbols(). */
int dvbs2x_demodulate_symbols_ex(struct dvbs2x_demodulator *demod,
				 const struct dvbs2x_complex *input,
				 unsigned int in_len,
				 double noise_var,
				 uint8_t *user_data,
				 unsigned int user_capacity,
				 unsigned int *user_len);

/*
 * dvbs2x_demodulate_stream - Continuous-mode streaming demodulation
 * @demod: initialized demodulator (state persists across calls)
 * @input: input IQ samples at configured sps rate
 * @in_len: number of input samples available
 * @user_data: output user data bits (one decoded frame)
 * @user_len: output user data length in bits
 * @consumed: number of input samples consumed (caller advances)
 *
 * Processes a stream of samples, attempting to decode one frame per
 * call.  The demodulator maintains lock state across calls:
 *
 *   SEARCH:  full acquisition window scan, wide timing estimate
 *   ACQUIRE: AFC accumulating, narrowing frequency estimate
 *   TRACK:   predicted frame boundaries, persistent timing/carrier
 *
 * Input is copied into an internal read-ahead buffer.  On return,
 * *consumed is either in_len (the samples are owned by the demodulator)
 * or zero if they could not be buffered.  At most one frame is returned
 * per call.  Call again with in_len == 0 to drain buffered frames.
 *
 * DVBS2X_ERR_SHORT means that no complete frame is buffered yet.  The
 * caller should retain no consumed samples and should feed only new,
 * contiguous input on the next call.
 *
 * Returns 0 on a successful frame decode, a negative DVBS2X_ERR_*
 * code on failure.
 */
int dvbs2x_demodulate_stream(struct dvbs2x_demodulator *demod,
			     const struct dvbs2x_complex *input,
			     unsigned int in_len,
			     uint8_t *user_data,
			     unsigned int *user_len,
			     unsigned int *consumed);

/* Capacity-checked variant of dvbs2x_demodulate_stream(). */
int dvbs2x_demodulate_stream_ex(struct dvbs2x_demodulator *demod,
				const struct dvbs2x_complex *input,
				unsigned int in_len,
				uint8_t *user_data,
				unsigned int user_capacity,
				unsigned int *user_len,
				unsigned int *consumed);

/* Continuous-mode variant returning the mode-adapted BBFRAME. */
int dvbs2x_demodulate_bbframe_stream_ex(struct dvbs2x_demodulator *demod,
					const struct dvbs2x_complex *input,
					unsigned int in_len,
					uint8_t *bbframe,
					unsigned int frame_capacity,
					unsigned int *frame_len,
					unsigned int *consumed);

#ifdef __cplusplus
}

#endif

#endif /* DVBS2X_VLSNR_H */
