/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Synchronization
 *
 * Implements the receiver synchronization chain:
 * 1. Symbol timing recovery (Gardner TED)
 * 2. Coarse frequency estimation (BQ-FED in FLL)
 * 3. Frame synchronization (WH header correlation)
 * 4. Fine frequency estimation (pilot-aided)
 * 5. Phase estimation (pilot-aided)
 *
 * Reference: ETSI TR 102 376-1 (implementation guidelines)
 */

#ifndef DVBS2X_SYNC_H
#define DVBS2X_SYNC_H

#include "dvbs2x_types.h"

/* Symbol timing recovery state */
struct dvbs2x_timing_sync {
	double		mu;		/* fractional delay */
	double		loop_bw;	/* normalized loop bandwidth */
	double		kp;		/* proportional gain */
	double		ki;		/* integral gain */
	double		integrator;
	unsigned int	sps;		/* samples per symbol */
};

/* Coarse frequency estimator (FLL) state */
struct dvbs2x_freq_coarse {
	double		freq_est;	/* current frequency estimate */
	double		loop_bw;	/* normalized loop bandwidth */
	double		phase_acc;	/* phase accumulator */
	double		integrator;
};

/* Fine frequency estimator state */
struct dvbs2x_freq_fine {
	double		freq_est;
	unsigned int	num_lags;	/* correlation lags */
	double		corr_i;		/* accumulated correlation I */
	double		corr_q;		/* accumulated correlation Q */
	unsigned int	frame_count;
};

/* Phase estimator state */
struct dvbs2x_phase_est {
	double		phase;
	double		alpha;		/* filter coefficient */
	int		need_smooth;	/* enable smoothing filter */
};

/* Multi-frame AFC (Automatic Frequency Control) state */
struct dvbs2x_afc {
	double		freq_est;	/* current estimate (cycles/sym) */
	double		acc_i;		/* coherent single-lag accumulator I */
	double		acc_q;		/* coherent single-lag accumulator Q */
	unsigned int	frames;		/* frames accumulated */
	int		locked;		/* convergence flag */
	double		alpha;		/* IIR decay factor */
};

/*
 * dvbs2x_timing_sync_init - Initialize symbol timing recovery
 * @ts: timing sync state
 * @sps: samples per symbol
 * @loop_bw: normalized loop bandwidth (e.g. 1e-4)
 */
void dvbs2x_timing_sync_init(struct dvbs2x_timing_sync *ts,
			     unsigned int sps, double loop_bw);

/*
 * dvbs2x_timing_sync_process - Process samples and output symbols
 * @ts: timing sync state
 * @in: input samples
 * @in_len: number of input samples
 * @out: output symbols (at symbol rate)
 * @out_len: number of output symbols produced
 */
void dvbs2x_timing_sync_process(struct dvbs2x_timing_sync *ts,
				const struct dvbs2x_complex *in,
				unsigned int in_len,
				struct dvbs2x_complex *out,
				unsigned int *out_len);

/*
 * dvbs2x_freq_coarse_init - Initialize coarse frequency estimator
 * @fc: frequency estimator state
 * @loop_bw: normalized loop bandwidth
 */
void dvbs2x_freq_coarse_init(struct dvbs2x_freq_coarse *fc,
			     double loop_bw);

/*
 * dvbs2x_freq_coarse_process - Apply coarse frequency correction
 * @fc: frequency estimator state
 * @symbols: symbols to correct (modified in place)
 * @len: number of symbols
 */
void dvbs2x_freq_coarse_process(struct dvbs2x_freq_coarse *fc,
				struct dvbs2x_complex *symbols,
				unsigned int len);

/*
 * dvbs2x_freq_fine_init - Initialize fine frequency estimator
 * @ff: fine frequency estimator state
 * @num_lags: number of correlation lags
 */
void dvbs2x_freq_fine_init(struct dvbs2x_freq_fine *ff,
			   unsigned int num_lags);

/*
 * dvbs2x_freq_fine_estimate - Estimate fine frequency offset from pilots
 * @ff: fine frequency estimator state
 * @pilots: extracted pilot symbols
 * @ref_pilots: reference pilot symbols
 * @num_pilots: number of pilot symbols
 *
 * Returns estimated frequency offset (normalized to symbol rate).
 */
double dvbs2x_freq_fine_estimate(struct dvbs2x_freq_fine *ff,
				 const struct dvbs2x_complex *pilots,
				 const struct dvbs2x_complex *ref_pilots,
				 unsigned int num_pilots);

/*
 * dvbs2x_phase_est_init - Initialize phase estimator
 * @pe: phase estimator state
 * @alpha: filter coefficient (< 0.5 for Es/N0 < -8 dB)
 * @smooth: enable moving average smoothing
 */
void dvbs2x_phase_est_init(struct dvbs2x_phase_est *pe,
			   double alpha, int smooth);

/*
 * dvbs2x_phase_est_process - Estimate and correct phase
 * @pe: phase estimator state
 * @symbols: symbols to correct (modified in place)
 * @pilots: pilot symbols for estimation
 * @ref_pilots: reference pilot symbols
 * @num_pilots: number of pilot symbols
 * @num_symbols: total symbols to correct
 */
void dvbs2x_phase_est_process(struct dvbs2x_phase_est *pe,
			      struct dvbs2x_complex *symbols,
			      const struct dvbs2x_complex *pilots,
			      const struct dvbs2x_complex *ref_pilots,
			      unsigned int num_pilots,
			      unsigned int num_symbols);

/*
 * dvbs2x_afc_init - Initialize multi-frame AFC
 * @afc: AFC state
 * @alpha: IIR decay factor (0.9-0.99, higher = more averaging)
 */
void dvbs2x_afc_init(struct dvbs2x_afc *afc, double alpha);

/*
 * dvbs2x_afc_update - Update AFC from WH header correlation
 * @afc: AFC state
 * @sym: received symbols (after pre-correction)
 * @wh_start: header start offset in sym[]
 * @wh_ref: reference WH symbols (DVBS2X_VLSNR_WH_LEN)
 *
 * Computes single-lag correlation on the de-rotated header and
 * updates the coherent accumulator.  The frequency estimate is
 * refined after each frame.
 */
void dvbs2x_afc_update(struct dvbs2x_afc *afc,
			const struct dvbs2x_complex *sym,
			unsigned int wh_start,
			const struct dvbs2x_complex *wh_ref);

#endif /* DVBS2X_SYNC_H */
