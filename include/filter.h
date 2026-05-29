/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR RRC Pulse Shaping Filter
 *
 * Root Raised Cosine filter for transmit pulse shaping
 * and receive matched filtering.
 *
 * Reference: ETSI EN 302 307-1 clause 4.2
 */

#ifndef DVBS2X_FILTER_H
#define DVBS2X_FILTER_H

#include "dvbs2x_types.h"

/* Maximum filter length (taps) */
#define DVBS2X_FILTER_MAX_TAPS	256

/* RRC filter state */
struct dvbs2x_rrc_filter {
	double		coeffs[DVBS2X_FILTER_MAX_TAPS];
	double		delay_i[DVBS2X_FILTER_MAX_TAPS];
	double		delay_q[DVBS2X_FILTER_MAX_TAPS];
	unsigned int	num_taps;
	unsigned int	sps;		/* samples per symbol */
	double		rolloff;
	unsigned int	delay_idx;
};

/*
 * dvbs2x_rrc_filter_init - Design and initialize RRC filter
 * @flt: filter state
 * @rolloff: roll-off factor (0.05 to 0.35)
 * @sps: samples per symbol (oversampling factor)
 * @span: filter span in symbols (each side)
 *
 * Total taps = 2 * span * sps + 1
 * Returns 0 on success, -1 if parameters exceed max taps.
 */
int dvbs2x_rrc_filter_init(struct dvbs2x_rrc_filter *flt,
			   double rolloff, unsigned int sps,
			   unsigned int span);

/*
 * dvbs2x_rrc_filter_reset - Clear filter delay line
 * @flt: filter state
 */
void dvbs2x_rrc_filter_reset(struct dvbs2x_rrc_filter *flt);

/*
 * dvbs2x_rrc_upsample - Upsample and filter (transmit)
 * @flt: filter state
 * @in: input symbols (at symbol rate)
 * @in_len: number of input symbols
 * @out: output samples (at sps * symbol rate)
 * @out_len: number of output samples produced
 */
void dvbs2x_rrc_upsample(struct dvbs2x_rrc_filter *flt,
			  const struct dvbs2x_complex *in,
			  unsigned int in_len,
			  struct dvbs2x_complex *out,
			  unsigned int *out_len);

/*
 * dvbs2x_rrc_filter_apply - Apply matched filter (receive)
 * @flt: filter state
 * @in: input samples
 * @in_len: number of input samples
 * @out: filtered output samples
 * @out_len: number of output samples
 */
void dvbs2x_rrc_filter_apply(struct dvbs2x_rrc_filter *flt,
			     const struct dvbs2x_complex *in,
			     unsigned int in_len,
			     struct dvbs2x_complex *out,
			     unsigned int *out_len);

#endif /* DVBS2X_FILTER_H */
