// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR LDPC Decoder
 *
 * Layered offset min-sum belief propagation decoder.
 *
 * The layered schedule processes one check node row at a time,
 * updating variable node messages immediately. This converges
 * roughly twice as fast as the flooding schedule.
 *
 * Offset min-sum approximates the check node update:
 *   min(|a|, |b|) - offset  (clamped to 0)
 * instead of the exact tanh rule, reducing complexity.
 *
 * Reference: ETSI TR 102 376-1 (implementation guidelines)
 */

#include "ldpc.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Default min-sum offset */
#define LDPC_MS_OFFSET	0.15

/* Maximum check node degree */
#define MAX_CN_DEGREE	32

/*
 * Build the parity check matrix structure from the LDPC table.
 * Returns arrays of check-to-variable connections.
 *
 * The H matrix has structure:
 * - Information part: defined by the address table (quasi-cyclic)
 * - Parity part: dual-diagonal (staircase)
 */
struct ldpc_cn_row {
	unsigned int	degree;
	unsigned int	vn_idx[MAX_CN_DEGREE];
};

static int build_h_matrix(const struct dvbs2x_ldpc_code *code,
			  struct ldpc_cn_row *rows, unsigned int m)
{
	unsigned int group, j, a;
	unsigned int q = code->q;
	unsigned int i;

	/* Initialize all rows */
	for (i = 0; i < m; i++)
		rows[i].degree = 0;

	/* Information part: from address table */
	for (group = 0; group < code->num_groups; group++) {
		const struct dvbs2x_ldpc_table_entry *entry;

		entry = &code->table[group];

		for (j = 0; j < DVBS2X_LDPC_PARALLEL; j++) {
			unsigned int vn = group * DVBS2X_LDPC_PARALLEL + j;

			for (a = 0; a < entry->num_addrs; a++) {
				unsigned int cn;

				cn = (entry->addrs[a] + j * q) % m;
				if (rows[cn].degree >= MAX_CN_DEGREE)
					return -1;
				rows[cn].vn_idx[rows[cn].degree] = vn;
				rows[cn].degree++;
			}
		}
	}

	/* Parity part: dual-diagonal (staircase) */
	/* First parity bit connects to first check */
	if (rows[0].degree >= MAX_CN_DEGREE)
		return -1;
	rows[0].vn_idx[rows[0].degree] = code->k;
	rows[0].degree++;

	/* Subsequent parity bits: p[i] connects to check i and i-1 */
	for (i = 1; i < m; i++) {
		unsigned int pn = code->k + i;

		/* Connection to check i */
		if (rows[i].degree >= MAX_CN_DEGREE)
			return -1;
		rows[i].vn_idx[rows[i].degree] = pn;
		rows[i].degree++;

		/* Connection to check i-1 (from accumulator) */
		if (rows[i - 1].degree >= MAX_CN_DEGREE)
			return -1;
		rows[i - 1].vn_idx[rows[i - 1].degree] = pn;
		rows[i - 1].degree++;
	}

	return 0;
}

int dvbs2x_ldpc_decoder_init(struct dvbs2x_ldpc_decoder *dec,
			     const struct dvbs2x_modcod *modcod,
			     unsigned int max_iter)
{
	unsigned int num_groups;
	const struct dvbs2x_ldpc_table_entry *table;

	table = dvbs2x_ldpc_get_table(modcod, &num_groups);
	if (!table)
		return -1;

	dec->code.n = modcod->fec_len;
	dec->code.k = modcod->k_ldpc;
	dec->code.q = modcod->q_ldpc;
	dec->code.num_groups = num_groups;
	dec->code.table = table;
	dec->max_iter = max_iter;
	dec->offset = LDPC_MS_OFFSET;

	return 0;
}

int dvbs2x_ldpc_decode(const struct dvbs2x_ldpc_decoder *dec,
		       const double *llr, uint8_t *output,
		       unsigned int *iter_used)
{
	unsigned int n = dec->code.n;
	unsigned int k = dec->code.k;
	unsigned int m = n - k;
	struct ldpc_cn_row *rows;
	double *vn_llr;		/* variable node total LLR */
	double *cn_msg;		/* check-to-variable messages */
	unsigned int iter;
	unsigned int i, j;
	int converged;
	int ret = -1;

	/* Allocate working memory */
	rows = calloc(m, sizeof(*rows));
	vn_llr = calloc(n, sizeof(*vn_llr));
	cn_msg = calloc(m * MAX_CN_DEGREE, sizeof(*cn_msg));

	if (!rows || !vn_llr || !cn_msg)
		goto out;

	/* Build H matrix structure */
	if (build_h_matrix(&dec->code, rows, m) < 0)
		goto out;

	/* Initialize variable node LLRs from channel */
	for (i = 0; i < n; i++)
		vn_llr[i] = llr[i];

	/* Initialize check-to-variable messages to zero */
	memset(cn_msg, 0, m * MAX_CN_DEGREE * sizeof(double));

	/* Iterative decoding */
	for (iter = 0; iter < dec->max_iter; iter++) {
		/* Process each check node (layered schedule) */
		for (i = 0; i < m; i++) {
			unsigned int deg = rows[i].degree;
			double vn_to_cn[MAX_CN_DEGREE];
			double sign;
			double min1, min2;
			unsigned int min1_idx;

			/* Compute variable-to-check messages */
			for (j = 0; j < deg; j++) {
				unsigned int vn = rows[i].vn_idx[j];

				vn_to_cn[j] = vn_llr[vn] -
					      cn_msg[i * MAX_CN_DEGREE + j];
			}

			/* Check node update: offset min-sum */
			sign = 1.0;
			min1 = 1e30;
			min2 = 1e30;
			min1_idx = 0;

			for (j = 0; j < deg; j++) {
				double abs_val = fabs(vn_to_cn[j]);

				if (vn_to_cn[j] < 0)
					sign = -sign;

				if (abs_val < min1) {
					min2 = min1;
					min1 = abs_val;
					min1_idx = j;
				} else if (abs_val < min2) {
					min2 = abs_val;
				}
			}

			/* Update check-to-variable messages */
			for (j = 0; j < deg; j++) {
				double msg_sign;
				double msg_abs;
				unsigned int vn = rows[i].vn_idx[j];
				double old_msg;

				/* Sign: product of all other signs */
				msg_sign = sign;
				if (vn_to_cn[j] < 0)
					msg_sign = -msg_sign;

				/* Magnitude: min of all others - offset */
				if (j == min1_idx)
					msg_abs = min2 - dec->offset;
				else
					msg_abs = min1 - dec->offset;

				if (msg_abs < 0.0)
					msg_abs = 0.0;

				/* Store new message */
				old_msg = cn_msg[i * MAX_CN_DEGREE + j];
				cn_msg[i * MAX_CN_DEGREE + j] =
					msg_sign * msg_abs;

				/* Update variable node LLR (layered) */
				vn_llr[vn] += cn_msg[i * MAX_CN_DEGREE + j] -
					      old_msg;
			}
		}

		/* Check convergence: all parity checks satisfied? */
		converged = 1;
		for (i = 0; i < m; i++) {
			int parity = 0;

			for (j = 0; j < rows[i].degree; j++) {
				if (vn_llr[rows[i].vn_idx[j]] < 0)
					parity ^= 1;
			}
			if (parity) {
				converged = 0;
				break;
			}
		}

		if (converged) {
			ret = 0;
			if (iter_used)
				*iter_used = iter + 1;
			break;
		}
	}

	if (!converged && iter_used)
		*iter_used = dec->max_iter;

	/* Hard decision on information bits */
	for (i = 0; i < k; i++)
		output[i] = (vn_llr[i] < 0) ? 1 : 0;

out:
	free(rows);
	free(vn_llr);
	free(cn_msg);
	return ret;
}
