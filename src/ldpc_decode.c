// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR LDPC Decoder
 *
 * Flooding schedule offset min-sum belief propagation decoder.
 *
 * Uses Compressed Sparse Row (CSR) format for the H matrix to
 * handle the high check node degrees that arise in VL-SNR codes
 * (where Q*360 > M causes address wrapping).
 *
 * The H matrix is built from:
 * 1. Information part: quasi-cyclic structure from address table
 * 2. Parity part: dual-diagonal (staircase/accumulator)
 *
 * Reference: ETSI TR 102 376-1 (implementation guidelines)
 */

#include "ldpc.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Default min-sum offset */
#define LDPC_MS_OFFSET	0.15

/* CSR sparse matrix */
struct ldpc_csr {
	unsigned int	num_rows;	/* M (number of check nodes) */
	unsigned int	num_cols;	/* N (number of variable nodes) */
	unsigned int	*row_ptr;	/* row_ptr[i] = start of row i */
	unsigned int	*col_idx;	/* column indices */
	unsigned int	num_edges;	/* total non-zeros */
};

/*
 * Build CSR representation of the parity check matrix.
 *
 * H matrix structure:
 * - Information part: quasi-cyclic from address table
 * - Parity part: lower bidiagonal (staircase from accumulator)
 *   Row 0: p[0]
 *   Row i (i>0): p[i-1], p[i]
 */
static int build_csr(const struct dvbs2x_ldpc_code *code,
		     struct ldpc_csr *csr)
{
	unsigned int m = code->n - code->k;
	unsigned int q = code->q;
	unsigned int group, j, a, i;
	unsigned int *row_count;

	csr->num_rows = m;
	csr->num_cols = code->n;

	/* First pass: count edges per row */
	row_count = calloc(m, sizeof(unsigned int));
	if (!row_count)
		return -1;

	/* Information part */
	for (group = 0; group < code->num_groups; group++) {
		const struct dvbs2x_ldpc_table_entry *entry;

		entry = &code->table[group];
		for (j = 0; j < DVBS2X_LDPC_PARALLEL; j++) {
			for (a = 0; a < entry->num_addrs; a++) {
				unsigned int cn;

				cn = (entry->addrs[a] + j * q) % m;
				row_count[cn]++;
			}
		}
	}

	/* Parity part: staircase
	 * Row 0: p[0] only (1 parity connection)
	 * Row i>0: p[i-1] and p[i] (2 parity connections)
	 */
	row_count[0] += 1;
	for (i = 1; i < m; i++)
		row_count[i] += 2;

	/* Build row_ptr */
	csr->row_ptr = malloc((m + 1) * sizeof(unsigned int));
	if (!csr->row_ptr) {
		free(row_count);
		return -1;
	}

	csr->row_ptr[0] = 0;
	for (i = 0; i < m; i++)
		csr->row_ptr[i + 1] = csr->row_ptr[i] + row_count[i];

	csr->num_edges = csr->row_ptr[m];

	/* Allocate column indices */
	csr->col_idx = malloc(csr->num_edges * sizeof(unsigned int));
	if (!csr->col_idx) {
		free(row_count);
		free(csr->row_ptr);
		return -1;
	}

	/* Second pass: fill column indices */
	memset(row_count, 0, m * sizeof(unsigned int));

	/* Information part */
	for (group = 0; group < code->num_groups; group++) {
		const struct dvbs2x_ldpc_table_entry *entry;

		entry = &code->table[group];
		for (j = 0; j < DVBS2X_LDPC_PARALLEL; j++) {
			unsigned int vn = group * DVBS2X_LDPC_PARALLEL + j;

			for (a = 0; a < entry->num_addrs; a++) {
				unsigned int cn;
				unsigned int pos;

				cn = (entry->addrs[a] + j * q) % m;
				pos = csr->row_ptr[cn] + row_count[cn];
				csr->col_idx[pos] = vn;
				row_count[cn]++;
			}
		}
	}

	/* Parity part: staircase */
	/* Row 0: p[0] */
	{
		unsigned int pos = csr->row_ptr[0] + row_count[0];

		csr->col_idx[pos] = code->k + 0;
		row_count[0]++;
	}
	/* Row i (i>=1): p[i-1] and p[i] */
	for (i = 1; i < m; i++) {
		unsigned int pos;

		pos = csr->row_ptr[i] + row_count[i];
		csr->col_idx[pos] = code->k + i - 1;
		row_count[i]++;

		pos = csr->row_ptr[i] + row_count[i];
		csr->col_idx[pos] = code->k + i;
		row_count[i]++;
	}

	free(row_count);
	return 0;
}

static void free_csr(struct ldpc_csr *csr)
{
	free(csr->row_ptr);
	free(csr->col_idx);
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
	dec->xp = modcod->xp;
	dec->p_period = modcod->p_period;
	dec->xs = modcod->xs;

	return 0;
}

int dvbs2x_ldpc_decode(const struct dvbs2x_ldpc_decoder *dec,
		       const double *llr, uint8_t *output,
		       unsigned int *iter_used)
{
	unsigned int n = dec->code.n;
	unsigned int k = dec->code.k;
	unsigned int m = n - k;		/* transmitted parity bits */
	struct ldpc_csr csr;
	double *vn_llr;		/* total VN belief */
	double *edge_msg;	/* CN-to-VN messages (one per edge) */
	unsigned int iter;
	unsigned int i, j;
	int ret = -1;
	int converged;

	/*
	 * Build H matrix in CSR format.
	 * Use the transmitted code dimensions (k info + m parity).
	 * The staircase parity structure handles the accumulator.
	 * Puncturing is handled by the encoder producing the correct
	 * codeword for the transmitted (post-puncture) code.
	 */
	if (build_csr(&dec->code, &csr) < 0)
		return -1;

	/* Allocate working memory */
	vn_llr = malloc(n * sizeof(double));
	edge_msg = calloc(csr.num_edges, sizeof(double));

	if (!vn_llr || !edge_msg)
		goto out;

	/* Initialize VN LLRs from channel */
	for (i = 0; i < n; i++)
		vn_llr[i] = llr[i];

	/*
	 * Note: shortening (xs > 0) is handled at the system level.
	 * The caller should set the first xs LLR positions to a large
	 * positive value before calling this function if shortening
	 * is used.
	 */

	/* Iterative decoding (layered schedule) */
	for (iter = 0; iter < dec->max_iter; iter++) {

		/* For each check node (row of H) */
		for (i = 0; i < m; i++) {
			unsigned int row_start = csr.row_ptr[i];
			unsigned int row_end = csr.row_ptr[i + 1];
			unsigned int degree = row_end - row_start;
			double sign_prod;
			double min1, min2;
			unsigned int min1_pos;

			if (degree == 0)
				continue;

			/* Find min and sign product */
			sign_prod = 1.0;
			min1 = 1e30;
			min2 = 1e30;
			min1_pos = row_start;

			for (j = row_start; j < row_end; j++) {
				unsigned int vn = csr.col_idx[j];
				double v2c = vn_llr[vn] - edge_msg[j];
				double abs_v2c = fabs(v2c);

				if (v2c < 0)
					sign_prod = -sign_prod;

				if (abs_v2c < min1) {
					min2 = min1;
					min1 = abs_v2c;
					min1_pos = j;
				} else if (abs_v2c < min2) {
					min2 = abs_v2c;
				}
			}

			/* Update CN-to-VN messages */
			for (j = row_start; j < row_end; j++) {
				unsigned int vn = csr.col_idx[j];
				double v2c = vn_llr[vn] - edge_msg[j];
				double msg_sign;
				double msg_abs;
				double new_msg;

				msg_sign = sign_prod;
				if (v2c < 0)
					msg_sign = -msg_sign;

				if (j == min1_pos)
					msg_abs = min2 - dec->offset;
				else
					msg_abs = min1 - dec->offset;

				if (msg_abs < 0.0)
					msg_abs = 0.0;

				new_msg = msg_sign * msg_abs;

				vn_llr[vn] += new_msg - edge_msg[j];
				edge_msg[j] = new_msg;
			}
		}

		/* Check convergence */
		converged = 1;
		for (i = 0; i < m; i++) {
			unsigned int row_start = csr.row_ptr[i];
			unsigned int row_end = csr.row_ptr[i + 1];
			int parity = 0;

			for (j = row_start; j < row_end; j++) {
				if (vn_llr[csr.col_idx[j]] < 0)
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
	free(vn_llr);
	free(edge_msg);
	free_csr(&csr);
	return ret;
}

