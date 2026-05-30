// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR LDPC Decoder
 *
 * Layered sum-product (belief propagation) decoder using the
 * box-plus operator for exact check-node updates.
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

/* LLR clamp magnitude -- prevents Inf-Inf = NaN in boxplus */
#define LLR_CLAMP	40.0

/*
 * Exact check-node update in the LLR domain (the "box-plus" operator):
 *
 *   a [+] b = sign(a) sign(b) min(|a|,|b|)
 *             + log(1+e^{-|a+b|}) - log(1+e^{-|a-b|})
 *
 * This is the sum-product (belief-propagation) check rule.  Compared to
 * min-sum it recovers the ~0.5-1 dB that matters at VL-SNR operating
 * points.  log1p(exp(-x)) is numerically stable for all x >= 0.
 */
static inline double boxplus(double a, double b)
{
	double sgn, aa, ab, mn, corr;

	if (a > LLR_CLAMP)
		a = LLR_CLAMP;
	else if (a < -LLR_CLAMP)
		a = -LLR_CLAMP;
	if (b > LLR_CLAMP)
		b = LLR_CLAMP;
	else if (b < -LLR_CLAMP)
		b = -LLR_CLAMP;

	sgn = ((a < 0.0) ? -1.0 : 1.0) * ((b < 0.0) ? -1.0 : 1.0);
	aa = fabs(a);
	ab = fabs(b);
	mn = (aa < ab) ? aa : ab;
	corr = log1p(exp(-fabs(a + b))) - log1p(exp(-fabs(a - b)));

	return sgn * mn + corr;
}

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
	double *v2c = NULL;	/* per-row VN->CN messages */
	double *fwd = NULL;	/* forward box-plus accumulation */
	double *bwd = NULL;	/* backward box-plus accumulation */
	unsigned int max_deg = 0;
	unsigned int iter;
	unsigned int i, j;
	int ret = -1;
	int converged = 0;

	/*
	 * Build H matrix in CSR format.
	 * Use the transmitted code dimensions (k info + m parity).
	 * The staircase parity structure handles the accumulator.
	 * Puncturing is handled by the encoder producing the correct
	 * codeword for the transmitted (post-puncture) code.
	 */
	if (build_csr(&dec->code, &csr) < 0)
		return -1;

	/* Largest check-node degree (sizes the per-row scratch buffers) */
	for (i = 0; i < m; i++) {
		unsigned int deg = csr.row_ptr[i + 1] - csr.row_ptr[i];

		if (deg > max_deg)
			max_deg = deg;
	}

	/* Allocate working memory */
	vn_llr = malloc(n * sizeof(double));
	edge_msg = calloc(csr.num_edges, sizeof(double));
	v2c = malloc((max_deg + 1) * sizeof(double));
	fwd = malloc((max_deg + 1) * sizeof(double));
	bwd = malloc((max_deg + 1) * sizeof(double));

	if (!vn_llr || !edge_msg || !v2c || !fwd || !bwd)
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

		/* For each check node (row of H): layered sum-product */
		for (i = 0; i < m; i++) {
			unsigned int row_start = csr.row_ptr[i];
			unsigned int row_end = csr.row_ptr[i + 1];
			unsigned int degree = row_end - row_start;
			unsigned int d;

			if (degree == 0)
				continue;

			/* Gather extrinsic VN->CN messages for this row */
			for (d = 0; d < degree; d++) {
				unsigned int vn = csr.col_idx[row_start + d];

				v2c[d] = vn_llr[vn] - edge_msg[row_start + d];
			}

			/* Forward / backward box-plus accumulation */
			fwd[0] = v2c[0];
			for (d = 1; d < degree; d++)
				fwd[d] = boxplus(fwd[d - 1], v2c[d]);
			bwd[degree - 1] = v2c[degree - 1];
			for (d = degree - 1; d-- > 0; )
				bwd[d] = boxplus(bwd[d + 1], v2c[d]);

			/* Extrinsic CN->VN message excludes the target edge */
			for (d = 0; d < degree; d++) {
				unsigned int vn = csr.col_idx[row_start + d];
				double new_msg;

				if (degree == 1)
					new_msg = 0.0;
				else if (d == 0)
					new_msg = bwd[1];
				else if (d == degree - 1)
					new_msg = fwd[degree - 2];
				else
					new_msg = boxplus(fwd[d - 1],
							  bwd[d + 1]);

				vn_llr[vn] += new_msg - edge_msg[row_start + d];
				edge_msg[row_start + d] = new_msg;
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
	free(v2c);
	free(fwd);
	free(bwd);
	free_csr(&csr);
	return ret;
}

