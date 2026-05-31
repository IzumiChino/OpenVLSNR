// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Robustness Test
 *
 * Feeds random IQ data to the demodulator to verify it never
 * crashes, only returns error codes gracefully.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "dvbs2x_vlsnr.h"

#define FUZZ_ITERATIONS	100
#define MAX_BUF_LEN	80000

int main(int argc, char *argv[])
{
	struct dvbs2x_demodulator demod;
	struct dvbs2x_complex *buf;
	uint8_t *out;
	unsigned int iterations = FUZZ_ITERATIONS;
	unsigned int i, n;
	unsigned int nosync = 0, other = 0, ok = 0;
	int ret;

	if (argc >= 2)
		iterations = (unsigned int)atoi(argv[1]);

	srand((unsigned int)time(NULL));

	dvbs2x_library_init();
	dvbs2x_demodulator_init(&demod, 0.35, 2, 0);

	buf = malloc(MAX_BUF_LEN * sizeof(*buf));
	out = malloc(65536);
	if (!buf || !out)
		return 1;

	printf("DVB-S2X VL-SNR Robustness Test\n");
	printf("==============================\n");
	printf("Feeding %u random buffers to demodulator...\n",
	       iterations);

	for (i = 0; i < iterations; i++) {
		unsigned int len = (unsigned int)(rand() % MAX_BUF_LEN) + 100;
		unsigned int rec_len = 0;

		/* Fill with random IQ */
		for (n = 0; n < len; n++) {
			buf[n].i = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
			buf[n].q = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
		}

		ret = dvbs2x_demodulate(&demod, buf, len, out, &rec_len);

		if (ret == DVBS2X_ERR_NOSYNC || ret == DVBS2X_ERR_SHORT)
			nosync++;
		else if (ret == DVBS2X_OK)
			ok++;
		else
			other++;
	}

	printf("  NOSYNC/SHORT: %u\n", nosync);
	printf("  Other error:  %u\n", other);
	printf("  Decoded(!):   %u\n", ok);
	printf("\nNo crashes.  PASS\n");

	free(buf);
	free(out);
	dvbs2x_demodulator_destroy(&demod);
	return 0;
}
