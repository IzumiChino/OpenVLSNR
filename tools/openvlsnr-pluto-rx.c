// SPDX-License-Identifier: GPL-2.0-or-later
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pluto.h"
#include "pluto_rx_queue.h"

#define DEFAULT_URI		"ip:192.168.2.1"
#define DEFAULT_SYMBOL_RATE	1250000
#define DEFAULT_SPS		2
#define DEFAULT_GAIN		40.0
#define PLUTO_BUFFER_SAMPLES	16384
#define RX_QUEUE_BLOCKS		256
#define RX_PACKET_CAPACITY	16

struct rx_options {
	const char	*uri;
	const char	*path;
	const char	*iq_path;
	const char	*symbol_path;
	long long	frequency;
	long long	symbol_rate;
	unsigned int	sps;
	double		gain;
	unsigned int	max_frames;
	unsigned int	duration;
};

static volatile sig_atomic_t stop_requested;
static struct pluto_stream *active_stream;
static struct dvbs2x_demodulator *active_demod;

static void stop_handler(int signal)
{
	(void)signal;
	stop_requested = 1;
	dvbs2x_demodulator_request_cancel(active_demod);
	pluto_stream_cancel(active_stream);
}

static void usage(const char *name)
{
	fprintf(stderr,
		"Usage: %s -f HZ [options] FILE.ts\n"
		"  -u, --uri URI          IIO URI (default %s)\n"
		"  -f, --frequency HZ     RF center frequency\n"
		"  -r, --symbol-rate HZ   symbol rate (default %d)\n"
		"  -s, --sps N            samples per symbol (default %d)\n"
		"  -g, --gain DB          manual RX gain (default %.1f)\n"
		"  -n, --frames N         stop after N decoded PL frames\n"
		"  -t, --seconds N        stop after N seconds\n"
		"  -i, --iq FILE          capture interleaved float32 IQ\n"
		"  -S, --symbols FILE     capture corrected float32 symbols\n",
		name, DEFAULT_URI, DEFAULT_SYMBOL_RATE, DEFAULT_SPS,
		DEFAULT_GAIN);
}

static int parse_longlong(const char *value, long long *result)
{
	char *end;
	long long parsed;

	errno = 0;
	parsed = strtoll(value, &end, 10);
	if (errno || *end || parsed <= 0)
		return -1;
	*result = parsed;
	return 0;
}

static int parse_uint(const char *value, unsigned int *result)
{
	long long parsed;

	if (parse_longlong(value, &parsed) < 0 || parsed > UINT_MAX)
		return -1;
	*result = (unsigned int)parsed;
	return 0;
}

static int parse_double(const char *value, double *result)
{
	char *end;
	double parsed;

	errno = 0;
	parsed = strtod(value, &end);
	if (errno || *end)
		return -1;
	*result = parsed;
	return 0;
}

static int parse_options(int argc, char **argv, struct rx_options *options)
{
	static const struct option long_options[] = {
		{ "uri", required_argument, NULL, 'u' },
		{ "frequency", required_argument, NULL, 'f' },
		{ "symbol-rate", required_argument, NULL, 'r' },
		{ "sps", required_argument, NULL, 's' },
		{ "gain", required_argument, NULL, 'g' },
		{ "frames", required_argument, NULL, 'n' },
		{ "seconds", required_argument, NULL, 't' },
		{ "iq", required_argument, NULL, 'i' },
		{ "symbols", required_argument, NULL, 'S' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int option;

	memset(options, 0, sizeof(*options));
	options->uri = DEFAULT_URI;
	options->symbol_rate = DEFAULT_SYMBOL_RATE;
	options->sps = DEFAULT_SPS;
	options->gain = DEFAULT_GAIN;
	while ((option = getopt_long(argc, argv, "u:f:r:s:g:n:t:i:S:h",
				      long_options, NULL)) != -1) {
		switch (option) {
		case 'u':
			options->uri = optarg;
			break;
		case 'f':
			if (parse_longlong(optarg, &options->frequency) < 0)
				return -1;
			break;
		case 'r':
			if (parse_longlong(optarg, &options->symbol_rate) < 0)
				return -1;
			break;
		case 's':
			if (parse_uint(optarg, &options->sps) < 0)
				return -1;
			break;
		case 'g':
			if (parse_double(optarg, &options->gain) < 0)
				return -1;
			break;
		case 'n':
			if (parse_uint(optarg, &options->max_frames) < 0)
				return -1;
			break;
		case 't':
			if (parse_uint(optarg, &options->duration) < 0)
				return -1;
			break;
		case 'i':
			options->iq_path = optarg;
			break;
		case 'S':
			options->symbol_path = optarg;
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
		default:
			return -1;
		}
	}
	if (!options->frequency || optind + 1 != argc ||
	    options->sps < 2 || options->sps > 16 ||
	    options->symbol_rate > LLONG_MAX / options->sps ||
	    options->gain < 0.0 || options->gain > 73.0)
		return -1;
	options->path = argv[optind];
	return 0;
}

static int write_packets(FILE *output, const uint8_t *packets,
			 unsigned int packet_count)
{
	if (!packet_count)
		return 0;
	if (fwrite(packets, DVBS2X_TS_PACKET_SIZE, packet_count, output) !=
	    packet_count) {
		perror("TS output");
		return -1;
	}
	if (fflush(output) == EOF) {
		perror("TS output");
		return -1;
	}
	return 0;
}

static const char *state_name(enum dvbs2x_demod_state state)
{
	switch (state) {
	case DVBS2X_DEMOD_SEARCH:
		return "SEARCH";
	case DVBS2X_DEMOD_ACQUIRE:
		return "ACQUIRE";
	case DVBS2X_DEMOD_TRACK:
		return "TRACK";
	default:
		return "UNKNOWN";
	}
}

static void report_status(const struct dvbs2x_demodulator *demod,
			  const struct pluto_rx_queue *queue,
			  unsigned long long samples,
			  double power_sum,
			  unsigned long long decoded,
			  unsigned long long sync_failures,
			  unsigned long long fec_failures,
			  unsigned long long ts_failures,
			  unsigned long long packets)
{
	struct dvbs2x_demod_stats stats;
	unsigned long long received_samples = samples;
	unsigned int queued = 0, high_water = 0, backpressure = 0;
	double power_dbfs;

	if (dvbs2x_demodulator_get_stats(demod, &stats) < 0)
		return;
	pluto_rx_queue_get_stats(queue, &received_samples, &queued, &high_water,
				 &backpressure);
	power_dbfs = samples ? 10.0 * log10(power_sum / (double)samples +
					      1e-15) : -150.0;
	fprintf(stderr,
		"samples=%llu input=%llu queue=%u/%u wait=%u "
		"power=%.1f dBFS state=%s modcod=%u sync=%.3f "
		"Es/N0=%.2f dB LDPC=%u frames=%llu "
		"fail(sync/fec/ts)=%llu/%llu/%llu packets=%llu\r",
		samples, received_samples, queued, high_water, backpressure,
		power_dbfs, state_name(demod->state), stats.modcod,
		stats.sync_confidence, stats.esn0_db, stats.ldpc_iterations,
		decoded, sync_failures, fec_failures, ts_failures, packets);
	fflush(stderr);
}

static int process_bbframe(struct dvbs2x_ts_rx *ts, const uint8_t *bbframe,
			   uint8_t *packets, FILE *output,
			   unsigned long long *packet_total)
{
	unsigned int packet_count;
	int ret;

	ret = dvbs2x_ts_rx_push(ts, bbframe, packets, RX_PACKET_CAPACITY,
				&packet_count);
	if (ret < 0)
		return ret;
	if (write_packets(output, packets, packet_count) < 0)
		return -1;
	*packet_total += packet_count;
	return 0;
}

struct symbol_capture {
	FILE	*file;
	int	failed;
};

static void capture_symbols(const struct dvbs2x_complex *symbols,
			    unsigned int len, void *opaque)
{
	struct symbol_capture *capture = opaque;
	float buffer[2048];
	unsigned int offset = 0;

	while (offset < len && !capture->failed) {
		unsigned int count = len - offset;
		unsigned int i;

		if (count > sizeof(buffer) / sizeof(buffer[0]) / 2)
			count = sizeof(buffer) / sizeof(buffer[0]) / 2;
		for (i = 0; i < count; i++) {
			buffer[2 * i] = (float)symbols[offset + i].i;
			buffer[2 * i + 1] = (float)symbols[offset + i].q;
		}
		if (fwrite(buffer, 2 * sizeof(buffer[0]), count,
			   capture->file) != count)
			capture->failed = 1;
		offset += count;
	}
}

int main(int argc, char **argv)
{
	struct rx_options options;
	struct pluto_config config;
	struct pluto_stream stream = { 0 };
	struct pluto_rx_queue *queue = NULL;
	struct dvbs2x_demodulator demod = { 0 };
	struct dvbs2x_ts_rx ts = { 0 };
	struct symbol_capture symbol_capture = { 0 };
	uint8_t packets[RX_PACKET_CAPACITY * DVBS2X_TS_PACKET_SIZE];
	uint8_t *bbframe = NULL;
	unsigned long long frame_count = 0;
	unsigned long long packet_count = 0;
	unsigned long long sample_count = 0;
	double power_sum = 0.0;
	unsigned long long sync_failures = 0;
	unsigned long long fec_failures = 0;
	unsigned long long ts_failures = 0;
	unsigned int refill_count = 0;
	unsigned int announced_modcod = 0;
	time_t end_time = 0;
	FILE *output = NULL;
	FILE *iq_output = NULL;
	FILE *symbol_output = NULL;
	int ret = 1;

	if (parse_options(argc, argv, &options) < 0) {
		usage(argv[0]);
		return 2;
	}
	output = strcmp(options.path, "-") ? fopen(options.path, "wb") : stdout;
	if (!output) {
		perror(options.path);
		return 1;
	}
	if (options.iq_path) {
		iq_output = fopen(options.iq_path, "wb");
		if (!iq_output) {
			perror(options.iq_path);
			goto out;
		}
	}
	if (options.symbol_path) {
		symbol_output = fopen(options.symbol_path, "wb");
		if (!symbol_output) {
			perror(options.symbol_path);
			goto out;
		}
		symbol_capture.file = symbol_output;
	}
	dvbs2x_library_init();
	if (dvbs2x_demodulator_init(&demod, 0.35, options.sps, 0) < 0)
		goto out;
	if (symbol_output)
		dvbs2x_demodulator_set_symbol_sink(&demod, capture_symbols,
						   &symbol_capture);
	bbframe = malloc(DVBS2X_LDPC_NORMAL);
	if (!bbframe)
		goto out;
	config.uri = options.uri;
	config.frequency = options.frequency;
	config.sample_rate = options.symbol_rate * options.sps;
	config.bandwidth = options.symbol_rate * 135 / 100;
	config.gain = options.gain;
	if (pluto_rx_open(&stream, &config, PLUTO_BUFFER_SAMPLES) < 0)
		goto out;
	active_stream = &stream;
	active_demod = &demod;
	if (signal(SIGINT, stop_handler) == SIG_ERR ||
	    signal(SIGTERM, stop_handler) == SIG_ERR)
		goto out;
	fprintf(stderr, "RX %.3f MHz, %.3f Msym/s, %.3f MS/s\n",
		(double)options.frequency / 1e6,
		(double)options.symbol_rate / 1e6,
		(double)config.sample_rate / 1e6);
	fprintf(stderr, "MODCOD is detected from each VL-SNR header\n");
	if (options.duration)
		end_time = time(NULL) + options.duration;
	if (pluto_rx_queue_start(&queue, &stream,
				 PLUTO_BUFFER_SAMPLES, RX_QUEUE_BLOCKS,
				 iq_output, end_time) < 0)
		goto out;
	while (!stop_requested) {
		const struct dvbs2x_complex *samples;
		unsigned int sample_len;
		unsigned int consumed;
		unsigned int frame_len;
		int dret;

		dret = pluto_rx_queue_acquire(queue, &samples, &sample_len);
		if (dret > 0 || (dret < 0 && stop_requested))
			break;
		if (dret < 0)
			goto out;
		if (symbol_capture.failed) {
			perror("symbol capture");
			goto out;
		}
		sample_count += sample_len;
		{
			unsigned int i;

			for (i = 0; i < sample_len; i++)
				power_sum += samples[i].i * samples[i].i +
					samples[i].q * samples[i].q;
		}
		refill_count++;
		if (refill_count <= 4)
			report_status(&demod, queue, sample_count, power_sum,
				      frame_count, sync_failures, fec_failures,
				      ts_failures, packet_count);
		dret = dvbs2x_demodulate_bbframe_stream_ex(
			&demod, samples, sample_len, bbframe,
			DVBS2X_LDPC_NORMAL, &frame_len, &consumed);
		while (dret != DVBS2X_ERR_SHORT) {
			if (dret == 0) {
				frame_count++;
				if (demod.modcod &&
				    announced_modcod != demod.modcod->index) {
					announced_modcod = demod.modcod->index;
					fprintf(stderr, "\ndetected MODCOD %u: %s\n",
						demod.modcod->index,
							dvbs2x_vlsnr_get_modcod_name(
							demod.modcod->index));
				}
				if (!demod.modcod ||
				    (!ts.bb.k_bch &&
				     dvbs2x_ts_rx_init(&ts, demod.modcod) < 0))
					goto out;
				if (ts.bb.k_bch != demod.modcod->k_bch) {
					dvbs2x_ts_rx_reset(&ts);
					if (dvbs2x_ts_rx_init(&ts, demod.modcod) < 0)
						goto out;
				}
				dret = process_bbframe(&ts, bbframe, packets, output,
							&packet_count);
				if (dret < 0) {
					ts_failures++;
					dvbs2x_ts_rx_reset(&ts);
				}
			} else if (dret == DVBS2X_ERR_NOSYNC) {
				sync_failures++;
			} else {
				fec_failures++;
			}
			if (options.max_frames &&
			    frame_count >= options.max_frames) {
				stop_requested = 1;
				break;
			}
			frame_len = 0;
			consumed = 0;
			dret = dvbs2x_demodulate_bbframe_stream_ex(
				&demod, NULL, 0, bbframe, DVBS2X_LDPC_NORMAL,
				&frame_len, &consumed);
		}
		if (symbol_capture.failed) {
			perror("symbol capture");
			goto out;
		}
		if (!(refill_count % 64))
			report_status(&demod, queue, sample_count, power_sum,
				      frame_count,
				      sync_failures, fec_failures, ts_failures,
					      packet_count);
		pluto_rx_queue_release(queue);
	}
	if (ts.bb.k_bch) {
		unsigned int final_count;

		if (dvbs2x_ts_rx_finalize_unchecked(
			    &ts, packets, 1, &final_count) < 0 ||
		    write_packets(output, packets, final_count) < 0)
			goto out;
		packet_count += final_count;
	}
	report_status(&demod, queue, sample_count, power_sum, frame_count,
		      sync_failures,
		      fec_failures, ts_failures, packet_count);
	fprintf(stderr, "\nreceived %llu TS packets from %llu PL frames\n",
		packet_count, frame_count);
	ret = 0;
out:
	active_stream = NULL;
	active_demod = NULL;
	pluto_rx_queue_stop(queue);
	pluto_stream_close(&stream);
	dvbs2x_demodulator_destroy(&demod);
	free(bbframe);
	if (iq_output)
		fclose(iq_output);
	if (symbol_output)
		fclose(symbol_output);
	if (output && output != stdout)
		fclose(output);
	return ret;
}
