/* 
 * XTRX driver for amarisoft LTEENB
 *
 * Copyright (C) 2017 Fairwaves,
 * Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <getopt.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdbool.h>

#include <xtrx_api.h>

#include "trx_driver.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

typedef struct trx_xtrx_state {
	struct xtrx_dev* xtrx;
	unsigned tx_inter;
	unsigned rx_decim;
	int64_t  rx_ts;
	int64_t  tx_ts_off;
	int      mimo_mode;
	unsigned packetsize;
	int      softfilt;
} trx_xtrx_state_t;

static void trx_xtrx_write(TRXState *s1, trx_timestamp_t timestamp, const void **psamples, int count, int flags, int tx_port_index)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	assert(tx_port_index == 0);

	if (flags & TRX_WRITE_FLAG_PADDING) {
		return;
	}

	xtrx_send_ex_info_t sei;
	sei.buffer_count = (s->mimo_mode ? 2 : 1);
	sei.buffers = psamples;
	sei.flags = XTRX_TX_DONT_BUFFER;
	sei.samples = count;
	sei.ts = timestamp - s->tx_ts_off;
	int res = xtrx_send_sync_ex(s->xtrx, &sei);
	if (res) {
		fprintf(stderr, "trx_xtrx_write: xtrx_send_burst_sync count=%d err=%d\n", count, res);
		exit(1);
	}
}

static int trx_xtrx_read(TRXState *s1, trx_timestamp_t *ptimestamp, void **psamples, int count, int rf_port_index)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	assert(rf_port_index == 0);

	xtrx_recv_ex_info_t ei;
	ei.buffers = psamples;
	ei.buffer_count = (s->mimo_mode ? 2 : 1);
	ei.samples = count;
	ei.flags = 0;

	int res = xtrx_recv_sync_ex(s->xtrx, &ei);
	if (res) {
		fprintf(stderr, "trx_xtrx_read: xtrx_recv_sync count=%d err=%d\n", count, res);
		exit(1);
	}

	*ptimestamp = s->rx_ts;
	s->rx_ts += count;
	return count;
}

static void trx_xtrx_set_tx_gain(TRXState *s1, double gain, int chain)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	xtrx_set_gain(s->xtrx, XTRX_CH_AB, XTRX_TX_PAD_GAIN, gain, NULL);
}

static void trx_xtrx_set_rx_gain(TRXState *s1, double gain, int chain)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	xtrx_set_gain(s->xtrx, XTRX_CH_AB, XTRX_RX_LNA_GAIN, gain, NULL);
}

static int trx_xtrx_start(TRXState *s1, const TRXDriverParams *p)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	double actual_gain_tx;
	double actual_gain_rx;
	double actual_tx_freq;
	double actual_rx_freq;
	int i = 0;
	int res;

	if (p->rf_port_count == 1 && p->tx_channel_count == 1 && p->rx_channel_count == 1) {
		fprintf(stderr, "XTRX SISO MODE!\n");
		s->mimo_mode = 0;
	} else if (p->rf_port_count == 1 && p->tx_channel_count == 2 && p->rx_channel_count == 2) {
		fprintf(stderr, "XTRX 2x2 MIMO MODE!\n");
		s->mimo_mode = 1;
	} else {
		fprintf(stderr, "XTRX mode %d/%d/%d isn't supported now!\n",
				p->rf_port_count, p->tx_channel_count, p->rx_channel_count);
		return -1;
	}

	double smaple_rate = p->sample_rate[i].num / p->sample_rate[i].den;

	fprintf(stderr, "XTRX bandwidth Rx:%.3f Tx:%.3f SR:%.3f\n",
			p->rx_bandwidth[i] / 1e6,
			p->tx_bandwidth[i] / 1e6,
			smaple_rate / 1e6);

	xtrx_tune_tx_bandwidth(s->xtrx, XTRX_CH_AB, p->tx_bandwidth[i], NULL);
	xtrx_tune_rx_bandwidth(s->xtrx, XTRX_CH_AB, p->rx_bandwidth[i], NULL);

	/*
	 * For now asume that LO on first and second channel are the same, TODO check this!
	 */
	if (p->tx_freq[0] == p->rx_freq[0]) {
		fprintf(stderr, "XTRX configured for TDD mode (sharing LO)\n");
		res = xtrx_tune(s->xtrx, XTRX_TUNE_TX_AND_RX_TDD, p->tx_freq[i], &actual_tx_freq);
		actual_rx_freq = actual_tx_freq;
	} else {
		res = xtrx_tune(s->xtrx, XTRX_TUNE_TX_FDD, p->tx_freq[i], &actual_tx_freq);
		if (res == 0) {
			res = xtrx_tune(s->xtrx, XTRX_TUNE_RX_FDD, p->rx_freq[i], &actual_rx_freq);
		}
	}

	if (res) {
		fprintf(stderr, "Unable to tune! RX=%ld TX=%ld err:%d\n", p->rx_freq[i], p->tx_freq[i], res);
		return -1;
	}

	xtrx_set_gain(s->xtrx, XTRX_CH_AB, XTRX_TX_PAD_GAIN, p->tx_gain[i], &actual_gain_tx);
	xtrx_set_gain(s->xtrx, XTRX_CH_AB, XTRX_RX_LNA_GAIN, 30 /*p->rx_gain[i]*/, &actual_gain_rx);

	printf("Chan Gain Freq(MHz)\n");
	printf(" TX %3.1f %5.6f\n", actual_gain_tx, actual_tx_freq / 1e6);
	printf(" RX %3.1f %5.6f\n", actual_gain_rx, actual_rx_freq / 1e6);

	/* delay actual streaming for 0.5 sec */
	//s->tx_ts = s->rx_ts = 50; // smaple_rate * 0.001;

	s->tx_ts_off = -8192+35;
	s->rx_ts = -35;

	xtrx_run_params_t params;
	params.dir = XTRX_TRX;
	params.nflags = 0;
	params.rx.wfmt = XTRX_WF_16;
	params.rx.hfmt = XTRX_IQ_FLOAT32;
	params.rx.chs = XTRX_CH_AB;
	params.rx.flags = (s->mimo_mode) ? 0 : XTRX_RSP_SISO_MODE;
	params.rx.paketsize = (s->mimo_mode) ? s->packetsize : s->packetsize;
	params.tx.wfmt = XTRX_WF_16;
	params.tx.hfmt = XTRX_IQ_FLOAT32;
	params.tx.chs = XTRX_CH_AB;
	params.tx.flags = (s->mimo_mode) ? 0 : XTRX_RSP_SISO_MODE;
	params.tx.paketsize = (s->mimo_mode) ? s->packetsize : s->packetsize;
	params.rx_stream_start = 8192;
	params.tx_repeat_buf = NULL;

	res = xtrx_run_ex(s->xtrx, &params);
	if (res) {
		fprintf(stderr, "Unable to start streaming, err: %d", res);
		return -1;
	}

	xtrx_set_antenna(s->xtrx, XTRX_RX_W);
	xtrx_set_antenna(s->xtrx, XTRX_TX_W); //B2

	return 0;
}

static void trx_xtrx_end(TRXState *s1)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;

	xtrx_stop(s->xtrx, XTRX_TRX);
	free(s);
}

static int is_allowed_sample_rate_num(unsigned int n)
{
	/* must only have 2, 3 or 5 as prime factors */
	while ((n % 2) == 0)
		n /= 2;
	while ((n % 3) == 0)
		n /= 3;
	while ((n % 5) == 0)
		n /= 5;
	return (n == 1);
}

static int trx_xtrx_get_sample_rate(TRXState *s1, TRXFraction *psample_rate,
									int *psample_rate_num, int sample_rate_min)
{
	int i, res;
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	double cgen_freq = 0;
	double cgen_actual;
	int sample_rate;
	int soft_filter = s->softfilt;

	sample_rate_min = lround((soft_filter) ? sample_rate_min * 1.15 : sample_rate_min * 1.35);

	/* from 2 to 62 MHz */
	for(i = 1; i <= 32; i++) {
		if (!is_allowed_sample_rate_num(i))
			continue;
		sample_rate = i * 1920000;
		if (sample_rate_min <= sample_rate)
			break;
	}
	if (i > 32)
		return -1;

	*psample_rate_num = i;
	psample_rate->num = sample_rate;
	psample_rate->den = 1;

	if (s->rx_decim > 1 && s->tx_inter > 1) {
		cgen_freq = sample_rate * 16 / (MAX(s->rx_decim, s->tx_inter));
	} else {
		cgen_freq = sample_rate * 32;
	}

	res = xtrx_set_samplerate(s->xtrx, cgen_freq, sample_rate, sample_rate,
							  (soft_filter) ? XTRX_SAMPLERATE_FORCE_TX_INTR | XTRX_SAMPLERATE_FORCE_RX_DECIM : 0,
							  &cgen_actual, NULL, NULL);
	if (res) {
		fprintf(stderr, "trx_xtrx_get_sample_rate: can't deliver freq %d, err: %d",
				sample_rate, res);
		return -1;
	}

	fprintf(stderr, "trx_xtrx_get_sample_rate min=%d set=%d master=%.3f MHz\n",
			sample_rate_min, sample_rate, cgen_actual / 1e6);
	return 0;
}

static int trx_xtrx_get_tx_samples_per_packet(TRXState *s1)
{
	trx_xtrx_state_t *s = (trx_xtrx_state_t *)s1->opaque;
	return s->packetsize;
}

static int trx_xtrx_get_stats(TRXState *s1, TRXStatistics *m)
{
	m->tx_underflow_count = 0; //TODO FILLME
	m->rx_overflow_count = 0;  //TODO FILLME
	return 0;
}


static void trx_xtrx_dump_info(TRXState *s, trx_printf_cb cb, void *opaque)
{
}

static int trx_xtrx_get_abs_tx_power_func(TRXState *s, float *presult, int channel_num)
{
	return -1;
}

static int trx_xtrx_get_abs_rx_power_func(TRXState *s, float *presult, int channel_num)
{
	return -1;
}

static void trx_xtrx_write_func2(TRXState *s, trx_timestamp_t timestamp, const void **samples,
								 int count, int tx_port_index, TRXWriteMetadata *md)
{
	//TODO correct processing TDD flags
	trx_xtrx_write(s, timestamp, samples, count, md->flags, tx_port_index);
}

static int trx_xtrx_read_func2(TRXState *s, trx_timestamp_t *ptimestamp, void **samples, int count,
							   int rx_port_index, TRXReadMetadata *md)
{
	return trx_xtrx_read(s, ptimestamp, samples, count, rx_port_index);
}



int trx_driver_init(TRXState *s1)
{
	trx_xtrx_state_t *s;
	const char* args;
	const char* devs;
	double loglevel;
	double loglevel_lms;
	double refclk;
	double tx_inter;
	double rx_decim;
	double packetsize;
	double softfilt;
	double dacvalue;
	struct xtrx_dev* dev;
	int err;
	unsigned flags;

	fprintf(stderr, "FAIRWAVES XTRX EXPERIMENTAL DRIVER!\n");

	if (s1->trx_api_version != TRX_API_VERSION) {
		fprintf(stderr, "ABI compatibility mismatch between LTEENB and TRX "
						"XTRX driver (LTEENB ABI version=%d, "
						"TRX XTRX driver ABI version=%d)\n",
				s1->trx_api_version, TRX_API_VERSION);
		return -1;
	}

	devs = trx_get_param_string(s1, "devs");
	if (!devs) {
		fprintf(stderr, "XTRX driver: No 'devs' parameter, defaulting to none\n");
		devs = "/dev/xtrx0";
	}

	args = trx_get_param_string(s1, "args");
	if (!args) {
		fprintf(stderr, "XTRX driver: No 'args' parameter, defaulting to none\n");
		args = "";
	}

	if (trx_get_param_double(s1, &loglevel, "xtrx_loglevel") != 0) {
		loglevel = 5;
	}

	if (trx_get_param_double(s1, &loglevel_lms, "xtrx_loglevel_lms") != 0) {
		loglevel_lms = 0;
	}

	if (trx_get_param_double(s1, &packetsize, "pktsz") != 0) {
		packetsize = 4096;
	}

	if (packetsize < 512 && packetsize != 0) {
		packetsize = 512;
	}

	flags = ((unsigned)loglevel) & XTRX_O_LOGLVL_MASK
			;//| (((unsigned)xtrx_loglevel_lms) << XTRX_O_LOGLVL_LMS7_OFF) & XTRX_O_LOGLVL_LMS7_MASK;

	err = xtrx_open(devs, flags, &dev);
	if (err) {
		fprintf(stderr, "XTRX driver: can't open \"%s\" with \"%s\": %d error\n",
				devs, args, err);
		return -1;
	}

	if (trx_get_param_double(s1, &refclk, "xtrx_refclk") == 0) {
		fprintf(stderr, "XTRX driver: %.3f MHz\n", refclk/1e6);
		xtrx_set_ref_clk(dev, refclk, XTRX_CLKSRC_INT);
	}

	if (trx_get_param_double(s1, &tx_inter, "xtrx_tx_inter") != 0) {
		tx_inter = 1;
	}

	if (trx_get_param_double(s1, &rx_decim, "xtrx_rx_decim") != 0) {
		rx_decim = 1;
	}

	if (trx_get_param_double(s1, &softfilt, "softfilt") != 0) {
		softfilt = 0;
	}

	if (trx_get_param_double(s1, &dacvalue, "dacvalue") != 0) {
		xtrx_val_set(dev, XTRX_TRX, XTRX_CH_AB, XTRX_VCTCXO_DAC_VAL, (int)dacvalue);
	}

	s = malloc(sizeof(trx_xtrx_state_t));
	s->xtrx = dev;
	s->tx_inter = tx_inter;
	s->rx_decim = rx_decim;
	s->packetsize = packetsize;
	s->softfilt = softfilt;

	s1->opaque = s;
	s1->trx_end_func = trx_xtrx_end;
	s1->trx_write_func = trx_xtrx_write;
	s1->trx_start_func = trx_xtrx_start;
	s1->trx_read_func = trx_xtrx_read;
	s1->trx_set_tx_gain_func = trx_xtrx_set_tx_gain;
	s1->trx_set_rx_gain_func = trx_xtrx_set_rx_gain;
	s1->trx_get_sample_rate_func = trx_xtrx_get_sample_rate;
	s1->trx_get_tx_samples_per_packet_func = trx_xtrx_get_tx_samples_per_packet;
	s1->trx_get_stats = trx_xtrx_get_stats;

	s1->trx_dump_info = trx_xtrx_dump_info;
	s1->trx_get_abs_tx_power_func = trx_xtrx_get_abs_tx_power_func;
	s1->trx_get_abs_rx_power_func = trx_xtrx_get_abs_rx_power_func;

	s1->trx_write_func2 = trx_xtrx_write_func2;
	s1->trx_read_func2 = trx_xtrx_read_func2;

	return 0;
}
