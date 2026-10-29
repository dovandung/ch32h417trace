/*
 * ch32trace - Cortex-M parallel trace (TPIU, 1/2/4 bit) capture with the
 * CH32H417 USB3 logic analyzer (OpenSourceLogic-CH32H417 APP firmware,
 * version 3 or later for 225..300 MS/s).
 *
 * The analyzer oversamples TRACECLK and TRACEDATA[n:0] as plain logic
 * channels (8-channel mode, 1 byte per sample, bit k = channel k). This
 * tool recovers both clock edges (trace is DDR), picks the sampling point
 * inside each half period from the measured data transition positions,
 * finds the TPIU full sync (0x7FFFFFFF) to get bit and byte alignment and
 * writes the aligned TPIU byte stream for Orbuculum:
 *
 *     orbuculum -f trace.fifo -T -t 1
 *
 * It also deframes the TPIU stream itself, decodes the ITM stream (ID 1 by
 * default) and can print a stimulus port as text or check a stimulus port
 * that carries an incrementing 32-bit counter, to measure lost data.
 *
 * Input is either the device (USB, VID 1A86 PID 5538) or a raw capture file
 * with one byte per sample (sigrok-cli -O binary, 8 channels).
 *
 * Copyright (C) 2026 Do Van Dung <do_van_dzung@yahoo.com>
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
typedef SOCKET sock_t;
#define SOCK_BAD	INVALID_SOCKET
#define sock_close	closesocket
#define MSG_NOSIGNAL	0
#else
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int sock_t;
#define SOCK_BAD	(-1)
#define sock_close	close
#endif
#include <libusb.h>
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define CH32_X86 1
#endif

#include "vth_lut.h"

/* ------------------------------------------------------------ device */

#define CH32_VID		0x1a86
#define CH32_PID_USB2		0x5537
#define CH32_PID_USB3		0x5538
#define EP_CMD_OUT		0x01
#define EP_CMD_IN		0x81
#define EP_LOGIC_IN		0x82

#define CMD_SET_LOGIC_PARA	0xa0	/* {bit width 8/16, rate index} */
#define CMD_SET_LOGIC_LEVEL	0xa1	/* {DAC hi, DAC lo} */
#define CMD_SET_START		0xa2
#define CMD_SET_STOP		0xa3
#define CMD_GET_VERSION		0xac
#define CMD_RETURN_PARA		0x10
#define FW_FAST8_VERSION	3

static const struct { uint8_t sys; uint32_t mhz; int fast8; } rates[] = {
	{ 0x18, 300, 1 }, { 0x19, 275, 1 }, { 0x1a, 250, 1 }, { 0x1b, 225, 1 },
	{ 0x00, 200, 0 }, { 0x02, 175, 0 }, { 0x05, 150, 0 }, { 0x07, 125, 0 },
	{ 0x09, 100, 0 }, { 0x0e, 75, 0 }, { 0x10, 50, 0 }, { 0x11, 40, 0 },
	{ 0x12, 25, 0 }, { 0x13, 20, 0 }, { 0x15, 10, 0 },
};

#define NXFER		16
#define XFER_SIZE	(1024 * 1024)	/* multiple of 16 KiB */
#define NPOOL		160		/* ~0.5 s of buffering at 300 MB/s */

/* ------------------------------------------------------------ options */

struct opts {
	const char *in_file;	/* raw capture instead of the device */
	uint32_t rate_mhz;
	int clk_ch;
	int d_ch[4];
	int width;
	int phase;		/* INT32_MIN: automatic, <0: before the edge */
	double vio;		/* VIO18: supply of the capture pins */
	const char *tpiu_out;
	const char *itm_out;
	const char *raw_out;
	int tcp_port;		/* serve the TPIU stream on this port, 0: off */
	int tcp_any;		/* listen on all interfaces, not just localhost */
	int itm_id;
	int print_port;
	int check_port;
	double seconds;
	int quiet;
	int swo;		/* SWO mode instead of parallel trace */
	int swo_ch;		/* ... on this channel */
	int swo_manch;		/* Manchester, else UART / NRZ */
	double swo_baud;	/* UART bit rate, 0: measure it */
	int swo_fmt;		/* SWO carries TPIU frames (formatter on) */
};

/* ------------------------------------------------------------ decoder */

#define CAL_SAMPLES	(1u * 1024 * 1024)	/* ~3.5 ms at 300 MS/s */
#define CAL_MAX		(64u * 1024 * 1024)
#define HIST_BINS	64

struct dec {
	const struct opts *o;
	int clk;
	uint8_t nib_lut[256];	/* sample byte -> TRACEDATA value */
	uint8_t dmask;

	/* calibration */
	int calibrated;
	size_t cal_next;	/* try to calibrate again at this many samples */
	int waiting;		/* "waiting for the clock" was said */
	uint64_t idle;		/* samples since the last clock edge */
	uint8_t *cal;
	size_t cal_len;
	double half;		/* samples per half period of TRACECLK */
	int backward;		/* sample 'offset' samples before the next edge */
	int offset;		/* ... or 'offset' samples after the edge */

	/* stage A: clock edges -> one TRACEDATA value per half period */
	int width;
	unsigned prevc;
	int have_prevc, have_edge;
	uint8_t tail[64];	/* last 64 samples of the previous block */
	int carry_ok;		/* forward mode: last pick of the previous block */
	uint8_t carry_v;
	uint64_t carry_x;
	uint64_t last_edge, gap_thr;
	uint8_t *vbuf;		/* TRACEDATA values picked in this block */
	size_t nv;
	int pending_reset;
	uint64_t samples, edges, long_gaps, discont;
	void (*edges_fn)(struct dec *, const uint8_t *, size_t);
	const char *isa;
	uint8_t *valbuf;	/* values of a block (per-edge path) */
	uint64_t a_acc;		/* stage A: bits not yet in a whole byte */
	unsigned a_nb;

	/* stage A fast path (AVX2 + BMI2): 64 samples per step, one step late */
	int fast;
	unsigned f_lc;		/* edges closer than this need the exact path */
	unsigned f_thr;		/* gap threshold */
	uint8_t lut_lo[16], lut_hi[16];	/* nib_lut split by sample nibble */
	uint64_t f_wnext;	/* sample index of the next 64-sample word */
	int f_pending;		/* word W waits for the edges of word W + 1 */
	int f_have;		/* an edge seen since the last reset */
	int64_t f_first;	/* that first edge, until handled (-1: none) */
	uint64_t f_wpos, f_em1, f_e0;	/* W: start, edges of W - 1 and W */
	uint64_t f_q, f_s, f_g;	/* W: edges with a close / a near edge before,
				 * W - 1: edges with a close edge after */
	unsigned f_lk, f_l2, f_tk, f_t2;	/* doubling steps, last shift */
	uint64_t f_F[4];	/* W: TRACEDATA values, w bits per sample */
	uint8_t f_rem[64];	/* samples of a part word */
	unsigned f_nrem;

	/* stage B: values -> bytes, bit / byte alignment from the full sync */
	uint64_t acc;		/* bits not yet in a whole byte */
	unsigned nb;		/* ... how many (< 8 between blocks) */
	uint32_t run;		/* ones before the current byte */
	int synced;
	uint8_t *pb;		/* bytes of this block */
	size_t pb_size;
	uint8_t *ib;		/* ITM stream bytes of this block */
	size_t ib_n;
	uint64_t tpiu_bytes, bit_resyncs;

	/* TPIU frames */
	int fsync, fpos, cur_id, itm_sel;
	uint8_t frame[16];
	uint64_t frames, frame_errs, id_bytes[128];

	/* ITM */
	int istate, ineed, ilen, iport, ihw, zeros;
	uint32_t ival;
	uint64_t itm_sw[32], itm_hw, itm_ovf, itm_sync, itm_resv;
	int chk_have, chk_locked, chk_have_conf;
	uint32_t chk_last, chk_conf;
	uint64_t chk_n, chk_err, chk_missing, chk_back, chk_bad;

	/* SWO */
	unsigned swo_prev, swo_level;
	int swo_have;
	uint64_t swo_pos, swo_edges;
	double swo_T;		/* UART: samples per bit */
	int u_state, u_bit;
	unsigned u_byte;
	double u_next;		/* next sampling point */
	int m_state, m_nb;
	unsigned m_cur;
	double m_t0, m_h, m_exp, m_hsum, m_havg, m_mid;
	uint64_t m_hn;
	uint8_t *swo_buf;
	size_t swo_n, swo_cap;
	uint64_t swo_bytes, swo_ferr, swo_glitch, swo_packets, swo_perr;

	FILE *tpiu_f, *itm_f, *raw_f, *print_f;
};

static volatile sig_atomic_t g_stop;
static int g_silent;	/* -s: errors and warnings only */
#define INFO(...) do { if (!g_silent) fprintf(stderr, __VA_ARGS__); } while (0)

static void on_sigint(int s) { (void)s; g_stop = 1; }

/*
 * Counter check. A run of +1 steps is 'locked'. A value out of sequence
 * unlocks (one error); the check locks again on the next two consecutive
 * values a, a + 1, and the values between the end of the last locked run
 * and a count as missing. Garbage decoded while the stream was out of sync
 * counts as bad values, not as missing.
 */
static void check_value(struct dec *d, uint32_t v)
{
	d->chk_n++;
	if (!d->chk_have) {
		d->chk_have = 1;
		d->chk_last = v;
		return;
	}
	if (v == d->chk_last + 1) {
		if (!d->chk_locked) {
			d->chk_locked = 1;
			if (d->chk_have_conf) {
				uint32_t lost = d->chk_last - d->chk_conf - 1;
				if ((int32_t)lost >= 0)
					d->chk_missing += lost;
				else
					d->chk_back++;	/* target restarted? */
			}
		}
		d->chk_conf = v;
		d->chk_have_conf = 1;
	} else {
		if (d->chk_locked)
			d->chk_err++;
		d->chk_locked = 0;
		d->chk_bad++;
	}
	d->chk_last = v;
}

static void itm_run(struct dec *d, const uint8_t *p, size_t n);

/* decode the ITM bytes collected so far */
static void itm_flush(struct dec *d)
{
	if (!d->ib_n)
		return;
	if (d->itm_f)
		fwrite(d->ib, 1, d->ib_n, d->itm_f);
	itm_run(d, d->ib, d->ib_n);
	d->ib_n = 0;
}

static void itm_reset(struct dec *d)
{
	itm_flush(d);
	d->istate = 0;
	d->zeros = 0;
}

static void itm_packet(struct dec *d)
{
	const struct opts *o = d->o;
	int k;

	if (d->ihw) {
		d->itm_hw++;
		return;
	}
	d->itm_sw[d->iport]++;
	if (d->iport == o->print_port && d->print_f) {
		for (k = 0; k < d->ineed; k++) {
			uint8_t c = (uint8_t)(d->ival >> (8 * k));
			if (c)
				fputc(c, d->print_f);
		}
		fflush(d->print_f);
	}
	if (d->iport == o->check_port && d->ineed == 4)
		check_value(d, d->ival);
}

enum { I_HDR, I_PAY, I_CONT };

static void itm_byte(struct dec *d, uint8_t x)
{
	switch (d->istate) {
	case I_PAY:
		d->ival |= (uint32_t)x << (8 * d->ilen);
		if (++d->ilen == d->ineed) {
			itm_packet(d);
			d->istate = I_HDR;
		}
		return;
	case I_CONT:
		if (!(x & 0x80))
			d->istate = I_HDR;
		return;
	}

	if (x == 0x00) {
		d->zeros++;
		return;
	}
	if (x == 0x80 && d->zeros >= 5) {
		d->itm_sync++;
		d->zeros = 0;
		return;
	}
	d->zeros = 0;
	if (x == 0x70) {
		d->itm_ovf++;
	} else if (x & 0x03) {			/* SW or HW source packet */
		d->ineed = (x & 0x03) == 3 ? 4 : (x & 0x03);
		d->iport = x >> 3;
		d->ihw = x & 0x04;
		d->ilen = 0;
		d->ival = 0;
		d->istate = I_PAY;
	} else if ((x & 0x0f) == 0) {		/* local timestamp */
		if (x & 0x80)
			d->istate = I_CONT;
	} else if ((x & 0x0b) == 0x08) {	/* extension */
		if (x & 0x80)
			d->istate = I_CONT;
	} else if ((x & 0xdf) == 0x94) {	/* global timestamp 1/2 */
		d->istate = I_CONT;
	} else {
		d->itm_resv++;
	}
}

/* ITM bytes are collected per block and decoded in one pass */
static inline void itm_in(struct dec *d, uint8_t x)
{
	d->ib[d->ib_n++] = x;
}

static void itm_run(struct dec *d, const uint8_t *p, size_t n)
{
	size_t i = 0;

	while (i < n) {
		/* the common packet: stimulus port, 32-bit write */
		if (d->istate == I_HDR && (p[i] & 7) == 3 && i + 5 <= n) {
			uint32_t v;
			memcpy(&v, p + i + 1, 4);
			d->zeros = 0;
			d->iport = p[i] >> 3;
			d->ihw = 0;
			d->ineed = 4;
			d->ival = v;
			itm_packet(d);
			i += 5;
			continue;
		}
		itm_byte(d, p[i++]);
	}
}

static inline void stream_byte(struct dec *d, int id, uint8_t x)
{
	if (id == 0 || id == 0x7f)
		return;
	d->id_bytes[id]++;
	if (id == d->itm_sel)
		itm_in(d, x);
}

/* 4 bits -> bit 0 of bytes 0, 2, 4, 6 */
static const uint64_t spread4[16] = {
#define S4(x) ((uint64_t)((x) & 1) | (uint64_t)((x) >> 1 & 1) << 16 | \
	       (uint64_t)((x) >> 2 & 1) << 32 | (uint64_t)((x) >> 3 & 1) << 48)
	S4(0), S4(1), S4(2), S4(3), S4(4), S4(5), S4(6), S4(7),
	S4(8), S4(9), S4(10), S4(11), S4(12), S4(13), S4(14), S4(15)
#undef S4
};

/* one 16-byte TPIU frame */
static inline void decode_frame(struct dec *d, const uint8_t *f)
{
	uint64_t w0, w1;
	uint8_t aux = f[15];
	int i;

	d->frames++;
	memcpy(&w0, f, 8);
	memcpy(&w1, f + 8, 8);
	if (!((w0 | w1) & 0x0001000100010001ull)) {	/* no ID change: 15 data bytes */
		const int id = d->cur_id;
		if (id == d->itm_sel) {
			/* bit 0 of the even bytes comes from the aux byte */
			uint8_t *o = d->ib + d->ib_n;
			w0 |= spread4[aux & 15];
			w1 |= spread4[aux >> 4];
			memcpy(o, &w0, 8);
			memcpy(o + 8, &w1, 8);	/* byte 15 (aux) is not counted */
			d->ib_n += 15;
		}
		if (id != 0 && id != 0x7f)
			d->id_bytes[id] += 15;
		return;
	}
	for (i = 0; i < 8; i++) {
		uint8_t b = f[2 * i];
		int a = (aux >> i) & 1;

		if (b & 1) {			/* ID change */
			int nid = b >> 1;
			if (i == 7) {
				d->cur_id = nid;
			} else if (a) {		/* next byte is still old ID */
				stream_byte(d, d->cur_id, f[2 * i + 1]);
				d->cur_id = nid;
			} else {
				d->cur_id = nid;
				stream_byte(d, d->cur_id, f[2 * i + 1]);
			}
		} else {
			stream_byte(d, d->cur_id, b | a);
			if (i < 7)
				stream_byte(d, d->cur_id, f[2 * i + 1]);
		}
	}
}

static inline void frame_byte(struct dec *d, uint8_t b)
{
	d->frame[d->fpos++] = b;
	if (d->fpos == 16) {
		decode_frame(d, d->frame);
		d->fpos = 0;
	}
}

/* a full sync ends here: the next byte starts a frame */
static void frame_sync(struct dec *d)
{
	if (d->fsync && d->fpos != 3) {	/* 3: the FF FF FF of this sync */
		d->frame_errs++;
		itm_reset(d);
	}
	d->fsync = 1;
	d->fpos = 0;
}

/* bit / frame / ITM side of a discontinuity (runs in the bits thread) */
static void bits_reset(struct dec *d)
{
	d->discont++;
	d->acc = 0;
	d->nb = 0;
	d->run = 0;
	d->synced = 0;
	d->fsync = 0;
	itm_reset(d);
}

/* edge side; the bit side is reset with the next block of values */
static void dec_discontinuity(struct dec *d)
{
	d->have_prevc = 0;
	d->have_edge = 0;
	d->carry_ok = 0;
	d->a_acc = 0;
	d->a_nb = 0;
	d->f_pending = 0;
	d->f_have = 0;
	d->f_first = -1;
	d->f_em1 = d->f_e0 = 0;
	d->f_q = d->f_s = d->f_g = 0;
	d->f_nrem = 0;
	d->pending_reset = 1;
}

/* ------------------------------------------------------------ stage A */

/* a sample before this block: the last 64 are kept, plus one forward pick */
static uint8_t old_sample(const struct dec *d, uint64_t base, int64_t x)
{
	if (x >= -64)
		return d->tail[64 + x];
	if (d->carry_ok && base + x == d->carry_x)
		return d->carry_v;
	return d->tail[0];
}

/*
 * One TRACEDATA value per half period of TRACECLK, picked when the edge
 * that ends the half period is seen: 'offset' samples before that edge
 * (backward, offset <= 64), or 'offset' samples after the edge that started
 * it but no later than the sample before the next edge (forward). Every
 * half period gives exactly one value, even when quantization makes it a
 * sample shorter or longer.
 *
 * The clock channel of 64 samples is gathered into one 64-bit mask (AVX2:
 * 2 x vpmovmskb, SSE2: 4 x pmovmskb, else 8 multiplies); the edges are the
 * set bits of mask ^ (mask << 1), walked with tzcnt.
 */
#define EDGE_LOOP(MASK64, BACK)						\
{									\
	const uint64_t base = d->samples, thr = d->gap_thr;		\
	const int64_t off = d->offset;					\
	const uint8_t *lut = d->nib_lut;				\
	uint8_t *vb = d->vbuf;						\
	size_t nv = d->nv, i = 0;					\
	int64_t pe = d->have_edge ? (int64_t)(d->last_edge - base)	\
				  : INT64_MIN / 4;			\
	uint64_t gaps = 0, nedge = 0, prev, have = d->have_edge;	\
									\
	if (!d->have_prevc) {						\
		d->prevc = (s[0] >> d->clk) & 1;			\
		d->have_prevc = 1;					\
	}								\
	prev = d->prevc;						\
									\
	for (;; i += 64) {						\
		uint64_t c, e;						\
		if (i + 64 <= n) {					\
			c = MASK64(s + i);				\
		} else if (i < n) {					\
			size_t k;					\
			for (c = 0, k = 0; k < n - i; k++)		\
				c |= (uint64_t)((s[i + k] >> d->clk) & 1) << k; \
			c |= (c >> (n - i - 1) & 1) ?			\
				~0ull << (n - i) : 0;	/* no edge past n */ \
		} else {						\
			break;						\
		}							\
		e = c ^ ((c << 1) | prev);				\
		prev = c >> 63;						\
		nedge += (uint64_t)__builtin_popcountll(e);		\
		while (e) {						\
			const int64_t at = (int64_t)i + __builtin_ctzll(e); \
			int64_t x;					\
			uint8_t sv;					\
			e &= e - 1;					\
			if (BACK)					\
				x = at - off > pe ? at - off : pe;	\
			else						\
				x = pe + off < at - 1 ? pe + off : at - 1; \
			sv = __builtin_expect(x >= 0, 1) ? s[x]		\
				: old_sample(d, base, x);		\
			vb[nv] = lut[sv];				\
			nv += have;					\
			gaps += have & ((uint64_t)(at - pe) > thr);	\
			pe = at;					\
			have = 1;					\
		}							\
	}								\
	d->prevc = (unsigned)((s[n - 1] >> d->clk) & 1);		\
	d->nv = nv;							\
	d->edges += nedge;						\
	d->long_gaps += gaps;						\
	d->have_edge = (int)have;					\
	if (have)							\
		d->last_edge = base + (uint64_t)pe;			\
	if (!(BACK) && have && pe + off >= 0 && pe + off < (int64_t)n) { \
		d->carry_x = base + (uint64_t)(pe + off);		\
		d->carry_v = s[pe + off];				\
		d->carry_ok = 1;					\
	}								\
}

/* portable: 8 samples per multiply */
static inline uint64_t mask64_scalar(const uint8_t *p, int clk)
{
	uint64_t c = 0;
	int k;

	for (k = 0; k < 8; k++) {
		uint64_t w;
		memcpy(&w, p + 8 * k, 8);
		w = (w >> clk) & 0x0101010101010101ull;
		c |= ((w * 0x0102040810204080ull) >> 56) << (8 * k);
	}
	return c;
}

#define M_SCALAR(p) mask64_scalar((p), clk)
static void edges_scalar_b(struct dec *d, const uint8_t *s, size_t n)
{ const int clk = d->clk; EDGE_LOOP(M_SCALAR, 1) }
static void edges_scalar_f(struct dec *d, const uint8_t *s, size_t n)
{ const int clk = d->clk; EDGE_LOOP(M_SCALAR, 0) }

#ifdef CH32_X86
/* the clock bit is shifted to bit 7 of each byte, then pmovmskb */
static inline __attribute__((always_inline)) uint64_t
mask64_sse2(const uint8_t *p, __m128i sh)
{
	__m128i a = _mm_sll_epi16(_mm_loadu_si128((const __m128i *)p), sh);
	__m128i b = _mm_sll_epi16(_mm_loadu_si128((const __m128i *)(p + 16)), sh);
	__m128i c = _mm_sll_epi16(_mm_loadu_si128((const __m128i *)(p + 32)), sh);
	__m128i e = _mm_sll_epi16(_mm_loadu_si128((const __m128i *)(p + 48)), sh);

	return (uint64_t)(uint16_t)_mm_movemask_epi8(a) |
	       (uint64_t)(uint16_t)_mm_movemask_epi8(b) << 16 |
	       (uint64_t)(uint16_t)_mm_movemask_epi8(c) << 32 |
	       (uint64_t)(uint16_t)_mm_movemask_epi8(e) << 48;
}

#define M_SSE2(p) mask64_sse2((p), sh)
static void edges_sse2_b(struct dec *d, const uint8_t *s, size_t n)
{ const __m128i sh = _mm_cvtsi32_si128(7 - d->clk); EDGE_LOOP(M_SSE2, 1) }
static void edges_sse2_f(struct dec *d, const uint8_t *s, size_t n)
{ const __m128i sh = _mm_cvtsi32_si128(7 - d->clk); EDGE_LOOP(M_SSE2, 0) }

static inline __attribute__((always_inline, target("avx2"))) uint64_t
mask64_avx2(const uint8_t *p, __m128i sh)
{
	__m256i a = _mm256_sll_epi16(_mm256_loadu_si256((const __m256i *)p), sh);
	__m256i b = _mm256_sll_epi16(_mm256_loadu_si256((const __m256i *)(p + 32)), sh);

	return (uint64_t)(uint32_t)_mm256_movemask_epi8(a) |
	       (uint64_t)(uint32_t)_mm256_movemask_epi8(b) << 32;
}

#define M_AVX2(p) mask64_avx2((p), sh)
static __attribute__((target("avx2,bmi,popcnt"))) void
edges_avx2_b(struct dec *d, const uint8_t *s, size_t n)
{ const __m128i sh = _mm_cvtsi32_si128(7 - d->clk); EDGE_LOOP(M_AVX2, 1) }
static __attribute__((target("avx2,bmi,popcnt"))) void
edges_avx2_f(struct dec *d, const uint8_t *s, size_t n)
{ const __m128i sh = _mm_cvtsi32_si128(7 - d->clk); EDGE_LOOP(M_AVX2, 0) }
#endif

/*
 * Values -> bytes, LSB first (TRACEDATA0 is the first bit on the wire).
 * 8 / width values make one byte with one multiply; bytes go into a 64-bit
 * accumulator at the current bit position, 7 bytes per store.
 */
static size_t pack_values(struct dec *d, const uint8_t *v, size_t nv,
		uint8_t *q0)
{
	const unsigned w = (unsigned)d->width;
	uint64_t acc = d->a_acc;
	unsigned nb = d->a_nb;
	uint8_t *q = q0;
	size_t k = 0;

#define PUT_BYTE(byte) do {						\
		acc |= (uint64_t)(byte) << nb;				\
		nb += 8;						\
		if (nb >= 56) {						\
			memcpy(q, &acc, 8);				\
			q += 7;						\
			acc >>= 56;					\
			nb -= 56;					\
		}							\
	} while (0)

	if (w == 4) {
		for (; k + 2 <= nv; k += 2) {
			uint16_t x;
			memcpy(&x, v + k, 2);
			PUT_BYTE(((uint32_t)x * 0x110u) >> 8 & 0xff);
		}
	} else if (w == 2) {
		for (; k + 4 <= nv; k += 4) {
			uint32_t x;
			memcpy(&x, v + k, 4);
			PUT_BYTE((x * 0x01041040u) >> 24);
		}
	} else {
		for (; k + 8 <= nv; k += 8) {
			uint64_t x;
			memcpy(&x, v + k, 8);
			PUT_BYTE((x * 0x0102040810204080ull) >> 56);
		}
	}
#undef PUT_BYTE
	for (; k < nv; k++) {
		acc |= (uint64_t)v[k] << nb;
		nb += w;
		if (nb >= 56) {
			memcpy(q, &acc, 8);
			q += 7;
			acc >>= 56;
			nb -= 56;
		}
	}
	while (nb >= 8) {
		*q++ = (uint8_t)acc;
		acc >>= 8;
		nb -= 8;
	}
	d->a_acc = acc;
	d->a_nb = nb;
	return (size_t)(q - q0);
}

#ifdef CH32_X86
#include <cpuid.h>

/*
 * Fast path, x86 with AVX2 + BMI2. Per 64-sample word:
 *  - the clock channel -> 64-bit mask (2 x vpmovmskb), edges = c ^ c << 1;
 *  - all samples -> w-bit TRACEDATA values, packed (pshufb + pmaddubsw);
 *  - the samples to take are the edge mask shifted by the sampling offset
 *    (backward: e >> off, forward: e << off), and pext takes their values
 *    straight into the output bit stream: no loop over the edges.
 * The masks need the next word's edges, so each word is finished one word
 * late. Edges closer than the offset (glitches, jitter) are handled in the
 * same masks; see fw_finish().
 */

struct fctx { uint64_t acc; unsigned nb; uint8_t *q; uint64_t gaps; };

#define FAST_TARGET __attribute__((target("avx2,bmi,bmi2,popcnt")))

/*
 * Edge-distance masks over two consecutive words a (earlier) and b, for a
 * fixed distance L < 64 given as k doubling steps and a last shift s2
 * (L = 2^k - 1 + s2): [1, 2^k] U [s2, s2 + 2^k - 1] = [1, L].
 * near_before: bits of b with an edge 1..L samples before them.
 * near_after:  bits of a with an edge 1..L samples after them.
 */
static inline __attribute__((always_inline)) FAST_TARGET uint64_t
near_before(uint64_t a, uint64_t b, unsigned k, unsigned s2)
{
	unsigned span = 1;

	while (k--) {
		b |= (b << span) | (a >> (64 - span));
		a |= a << span;
		span *= 2;
	}
	return (b << 1) | (a >> 63) | (b << s2) | (a >> (64 - s2));
}

static inline __attribute__((always_inline)) FAST_TARGET uint64_t
near_after(uint64_t a, uint64_t b, unsigned k, unsigned s2)
{
	unsigned span = 1;

	while (k--) {
		a |= (a >> span) | (b << (64 - span));
		b |= b >> span;
		span *= 2;
	}
	return (a >> 1) | (b << 63) | (a >> s2) | (b << (64 - s2));
}

/* L (1..63) -> doubling steps and last shift */
static void near_steps(unsigned L, unsigned *k, unsigned *s2)
{
	unsigned span = 1;

	*k = 0;
	while (span * 2 <= L) {
		span *= 2;
		(*k)++;
	}
	*s2 = L - span + 1;
}

/* append n (<= 64) bits; branch free: always store, advance when full */
static inline __attribute__((always_inline)) FAST_TARGET void
fw_put(struct fctx *c, uint64_t bits, unsigned n)
{
	const uint64_t lo = c->acc | (bits << c->nb);
	const uint64_t hi = (bits >> 1) >> (63 - c->nb);	/* 0 when nb = 0 */
	const unsigned full = (c->nb + n) >> 6;

	memcpy(c->q, &lo, 8);
	c->q += 8 * full;
	c->acc = full ? hi : lo;
	c->nb = (c->nb + n) & 63;
}

/* the values of word W at the positions in P, in order */
static inline __attribute__((always_inline)) FAST_TARGET void
fw_take(struct dec *d, struct fctx *c, uint64_t P)
{
	const uint64_t *F = d->f_F;
	int q;

	switch (d->width) {
	case 4:
		for (q = 0; q < 4; q++) {
			uint64_t pq = (P >> (16 * q)) & 0xffff;
			fw_put(c, _pext_u64(F[q],
				_pdep_u64(pq, 0x1111111111111111ull) * 15),
				4 * (unsigned)_mm_popcnt_u64(pq));
		}
		break;
	case 2:
		for (q = 0; q < 2; q++) {
			uint64_t pq = (P >> (32 * q)) & 0xffffffffull;
			fw_put(c, _pext_u64(F[q],
				_pdep_u64(pq, 0x5555555555555555ull) * 3),
				2 * (unsigned)_mm_popcnt_u64(pq));
		}
		break;
	default:
		fw_put(c, _pext_u64(F[0], P), (unsigned)_mm_popcnt_u64(P));
		break;
	}
}

/*
 * Finish word W, now that the edges of W + 1 (en) are known. The pick for
 * a half period (pe, at) is, per edge, exactly as in the per-edge path:
 *   backward  max(at - off, pe): at - off, or pe when at - pe < off
 *   forward   min(pe + off, at - 1): pe + off, or at - 1 when at - pe <= off
 * As masks: the unclamped picks are the edge mask shifted by off; the
 * clamped ones are the edge before a close edge (backward) or one sample
 * before the close edge (forward). Picks of different half periods never
 * coincide, so the union of the masks is the list of picks in order.
 * Each pair of words is compared once and the results are kept for the
 * next word (f_q, f_s, f_g).
 */
static inline __attribute__((always_inline)) FAST_TARGET void
fw_finish(struct dec *d, struct fctx *c, uint64_t en)
{
	const uint64_t em = d->f_em1, e = d->f_e0;
	const int64_t w0 = (int64_t)d->f_wpos, first = d->f_first;
	const unsigned off = (unsigned)d->offset;
	uint64_t q = 0, sa = 0, lone, P;

	if (d->f_lc) {
		q = en & near_before(e, en, d->f_lk, d->f_l2);	/* in W + 1 */
		sa = e & near_after(e, en, d->f_lk, d->f_l2);	/* in W */
	}
	lone = e & ~d->f_g;
	d->f_g = near_before(e, en, d->f_tk, d->f_t2);
	if (first >= w0 && first < w0 + 64)
		lone &= ~(1ull << (first - w0));	/* no edge before it */
	c->gaps += (uint64_t)_mm_popcnt_u64(lone);

	if (d->backward) {			/* off >= 1, f_lc = off - 1 */
		P = ((e & ~d->f_q) >> off) | ((en & ~q) << (64 - off)) | sa;
		if (first >= 0 && first - (int64_t)off >= w0 &&
		    first - (int64_t)off < w0 + 64)
			P &= ~(1ull << (first - off - w0));	/* no edge before it */
	} else if (off) {			/* f_lc = off */
		P = ((e & ~sa) << off) | ((em & ~d->f_s) >> (64 - off)) |
		    (d->f_q >> 1) | (q << 63);
	} else {
		P = e;
	}
	d->f_q = q;
	d->f_s = sa;
	fw_take(d, c, P);
	if (first >= 0 && first < w0 + 64)
		d->f_first = -1;
}

static inline __attribute__((always_inline)) FAST_TARGET void
fw_word(struct dec *d, struct fctx *c, const uint8_t *p)
{
	const __m256i a = _mm256_loadu_si256((const __m256i *)p);
	const __m256i b = _mm256_loadu_si256((const __m256i *)(p + 32));
	const __m128i sh = _mm_cvtsi32_si128(7 - d->clk);
	const __m256i m4 = _mm256_set1_epi8(0x0f);
	const __m256i llo = _mm256_broadcastsi128_si256(_mm_loadu_si128((const __m128i *)d->lut_lo));
	const __m256i lhi = _mm256_broadcastsi128_si256(_mm_loadu_si128((const __m128i *)d->lut_hi));
	const uint64_t pos = d->f_wnext;
	uint64_t cm, e, F[4];
	__m256i va, vb;

	cm = (uint64_t)(uint32_t)_mm256_movemask_epi8(_mm256_sll_epi16(a, sh)) |
	     (uint64_t)(uint32_t)_mm256_movemask_epi8(_mm256_sll_epi16(b, sh)) << 32;
	if (!d->have_prevc) {
		d->prevc = (unsigned)(cm & 1);
		d->have_prevc = 1;
	}
	e = cm ^ ((cm << 1) | d->prevc);
	d->prevc = (unsigned)(cm >> 63);
	d->f_wnext += 64;
	d->edges += (uint64_t)_mm_popcnt_u64(e);
	if (e && !d->f_have) {
		d->f_have = 1;
		d->f_first = (int64_t)(pos + (uint64_t)__builtin_ctzll(e));
	}

	/* TRACEDATA value of every sample, w bits each, LSB first */
	va = _mm256_or_si256(_mm256_shuffle_epi8(llo, _mm256_and_si256(a, m4)),
		_mm256_shuffle_epi8(lhi, _mm256_and_si256(_mm256_srli_epi16(a, 4), m4)));
	vb = _mm256_or_si256(_mm256_shuffle_epi8(llo, _mm256_and_si256(b, m4)),
		_mm256_shuffle_epi8(lhi, _mm256_and_si256(_mm256_srli_epi16(b, 4), m4)));
	if (d->width == 4) {
		const __m256i k = _mm256_set1_epi16(0x1001);
		__m256i x = _mm256_packus_epi16(_mm256_maddubs_epi16(va, k),
						_mm256_maddubs_epi16(vb, k));
		x = _mm256_permute4x64_epi64(x, 0xd8);
		_mm256_storeu_si256((__m256i *)F, x);
	} else if (d->width == 2) {
		const __m256i k1 = _mm256_set1_epi16(0x0401);
		const __m256i k2 = _mm256_set1_epi32(0x00100001);
		__m256i x = _mm256_packus_epi32(
			_mm256_madd_epi16(_mm256_maddubs_epi16(va, k1), k2),
			_mm256_madd_epi16(_mm256_maddubs_epi16(vb, k1), k2));
		uint64_t l0, l1;
		x = _mm256_packus_epi16(x, x);
		l0 = (uint64_t)_mm256_extract_epi64(x, 0);
		l1 = (uint64_t)_mm256_extract_epi64(x, 2);
		F[0] = (l0 & 0xffffffffull) | (l1 << 32);
		F[1] = (l0 >> 32) | (l1 & 0xffffffff00000000ull);
	} else {
		F[0] = (uint64_t)(uint32_t)_mm256_movemask_epi8(_mm256_slli_epi16(va, 7)) |
		       (uint64_t)(uint32_t)_mm256_movemask_epi8(_mm256_slli_epi16(vb, 7)) << 32;
	}

	if (d->f_pending) {
		fw_finish(d, c, e);
	} else {				/* first word: nothing before it */
		d->f_g = near_before(0, e, d->f_tk, d->f_t2);
		d->f_q = d->f_lc ? e & near_before(0, e, d->f_lk, d->f_l2) : 0;
		d->f_s = 0;
	}
	d->f_em1 = d->f_pending ? d->f_e0 : 0;
	d->f_e0 = e;
	memcpy(d->f_F, F, sizeof(F));
	d->f_wpos = pos;
	d->f_pending = 1;
}

static FAST_TARGET void fast_block(struct dec *d, const uint8_t *s, size_t n)
{
	struct fctx c = { d->a_acc, d->a_nb, d->vbuf + d->nv, 0 };
	size_t i = 0;

	if (d->f_nrem) {			/* complete a part word */
		size_t k = 64 - d->f_nrem;
		if (k > n)
			k = n;
		memcpy(d->f_rem + d->f_nrem, s, k);
		d->f_nrem += (unsigned)k;
		i = k;
		if (d->f_nrem == 64) {
			fw_word(d, &c, d->f_rem);
			d->f_nrem = 0;
		}
	}
	for (; i + 64 <= n; i += 64)
		fw_word(d, &c, s + i);
	if (i < n) {
		memcpy(d->f_rem, s + i, n - i);
		d->f_nrem = (unsigned)(n - i);
	}
	d->a_acc = c.acc;
	d->a_nb = c.nb;
	d->nv = (size_t)(c.q - d->vbuf);
	d->long_gaps += c.gaps;
}

/* end of input: the part word (padded with its last sample), then W */
static FAST_TARGET void fast_flush(struct dec *d)
{
	struct fctx c = { d->a_acc, d->a_nb, d->vbuf + d->nv, 0 };

	if (d->f_nrem) {
		memset(d->f_rem + d->f_nrem, d->f_rem[d->f_nrem - 1], 64 - d->f_nrem);
		fw_word(d, &c, d->f_rem);
		d->f_nrem = 0;
	}
	if (d->f_pending) {
		fw_finish(d, &c, 0);
		d->f_pending = 0;
	}
	while (c.nb >= 8) {			/* whole bytes still in acc */
		*c.q++ = (uint8_t)c.acc;
		c.acc >>= 8;
		c.nb -= 8;
	}
	d->a_acc = c.acc;
	d->a_nb = c.nb;
	d->nv = (size_t)(c.q - d->vbuf);
	d->long_gaps += c.gaps;
}

/* pdep / pext are microcoded (slow) on AMD before Zen 3 */
static int fast_pdep(void)
{
	unsigned a, b, c, dd, fam;

	if (!__get_cpuid(0, &a, &b, &c, &dd))
		return 0;
	if (b != 0x68747541)			/* not "AuthenticAMD" */
		return 1;
	__get_cpuid(1, &a, &b, &c, &dd);
	fam = ((a >> 8) & 0xf) + ((a >> 20) & 0xff);
	return fam >= 0x19;
}
#endif

/*
 * Pick the stage A code for this CPU and sampling point. CH32TRACE_ISA =
 * fast | avx2 | sse2 | scalar forces one (for testing).
 */
static const char *g_isa;	/* the speed test forces a path */

static void select_edges(struct dec *d)
{
	const char *f = g_isa ? g_isa : getenv("CH32TRACE_ISA");
	const int back = d->backward;

	d->fast = 0;
	d->edges_fn = back ? edges_scalar_b : edges_scalar_f;
	d->isa = "scalar";
#ifdef CH32_X86
	if (f && !strcmp(f, "scalar"))
		return;
	d->edges_fn = back ? edges_sse2_b : edges_sse2_f;
	d->isa = "sse2";
	if (f && !strcmp(f, "sse2"))
		return;
	__builtin_cpu_init();
	if (!__builtin_cpu_supports("avx2") || !__builtin_cpu_supports("bmi") ||
	    !__builtin_cpu_supports("popcnt"))
		return;
	d->edges_fn = back ? edges_avx2_b : edges_avx2_f;
	d->isa = "avx2";
	if (f && !strcmp(f, "avx2"))
		return;
	if (__builtin_cpu_supports("bmi2") && (fast_pdep() || (f && !strcmp(f, "fast"))) &&
	    d->offset <= 32 && (!back || d->offset >= 1) &&
	    d->gap_thr >= 1 && d->gap_thr <= 63) {
		int k;
		d->fast = 1;
		d->isa = "avx2+bmi2 (pext)";
		d->f_lc = back ? (unsigned)d->offset - 1 : (unsigned)d->offset;
		d->f_thr = (unsigned)d->gap_thr;
		if (d->f_lc)
			near_steps(d->f_lc, &d->f_lk, &d->f_l2);
		near_steps(d->f_thr, &d->f_tk, &d->f_t2);
		d->f_first = -1;
		for (k = 0; k < 16; k++) {
			d->lut_lo[k] = d->nib_lut[k];
			d->lut_hi[k] = d->nib_lut[k << 4];
		}
	}
#else
	(void)f;
#endif
}

static __attribute__((noinline)) void run_edges(struct dec *d,
		const uint8_t *s, size_t n)
{
	if (!n)
		return;
#ifdef CH32_X86
	if (d->fast) {
		fast_block(d, s, n);
		d->samples += n;
		return;
	}
#endif
	{
		uint8_t *out = d->vbuf;
		size_t nb0 = d->nv;

		d->vbuf = d->valbuf;		/* values first, then bytes */
		d->nv = 0;
		d->edges_fn(d, s, n);
		d->vbuf = out;
		d->nv = nb0 + pack_values(d, d->valbuf, d->nv, out + nb0);
	}
	if (n >= 64) {
		memcpy(d->tail, s + n - 64, 64);
	} else {
		memmove(d->tail, d->tail + n, 64 - n);
		memcpy(d->tail + 64 - n, s, n);
	}
	d->samples += n;
}

/* ------------------------------------------------------------ stage B */

/*
 * Bytes from stage A are cut wherever stage A started; the TPIU byte
 * boundary is 'nb' bits later (set by the last full sync). Re-cut the block
 * by that shift, 8 bytes per step; 'acc' holds the nb bits carried over.
 */
static size_t recut(struct dec *d, const uint8_t *in, size_t n)
{
	const unsigned nb = d->nb;
	uint64_t carry = d->acc;
	uint8_t *q = d->pb;
	size_t k = 0;

	if (!nb) {
		memcpy(q, in, n);
		return n;
	}
	for (; k + 8 <= n; k += 8) {
		uint64_t x, o;
		memcpy(&x, in + k, 8);
		o = (x << nb) | carry;
		carry = x >> (64 - nb);
		memcpy(q + k, &o, 8);
	}
	for (; k < n; k++) {
		uint64_t o = ((uint64_t)in[k] << nb) | carry;
		q[k] = (uint8_t)o;
		carry = o >> 8;
	}
	d->acc = carry;
	return n;
}

/*
 * A full sync ended at bit z (< 7) of byte j: the bits after it start the
 * next byte. Re-cut the rest of the block (and the pending bits) in place;
 * the block can grow by one byte.
 */
static size_t realign(struct dec *d, size_t j, unsigned z, size_t m)
{
	uint8_t *pb = d->pb;
	uint64_t acc = pb[j] >> (z + 1);
	unsigned nb = 7 - z;
	size_t k;

	for (k = j + 1; k < m; k++) {
		acc |= (uint64_t)pb[k] << nb;
		pb[k] = (uint8_t)acc;
		acc >>= 8;
	}
	acc |= d->acc << nb;
	nb += d->nb;
	if (nb >= 8) {
		pb[m++] = (uint8_t)acc;
		acc >>= 8;
		nb -= 8;
	}
	d->acc = acc;
	d->nb = nb;
	return m;
}

/*
 * TCP server for the aligned TPIU stream: Orbuculum connects to it with
 * "orbuculum -s localhost:PORT -T -t 1" (on Windows too, where there is
 * no FIFO). One client at a time; a new one replaces the old one.
 */
static struct {
	sock_t ls, cl;
	int on;
	pthread_t th;
	pthread_mutex_t mx;
} N = { .ls = SOCK_BAD, .cl = SOCK_BAD, .mx = PTHREAD_MUTEX_INITIALIZER };

static void *net_accept(void *arg)
{
	(void)arg;
	while (N.on) {
		sock_t c = accept(N.ls, NULL, NULL);
		if (c == SOCK_BAD)
			continue;
		pthread_mutex_lock(&N.mx);
		if (N.cl != SOCK_BAD)
			sock_close(N.cl);
		N.cl = c;
		pthread_mutex_unlock(&N.mx);
		INFO("TCP client connected\n");
	}
	return NULL;
}

static int net_start(int port, int any)
{
	struct sockaddr_in a;
	int one = 1;

#ifdef _WIN32
	WSADATA w;
	if (WSAStartup(MAKEWORD(2, 2), &w)) {
		fprintf(stderr, "WSAStartup failed\n");
		return -1;
	}
#endif
	N.ls = socket(AF_INET, SOCK_STREAM, 0);
	if (N.ls == SOCK_BAD)
		return -1;
	setsockopt(N.ls, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one));
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons((uint16_t)port);
	a.sin_addr.s_addr = htonl(any ? INADDR_ANY : INADDR_LOOPBACK);
	if (bind(N.ls, (struct sockaddr *)&a, sizeof(a)) || listen(N.ls, 1)) {
		fprintf(stderr, "cannot listen on TCP port %d\n", port);
		sock_close(N.ls);
		return -1;
	}
	N.on = 1;
	if (pthread_create(&N.th, NULL, net_accept, NULL))
		return -1;
	pthread_detach(N.th);
	INFO("TPIU stream on TCP %s:%d (orbuculum -s localhost:%d -T -t 1)\n",
		any ? "*" : "127.0.0.1", port, port);
	return 0;
}

static void net_send(const uint8_t *p, size_t n)
{
	pthread_mutex_lock(&N.mx);
	while (N.cl != SOCK_BAD && n) {
		int k = send(N.cl, (const char *)p, n > (1u << 20) ? (1 << 20) : (int)n,
			     MSG_NOSIGNAL);
		if (k <= 0) {
			sock_close(N.cl);
			N.cl = SOCK_BAD;
			INFO("TCP client gone\n");
			break;
		}
		p += k;
		n -= (size_t)k;
	}
	pthread_mutex_unlock(&N.mx);
}

static void out_bytes(struct dec *d, const uint8_t *p, size_t n)
{
	d->tpiu_bytes += n;
	if (d->tpiu_f && n)
		fwrite(p, 1, n, d->tpiu_f);
	if (N.on && n)
		net_send(p, n);
}

/*
 * Find full syncs (31 ones, then a zero that must be bit 7 of a byte) at
 * byte granularity: 'run' counts the ones before the current byte. A sync
 * can only end in a byte when run >= 24, so most bytes cost one compare.
 */
static void scan_bytes(struct dec *d, size_t m)
{
	static const uint8_t full_sync[4] = { 0xff, 0xff, 0xff, 0x7f };
	uint8_t *pb = d->pb;
	uint32_t run = d->run;
	size_t j, out0 = 0;

	for (j = 0; j < m; j++) {
		const unsigned b = pb[j];

		/* a whole frame with no 0xFF byte cannot hold a sync */
		if (d->fpos == 0 && d->fsync && run < 24 && j + 16 <= m) {
#ifdef CH32_X86
			const __m128i f = _mm_loadu_si128((const __m128i *)(pb + j));
			const int noff = !_mm_movemask_epi8(_mm_cmpeq_epi8(f,
					_mm_set1_epi8((char)0xff)));
#else
			int noff = 1, t;
			for (t = 0; t < 16; t++)
				noff &= pb[j + t] != 0xff;
#endif
			if (noff) {
				decode_frame(d, pb + j);
				run = (uint32_t)__builtin_clz((~(unsigned)pb[j + 15] & 0xffu) << 24);
				j += 15;
				continue;
			}
		}
		if (b == 0xff) {
			run = run < 64 ? run + 8 : run;
			if (d->fsync)
				frame_byte(d, (uint8_t)b);
			continue;
		}
		if (run >= 24) {
			const unsigned z = (unsigned)__builtin_ctz(~b);

			if (run + z >= 31) {
				if (z == 7 && d->synced) {	/* aligned: the usual case */
					frame_sync(d);
					run = 0;
					continue;
				}
				if (d->synced) {		/* bit slip */
					d->bit_resyncs++;
					out_bytes(d, pb + out0, j - out0);
					out_bytes(d, full_sync + 3, 1);
				} else {			/* first sync */
					out_bytes(d, full_sync, 4);
				}
				d->synced = 1;
				frame_sync(d);
				if (z != 7)
					m = realign(d, j, z, m);
				out0 = j + 1;
				run = 0;
				continue;
			}
		}
		/* ones at the top of b carry into the next byte */
		run = (uint32_t)__builtin_clz((~b & 0xffu) << 24);
		if (d->fsync)
			frame_byte(d, (uint8_t)b);
	}
	if (d->synced)
		out_bytes(d, pb + out0, m - out0);
	d->run = run;
	itm_flush(d);
}

static void bits_bufs(struct dec *d, size_t n)
{
	/* plus one byte per possible re-cut (each needs >= 4 bytes) */
	size_t need = n + n / 4 + 64;

	if (need > d->pb_size) {
		free(d->pb);
		free(d->ib);
		d->pb = malloc(need);
		d->ib = malloc(need);
		d->pb_size = need;
		if (!d->pb || !d->ib) {
			fprintf(stderr, "out of memory\n");
			exit(1);
		}
	}
}

static __attribute__((noinline)) void run_bits(struct dec *d,
		const uint8_t *in, size_t n)
{
	bits_bufs(d, n);
	scan_bytes(d, recut(d, in, n));
}

/*
 * Two stage pipeline: the caller's thread finds the edges and picks one
 * value per half period; the bits thread does alignment, TPIU frames and
 * ITM. Blocks of values go between them through a small queue.
 */
#define NVB		8
#define VB_SIZE		(XFER_SIZE + 8)

static struct {
	pthread_mutex_t mx;
	pthread_cond_t cv;
	uint8_t *free_b[NVB], *q_b[NVB];
	size_t q_len[NVB];
	int q_reset[NVB];
	int nfree, qhead, qlen, done, started;
	pthread_t th;
} V = { .mx = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

static void *bits_thread(void *arg)
{
	struct dec *d = arg;

	for (;;) {
		uint8_t *b;
		size_t len;
		int reset;

		pthread_mutex_lock(&V.mx);
		while (!V.qlen && !V.done)
			pthread_cond_wait(&V.cv, &V.mx);
		if (!V.qlen) {
			pthread_mutex_unlock(&V.mx);
			return NULL;
		}
		b = V.q_b[V.qhead];
		len = V.q_len[V.qhead];
		reset = V.q_reset[V.qhead];
		V.qhead = (V.qhead + 1) % NVB;
		V.qlen--;
		pthread_mutex_unlock(&V.mx);

		if (reset)
			bits_reset(d);
		run_bits(d, b, len);

		pthread_mutex_lock(&V.mx);
		V.free_b[V.nfree++] = b;
		pthread_cond_broadcast(&V.cv);
		pthread_mutex_unlock(&V.mx);
	}
}

static int pipe_start(struct dec *d)
{
	int i;

	V.done = 0;
	V.qhead = V.qlen = 0;

	for (i = 0; i < NVB; i++)
		if (!(V.free_b[V.nfree++] = malloc(VB_SIZE)))
			return -1;
	if (pthread_create(&V.th, NULL, bits_thread, d))
		return -1;
	V.started = 1;
	return 0;
}

static void pipe_stop(void)
{
	int i;

	if (!V.started)
		return;
	pthread_mutex_lock(&V.mx);
	V.done = 1;
	pthread_cond_broadcast(&V.cv);
	pthread_mutex_unlock(&V.mx);
	pthread_join(V.th, NULL);
	V.started = 0;
	for (i = 0; i < V.nfree; i++)
		free(V.free_b[i]);
	V.nfree = 0;
}

static void run_sampler(struct dec *d, const uint8_t *s, size_t n)
{
	while (n) {
		size_t m = n < XFER_SIZE ? n : XFER_SIZE;
		int slot;

		pthread_mutex_lock(&V.mx);
		while (!V.nfree)
			pthread_cond_wait(&V.cv, &V.mx);
		d->vbuf = V.free_b[--V.nfree];
		pthread_mutex_unlock(&V.mx);

		d->nv = 0;
		run_edges(d, s, m);

		pthread_mutex_lock(&V.mx);
		slot = (V.qhead + V.qlen) % NVB;
		V.q_b[slot] = d->vbuf;
		V.q_len[slot] = d->nv;
		V.q_reset[slot] = d->pending_reset;
		d->pending_reset = 0;
		V.qlen++;
		pthread_cond_broadcast(&V.cv);
		pthread_mutex_unlock(&V.mx);

		s += m;
		n -= m;
	}
}

/*
 * Find the clock half period and where the data lines change relative to
 * the clock edges, then pick the sample furthest from data transitions.
 * hf[p]: transitions p samples after the last edge (edge sample = 0).
 * hb[r]: transitions r samples before the next edge (edge sample = 0).
 * A transition 'at j' means s[j] != s[j - 1]; sampling at x is at risk from
 * transitions at x and x + 1, and less so at x - 1 and x + 2.
 */
static int calibrate(struct dec *d)
{
	const uint8_t *s = d->cal;
	const size_t n = d->cal_len;
	const unsigned cm = 1u << d->clk, dm = d->dmask;
	size_t j, nedge = 0, first = 0, last = 0, np = 0;
	size_t pend[256];	/* data transitions since the last clock edge */
	uint64_t hf[HIST_BINS] = { 0 }, hb[HIST_BINS] = { 0 }, ntrans = 0;
	uint64_t best = UINT64_MAX;
	int P, p, r;

	for (j = 1; j < n; j++) {
		unsigned x;
		size_t t;

		if (j + 8 <= n) {		/* skip runs of unchanged samples */
			uint64_t u, v;
			memcpy(&u, s + j, 8);
			memcpy(&v, s + j - 1, 8);
			if (u == v) {
				j += 7;
				continue;
			}
		}
		x = s[j] ^ s[j - 1];
		if (!x)
			continue;
		if (x & cm) {
			/* transitions since the last edge: their distance to it
			 * and to this one */
			for (t = 0; t < np; t++) {
				size_t pos = pend[t] - last, rpos = j - pend[t];
				hf[pos < HIST_BINS ? pos : HIST_BINS - 1]++;
				hb[rpos < HIST_BINS ? rpos : HIST_BINS - 1]++;
				ntrans++;
			}
			np = 0;
			if (!nedge)
				first = j;
			last = j;
			nedge++;
		}
		if (nedge && (x & dm)) {
			if (j == last) {	/* on the edge itself */
				hf[0]++;
				hb[0]++;
				ntrans++;
			} else if (np < 256) {
				pend[np++] = j;
			}
		}
	}
	if (nedge < 10)
		return -2;		/* no clock (or only noise) */
	if (nedge < 1000)
		return -1;		/* a clock, but too slow for this window */

	d->half = (double)(last - first) / (double)(nedge - 1);

	P = (int)floor(d->half);
	d->gap_thr = (uint64_t)(d->half * 1.5 + 1.5);

	if (d->o->phase != INT32_MIN) {
		d->backward = d->o->phase < 0;
		d->offset = abs(d->o->phase);
		if (d->backward && d->offset > 64) {
			fprintf(stderr, "-p: at most 64 samples before the edge.\n");
			d->offset = 64;
		}
	} else if (P < 2) {
		d->backward = 1;
		d->offset = 1;
	} else {
#define HF(i) ((i) >= 0 && (i) < HIST_BINS ? hf[i] : 0)
#define HB(i) ((i) >= 0 && (i) < HIST_BINS ? hb[i] : 0)
		for (r = 1; r <= P && r <= 64; r++) {	/* x = next edge - r */
			uint64_t sc = 2 * (HB(r) + HB(r - 1)) + HB(r + 1) + HB(r - 2);
			if (sc < best) {
				best = sc;
				d->backward = 1;
				d->offset = r;
			}
		}
		for (p = 0; p < P; p++) {	/* x = edge + p */
			uint64_t sc = 2 * (HF(p) + HF(p + 1)) + HF(p - 1) + HF(p + 2);
			if (sc < best) {
				best = sc;
				d->backward = 0;
				d->offset = p;
			}
		}
#undef HF
#undef HB
	}

	if (!d->o->quiet) {
		double rate = d->o->rate_mhz * 1e6;
		fprintf(stderr, "TRACECLK  %.3f MHz (%.2f samples per half period at %u MS/s)\n",
			rate / (2.0 * d->half) / 1e6, d->half, d->o->rate_mhz);
		fprintf(stderr, "data transitions, samples after an edge: ");
		for (p = 0; p <= P && p < HIST_BINS; p++)
			fprintf(stderr, " %d:%.1f%%", p, ntrans ? 100.0 * hf[p] / ntrans : 0.0);
		fprintf(stderr, "\n                  samples before an edge:");
		for (r = 1; r <= P && r < HIST_BINS; r++)
			fprintf(stderr, " %d:%.1f%%", r, ntrans ? 100.0 * hb[r] / ntrans : 0.0);
		fprintf(stderr, "\nsampling %d sample(s) %s each edge%s\n", d->offset,
			d->backward ? "before" : "after",
			d->o->phase != INT32_MIN ? " (forced)" : "");
		if (d->half < 2.5)
			fprintf(stderr, "WARNING: under 2.5 samples per half period, "
				"decoding will be unreliable; lower TRACECLK.\n");
		if (!ntrans)
			fprintf(stderr, "WARNING: no activity on the data channels.\n");
	}
	select_edges(d);
	if (!d->o->quiet)
		fprintf(stderr, "edge finder: %s\n", d->isa);
	return 0;
}

/* ------------------------------------------------------------ SWO */

/*
 * SWO on one analyzer channel (-W), decoded from the samples edge by edge.
 *  UART / NRZ: idle high; 1 start bit (low), 8 data bits LSB first, 1 stop
 *    bit (high). The bit time comes from -b or is measured (auto-baud).
 *  Manchester (ARM TPIU): idle low; a packet starts with a 1 and ends when
 *    the line stays low for a bit; a 1 is high then low, a 0 low then high,
 *    so every bit has a transition in its middle. Self-clocking: the half
 *    bit time is measured on each start bit and the phase follows every
 *    mid-bit edge.
 * An edge at sample t (the first sample with the new level) is at t - 0.5.
 * SWO without the TPIU formatter (the usual setup) is the ITM stream
 * itself; with it (-F), the bytes go through the same TPIU deframer as
 * parallel trace.
 */
enum { U_IDLE, U_BITS };
enum { M_IDLE, M_START, M_BITS };

static void swo_put(struct dec *d, uint8_t b)
{
	if (d->swo_n == d->swo_cap) {
		d->swo_cap = d->swo_cap ? 2 * d->swo_cap : 1 << 16;
		d->swo_buf = realloc(d->swo_buf, d->swo_cap);
		if (!d->swo_buf) {
			fprintf(stderr, "out of memory\n");
			exit(1);
		}
	}
	d->swo_buf[d->swo_n++] = b;
	d->swo_bytes++;
}

/* UART: the sampling points before time t, at the current level */
static void uart_until(struct dec *d, double t)
{
	while (d->u_state == U_BITS && d->u_next < t) {
		const unsigned lv = d->swo_level;

		if (d->u_bit < 0) {			/* middle of the start bit */
			if (lv) {
				d->swo_glitch++;
				d->u_state = U_IDLE;
				return;
			}
		} else if (d->u_bit < 8) {
			d->u_byte |= lv << d->u_bit;
		} else {				/* stop bit */
			if (lv)
				swo_put(d, (uint8_t)d->u_byte);
			else
				d->swo_ferr++;
			d->u_state = U_IDLE;
			return;
		}
		d->u_bit++;
		d->u_next += d->swo_T;
	}
}

static void uart_edge(struct dec *d, double b, unsigned lv)
{
	uart_until(d, b);
	d->swo_level = lv;
	if (d->u_state == U_IDLE && !lv) {	/* start bit */
		d->u_state = U_BITS;
		d->u_bit = -1;
		d->u_byte = 0;
		d->u_next = b + 0.5 * d->swo_T;
	}
}

static void manch_end(struct dec *d)
{
	if (d->m_nb)
		d->swo_perr++;			/* not a whole number of bytes */
	d->swo_packets++;
	d->m_state = M_IDLE;
}

static void manch_edge(struct dec *d, double b, unsigned lv)
{
	switch (d->m_state) {
	case M_BITS:
		if (b < d->m_exp - 0.5 * d->m_h)
			break;			/* between two bits */
		if (b <= d->m_exp + 0.5 * d->m_h) {	/* middle of a bit */
			/* two mid-bit edges are one bit apart: the best
			 * measure of the bit time, kept across packets */
			const double est = 0.5 * (b - d->m_mid);
			if (est > 0.75 * d->m_h && est < 1.25 * d->m_h) {
				d->m_h += 0.25 * (est - d->m_h);
				d->m_havg += (est - d->m_havg) / 64;
				d->m_hsum += est;
				d->m_hn++;
			}
			d->m_mid = b;
			d->m_cur |= (unsigned)!lv << d->m_nb;	/* falling: 1 */
			if (++d->m_nb == 8) {
				swo_put(d, (uint8_t)d->m_cur);
				d->m_cur = 0;
				d->m_nb = 0;
			}
			d->m_exp = b + 2 * d->m_h;
			break;
		}
		manch_end(d);			/* the line was low for a bit */
		/* this edge can start the next packet */
		__attribute__((fallthrough));
	case M_IDLE:
		if (lv) {
			d->m_t0 = b;
			d->m_state = M_START;
		}
		break;
	case M_START:				/* end of the start bit's high half */
		if (!lv && b - d->m_t0 >= 0.75) {
			/* the high half of the start bit: coarse (whole
			 * samples), so the learned bit time wins if close */
			const double hm = b - d->m_t0;
			if (d->m_havg > 0 && hm > 0.5 * d->m_havg && hm < 1.5 * d->m_havg)
				d->m_h = d->m_havg;
			else
				d->m_h = d->m_havg = hm;
			d->m_mid = b;
			d->m_exp = b + 2 * d->m_h;
			d->m_cur = 0;
			d->m_nb = 0;
			d->m_state = M_BITS;
		} else {
			d->m_state = M_IDLE;
		}
		break;
	}
	d->swo_level = lv;
}

static void manch_until(struct dec *d, double t)
{
	if (d->m_state == M_BITS && t > d->m_exp + 0.5 * d->m_h)
		manch_end(d);
}

static void swo_run(struct dec *d, const uint8_t *s, size_t n)
{
	const int ch = d->o->swo_ch, manch = d->o->swo_manch;
	const uint64_t base = d->swo_pos;
	size_t i;

	if (!n)
		return;
	if (!d->swo_have) {
		d->swo_prev = d->swo_level = (s[0] >> ch) & 1;
		d->swo_have = 1;
	}
	for (i = 0; i < n; i += 64) {
		const unsigned cnt = n - i < 64 ? (unsigned)(n - i) : 64;
		uint64_t m, e;
		double tend;

		if (cnt == 64) {
			m = mask64_scalar(s + i, ch);
		} else {
			unsigned k;
			for (m = 0, k = 0; k < cnt; k++)
				m |= (uint64_t)((s[i + k] >> ch) & 1) << k;
		}
		e = m ^ ((m << 1) | d->swo_prev);
		if (cnt < 64)
			e &= (1ull << cnt) - 1;
		d->swo_prev = (unsigned)(m >> (cnt - 1)) & 1;
		while (e) {
			const unsigned k = (unsigned)__builtin_ctzll(e);
			const double b = (double)(base + i + k) - 0.5;
			e &= e - 1;
			d->swo_edges++;
			if (manch)
				manch_edge(d, b, (unsigned)(m >> k) & 1);
			else
				uart_edge(d, b, (unsigned)(m >> k) & 1);
		}
		tend = (double)(base + i + cnt) - 0.5;
		if (manch)
			manch_until(d, tend);
		else
			uart_until(d, tend);
	}
	d->swo_pos += n;
	d->samples += n;
}

/* the decoded SWO bytes: to -o / -S, and to the ITM decoder */
static void swo_flush(struct dec *d)
{
	if (!d->swo_n)
		return;
	if (d->o->swo_fmt) {
		run_bits(d, d->swo_buf, d->swo_n);	/* TPIU frames */
	} else {
		out_bytes(d, d->swo_buf, d->swo_n);
		if (d->itm_sel >= 0) {
			bits_bufs(d, d->swo_n);
			memcpy(d->ib + d->ib_n, d->swo_buf, d->swo_n);
			d->ib_n += d->swo_n;
			itm_flush(d);
		}
	}
	d->swo_n = 0;
}

/*
 * UART auto-baud: run lengths between edges are whole numbers of bits.
 * Start from the shortest common run, then fit the bit time to all runs
 * up to 12 bits long.
 */
static int swo_calibrate(struct dec *d)
{
	const uint8_t *s = d->cal;
	const int ch = d->o->swo_ch;
	static uint32_t hist[4096];
	uint64_t nr = 0, k;
	size_t j, last = 0;
	int have = 0;
	double T = 0;
	unsigned r, r1 = 0, it;

	memset(hist, 0, sizeof(hist));
	for (j = 1; j < d->cal_len; j++) {
		if (!(((s[j] ^ s[j - 1]) >> ch) & 1))
			continue;
		if (have) {
			r = (unsigned)(j - last < 4095 ? j - last : 4095);
			hist[r]++;
			nr++;
		}
		have = 1;
		last = j;
	}
	if (nr < 10)
		return -2;		/* no SWO traffic */
	if (nr < 200)
		return -1;
	for (r = 1; r < 4095; r++)
		if (hist[r] >= 2 && hist[r] * 200 >= nr) {
			r1 = r;
			break;
		}
	if (!r1)
		return -1;
	/* a run of r samples is round(r / T) bits; a bit time between two
	 * whole samples shows up as runs of both lengths */
	T = r1 + (hist[r1 + 1] > hist[r1] / 4 ? 0.5 : 0.0);
	for (it = 0; it < 4; it++) {
		double num = 0, den = 0;
		for (r = 1; r < 4095 && r <= 12 * T; r++) {
			k = (uint64_t)(r / T + 0.5);
			if (k >= 1 && hist[r]) {
				num += (double)r * hist[r];
				den += (double)k * hist[r];
			}
		}
		if (den > 0)
			T = num / den;
	}
	if (d->o->swo_manch) {		/* runs are 1 or 2 half bits */
		d->m_havg = T;
		T *= 2;
		if (!d->o->quiet)
			fprintf(stderr, "SWO Manchester: %.1f samples per bit -> %.0f baud "
				"(measured, then followed per bit)\n", T, d->o->rate_mhz * 1e6 / T);
	} else {
		d->swo_T = T;
		if (!d->o->quiet)
			fprintf(stderr, "SWO UART: %.1f samples per bit -> %.0f baud (measured)\n",
				T, d->o->rate_mhz * 1e6 / T);
	}
	if (T < 4)
		fprintf(stderr, "WARNING: under 4 samples per SWO bit; raise the "
			"sample rate (-r).\n");
	return 0;
}

/*
 * Collect samples until the clock (parallel trace) or the SWO bit rate
 * (UART auto-baud) can be measured, however long that takes. With no edges
 * at all the samples are dropped; with too few for a good measurement the
 * window grows (up to CAL_MAX, then its older half is dropped). Returns the
 * number of samples of s used; d->calibrated says if it worked.
 */
static void fast_drain(struct dec *d);

static size_t cal_collect(struct dec *d, const uint8_t *s, size_t n,
		int (*cal)(struct dec *), const char *what)
{
	size_t used = 0;

	if (!d->cal_next)
		d->cal_next = CAL_SAMPLES;
	while (used < n) {
		size_t m = n - used, room = CAL_MAX - d->cal_len;
		int r;

		if (m > room)
			m = room;
		if (m > d->cal_next - d->cal_len)
			m = d->cal_next - d->cal_len;
		memcpy(d->cal + d->cal_len, s + used, m);
		d->cal_len += m;
		used += m;
		if (d->cal_len < d->cal_next)
			continue;
		r = cal(d);
		if (r == 0) {
			if (d->waiting && !d->o->quiet)
				INFO("%s found.\n", what);
			d->calibrated = 1;
			d->waiting = 0;
			d->cal_next = 0;
			return used;
		}
		if (!d->waiting) {
			INFO("waiting for %s ... (Ctrl-C to stop)\n", what);
			d->waiting = 1;
		}
		if (r == -2) {				/* nothing: drop it */
			d->cal_len = 0;
			d->cal_next = CAL_SAMPLES;
		} else if (d->cal_len >= CAL_MAX) {	/* keep the newer half */
			memmove(d->cal, d->cal + CAL_MAX / 2, CAL_MAX / 2);
			d->cal_len = CAL_MAX / 2;
			d->cal_next = CAL_MAX;
		} else {				/* slow: a bigger window */
			d->cal_next = d->cal_len * 2 < CAL_MAX ? d->cal_len * 2 : CAL_MAX;
		}
	}
	return used;
}

static int swo_feed(struct dec *d, const uint8_t *s, size_t n)
{
	if (!d->calibrated) {
		if (d->o->swo_baud > 0) {
			const double T = d->o->rate_mhz * 1e6 / d->o->swo_baud;
			if (d->o->swo_manch)
				d->m_havg = T / 2;	/* then followed per bit */
			else
				d->swo_T = T;
			if (!d->o->quiet)
				fprintf(stderr, "SWO %s: %.0f baud, %.1f samples per bit\n",
					d->o->swo_manch ? "Manchester" : "UART",
					d->o->swo_baud, T);
			d->calibrated = 1;
		} else {
			char what[64];
			size_t m;

			snprintf(what, sizeof(what), "SWO traffic on channel %d", d->o->swo_ch);
			m = cal_collect(d, s, n, swo_calibrate, what);
			if (!d->calibrated)
				return 0;
			swo_run(d, d->cal, d->cal_len);
			d->cal_len = 0;
			s += m;
			n -= m;
		}
	}
	swo_run(d, s, n);
	swo_flush(d);
	return 0;
}

/*
 * The clock stopped (target reset, trace port turned off ...): no edge for
 * this many samples. Then wait for it again and measure it again, as its
 * frequency may have changed.
 */
#define CLOCK_STOP_SAMPLES	(8u * 1024 * 1024)	/* 28 ms at 300 MS/s */

static int dec_feed(struct dec *d, const uint8_t *s, size_t n)
{
	char what[48];

	if (d->raw_f)
		fwrite(s, 1, n, d->raw_f);
	if (d->o->swo)
		return swo_feed(d, s, n);

	snprintf(what, sizeof(what), "TRACECLK on channel %d", d->clk);
	while (n) {
		if (d->calibrated) {
			/* in steps, so a stop is seen within a block */
			const size_t m = n < XFER_SIZE ? n : XFER_SIZE;
			const uint64_t e0 = d->edges;

			run_sampler(d, s, m);
			s += m;
			n -= m;
			if (d->edges != e0) {
				d->idle = 0;
				continue;
			}
			d->idle += m;
			if (d->idle < CLOCK_STOP_SAMPLES)
				continue;
			INFO("TRACECLK stopped; ");
			fast_drain(d);
			dec_discontinuity(d);
			d->calibrated = 0;
			d->idle = 0;
			d->cal_len = 0;
			d->cal_next = 0;
			continue;
		}
		{
			const size_t m = cal_collect(d, s, n, calibrate, what);
			s += m;
			n -= m;
			if (d->calibrated) {
				run_sampler(d, d->cal, d->cal_len);
				d->cal_len = 0;
			}
		}
	}
	return 0;
}

static void fast_drain(struct dec *d)
{
#ifdef CH32_X86
	if (d->calibrated && d->fast) {		/* the word still in stage A */
		int slot;

		pthread_mutex_lock(&V.mx);
		while (!V.nfree)
			pthread_cond_wait(&V.cv, &V.mx);
		d->vbuf = V.free_b[--V.nfree];
		pthread_mutex_unlock(&V.mx);
		d->nv = 0;
		fast_flush(d);
		pthread_mutex_lock(&V.mx);
		slot = (V.qhead + V.qlen) % NVB;
		V.q_b[slot] = d->vbuf;
		V.q_len[slot] = d->nv;
		V.q_reset[slot] = d->pending_reset;
		d->pending_reset = 0;
		V.qlen++;
		pthread_cond_broadcast(&V.cv);
		pthread_mutex_unlock(&V.mx);
	}
#endif
}

/* end of input: decode what is left, wait for the bits thread */
static int dec_finish(struct dec *d)
{
	int ret = 0;

	if (d->o->swo) {
		if (!d->calibrated && d->cal_len >= 1000 && swo_calibrate(d) == 0) {
			d->calibrated = 1;
			swo_run(d, d->cal, d->cal_len);
		}
		if (!d->calibrated && !d->swo_bytes) {
			fprintf(stderr, "No SWO traffic on channel %d to measure the "
				"baud rate; give it with -b.\n", d->o->swo_ch);
			ret = -1;
		}
		if (d->calibrated && d->o->swo_manch)
			manch_until(d, 1e30);
		swo_flush(d);
		pipe_stop();
		return ret;
	}

	if (!d->calibrated && d->cal_len >= 1000 && calibrate(d) == 0) {
		d->calibrated = 1;
		run_sampler(d, d->cal, d->cal_len);
	}
	if (!d->calibrated && !d->edges) {
		fprintf(stderr, "No clock found on channel %d.\n", d->clk);
		ret = -1;
	}
	fast_drain(d);
	pipe_stop();
	return ret;
}

static int dec_init(struct dec *d, const struct opts *o)
{
	int b, k;

	memset(d, 0, sizeof(*d));
	d->o = o;
	d->clk = o->clk_ch;
	d->itm_sel = (o->print_port >= 0 || o->check_port >= 0 || o->itm_out)
		? o->itm_id : -1;	/* ITM is decoded only when used */
	d->width = o->width;
	for (b = 0; b < 256; b++) {
		uint8_t v = 0;
		for (k = 0; k < o->width; k++)
			v |= ((b >> o->d_ch[k]) & 1) << k;
		d->nib_lut[b] = v;
	}
	for (k = 0; k < o->width; k++)
		d->dmask |= 1u << o->d_ch[k];
	d->cal = malloc(CAL_MAX);
	d->valbuf = malloc(XFER_SIZE + 64);
	d->f_first = -1;
	if (!d->cal || !d->valbuf || pipe_start(d) < 0)
		return -1;
	d->print_f = o->tpiu_out && !strcmp(o->tpiu_out, "-") ? stderr : stdout;
	return 0;
}

static void print_stats(const struct dec *d, double secs, uint64_t usb_bytes,
		uint64_t dropped, int final)
{
	const struct opts *o = d->o;
	double rate = o->rate_mhz * 1e6;

	if (g_silent || (o->quiet && !final))
		return;
	fprintf(stderr, "%s%6.1fs", final ? "\n== " : "", secs);
	if (usb_bytes)
		fprintf(stderr, " usb %.1f MB/s drop %" PRIu64 " B",
			secs > 0 ? usb_bytes / secs / 1e6 : 0.0, dropped);
	if (o->swo) {
		double baud = o->swo_manch ? (d->m_hn ? rate / (2 * d->m_hsum / d->m_hn) : 0)
				: (d->swo_T > 0 ? rate / d->swo_T : 0);
		fprintf(stderr, " | swo %" PRIu64 " B %.0f baud", d->swo_bytes, baud);
		if (o->swo_manch)
			fprintf(stderr, " packets %" PRIu64 " bad %" PRIu64,
				d->swo_packets, d->swo_perr);
		else
			fprintf(stderr, " framing err %" PRIu64 " false starts %" PRIu64,
				d->swo_ferr, d->swo_glitch);
		if (o->swo_fmt)
			fprintf(stderr, " | tpiu frames %" PRIu64 " ferr %" PRIu64,
				d->frames, d->frame_errs);
	} else {
		/* the clock now: edges since the last status line (the
		 * samples while waiting for a clock are not counted) */
		static uint64_t e0, s0;
		const uint64_t de = d->edges - e0, ds = d->samples - s0;

		if (!d->calibrated || (!final && (!ds || !de)))
			fprintf(stderr, " | clk %s", d->edges ? "stopped" : "none");
		else if (final)		/* the last measured clock */
			fprintf(stderr, " | clk %.2f MHz", rate / (2 * d->half) / 1e6);
		else
			fprintf(stderr, " | clk %.2f MHz", de / 2.0 / (ds / rate) / 1e6);
		e0 = d->edges;
		s0 = d->samples;
		fprintf(stderr, " | tpiu %" PRIu64 " B frames %" PRIu64
			" ferr %" PRIu64 " resync %" PRIu64 " gaps %" PRIu64,
			d->tpiu_bytes, d->frames, d->frame_errs, d->bit_resyncs,
			d->long_gaps);
	}
	if (o->check_port >= 0)
		fprintf(stderr, " | port%d n=%" PRIu64 " breaks=%" PRIu64
			" missing=%" PRIu64, o->check_port, d->chk_n,
			d->chk_err, d->chk_missing);
	fputc('\n', stderr);
	if (final) {
		int id, p;
		if (o->swo)
			fprintf(stderr, "   samples %" PRIu64 ", SWO edges %" PRIu64 "\n",
				d->samples, d->swo_edges);
		else
			fprintf(stderr, "   samples %" PRIu64 ", clock edges %" PRIu64
				", discontinuities %" PRIu64 "\n", d->samples, d->edges,
				d->discont);
		for (id = 1; id < 0x7f; id++)
			if (d->id_bytes[id])
				fprintf(stderr, "   TPIU stream %d: %" PRIu64 " bytes\n",
					id, d->id_bytes[id]);
		if (d->itm_sel >= 0)
			fprintf(stderr, "   ITM: syncs %" PRIu64 " overflows %" PRIu64
				" hw packets %" PRIu64 " reserved %" PRIu64 "\n",
				d->itm_sync, d->itm_ovf, d->itm_hw, d->itm_resv);
		for (p = 0; p < 32; p++)
			if (d->itm_sw[p])
				fprintf(stderr, "   ITM port %d: %" PRIu64 " packets\n",
					p, d->itm_sw[p]);
		if (o->check_port >= 0)
			fprintf(stderr, "   counter check port %d: %" PRIu64
				" values, %" PRIu64 " breaks, %" PRIu64
				" missing, %" PRIu64 " out-of-sequence, %" PRIu64
				" restarts\n", o->check_port, d->chk_n, d->chk_err,
				d->chk_missing, d->chk_bad, d->chk_back);
	}
}

static double now_s(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ------------------------------------------------------------ file input */

/* map a whole regular file read-only; NULL: read it instead */
#ifdef _WIN32
static const uint8_t *map_file(FILE *f, size_t *len)
{
	HANDLE fh = (HANDLE)_get_osfhandle(_fileno(f)), m;
	LARGE_INTEGER sz;
	void *v;

	if (fh == INVALID_HANDLE_VALUE || GetFileType(fh) != FILE_TYPE_DISK ||
	    !GetFileSizeEx(fh, &sz) || sz.QuadPart <= 0 ||
	    (uint64_t)sz.QuadPart > SIZE_MAX)
		return NULL;
	m = CreateFileMappingW(fh, NULL, PAGE_READONLY, 0, 0, NULL);
	if (!m)
		return NULL;
	v = MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0);
	CloseHandle(m);			/* the view keeps the mapping */
	*len = (size_t)sz.QuadPart;
	return v;
}

static void unmap_file(const uint8_t *p, size_t len)
{
	(void)len;
	UnmapViewOfFile(p);
}
#else
static const uint8_t *map_file(FILE *f, size_t *len)
{
	struct stat st;
	void *v;

	if (fstat(fileno(f), &st) || !S_ISREG(st.st_mode) || st.st_size <= 0)
		return NULL;
	v = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_PRIVATE, fileno(f), 0);
	if (v == MAP_FAILED)
		return NULL;
	madvise(v, (size_t)st.st_size, MADV_SEQUENTIAL);
	*len = (size_t)st.st_size;
	return v;
}

static void unmap_file(const uint8_t *p, size_t len)
{
	munmap((void *)p, len);
}
#endif

static int run_file(struct dec *d, const struct opts *o)
{
	FILE *f = strcmp(o->in_file, "-") ? fopen(o->in_file, "rb") : stdin;
	uint8_t *buf;
	size_t n, blk = XFER_SIZE;
	double t0 = now_s();
	int ret = 0;

	if (!f) {
		fprintf(stderr, "%s: %s\n", o->in_file, strerror(errno));
		return -1;
	}
	/* CH32TRACE_BLOCK: read size, to test block boundaries */
	if (getenv("CH32TRACE_BLOCK"))
		blk = (size_t)atol(getenv("CH32TRACE_BLOCK"));
	if (blk < 1 || blk > XFER_SIZE)
		blk = XFER_SIZE;
	if (f == stdin) {
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		/* a regular file: map it instead of copying it in */
		size_t len = 0;
		const uint8_t *map = map_file(f, &len);
		if (map) {
			size_t at;
			for (at = 0; !g_stop && at < len; at += n) {
				n = len - at < blk ? len - at : blk;
				if ((ret = dec_feed(d, map + at, n)) < 0)
					break;
			}
			if (dec_finish(d) < 0)
				ret = -1;
			print_stats(d, now_s() - t0, 0, 0, 1);
			unmap_file(map, len);
			fclose(f);
			return ret;
		}
	}
	buf = malloc(XFER_SIZE);
	{
		double tlast = t0;
		while (!g_stop && (n = fread(buf, 1, blk, f)) > 0) {
			const double t = now_s();
			if ((ret = dec_feed(d, buf, n)) < 0)
				break;
			if (t - tlast >= 1.0) {	/* a pipe: live status */
				print_stats(d, t - t0, 0, 0, 0);
				tlast = t;
			}
		}
	}
	if (dec_finish(d) < 0)
		ret = -1;
	print_stats(d, now_s() - t0, 0, 0, 1);
	free(buf);
	if (f != stdin)
		fclose(f);
	return ret;
}

/* ------------------------------------------------------------ USB input */

struct qent { uint8_t *buf; int len; int gap; };

static struct {
	pthread_mutex_t mx;
	pthread_cond_t cv;
	uint8_t *free_bufs[NPOOL];
	int nfree;
	struct qent q[NPOOL];
	int qhead, qtail, qlen;
	int pending_gap, running, done, inflight, xfer_err;
	uint64_t usb_bytes, dropped;
} U = { .mx = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

static void LIBUSB_CALL xfer_cb(struct libusb_transfer *x)
{
	int ok = x->status == LIBUSB_TRANSFER_COMPLETED ||
		 x->status == LIBUSB_TRANSFER_TIMED_OUT;

	if (!ok && x->status != LIBUSB_TRANSFER_CANCELLED) {
		fprintf(stderr, "USB transfer error (status %d)\n", x->status);
		U.xfer_err = 1;
		g_stop = 1;
	}
	pthread_mutex_lock(&U.mx);
	if (ok && x->actual_length > 0) {
		U.usb_bytes += x->actual_length;
		if (U.nfree > 0) {
			struct qent *e = &U.q[U.qtail];
			e->buf = x->buffer;
			e->len = x->actual_length;
			e->gap = U.pending_gap;
			U.pending_gap = 0;
			U.qtail = (U.qtail + 1) % NPOOL;
			U.qlen++;
			x->buffer = U.free_bufs[--U.nfree];
			pthread_cond_signal(&U.cv);
		} else {
			/* the decoder is behind: lose this block, mark a gap */
			U.dropped += x->actual_length;
			U.pending_gap = 1;
		}
	}
	pthread_mutex_unlock(&U.mx);

	if (ok && U.running && !g_stop && libusb_submit_transfer(x) == 0)
		return;
	U.inflight--;
}

static void *decoder_thread(void *arg)
{
	struct dec *d = arg;

	for (;;) {
		struct qent e;

		pthread_mutex_lock(&U.mx);
		while (!U.qlen && !U.done)
			pthread_cond_wait(&U.cv, &U.mx);
		if (!U.qlen && U.done) {
			pthread_mutex_unlock(&U.mx);
			break;
		}
		e = U.q[U.qhead];
		U.qhead = (U.qhead + 1) % NPOOL;
		U.qlen--;
		pthread_mutex_unlock(&U.mx);

		if (e.gap && d->calibrated)
			dec_discontinuity(d);
		if (dec_feed(d, e.buf, e.len) < 0)
			g_stop = 1;

		pthread_mutex_lock(&U.mx);
		U.free_bufs[U.nfree++] = e.buf;
		pthread_mutex_unlock(&U.mx);
	}
	return NULL;
}

static int cmd(libusb_device_handle *h, uint8_t c, const uint8_t *p,
		uint8_t plen, uint8_t *rsp, int rsp_size)
{
	uint8_t buf[8] = { c, plen }, rb[64];
	int xfer, ret;

	if (plen)
		memcpy(buf + 2, p, plen);
	ret = libusb_bulk_transfer(h, EP_CMD_OUT, buf, 2 + plen, &xfer, 1000);
	if (ret) {
		fprintf(stderr, "command 0x%02x: %s\n", c, libusb_error_name(ret));
		return -1;
	}
	if (c == CMD_SET_START || c == CMD_SET_STOP)
		return 0;
	memset(rb, 0, sizeof(rb));
	ret = libusb_bulk_transfer(h, EP_CMD_IN, rb, sizeof(rb), &xfer, 1000);
	if (ret || xfer < 3 || rb[0] != (c | CMD_RETURN_PARA) || rb[2]) {
		fprintf(stderr, "command 0x%02x: bad reply\n", c);
		return -1;
	}
	if (rsp)
		memcpy(rsp, rb, rsp_size < xfer ? rsp_size : xfer);
	return 0;
}

/*
 * The capture inputs (UHSIF pins PC1-PC3, PB0-PB1, PE10-PE12, ...) are in
 * the chip's VIO18 bank. They have no comparators: they switch at roughly
 * VIO18 / 2, so the "threshold" is set by setting VIO18, and VIO18 should
 * equal the target's I/O voltage. A signal above VIO18 drives the input
 * protection diodes and back-feeds the rail, so VIO18 must never be below
 * the target's signal swing.
 *
 * CMD_SET_LOGIC_LEVEL (0xA1) carries the DAC code for the Q2H2 board's
 * external VIO18 regulator (vth_lut.h: measured VIO18 volts -> code). The
 * nanoCH32H417 build of OpenSourceLogic-CH32H417 (BOARD_NANO_CH32H417)
 * turns the same code back into a voltage and sets the internal VIO18
 * regulator to the NEAREST of 1.2 / 1.8 / 2.5 / 3.3 V. So the request is
 * rounded UP to one of those levels here first: then both boards give the
 * same rail, and the nano never rounds down below the target swing.
 */
static const double vio_levels[4] = { 1.2, 1.8, 2.5, 3.3 };

static uint16_t vio_to_dac(double v)
{
	int mv;

	if (v >= 3.44)
		return 0;
	mv = (int)(v * 1000.0 + 0.5);
	if (mv < 0)
		mv = 0;
	if (mv > CH32_VTH_LUT_MV_MAX)
		mv = CH32_VTH_LUT_MV_MAX;
	return ch32_vth_mv_to_dac[mv];
}

static double vio_level(double v)
{
	int k;

	for (k = 0; k < 3 && vio_levels[k] < v - 0.05; k++)
		;
	return vio_levels[k];
}

static int set_vio18(libusb_device_handle *h, double want)
{
	double v = vio_level(want);
	uint16_t dac = vio_to_dac(v);
	uint8_t p[2] = { (uint8_t)(dac >> 8), (uint8_t)dac };

	if (fabs(v - want) > 0.05)
		fprintf(stderr, "VIO18 levels are 1.2 / 1.8 / 2.5 / 3.3 V: "
			"using %.1f V for -V %.2f\n", v, want);
	if (cmd(h, CMD_SET_LOGIC_LEVEL, p, 2, NULL, 0))
		return -1;
	INFO("VIO18 %.1f V (DAC %u): inputs switch at about %.2f V; "
		"keep target signals at or below %.1f V\n", v, dac, v / 2, v);
	return 0;
}

static int run_device(struct dec *d, const struct opts *o)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *h;
	struct libusb_transfer *xf[NXFER] = { 0 };
	pthread_t th;
	uint8_t rsp[8] = { 0 }, p[2];
	int usb3 = 1, i, sys = -1, ret = -1;
	unsigned xfer_size = XFER_SIZE;
	double t0, tlast;

	if ((i = libusb_init(&ctx)) != 0) {
		fprintf(stderr, "libusb_init: %s\n", libusb_error_name(i));
		return -1;
	}
	h = libusb_open_device_with_vid_pid(ctx, CH32_VID, CH32_PID_USB3);
	if (!h) {
		h = libusb_open_device_with_vid_pid(ctx, CH32_VID, CH32_PID_USB2);
		usb3 = 0;
	}
	if (!h) {
		fprintf(stderr, "No CH32H417 analyzer (1a86:5538 or 1a86:5537) "
			"found, or no permission to open it.\n");
		goto out_ctx;
	}
	libusb_set_auto_detach_kernel_driver(h, 1);
	if (libusb_claim_interface(h, 0)) {
		fprintf(stderr, "Cannot claim the USB interface.\n");
		goto out_h;
	}
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x0100010C)
	/* Windows / WinUSB: keep several reads in flight (as the sigrok driver) */
	if (libusb_endpoint_set_raw_io(h, EP_LOGIC_IN, 1) == LIBUSB_SUCCESS) {
		int max = libusb_get_max_raw_io_transfer_size(h, EP_LOGIC_IN);
		if (max > 0 && (unsigned)max < xfer_size)
			xfer_size = (unsigned)max & ~16383u;
	}
#endif

	/* stop a capture left running, drop what it already queued */
	cmd(h, CMD_SET_STOP, NULL, 0, NULL, 0);
	{
		uint8_t *junk = malloc(XFER_SIZE);
		int got, k;
		for (k = 0; junk && k < 64; k++)
			if (libusb_bulk_transfer(h, EP_LOGIC_IN, junk, XFER_SIZE,
					&got, 50) != 0 && got == 0)
				break;
		free(junk);
	}
	if (cmd(h, CMD_GET_VERSION, NULL, 0, rsp, sizeof(rsp)))
		goto out_if;
	INFO("analyzer: firmware %u, hardware %u, %s\n", rsp[3],
		rsp[4], usb3 ? "USB 3" : "USB 2");

	for (i = 0; i < (int)(sizeof(rates) / sizeof(rates[0])); i++)
		if (rates[i].mhz == o->rate_mhz)
			sys = i;
	if (sys < 0) {
		fprintf(stderr, "Rate %u MS/s not supported.\n", o->rate_mhz);
		goto out_if;
	}
	if (!usb3) {
		fprintf(stderr, "USB 2 link: the firmware samples at a fixed 40 MS/s. "
			"Use a USB 3 port for trace.\n");
		goto out_if;
	}
	if (rates[sys].fast8 && rsp[3] < FW_FAST8_VERSION) {
		fprintf(stderr, "%u MS/s needs firmware %d or later.\n",
			o->rate_mhz, FW_FAST8_VERSION);
		goto out_if;
	}

	p[0] = 8;
	p[1] = rates[sys].sys;
	if (cmd(h, CMD_SET_LOGIC_PARA, p, 2, NULL, 0))
		goto out_if;
	if (set_vio18(h, o->vio))
		goto out_if;

	for (i = 0; i < NPOOL; i++) {
		U.free_bufs[i] = malloc(XFER_SIZE);
		if (!U.free_bufs[i]) {
			fprintf(stderr, "out of memory\n");
			goto out_bufs;
		}
		U.nfree++;
	}
	pthread_create(&th, NULL, decoder_thread, d);

	U.running = 1;
	for (i = 0; i < NXFER; i++) {
		xf[i] = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xf[i], h, EP_LOGIC_IN,
			U.free_bufs[--U.nfree], (int)xfer_size, xfer_cb, NULL, 500);
		if (libusb_submit_transfer(xf[i]) == 0)
			U.inflight++;
	}
	if (cmd(h, CMD_SET_START, NULL, 0, NULL, 0) == 0) {
		INFO("capturing at %u MS/s, Ctrl-C to stop\n", o->rate_mhz);
		ret = 0;
	}

	t0 = tlast = now_s();
	while (!g_stop && ret == 0) {
		struct timeval tv = { 0, 100000 };
		double t;

		libusb_handle_events_timeout(ctx, &tv);
		t = now_s();
		if (o->seconds > 0 && t - t0 >= o->seconds)
			break;
		if (t - tlast >= 1.0) {
			tlast = t;
			pthread_mutex_lock(&U.mx);
			print_stats(d, t - t0, U.usb_bytes, U.dropped, 0);
			pthread_mutex_unlock(&U.mx);
		}
	}

	U.running = 0;
	cmd(h, CMD_SET_STOP, NULL, 0, NULL, 0);
	for (i = 0; i < NXFER; i++)
		if (xf[i])
			libusb_cancel_transfer(xf[i]);
	while (U.inflight > 0) {
		struct timeval tv = { 0, 100000 };
		libusb_handle_events_timeout(ctx, &tv);
	}

	pthread_mutex_lock(&U.mx);
	U.done = 1;
	pthread_cond_signal(&U.cv);
	pthread_mutex_unlock(&U.mx);
	pthread_join(th, NULL);
	if (dec_finish(d) < 0)
		ret = -1;
	print_stats(d, now_s() - t0, U.usb_bytes, U.dropped, 1);
	if (U.dropped)
		fprintf(stderr, "WARNING: %" PRIu64 " bytes lost on the host "
			"(decoder too slow). Device-side losses show up as "
			"counter gaps and clock gaps instead.\n", U.dropped);

	for (i = 0; i < NXFER; i++)
		if (xf[i]) {
			U.free_bufs[U.nfree++] = xf[i]->buffer;
			libusb_free_transfer(xf[i]);
		}
out_bufs:
	for (i = 0; i < U.nfree; i++)
		free(U.free_bufs[i]);
out_if:
	libusb_release_interface(h, 0);
out_h:
	libusb_close(h);
out_ctx:
	libusb_exit(ctx);
	return ret;
}

/* ------------------------------------------------------------ speed test */

/*
 * -B: decode speed on this PC, no device or files needed. Makes a 4-bit
 * TPIU trace in memory (TRACECLK 50 MHz sampled at 300 MS/s, with clock
 * jitter and data skew, ITM counter on port 1, continuous traffic: the
 * worst case), then decodes it with every code path this CPU has and
 * checks the counter.
 */
struct bgen { uint8_t *b; size_t n, cap; };

static void bput(struct bgen *g, uint8_t x)
{
	if (g->n == g->cap) {
		g->cap = g->cap ? 2 * g->cap : 1 << 20;
		g->b = realloc(g->b, g->cap);
		if (!g->b) {
			fprintf(stderr, "out of memory\n");
			exit(1);
		}
	}
	g->b[g->n++] = x;
}

static uint32_t brand(uint64_t *st)	/* xorshift */
{
	*st ^= *st << 13;
	*st ^= *st >> 7;
	*st ^= *st << 17;
	return (uint32_t)(*st >> 32);
}

static uint8_t *bench_make(size_t want, size_t *n_out, uint32_t *values)
{
	struct bgen itm = { 0 }, tp = { 0 };
	const double H = 3.0;			/* samples per half period */
	const size_t nvals = (size_t)(want / H) + 64;
	size_t k, p = 0, i, nf = 0;
	uint32_t v = 0;
	uint64_t st = 0x9e3779b97f4a7c15ull;
	uint8_t *s;
	int cur = -1;

	/* ITM: 32-bit writes to port 1, a sync every 500 */
	while (itm.n * 2 < nvals) {
		if (v % 500 == 0) {
			for (k = 0; k < 5; k++)
				bput(&itm, 0);
			bput(&itm, 0x80);
		}
		bput(&itm, (1 << 3) | 3);
		for (k = 0; k < 4; k++)
			bput(&itm, (uint8_t)(v >> (8 * k)));
		v++;
	}
	*values = v;

	/* TPIU: stream ID 1 in 16-byte frames, full syncs between them */
	for (k = 0; k < 16; k++) {
		bput(&tp, 0xff); bput(&tp, 0xff); bput(&tp, 0xff); bput(&tp, 0x7f);
	}
	while (p + 16 <= itm.n) {
		uint8_t f[16] = { 0 }, aux = 0;
		int sl;
		for (sl = 0; sl < 8; sl++) {
			const uint8_t b0 = itm.b[p];
			if (sl == 7) {
				if (cur == 1) {
					f[14] = b0 & 0xfe;
					aux |= (uint8_t)((b0 & 1) << 7);
					p++;
				} else {
					f[14] = (1 << 1) | 1;
					cur = 1;
				}
			} else if (cur != 1) {
				f[0] = (1 << 1) | 1;	/* new ID, applies to f[1] */
				f[1] = b0;
				cur = 1;
				p++;
			} else {
				f[2 * sl] = b0 & 0xfe;
				aux |= (uint8_t)((b0 & 1) << sl);
				f[2 * sl + 1] = itm.b[p + 1];
				p += 2;
			}
		}
		f[15] = aux;
		for (k = 0; k < 16; k++)
			bput(&tp, f[k]);
		if (++nf % 64 == 0) {
			bput(&tp, 0xff); bput(&tp, 0xff); bput(&tp, 0xff); bput(&tp, 0x7f);
		}
	}
	free(itm.b);

	/* samples: D0 = TRACECLK, D1..D4 = TRACEDATA, D7 high */
	{
		const size_t nnib = tp.n * 2, ns = (size_t)(50 + nnib * H + 8);
		double t = 50.0;
		size_t e = 0, j = 0;
		unsigned clk = 0, data = 0;
		double next_edge, next_launch;

		s = malloc(ns);
		if (!s) {
			fprintf(stderr, "out of memory\n");
			exit(1);
		}
		next_edge = t + (brand(&st) / 4294967296.0 - 0.5) * 0.3;
		next_launch = next_edge + brand(&st) / 4294967296.0 * 0.5;
		for (i = 0; i < ns; i++) {
			while (e < nnib && next_edge <= (double)i) {	/* clock edge */
				clk ^= 1;
				e++;
				t += H;
				next_edge = t + (brand(&st) / 4294967296.0 - 0.5) * 0.3;
			}
			while (j < e && next_launch <= (double)i) {	/* data change */
				data = (tp.b[j >> 1] >> (4 * (j & 1))) & 15;
				j++;
				next_launch = (50.0 + (double)j * H) +
					brand(&st) / 4294967296.0 * 0.5;
			}
			s[i] = (uint8_t)(0x80 | clk | data << 1);
		}
		*n_out = ns;
	}
	free(tp.b);
	return s;
}

static void dec_free(struct dec *d)
{
	free(d->cal);
	free(d->valbuf);
	free(d->pb);
	free(d->ib);
	free(d->swo_buf);
}

static int run_bench(void)
{
	static const char *paths[] = { "fast", "avx2", "sse2", "scalar" };
	const size_t want = 128u << 20;
	size_t n, at, k;
	uint32_t nvals;
	double t0 = now_s(), tg;
	uint8_t *s = bench_make(want, &n, &nvals);
	const char *last = "";
	int itm;

	tg = now_s() - t0;
	fprintf(stderr, "speed test: 4-bit TPIU, TRACECLK 50 MHz sampled at 300 MS/s, "
		"busy trace, %.0f M samples (%.2f s of capture; made in %.1f s)\n",
		n / 1e6, n / 300e6, tg);
#ifdef CH32_X86
	__builtin_cpu_init();
	fprintf(stderr, "CPU: avx2 %s, bmi2 %s\n",
		__builtin_cpu_supports("avx2") ? "yes" : "no",
		__builtin_cpu_supports("bmi2") ? "yes" : "no");
#endif
	fprintf(stderr, "\n%-22s %-8s %8s %10s   %s\n", "edge finder", "ITM",
		"MS/s", "realtime", "check");

	for (k = 0; k < sizeof(paths) / sizeof(paths[0]); k++) {
		for (itm = 1; itm >= 0; itm--) {
			struct opts o = {
				.rate_mhz = 300, .clk_ch = 0, .d_ch = { 1, 2, 3, 4 },
				.width = 4, .phase = INT32_MIN, .vio = 3.3, .itm_id = 1,
				.print_port = -1, .check_port = itm ? 1 : -1, .quiet = 1,
			};
			struct dec *d = calloc(1, sizeof(*d));
			double t, dt;
			char res[96];

			g_isa = paths[k];
			if (!d || dec_init(d, &o) < 0) {
				fprintf(stderr, "out of memory\n");
				return 1;
			}
			t = now_s();
			for (at = 0; at < n; at += XFER_SIZE)
				dec_feed(d, s + at, n - at < XFER_SIZE ? n - at : XFER_SIZE);
			dec_finish(d);
			dt = now_s() - t;
			if (!d->isa || !strcmp(d->isa, last)) {	/* path not on this CPU */
				dec_free(d);
				free(d);
				break;
			}
			if (itm)
				snprintf(res, sizeof(res), "%s: %" PRIu64 " of %u values, "
					"%" PRIu64 " breaks", d->chk_err == 0 &&
					d->chk_n + 16 >= nvals ? "ok" : "FAILED",
					d->chk_n, nvals, d->chk_err);
			else
				snprintf(res, sizeof(res), "%" PRIu64 " frames, %" PRIu64
					" errors", d->frames, d->frame_errs);
			fprintf(stderr, "%-22s %-8s %8.0f %9.1fx   %s\n", d->isa,
				itm ? "decoded" : "off", n / dt / 1e6, n / dt / 300e6, res);
			if (!itm)
				last = d->isa;
			dec_free(d);
			free(d);
		}
	}
	fprintf(stderr, "\nrealtime = speed / 300 MS/s; above 1x keeps up with the "
		"analyzer.\n'ITM off' is the -o / -S case, 'decoded' adds -P / -C / -w.\n");
	free(s);
	g_isa = NULL;
	return 0;
}

/* ------------------------------------------------------------ main */

static void usage(const char *p)
{
	fprintf(stderr,
"usage: %s [options]\n"
"  -i FILE       read a raw capture (1 byte/sample, bit k = channel k) instead\n"
"                of the device; '-' = stdin\n"
"  -r MHZ        sample rate in MS/s (default 300; 300/275/250/225 need fw >= 3)\n"
"  -c CH         TRACECLK channel (default 0)\n"
"  -d CH[,CH..]  TRACEDATA channels, TRACEDATA0 first (default 1,2,3,4);\n"
"                1, 2 or 4 channels = trace port width\n"
"  -p N          take the data N samples after each clock edge, or with -N,\n"
"                N samples before the next edge (default: automatic)\n"
"  -V VOLT       VIO18, the supply of the capture pins = the target's I/O\n"
"                voltage (default 3.3; 1.8 for 1.8 V targets). The inputs\n"
"                switch at about VOLT/2. Never set it below the target swing.\n"
"  -o FILE       aligned TPIU byte stream (for: orbuculum -f FILE -T -t 1);\n"
"                a FIFO works (not on Windows), '-' = stdout\n"
"  -S PORT       serve the TPIU stream on TCP PORT, for\n"
"                orbuculum -s localhost:PORT -T -t 1 (Windows and Linux)\n"
"  -A            with -S: listen on all interfaces (default: localhost only)\n"
"  -I ID         TPIU stream ID that carries ITM (default 1)\n"
"  -w FILE       write the deframed ITM stream bytes to FILE\n"
"  -P PORT       print ITM stimulus port PORT as text on stdout\n"
"  -C PORT       check that ITM port PORT carries a +1 counter (32-bit writes)\n"
"  -R FILE       also save the raw samples (needs a disk that takes 300 MB/s)\n"
"  -t SEC        stop after SEC seconds (default: Ctrl-C)\n"
"  -q            quiet: no status line every second, only the final summary\n"
"  -s            silent: no status line, no summary, no info; errors and\n"
"                warnings only\n"
"  -B            speed test: decode a generated trace with each code path\n"
"SWO instead of parallel trace:\n"
"  -W CH         SWO is on channel CH\n"
"  -M MODE       uart (NRZ, default) or manchester\n"
"  -b BAUD       UART bit rate (default: measured)\n"
"  -F            the SWO stream has TPIU frames (formatter on, TPI->FFCR bit 1)\n"
"  -o / -S / -P / -C / -w / -R work as for parallel trace. For Orbuculum:\n"
"                orbuculum -s localhost:PORT     (add -T -t 1 with -F)\n", p);
}

static FILE *open_out(const char *path)
{
	FILE *f;

	if (!strcmp(path, "-")) {
		f = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdout), _O_BINARY);	/* no \n -> \r\n */
#endif
	} else if (!(f = fopen(path, "wb"))) {
		fprintf(stderr, "%s: %s\n", path, strerror(errno));
		exit(1);
	}
	setvbuf(f, NULL, _IOFBF, 4 << 20);
	return f;
}

int main(int argc, char **argv)
{
	struct opts o = {
		.rate_mhz = 300, .clk_ch = 0, .d_ch = { 1, 2, 3, 4 }, .width = 4,
		.phase = INT32_MIN, .vio = 3.3, .itm_id = 1, .print_port = -1,
		.check_port = -1,
	};
	static struct dec d;
	int c, k, ret;

	while ((c = getopt(argc, argv, "i:r:c:d:p:V:o:S:AI:w:P:C:R:t:qsBW:M:b:Fh")) != -1) {
		switch (c) {
		case 'i': o.in_file = optarg; break;
		case 'r': o.rate_mhz = (uint32_t)atoi(optarg); break;
		case 'c': o.clk_ch = atoi(optarg); break;
		case 'd': {
			char *s = optarg, *e;
			o.width = 0;
			while (*s && o.width < 5) {
				long v = strtol(s, &e, 10);
				if (e == s)
					break;
				if (o.width < 4)
					o.d_ch[o.width] = (int)v;
				o.width++;
				s = *e == ',' ? e + 1 : e;
			}
			break;
		}
		case 'p': o.phase = atoi(optarg); break;
		case 'V': o.vio = atof(optarg); break;
		case 'o': o.tpiu_out = optarg; break;
		case 'S': o.tcp_port = atoi(optarg); break;
		case 'A': o.tcp_any = 1; break;
		case 'I': o.itm_id = atoi(optarg); break;
		case 'w': o.itm_out = optarg; break;
		case 'P': o.print_port = atoi(optarg); break;
		case 'C': o.check_port = atoi(optarg); break;
		case 'R': o.raw_out = optarg; break;
		case 't': o.seconds = atof(optarg); break;
		case 'q': o.quiet = 1; break;
		case 's': o.quiet = 1; g_silent = 1; break;
		case 'B': return run_bench();
		case 'W': o.swo = 1; o.swo_ch = atoi(optarg); break;
		case 'M':
			if (!strncmp(optarg, "man", 3))
				o.swo_manch = 1;
			else if (!strncmp(optarg, "uart", 4) || !strncmp(optarg, "nrz", 3))
				o.swo_manch = 0;
			else {
				fprintf(stderr, "-M: uart or manchester\n");
				return 1;
			}
			break;
		case 'b': o.swo_baud = atof(optarg); break;
		case 'F': o.swo_fmt = 1; break;
		default: usage(argv[0]); return 1;
		}
	}
	if (o.width != 1 && o.width != 2 && o.width != 4) {
		fprintf(stderr, "-d needs 1, 2 or 4 channels.\n");
		return 1;
	}
	if (o.swo && (o.swo_ch < 0 || o.swo_ch > 7)) {
		fprintf(stderr, "-W: channel 0..7\n");
		return 1;
	}
	if (o.clk_ch < 0 || o.clk_ch > 7) {
		fprintf(stderr, "channels must be 0..7 (8-channel mode).\n");
		return 1;
	}
	for (k = 0; k < o.width; k++)
		if (o.d_ch[k] < 0 || o.d_ch[k] > 7 || o.d_ch[k] == o.clk_ch) {
			fprintf(stderr, "channels must be 0..7 and differ from the clock.\n");
			return 1;
		}
	if (o.vio < 1.1 || o.vio > 3.45) {
		fprintf(stderr, "-V: VIO18 must be 1.1 .. 3.4 V (chip rating 1.1 .. 3.6 V).\n");
		return 1;
	}
	if (o.itm_id < 1 || o.itm_id > 126 || o.print_port > 31 || o.check_port > 31) {
		fprintf(stderr, "bad -I / -P / -C value.\n");
		return 1;
	}

	if (dec_init(&d, &o) < 0) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}
	if (o.tpiu_out)
		d.tpiu_f = open_out(o.tpiu_out);
	if (o.itm_out)
		d.itm_f = open_out(o.itm_out);
	if (o.raw_out)
		d.raw_f = open_out(o.raw_out);

	signal(SIGINT, on_sigint);
#ifdef SIGPIPE
	signal(SIGPIPE, SIG_IGN);
#endif
	if (o.tcp_port && net_start(o.tcp_port, o.tcp_any) < 0)
		return 1;

	ret = o.in_file ? run_file(&d, &o) : run_device(&d, &o);

	if (d.tpiu_f) fclose(d.tpiu_f);
	if (d.itm_f) fclose(d.itm_f);
	if (d.raw_f) fclose(d.raw_f);
	return ret < 0 ? 1 : 0;
}
