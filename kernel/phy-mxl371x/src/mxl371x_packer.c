// SPDX-License-Identifier: GPL-2.0+
/*
 * MoCA boot-config payload packer (see mxl371x_packer.h).
 *
 * Each build function walks its config blob in a fixed field order and packs
 * each byte big-endian into the output word stream (byte i -> word[i/4] at
 * shift (3 - (i & 3)) * 8), producing the boot-config payload the SoC expects
 * for that command.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include "mxl371x_packer.h"

/* Minimum blob lengths: the highest byte offset each builder reads, + 1. */
#define MXL_CLINK_MIN		0x370
#define MXL_MCAST_MIN		0x184
#define MXL_PLATFORM_MIN	0x34
#define MXL_RLAPM_MIN		0x214
#define MXL_ENDET_MIN		0x238
#define MXL_SAPM_MIN		0x434
#define MXL_RSSI_MIN		0x34

struct packer {
	u32 *buf;	/* output words */
	u32 cap;	/* capacity in words */
	u32 bi;		/* current byte index */
	bool overflow;
	const struct mxl371x_cfg_blobs *b;
};

/* pack one byte: byte i -> word[i/4] at shift (3 - (i & 3)) * 8 (BE within word). */
static void pk_p8(struct packer *pk, u8 v)
{
	u32 w = pk->bi >> 2, sh = (~pk->bi & 3u) << 3;

	if (w >= pk->cap) {
		pk->overflow = true;
		return;
	}
	pk->buf[w] = (pk->buf[w] & ~(0xffu << sh)) | ((u32)v << sh);
	pk->bi++;
}

/* per-blob byte accessors / multi-byte helpers, mirroring the host packer */
#define P8(v)	 pk_p8(pk, (v))
#define PZ()	 pk_p8(pk, 0)
/* clink (idx 1) */
#define C(o)	 pk_p8(pk, pk->b->clink.data[o])
#define CSWAP(o) do { pk_p8(pk, pk->b->clink.data[(o)+1]); \
		      pk_p8(pk, pk->b->clink.data[o]); } while (0)
#define CREV8(o) do { int _k; for (_k = 7; _k >= 0; _k--) \
		      pk_p8(pk, pk->b->clink.data[(o)+_k]); } while (0)
/* mcast (idx 2) */
#define M(o)	 pk_p8(pk, pk->b->mcast.data[o])
#define MREV4(o) do { int _k; for (_k = 3; _k >= 0; _k--) \
		      pk_p8(pk, pk->b->mcast.data[(o)+_k]); } while (0)
/* platform (idx 7) */
#define Q(o)	 pk_p8(pk, pk->b->platform.data[o])
#define QSWAP(o) do { pk_p8(pk, pk->b->platform.data[(o)+1]); \
		      pk_p8(pk, pk->b->platform.data[o]); } while (0)
/* rlapm (idx 3) */
#define L(o)	 pk_p8(pk, pk->b->rlapm.data[o])
#define LSWAP(o) do { pk_p8(pk, pk->b->rlapm.data[(o)+1]); \
		      pk_p8(pk, pk->b->rlapm.data[o]); } while (0)
/* endet (idx 4) */
#define E(o)	 pk_p8(pk, pk->b->endet.data[o])
#define ESWAP(o) do { pk_p8(pk, pk->b->endet.data[(o)+1]); \
		      pk_p8(pk, pk->b->endet.data[o]); } while (0)
#define EREV4(o) do { int _k; for (_k = 3; _k >= 0; _k--) \
		      pk_p8(pk, pk->b->endet.data[(o)+_k]); } while (0)
/* sapm (idx 5) */
#define S(o)	 pk_p8(pk, pk->b->sapm.data[o])
#define SSWAP(o) do { pk_p8(pk, pk->b->sapm.data[(o)+1]); \
		      pk_p8(pk, pk->b->sapm.data[o]); } while (0)
/* rssi (idx 6) */
#define R(o)	 pk_p8(pk, pk->b->rssi.data[o])

/* ---- InitParamV2 (cmd 0x1010004): clink(1) + mcast(2) + platform(7) ---- */
static void build_v2(struct packer *pk, u32 guid_hi, u32 guid_lo)
{
	int b, i, o;

	pk->bi = 0;
	C(0); C(1); C(2); C(3);
	C(7); C(6); CSWAP(4);
	C(0xb); C(0xa); CSWAP(8);
	C(0xf); C(0xe); CSWAP(0xc);
	C(0x13); C(0x12); CSWAP(0x10);
	/* GUID inline: eMacAddrHi/Lo packed big-endian (word 5/6) */
	P8(guid_hi >> 24); P8(guid_hi >> 16); P8(guid_hi >> 8); P8(guid_hi);
	P8(guid_lo >> 24); P8(guid_lo >> 16); P8(guid_lo >> 8); P8(guid_lo);
	C(0x263); C(0x262); CSWAP(0x260);
	C(0x265); C(0x264); C(0x267); C(0x266);
	C(0x36c); C(0x36d); C(0x36e); C(0x36f);
	C(0x15); C(0x14); C(0x16); C(0x17);
	C(0x1b); C(0x1a); CSWAP(0x18);
	C(0x1f); C(0x1e); CSWAP(0x1c);
	C(0x23); C(0x22); CSWAP(0x20);
	P8(0);				/* devBusType */
	M(0x180);
	PZ(); PZ();
	for (o = 0; o < 0x180; o += 4) MREV4(o);
	Q(0x2f); Q(0x2e); QSWAP(0x2c);
	C(0x25f); C(0x25e); CSWAP(0x25c);
	C(0x27f); C(0x27e); CSWAP(0x27c);
	P8(pk->b->clink.data[0x24] & 0xfc);
	C(0x25); C(0x26); C(0x27);
	for (b = 0x28; b < 0xb8; b += 0x12) {
		for (i = 0; i < 0x11; i++)
			C(b + i);
		PZ(); PZ(); PZ();
	}
	for (o = 0xb8; o < 0xc8; o += 2) CSWAP(o);
	C(0xc9); C(0xc8); C(0xca); C(0xcb);
	for (o = 0xd0; o < 0x110; o += 8) CREV8(o);
	for (i = 0; i < 8; i++) C(0x110 + i);
	for (o = 0x118; o < 0x158; o += 8) CREV8(o);
	for (o = 0x158; o < 0x198; o += 8) CREV8(o);
	for (o = 0x198; o < 0x1d8; o += 8) CREV8(o);
	for (o = 0x1d8; o < 0x218; o += 8) CREV8(o);
	C(0x21b); C(0x21a); CSWAP(0x218);
	for (i = 0; i < 8; i++) C(0x21c + i);
	for (i = 0; i < 8; i++) C(0x224 + i);
	for (i = 0; i < 8; i++) C(0x268 + i);
	for (i = 0; i < 8; i++) C(0x270 + i);
	C(0x22c); C(0x22d);
	PZ(); PZ();
	C(0x233); C(0x232); CSWAP(0x230);
	C(0x237); C(0x236); CSWAP(0x234);
	C(0x23b); C(0x23a); CSWAP(0x238);
	for (i = 0; i < 8; i++) C(0x23c + i);
	for (i = 0; i < 8; i++) C(0x244 + i);
	for (i = 0; i < 8; i++) C(0x24c + i);
	C(0x255); C(0x254); C(0x257); C(0x256);
	C(0x259); C(0x258); C(0x25b); C(0x25a);
	for (i = 0; i < 16; i++) PZ();
	Q(0x2b); Q(0x2a); QSWAP(0x28);
	Q(0x33); Q(0x32); QSWAP(0x30);
	C(0x27b); C(0x27a); CSWAP(0x278);
}

/* one endet interleave block centred at byte offset @cur, half-span @hs bytes */
static void eblock(struct packer *pk, int cur, int hs)
{
	int p;

	for (p = cur - hs; p < cur; p += 2) ESWAP(p);
	for (p = cur; p < cur + hs; p += 2) ESWAP(p);
}

/* ---- RlapmV2 (cmd 0x1010005): rlapm(3) + endet(4) ---- */
static void build_rlapm(struct packer *pk)
{
	int b, i, o, cur;

	pk->bi = 0;
	L(0);
	PZ(); PZ(); PZ();
	for (o = 4; o < 0x214; o += 0x58) LSWAP(o);
	for (b = 8; b < 0x218; b += 0x58)
		for (i = 0; i < 0x54; i++)
			L(b + i);
	/* endet: 8 blocks, stride 0x28, full 0x10-half-span interleave */
	for (cur = 0x14; cur < 0x154; cur += 0x28) {
		E(cur - 0x13); E(cur - 0x14); E(cur - 0x11); E(cur - 0x12);
		eblock(pk, cur, 0x10);
		E(cur + 0x11); E(cur + 0x10); E(cur + 0x13); E(cur + 0x12);
	}
	for (i = 0; i < 6; i++) E(0x140 + i);
	E(0x147); E(0x146); E(0x149); E(0x148); E(0x14b); E(0x14a);
	for (i = 0; i < 4; i++) E(0x14c + i);
	EREV4(0x150);
	E(0x154); E(0x155); E(0x156); E(0x157); E(0x158); E(0x159);
	E(0x15b); E(0x15a); E(0x15d); E(0x15c); E(0x15f); E(0x15e);
	for (i = 0; i < 4; i++) E(0x160 + i);
	EREV4(0x164);
	/* endet: 6 blocks, stride 0x14, 0xa-half-span interleave */
	for (cur = 0x172; cur < 0x1ea; cur += 0x14)
		eblock(pk, cur, 0xa);
	E(0x1e1); E(0x1e0); E(0x1e3); E(0x1e2);
	for (i = 0; i < 4; i++) E(0x1e4 + i);
	for (i = 0; i < 4; i++) E(0x1e8 + i);
	for (i = 0; i < 7; i++) E(0x1ec + i);
	E(0x1f3); E(0x1f4);
	for (i = 0; i < 7; i++) E(0x1f5 + i);
	EREV4(0x1fc);
	E(0x201); E(0x200); E(0x203); E(0x202);
	for (i = 0; i < 4; i++) E(0x204 + i);
	for (i = 0; i < 4; i++) E(0x208 + i);
	for (i = 0; i < 0x10; i++) E(0x20c + i);
	EREV4(0x21c);
	EREV4(0x220);
	EREV4(0x224);
	E(0x22b); E(0x22a); E(0x229); E(0x228);
	E(0x22f); E(0x22e); E(0x22d); E(0x22c);
	EREV4(0x230);
	EREV4(0x234);
}

/* ---- SapmV2 (cmd 0x1010006): sapm(5) + rssi(6) ---- */
static void build_sapm(struct packer *pk)
{
	int b, i, o;

	pk->bi = 0;
	S(0);
	PZ(); PZ(); PZ();
	for (o = 4; o < 0x434; o += 0x10c) SSWAP(o);
	for (o = 8; o < 0x438; o += 0x10c) S(o);
	for (o = 0xc; o < 0x43c; o += 0x10c) SSWAP(o);
	for (b = 0x10; b < 0x430; b += 0x10c)
		for (i = 0; i < 0x100; i++)
			S(b + i);
	for (i = 0; i < 0x34; i++) R(i);
}

/* ---- InitParamV25 (cmd 0x101000a): clink(1) only ---- */
static void build_v25(struct packer *pk)
{
	int i;

	pk->bi = 0;
	C(0x280); C(0x281); C(0x282); C(0x283);
	C(0x2c7); C(0x2c6); C(0x2c9); C(0x2c8); C(0x2cf); C(0x2ce);
	CSWAP(0x2cc);
	C(0x2d3); C(0x2d2); CSWAP(0x2d0);
	C(0x284);
	PZ(); PZ(); PZ();
	for (i = 0; i < 0x40; i++) C(0x285 + i);
	for (i = 0; i < 0x20; i++) C(0x2d4 + i);
	C(0x2f7); C(0x2f6); CSWAP(0x2f4);
	for (i = 0; i < 0x48; i++) C(0x2f8 + i);
	C(0x343); C(0x342); CSWAP(0x340);
	C(0x347); C(0x346); CSWAP(0x344);
	PZ(); PZ();
	C(0x34b); C(0x34a);
	for (i = 0x34c; i < 0x36c; i += 2) CSWAP(i);
}

static bool blob_ok(const struct mxl371x_cfg_blobs *b)
{
	return b->clink.len >= MXL_CLINK_MIN &&
	       b->mcast.len >= MXL_MCAST_MIN &&
	       b->rlapm.len >= MXL_RLAPM_MIN &&
	       b->endet.len >= MXL_ENDET_MIN &&
	       b->sapm.len >= MXL_SAPM_MIN &&
	       b->rssi.len >= MXL_RSSI_MIN &&
	       b->platform.len >= MXL_PLATFORM_MIN;
}

int mxl371x_pack_payloads(const struct mxl371x_cfg_blobs *blobs,
			  u32 guid_hi, u32 guid_lo,
			  struct mxl371x_payloads *out)
{
	struct packer pk = { .b = blobs };

	if (!blob_ok(blobs))
		return -EINVAL;

	memset(out, 0, sizeof(*out));

	pk.buf = out->v2; pk.cap = ARRAY_SIZE(out->v2);
	build_v2(&pk, guid_hi, guid_lo);
	out->v2_len = pk.bi;

	pk.buf = out->rlapm; pk.cap = ARRAY_SIZE(out->rlapm);
	build_rlapm(&pk);
	out->rlapm_len = pk.bi;

	pk.buf = out->sapm; pk.cap = ARRAY_SIZE(out->sapm);
	build_sapm(&pk);
	out->sapm_len = pk.bi;

	pk.buf = out->v25; pk.cap = ARRAY_SIZE(out->v25);
	build_v25(&pk);
	out->v25_len = pk.bi;

	return pk.overflow ? -EINVAL : 0;
}
