/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * MoCA boot-config payload packer.
 *
 * The four boot-config mailbox payloads (InitParamV2, RlapmV2, SapmV2,
 * InitParamV25) are serialized at runtime from the per-board binary config
 * blobs (clink/mcast/rlapm/endet/sapm/rssi/platform), in the SoC wire format
 * (big-endian packed words).  This replaces the previously static, board-baked
 * payload arrays so a single image can drive any board that ships its own
 * blob set.  InitParamV2 carries the per-device MoCA GUID, injected inline.
 */
#ifndef _MXL371X_PACKER_H
#define _MXL371X_PACKER_H

#include <linux/types.h>

/* one config blob: data pointer + length in bytes */
struct mxl371x_blob {
	const u8 *data;
	size_t len;
};

/* the seven config blobs */
struct mxl371x_cfg_blobs {
	struct mxl371x_blob clink;	/* idx 1 */
	struct mxl371x_blob mcast;	/* idx 2 */
	struct mxl371x_blob rlapm;	/* idx 3 */
	struct mxl371x_blob endet;	/* idx 4 */
	struct mxl371x_blob sapm;	/* idx 5 */
	struct mxl371x_blob rssi;	/* idx 6 */
	struct mxl371x_blob platform;	/* idx 7 */
};

/* packed payloads (sizes in words) */
struct mxl371x_payloads {
	u32 v2[270];
	u32 rlapm[272];
	u32 sapm[275];
	u32 v25[59];
	u32 v2_len;	/* bytes */
	u32 rlapm_len;
	u32 sapm_len;
	u32 v25_len;
};

/*
 * Serialize all four payloads from @blobs into @out.  @guid_hi/@guid_lo are the
 * device MoCA GUID words, packed big-endian into InitParamV2 (word 5/6).
 * Returns 0 on success, -EINVAL if any blob is shorter than the fields read
 * from it.
 */
int mxl371x_pack_payloads(const struct mxl371x_cfg_blobs *blobs,
			  u32 guid_hi, u32 guid_lo,
			  struct mxl371x_payloads *out);

#endif /* _MXL371X_PACKER_H */
