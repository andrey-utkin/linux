/*
 *  TW5864 driver  - common header file
 *
 *  Copyright (C) 2015 Bluecherry, LLC <maintainers@bluecherrydvr.com>
 *  Author: Andrey Utkin <andrey.utkin@corp.bluecherry.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/pci.h>
#include <linux/videodev2.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-sg.h>

#include "tw5864-reg.h"

#define PCI_DEVICE_ID_TECHWELL_5864 0x5864

#define TW5864_NORMS ( \
		       V4L2_STD_NTSC  | V4L2_STD_PAL    | V4L2_STD_SECAM | \
		       V4L2_STD_PAL_M | V4L2_STD_PAL_Nc | V4L2_STD_PAL_60)

/* ----------------------------------------------------------- */
/* static data                                                 */

struct tw5864_tvnorm {
	char *name;
	v4l2_std_id id;

	/* video decoder */
	u32 sync_control;
	u32 luma_control;
	u32 chroma_ctrl1;
	u32 chroma_gain;
	u32 chroma_ctrl2;
	u32 vgate_misc;

	/* video scaler */
	u32 h_delay;
	u32 h_start;
	u32 h_stop;
	u32 v_delay;
	u32 video_v_start;
	u32 video_v_stop;
	u32 vbi_v_start_0;
	u32 vbi_v_stop_0;
	u32 vbi_v_start_1;

	/* Techwell specific */
	u32 format;
};

struct tw5864_format {
	char *name;
	u32 fourcc;
	u32 depth;
	u32 twformat;
};

/* ----------------------------------------------------------- */
/* card configuration   */

#define TW5864_INPUTS 4

#define H264_VLC_BUF_SIZE 0x80000
#define H264_MV_BUF_SIZE 0x40000
#define QP_VALUE 28
#define BITALIGN_VALUE_IN_TIMER 0
#define BITALIGN_VALUE_IN_INIT 0
#define GOP_SIZE 32

enum resolution {
	D1 = 1,
	HD1 = 2, /* half d1 - 360x(240|288) */
	CIF = 3,
	QCIF = 4,
};

/* ----------------------------------------------------------- */
/* device / file handle status                                 */

struct tw5864_dev; /* forward delclaration */

/* buffer for one video/vbi/ts frame */
struct tw5864_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;

	unsigned int size;
};

struct tw5864_fmt {
	char *name;
	u32 fourcc; /* v4l2 format id */
	int depth;
	int flags;
	u32 twformat;
};

struct tw5864_dma_buf {
	void *addr;
	dma_addr_t dma_addr;
};

enum tw5864_vid_std {
	STD_NTSC = 0,
	STD_PAL = 1,
	STD_SECAM = 2,

	STD_INVALID = 7,
	STD_AUTO = 7,
};

v4l2_std_id tw5864_get_v4l2_std(enum tw5864_vid_std std);
enum tw5864_vid_std tw5864_from_v4l2_std(v4l2_std_id v4l2_std);

struct tw5864_input {
	int input_number;
	struct tw5864_dev *root;
	struct mutex lock; /* used for vidq and vdev */
	spinlock_t slock; /* used for sync between ISR, tasklet & V4L2 API */
	struct video_device vdev;
	struct v4l2_ctrl_handler hdl;
	const struct tw5864_tvnorm *tvnorm;
	void *alloc_ctx;
	struct vb2_queue vidq;
	struct list_head active;
	const struct tw5864_format *fmt;
	enum resolution resolution;
	unsigned int width, height;
	unsigned int frame_seqno;
	unsigned int h264_idr_pic_id;
	unsigned int h264_frame_seqno_in_gop;
	int enabled;
	enum tw5864_vid_std std;
	v4l2_std_id v4l2_std;
	int tail_nb_bits;
	u8 tail;
	u8 *buf_cur_ptr;
	int buf_cur_space_left;

	u32 reg_interlacing;
	u32 reg_vlc;
	u32 reg_dsp_codec;
	u32 reg_dsp;
	u32 reg_emu;
	u32 reg_dsp_qp;
	u32 reg_dsp_ref_mvp_lambda;
	u32 reg_dsp_i4x4_weight;
	u32 buf_id;

	struct tw5864_buf *vb;

	struct v4l2_ctrl *md_threshold_grid_ctrl;
	u16 md_threshold_grid_values[12 * 16];
	int qp;
	int gop;

	/*
	 * In (1/MAX_FPS) units.
	 * For max FPS (default), set to 1.
	 * For 1 FPS, set to e.g. 32.
	 */
	int frame_interval;
	unsigned long new_frame_deadline;
};

struct tw5864_h264_frame {
	struct tw5864_dma_buf vlc;
	struct tw5864_dma_buf mv;

	int vlc_len;
	u32 checksum;
	struct tw5864_input *input;

	u64 timestamp;
};

/* global device status */
struct tw5864_dev {
	spinlock_t slock; /* used for sync between ISR, tasklet & V4L2 API */
	struct v4l2_device v4l2_dev;
	struct tw5864_input inputs[TW5864_INPUTS];
#define H264_BUF_CNT 64
	struct tw5864_h264_frame h264_buf[H264_BUF_CNT];
	int h264_buf_r_index;
	int h264_buf_w_index;

	struct tasklet_struct tasklet;

	int encoder_busy;
	/* Input number to check next (in RR fashion) */
	int next_i;

	/* pci i/o */
	char name[64];
	struct pci_dev *pci;
	void __iomem *mmio;
	u32 irqmask;
	u32 frame_seqno;

	u32 stored_len;

	struct dentry *debugfs_dir;
};

#define tw_readl(reg) readl(dev->mmio + reg)
#define tw_mask_readl(reg, mask) \
	(tw_readl(reg) & (mask))
#define tw_mask_shift_readl(reg, mask, shift) \
	(tw_mask_readl((reg), ((mask) << (shift))) >> (shift))

#define tw_writel(reg, value) writel((value), dev->mmio + reg)
#define tw_mask_writel(reg, mask, value) \
	tw_writel(reg, (tw_readl(reg) & ~(mask)) | ((value) & (mask)))
#define tw_mask_shift_writel(reg, mask, shift, value) \
	tw_mask_writel((reg), ((mask) << (shift)), ((value) << (shift)))

#define tw_setl(reg, bit) tw_writel((reg), tw_readl(reg) | (bit))
#define tw_clearl(reg, bit) tw_writel((reg), tw_readl(reg) & ~(bit))

u8 tw_indir_readb(struct tw5864_dev *dev, u16 addr);
void tw_indir_writeb(struct tw5864_dev *dev, u16 addr, u8 data);

void tw5864_set_tvnorm_hw(struct tw5864_dev *dev);

void tw5864_irqmask_apply(struct tw5864_dev *dev);
void tw5864_init_ad(struct tw5864_dev *dev);
int tw5864_video_init(struct tw5864_dev *dev, int *video_nr);
void tw5864_video_fini(struct tw5864_dev *dev);
void tw5864_prepare_frame_headers(struct tw5864_input *input);
void tw5864_h264_put_stream_header(u8 **buf, size_t *space_left, int qp,
				   int width, int height);
void tw5864_h264_put_slice_header(u8 **buf, size_t *space_left,
				  unsigned int idr_pic_id,
				  unsigned int frame_seqno_in_gop,
				  int *tail_nb_bits, u8 *tail);
void tw5864_request_encoded_frame(struct tw5864_input *input);
void tw5864_push_to_make_it_roll(struct tw5864_input *input);

static const unsigned int lambda_lookup_table[52] = {
	0x0020, 0x0020, 0x0020, 0x0020,
	0x0020, 0x0020, 0x0020, 0x0020,
	0x0020, 0x0020, 0x0020, 0x0020,
	0x0020, 0x0020, 0x0020, 0x0020,
	0x0040, 0x0040, 0x0040, 0x0040,
	0x0060, 0x0060, 0x0060, 0x0080,
	0x0080, 0x0080, 0x00a0, 0x00c0,
	0x00c0, 0x00e0, 0x0100, 0x0120,
	0x0140, 0x0160, 0x01a0, 0x01c0,
	0x0200, 0x0240, 0x0280, 0x02e0,
	0x0320, 0x03a0, 0x0400, 0x0480,
	0x0500, 0x05a0, 0x0660, 0x0720,
	0x0800, 0x0900, 0x0a20, 0x0b60
};

static const unsigned int intra4x4_lambda3[52] = {
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 3, 3, 3, 4,
	4, 4, 5, 6, 6, 7, 8, 9,
	10, 11, 13, 14, 16, 18, 20, 23,
	25, 29, 32, 36, 40, 45, 51, 57,
	64, 72, 81, 91
};
