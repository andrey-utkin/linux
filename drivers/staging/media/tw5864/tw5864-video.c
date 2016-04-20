/*
 *  TW5864 driver - video encoding functions
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

#include <linux/module.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "tw5864.h"
#include "tw5864-reg.h"
#include "tw5864-tables.h"

static void tw5864_handle_frame_task(unsigned long data);
static void tw5864_handle_frame(struct tw5864_h264_frame *frame);
static void tw5864_frame_interval_set(struct tw5864_input *input);

static int tw5864_queue_setup(struct vb2_queue *q,
			      unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      void *alloc_ctxs[])
{
	struct tw5864_input *dev = vb2_get_drv_priv(q);

	if (q->num_buffers + *num_buffers < 12)
		*num_buffers = 12 - q->num_buffers;

	alloc_ctxs[0] = dev->alloc_ctx;
	if (*num_planes)
		return sizes[0] < H264_VLC_BUF_SIZE ? -EINVAL : 0;

	sizes[0] = H264_VLC_BUF_SIZE;
	*num_planes = 1;

	return 0;
}

static void tw5864_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *vq = vb->vb2_queue;
	struct tw5864_input *dev = vb2_get_drv_priv(vq);
	struct tw5864_buf *buf = container_of(vbuf, struct tw5864_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	list_add_tail(&buf->list, &dev->active);
	spin_unlock_irqrestore(&dev->slock, flags);
}

static int tw5864_input_std_get(struct tw5864_input *input,
				enum tw5864_vid_std *std_arg)
{
	struct tw5864_dev *dev = input->root;
	enum tw5864_vid_std std;
	u8 indir_0x00e = tw_indir_readb(dev,
					0x00e + input->input_number * 0x010);
	std = (indir_0x00e & 0x70) >> 4;

	if (indir_0x00e & 0x80) {
		dev_err(&dev->pci->dev,
			"Video format detection is in progress, please wait\n");
		return -EAGAIN;
	}

	if (std == STD_INVALID) {
		dev_err(&dev->pci->dev, "No valid video format detected\n");
		return -1;
	}

	*std_arg = std;
	return 0;
}

static int tw5864_enable_input(struct tw5864_input *input)
{
	struct tw5864_dev *dev = input->root;
	int input_number = input->input_number;
	unsigned long flags;
	int ret;
	int d1_width = 720;
	int d1_height;
	int frame_width_bus_value = 0;
	int frame_height_bus_value = 0;
	int reg_frame_bus = 0x1c;
	int fmt_reg_value = 0;
	int downscale_enabled = 0;

	dev_dbg(&dev->pci->dev, "Enabling channel %d\n", input_number);

	ret = tw5864_input_std_get(input, &input->std);
	if (ret)
		return ret;
	input->v4l2_std = tw5864_get_v4l2_std(input->std);

	input->frame_seqno = 0;
	input->h264_idr_pic_id = 0;
	input->h264_frame_seqno_in_gop = 0;

	input->reg_dsp_qp = input->qp;
	input->reg_dsp_ref_mvp_lambda = lambda_lookup_table[input->qp];
	input->reg_dsp_i4x4_weight = intra4x4_lambda3[input->qp];
	input->reg_emu = TW5864_EMU_EN_LPF | TW5864_EMU_EN_BHOST
		| TW5864_EMU_EN_SEN | TW5864_EMU_EN_ME | TW5864_EMU_EN_DDR;
	input->reg_dsp = input_number /* channel id */
		| TW5864_DSP_CHROM_SW
		| ((0xa << 8) & TW5864_DSP_MB_DELAY)
		;

	input->resolution = D1;

	d1_height = (input->std == STD_NTSC) ? 480 : 576;

	input->width = d1_width;
	input->height = d1_height;

	input->reg_interlacing = 0x4;

	switch (input->resolution) {
	case D1:
		frame_width_bus_value = 0x2cf;
		frame_height_bus_value = input->height - 1;
		reg_frame_bus = 0x1c;
		fmt_reg_value = 0;
		downscale_enabled = 0;
		input->reg_dsp_codec |= TW5864_CIF_MAP_MD | TW5864_HD1_MAP_MD;
		input->reg_emu |= TW5864_DSP_FRAME_TYPE_D1;
		input->reg_interlacing = TW5864_DI_EN | TW5864_DSP_INTER_ST;

		tw_setl(TW5864_FULL_HALF_FLAG, 1 << input_number);
		break;
	case HD1:
		input->height /= 2;
		input->width /= 2;
		frame_width_bus_value = 0x2cf;
		frame_height_bus_value = input->height * 2 - 1;
		reg_frame_bus = 0x1c;
		fmt_reg_value = 0;
		downscale_enabled = 0;
		input->reg_dsp_codec |= TW5864_HD1_MAP_MD;
		input->reg_emu |= TW5864_DSP_FRAME_TYPE_D1;

		tw_clearl(TW5864_FULL_HALF_FLAG, 1 << input_number);

		break;
	case CIF:
		input->height /= 4;
		input->width /= 2;
		frame_width_bus_value = 0x15f;
		frame_height_bus_value = input->height * 2 - 1;
		reg_frame_bus = 0x07;
		fmt_reg_value = 1;
		downscale_enabled = 1;
		input->reg_dsp_codec |= TW5864_CIF_MAP_MD;

		tw_clearl(TW5864_FULL_HALF_FLAG, 1 << input_number);
		break;
	case QCIF:
		input->height /= 4;
		input->width /= 4;
		frame_width_bus_value = 0x15f;
		frame_height_bus_value = input->height * 2 - 1;
		reg_frame_bus = 0x07;
		fmt_reg_value = 1;
		downscale_enabled = 1;
		input->reg_dsp_codec |= TW5864_CIF_MAP_MD;

		tw_clearl(TW5864_FULL_HALF_FLAG, 1 << input_number);
		break;
	}

	/* analog input width / 4 */
	tw_indir_writeb(dev, TW5864_INDIR_IN_PIC_WIDTH(input_number),
			d1_width / 4);
	tw_indir_writeb(dev, TW5864_INDIR_IN_PIC_HEIGHT(input_number),
			d1_height / 4);

	/* output width / 4 */
	tw_indir_writeb(dev, TW5864_INDIR_OUT_PIC_WIDTH(input_number),
			input->width / 4);
	tw_indir_writeb(dev, TW5864_INDIR_OUT_PIC_HEIGHT(input_number),
			input->height / 4);

	tw_writel(TW5864_DSP_PIC_MAX_MB,
		  ((input->width / 16) << 8) | (input->height / 16));

	tw_writel(TW5864_FRAME_WIDTH_BUS_A(input_number),
		  frame_width_bus_value);
	tw_writel(TW5864_FRAME_WIDTH_BUS_B(input_number),
		  frame_width_bus_value);
	tw_writel(TW5864_FRAME_HEIGHT_BUS_A(input_number),
		  frame_height_bus_value);
	tw_writel(TW5864_FRAME_HEIGHT_BUS_B(input_number),
		  (frame_height_bus_value + 1) / 2 - 1);

	tw5864_frame_interval_set(input);

	if (downscale_enabled)
		tw_setl(TW5864_H264EN_CH_DNS, 1 << input_number);

	tw_mask_shift_writel(TW5864_H264EN_CH_FMT_REG1, 0x3, 2 * input_number,
			     fmt_reg_value);

	tw_mask_shift_writel(
			     (input_number < 2
			      ? TW5864_H264EN_RATE_MAX_LINE_REG1
			      : TW5864_H264EN_RATE_MAX_LINE_REG2),
			     0x1f, 5 * (input_number % 2),
			     input->std == STD_NTSC ? 29 : 24);

	tw_mask_shift_writel((input_number < 2) ? TW5864_FRAME_BUS1 :
			     TW5864_FRAME_BUS2, 0xff, (input_number % 2) * 8,
			     reg_frame_bus);

	spin_lock_irqsave(&dev->slock, flags);
	input->enabled = 1;
	spin_unlock_irqrestore(&dev->slock, flags);

	return 0;
}

void tw5864_request_encoded_frame(struct tw5864_input *input)
{
	struct tw5864_dev *dev = input->root;
	u32 enc_buf_id_new;

	tw_setl(TW5864_DSP_CODEC, TW5864_CIF_MAP_MD | TW5864_HD1_MAP_MD);
	tw_writel(TW5864_EMU, input->reg_emu);
	tw_writel(TW5864_INTERLACING, input->reg_interlacing);
	tw_writel(TW5864_DSP, input->reg_dsp);

	tw_writel(TW5864_DSP_QP, input->reg_dsp_qp);
	tw_writel(TW5864_DSP_REF_MVP_LAMBDA, input->reg_dsp_ref_mvp_lambda);
	tw_writel(TW5864_DSP_I4x4_WEIGHT, input->reg_dsp_i4x4_weight);
	/* 16x16 */
	tw_mask_shift_writel(TW5864_DSP_INTRA_MODE, TW5864_DSP_INTRA_MODE_MASK,
			     TW5864_DSP_INTRA_MODE_SHIFT,
			     TW5864_DSP_INTRA_MODE_16x16);

	if (input->frame_seqno % input->gop == 0) {
		/* Produce I-frame */
		tw_writel(TW5864_MOTION_SEARCH_ETC, TW5864_INTRA_EN);
		input->h264_frame_seqno_in_gop = 0;
		input->h264_idr_pic_id++;
		input->h264_idr_pic_id &= TW5864_DSP_REF_FRM;
	} else {
		/* Produce P-frame */
		tw_writel(TW5864_MOTION_SEARCH_ETC,
			  TW5864_INTRA_EN
			  | TW5864_ME_EN
			  | BIT(5) /* SRCH_OPT default */
			 );
		input->h264_frame_seqno_in_gop++;
	}
	tw5864_prepare_frame_headers(input);
	tw_writel(TW5864_VLC,
		  TW5864_VLC_PCI_SEL | ((input->tail_nb_bits + 24) <<
					TW5864_VLC_BIT_ALIGN_SHIFT) |
		  input->reg_dsp_qp);

	enc_buf_id_new = tw_mask_shift_readl(TW5864_ENC_BUF_PTR_REC1, 0x3,
					     2 * input->input_number);
	tw_writel(TW5864_DSP_ENC_ORG_PTR_REG,
		  ((enc_buf_id_new + 1) % 4) << TW5864_DSP_ENC_ORG_PTR_SHIFT);
	tw_writel(TW5864_DSP_ENC_REC,
		  (((enc_buf_id_new + 1) % 4) << 12) | (enc_buf_id_new & 0x3));

	tw_writel(TW5864_SLICE, TW5864_START_NSLICE);
	tw_writel(TW5864_SLICE, 0);
}

static int tw5864_disable_input(struct tw5864_input *input)
{
	struct tw5864_dev *dev = input->root;
	unsigned long flags;

	dev_dbg(&dev->pci->dev, "Disabling channel %d\n", input->input_number);

	spin_lock_irqsave(&dev->slock, flags);
	input->enabled = 0;
	spin_unlock_irqrestore(&dev->slock, flags);
	return 0;
}

static int tw5864_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct tw5864_input *input = vb2_get_drv_priv(q);

	tw5864_enable_input(input);
	return 0;
}

static void tw5864_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct tw5864_input *input = vb2_get_drv_priv(q);

	tw5864_disable_input(input);

	spin_lock_irqsave(&input->slock, flags);
	if (input->vb) {
		vb2_buffer_done(&input->vb->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		input->vb = NULL;
	}
	while (!list_empty(&input->active)) {
		struct tw5864_buf *buf = container_of(input->active.next,
						      struct tw5864_buf, list);

		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&input->slock, flags);
}

static struct vb2_ops tw5864_video_qops = {
	.queue_setup = tw5864_queue_setup,
	.buf_queue = tw5864_buf_queue,
	.start_streaming = tw5864_start_streaming,
	.stop_streaming = tw5864_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int tw5864_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tw5864_input *input =
		container_of(ctrl->handler, struct tw5864_input, hdl);
	struct tw5864_dev *dev = input->root;
	unsigned long flags;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		tw_indir_writeb(dev,
				TW5864_INDIR_VIN_A_BRIGHT(input->input_number),
				(u8)ctrl->val);
		break;
	case V4L2_CID_HUE:
		tw_indir_writeb(dev,
				TW5864_INDIR_VIN_7_HUE(input->input_number),
				(u8)ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		tw_indir_writeb(dev,
				TW5864_INDIR_VIN_9_CNTRST(input->input_number),
				(u8)ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		tw_indir_writeb(dev,
				TW5864_INDIR_VIN_B_SAT_U(input->input_number),
				(u8)ctrl->val);
		tw_indir_writeb(dev,
				TW5864_INDIR_VIN_C_SAT_V(input->input_number),
				(u8)ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		input->gop = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		spin_lock_irqsave(&input->slock, flags);
		input->qp = ctrl->val;
		input->reg_dsp_qp = input->qp;
		input->reg_dsp_ref_mvp_lambda = lambda_lookup_table[input->qp];
		input->reg_dsp_i4x4_weight = intra4x4_lambda3[input->qp];
		spin_unlock_irqrestore(&input->slock, flags);
		return 0;
	case V4L2_CID_DETECT_MD_GLOBAL_THRESHOLD:
		memset(input->md_threshold_grid_values, ctrl->val,
		       sizeof(input->md_threshold_grid_values));
		return 0;
	case V4L2_CID_DETECT_MD_MODE:
		return 0;
	case V4L2_CID_DETECT_MD_THRESHOLD_GRID:
		/* input->md_threshold_grid_ctrl->p_new.p_u16 contains data */
		memcpy(input->md_threshold_grid_values,
		       input->md_threshold_grid_ctrl->p_new.p_u16,
		       sizeof(input->md_threshold_grid_values));
		return 0;
	}
	return 0;
}

static int tw5864_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct tw5864_input *input = video_drvdata(file);
	enum tw5864_vid_std std;
	int ret;

	ret = tw5864_input_std_get(input, &std);
	if (ret)
		return ret;

	f->fmt.pix.width = 720;
	switch (std) {
	default:
		WARN_ON_ONCE(1);
	case STD_NTSC:
		f->fmt.pix.height = 480;
		break;
	case STD_PAL:
	case STD_SECAM:
		f->fmt.pix.height = 576;
		break;
	}
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
	f->fmt.pix.sizeimage = H264_VLC_BUF_SIZE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	f->fmt.pix.priv = 0;
	return 0;
}

static int tw5864_enum_input(struct file *file, void *priv,
			     struct v4l2_input *i)
{
	struct tw5864_input *dev = video_drvdata(file);

	u8 indir_0x000 = tw_indir_readb(dev->root,
			TW5864_INDIR_VIN_0(dev->input_number));
	u8 indir_0x00d = tw_indir_readb(dev->root,
			TW5864_INDIR_VIN_D(dev->input_number));
	u8 v1 = indir_0x000;
	u8 v2 = indir_0x00d;

	if (i->index)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	snprintf(i->name, sizeof(i->name), "Encoder %d", dev->input_number);
	i->std = TW5864_NORMS;
	if (v1 & (1 << 7))
		i->status |= V4L2_IN_ST_NO_SYNC;
	if (!(v1 & (1 << 6)))
		i->status |= V4L2_IN_ST_NO_H_LOCK;
	if (v1 & (1 << 2))
		i->status |= V4L2_IN_ST_NO_SIGNAL;
	if (v1 & (1 << 1))
		i->status |= V4L2_IN_ST_NO_COLOR;
	if (v2 & (1 << 2))
		i->status |= V4L2_IN_ST_MACROVISION;

	return 0;
}

static int tw5864_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int tw5864_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i)
		return -EINVAL;
	return 0;
}

static int tw5864_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct tw5864_input *dev = video_drvdata(file);

	strcpy(cap->driver, "tw5864");
	snprintf(cap->card, sizeof(cap->card), "TW5864 Encoder %d",
		 dev->input_number);
	sprintf(cap->bus_info, "PCI:%s", pci_name(dev->root->pci));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
		V4L2_CAP_STREAMING;

	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int tw5864_g_std(struct file *file, void *priv, v4l2_std_id *id)
{
	struct tw5864_input *input = video_drvdata(file);
	enum tw5864_vid_std std;
	int ret;

	ret = tw5864_input_std_get(input, &std);
	if (ret)
		return ret;

	*id = tw5864_get_v4l2_std(std);
	return 0;
}

static int tw5864_s_std(struct file *file, void *priv, v4l2_std_id id)
{
	struct tw5864_input *input = video_drvdata(file);
	enum tw5864_vid_std std;
	int ret;

	ret = tw5864_input_std_get(input, &std);
	if (ret)
		return ret;

	/* Allow only if matches with currently detected */
	if (id != tw5864_get_v4l2_std(std))
		return -EINVAL;

	return 0;
}

static int tw5864_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	return tw5864_g_fmt_vid_cap(file, priv, f);
}

static int tw5864_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return tw5864_try_fmt_vid_cap(file, priv, f);
}

static int tw5864_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_H264;

	return 0;
}

static int tw5864_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	case V4L2_EVENT_MOTION_DET:
		/*
		 * Allow for up to 30 events (1 second for NTSC) to be stored.
		 */
		return v4l2_event_subscribe(fh, sub, 30, NULL);
	}
	return -EINVAL;
}

static void tw5864_frame_interval_set(struct tw5864_input *input)
{
	/*
	 * This register value seems to follow such approach: In each second
	 * interval, when processing Nth frame, it checks Nth bit of register
	 * value and, if the bit is 1, it processes the frame, otherwise the
	 * frame is discarded.
	 * So unary representation would work, but more or less equal gaps
	 * between the frames should be preserved.
	 * For 1 FPS - 0x00000001
	 * 00000000 00000000 00000000 00000001
	 *
	 * For 2 FPS - 0x00010001.
	 * 00000000 00000001 00000000 00000001
	 *
	 * For 4 FPS - 0x01010101.
	 * 00000001 00000001 00000001 00000001
	 *
	 * For 8 FPS - 0x11111111.
	 * 00010001 00010001 00010001 00010001
	 *
	 * For 16 FPS - 0x55555555.
	 * 01010101 01010101 01010101 01010101
	 *
	 * For 32 FPS (not reached - capped by 25/30 limit) - 0xffffffff.
	 * 11111111 11111111 11111111 11111111
	 *
	 * Et cetera.
	 */
	struct tw5864_dev *dev = input->root;
	u32 unary_framerate = 0;
	int shift = 0;

	for (shift = 0; shift <= 32; shift += input->frame_interval)
		unary_framerate |= 0x00000001 << shift;

	tw_writel(TW5864_H264EN_RATE_CNTL_LO_WORD(input->input_number, 0),
		  unary_framerate >> 16);
	tw_writel(TW5864_H264EN_RATE_CNTL_HI_WORD(input->input_number, 0),
		  unary_framerate & 0xffff);
}

static int tw5864_frameinterval_get(struct tw5864_input *input,
				    struct v4l2_fract *frameinterval)
{
	int ret;
	enum tw5864_vid_std std;

	ret = tw5864_input_std_get(input, &std);
	if (ret)
		return ret;

	frameinterval->numerator = 1;

	switch (std) {
	case STD_NTSC:
	case STD_SECAM:
		frameinterval->denominator = 25;
		break;
	case STD_PAL:
		frameinterval->denominator = 30;
		break;
	default:
		WARN(1, "tw5864_frameinterval_get requested for unknown std %d\n",
		     std);
		return 1;
	}

	return 0;
}

static int tw5864_enum_frameintervals(struct file *file, void *priv,
				      struct v4l2_frmivalenum *fintv)
{
	struct tw5864_input *input = video_drvdata(file);

	if (fintv->pixel_format != V4L2_PIX_FMT_H264)
		return -EINVAL;
	if (fintv->index)
		return -EINVAL;

	fintv->type = V4L2_FRMIVAL_TYPE_DISCRETE;

	return tw5864_frameinterval_get(input, &fintv->discrete);
}

static int tw5864_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *sp)
{
	struct tw5864_input *input = video_drvdata(file);
	struct v4l2_captureparm *cp = &sp->parm.capture;
	int ret;

	cp->capability = V4L2_CAP_TIMEPERFRAME;

	ret = tw5864_frameinterval_get(input, &cp->timeperframe);
	cp->timeperframe.numerator *= input->frame_interval;
	cp->capturemode = 0;
	cp->readbuffers = 2;

	return ret;
}

static int tw5864_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *sp)
{
	struct tw5864_input *input = video_drvdata(file);
	struct v4l2_fract *t = &sp->parm.capture.timeperframe;
	struct v4l2_fract time_base;
	int ret;

	ret = tw5864_frameinterval_get(input, &time_base);
	if (ret)
		return ret;

	if (!t->numerator || !t->denominator) {
		dev_err(&input->root->pci->dev,
			"weird timeperframe %u/%u, using current %u/%u\n",
			t->numerator, t->denominator,
			input->frame_interval, time_base.denominator);
		t->numerator = input->frame_interval;
		t->denominator = time_base.denominator;
	} else if (t->denominator != time_base.denominator) {
		t->numerator = t->numerator * time_base.denominator /
			t->denominator;
		t->denominator = time_base.denominator;
	}

	input->frame_interval = t->numerator;
	tw5864_frame_interval_set(input);
	return tw5864_g_parm(file, priv, sp);
}

static const struct v4l2_ctrl_ops tw5864_ctrl_ops = {
	.s_ctrl = tw5864_s_ctrl,
};

static const struct v4l2_file_operations video_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG

#define INDIR_SPACE_MAP_SHIFT 0x100000

static int tw5864_g_reg(struct file *file, void *fh,
			struct v4l2_dbg_register *reg)
{
	struct tw5864_input *input = video_drvdata(file);
	struct tw5864_dev *dev = input->root;

	if (reg->reg < INDIR_SPACE_MAP_SHIFT) {
		if (reg->reg > 0x87FFF)
			return -EINVAL;
		reg->size = 4;
		reg->val = tw_readl(reg->reg);
	} else {
		__u64 indir_addr = reg->reg - INDIR_SPACE_MAP_SHIFT;
		if (indir_addr > 0xEFE)
			return -EINVAL;
		reg->size = 1;
		reg->val = tw_indir_readb(dev, reg->reg);
	}
	return 0;
}

static int tw5864_s_reg(struct file *file, void *fh,
			const struct v4l2_dbg_register *reg)
{
	struct tw5864_input *input = video_drvdata(file);
	struct tw5864_dev *dev = input->root;

	if (reg->reg < INDIR_SPACE_MAP_SHIFT) {
		if (reg->reg > 0x87FFF)
			return -EINVAL;
		tw_writel(reg->reg, reg->val);
	} else {
		__u64 indir_addr = reg->reg - INDIR_SPACE_MAP_SHIFT;
		if (indir_addr > 0xEFE)
			return -EINVAL;
		tw_indir_writeb(dev, reg->reg, reg->val);
	}
	return 0;
}
#endif

static const struct v4l2_ioctl_ops video_ioctl_ops = {
	.vidioc_querycap = tw5864_querycap,
	.vidioc_enum_fmt_vid_cap = tw5864_enum_fmt_vid_cap,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_s_std = tw5864_s_std,
	.vidioc_g_std = tw5864_g_std,
	.vidioc_enum_input = tw5864_enum_input,
	.vidioc_g_input = tw5864_g_input,
	.vidioc_s_input = tw5864_s_input,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_try_fmt_vid_cap = tw5864_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = tw5864_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = tw5864_g_fmt_vid_cap,
	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = tw5864_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_enum_frameintervals = tw5864_enum_frameintervals,
	.vidioc_s_parm = tw5864_s_parm,
	.vidioc_g_parm = tw5864_g_parm,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register = tw5864_g_reg,
	.vidioc_s_register = tw5864_s_reg,
#endif
};

static struct video_device tw5864_video_template = {
	.name = "tw5864_video",
	.fops = &video_fops,
	.ioctl_ops = &video_ioctl_ops,
	.release = video_device_release_empty,
	.tvnorms = TW5864_NORMS,
};

/* The TW5864 uses 192 (16x12) detection cells in full screen for motion
 * detection. Each detection cell is composed of 44 pixels and 20 lines for
 * NTSC and 24 lines for PAL.
 */
#define MD_CELLS_HOR 16
#define MD_CELLS_VERT 12

/* Motion Detection Threshold matrix */
static const struct v4l2_ctrl_config tw5864_md_thresholds = {
	.ops = &tw5864_ctrl_ops,
	.id = V4L2_CID_DETECT_MD_THRESHOLD_GRID,
	.dims = {MD_CELLS_HOR, MD_CELLS_VERT},
	.def = 14,
	/* See tw5864_md_metric_from_mvd() */
	.max = 2 * 0x0f,
	.step = 1,
};

static int tw5864_video_input_init(struct tw5864_input *dev, int video_nr);
static void tw5864_video_input_fini(struct tw5864_input *dev);
static void tw5864_tables_upload(struct tw5864_dev *dev);

int tw5864_video_init(struct tw5864_dev *dev, int *video_nr)
{
	int i;
	int ret = -1;

	for (i = 0; i < H264_BUF_CNT; i++) {
		dev->h264_buf[i].vlc.addr =
			dma_alloc_coherent(&dev->pci->dev, H264_VLC_BUF_SIZE,
					   &dev->h264_buf[i].vlc.dma_addr,
					   GFP_KERNEL | GFP_DMA32);
		dev->h264_buf[i].mv.addr =
			dma_alloc_coherent(&dev->pci->dev, H264_MV_BUF_SIZE,
					   &dev->h264_buf[i].mv.dma_addr,
					   GFP_KERNEL | GFP_DMA32);
		if (!dev->h264_buf[i].vlc.addr || !dev->h264_buf[i].mv.addr) {
			dev_err(&dev->pci->dev, "dma alloc & map fail\n");
			ret = -ENOMEM;
			goto dma_alloc_fail;
		}
	}

	tw5864_tables_upload(dev);
	tw5864_init_ad(dev);

	/* Picture is distorted without this block */
	/* use falling edge to sample 54M to 108M */
	tw_indir_writeb(dev, TW5864_INDIR_VD_108_POL,
			TW5864_INDIR_VD_108_POL_BOTH);
	tw_indir_writeb(dev, TW5864_INDIR_CLK0_SEL, 0x00);

	tw_indir_writeb(dev, TW5864_INDIR_DDRA_DLL_DQS_SEL0, 0x02);
	tw_indir_writeb(dev, TW5864_INDIR_DDRA_DLL_DQS_SEL1, 0x02);
	tw_indir_writeb(dev, TW5864_INDIR_DDRA_DLL_CLK90_SEL, 0x02);
	tw_indir_writeb(dev, TW5864_INDIR_DDRB_DLL_DQS_SEL0, 0x02);
	tw_indir_writeb(dev, TW5864_INDIR_DDRB_DLL_DQS_SEL1, 0x02);
	tw_indir_writeb(dev, TW5864_INDIR_DDRB_DLL_CLK90_SEL, 0x02);

	/* video input reset */
	tw_indir_writeb(dev, TW5864_INDIR_RESET, 0);
	tw_indir_writeb(dev, TW5864_INDIR_RESET, TW5864_INDIR_RESET_VD |
			TW5864_INDIR_RESET_DLL | TW5864_INDIR_RESET_MUX_CORE);
	mdelay(10);

	/*
	 * Select Part A mode for all channels.
	 * tw_setl instead of tw_clearl for Part B mode.
	 *
	 * I guess "Part B" is primarily for downscaled version of same channel
	 * which goes in Part A of same bus
	 */
	tw_writel(TW5864_FULL_HALF_MODE_SEL, 0);

	tw_indir_writeb(dev, TW5864_INDIR_PV_VD_CK_POL,
			TW5864_INDIR_PV_VD_CK_POL_VD(0) |
			TW5864_INDIR_PV_VD_CK_POL_VD(1) |
			TW5864_INDIR_PV_VD_CK_POL_VD(2) |
			TW5864_INDIR_PV_VD_CK_POL_VD(3));

	dev->h264_buf_r_index = 0;
	dev->h264_buf_w_index = 0;
	tw_writel(TW5864_VLC_STREAM_BASE_ADDR,
		  dev->h264_buf[dev->h264_buf_w_index].vlc.dma_addr);
	tw_writel(TW5864_MV_STREAM_BASE_ADDR,
		  dev->h264_buf[dev->h264_buf_w_index].mv.dma_addr);

	for (i = 0; i < TW5864_INPUTS; i++) {
		tw_indir_writeb(dev, TW5864_INDIR_VIN_E(i), 0x07);
		/* to initiate auto format recognition */
		tw_indir_writeb(dev, TW5864_INDIR_VIN_F(i), 0xff);
	}

	tw_writel(TW5864_SEN_EN_CH, 0x000f);
	tw_writel(TW5864_H264EN_CH_EN, 0x000f);

	tw_writel(TW5864_H264EN_BUS0_MAP, 0x00000000);
	tw_writel(TW5864_H264EN_BUS1_MAP, 0x00001111);
	tw_writel(TW5864_H264EN_BUS2_MAP, 0x00002222);
	tw_writel(TW5864_H264EN_BUS3_MAP, 0x00003333);

	/*
	 * Quote from Intersil (manufacturer):
	 * 0x0038 is managed by HW, and by default it won't pass the pointer set
	 * at 0x0010. So if you don't do encoding, 0x0038 should stay at '3'
	 * (with 4 frames in buffer). If you encode one frame and then move
	 * 0x0010 to '1' for example, HW will take one more frame and set it to
	 * buffer #0, and then you should see 0x0038 is set to '0'.  There is
	 * only one HW encoder engine, so 4 channels cannot get encoded
	 * simultaneously. But each channel does have its own buffer (for
	 * original frames and reconstructed frames). So there is no problem to
	 * manage encoding for 4 channels at same time and no need to force
	 * I-frames in switching channels.
	 * End of quote.
	 *
	 * If we set 0x0010 (TW5864_ENC_BUF_PTR_REC1) to 0 (for any channel), we
	 * have no "rolling" (until we change this value).
	 * If we set 0x0010 (TW5864_ENC_BUF_PTR_REC1) to 0x3, it starts to roll
	 * continuously together with 0x0038.
	 */
	tw_writel(TW5864_ENC_BUF_PTR_REC1, 0x00ff);
	tw_writel(TW5864_PCI_INTTM_SCALE, 0);

	tw_writel(TW5864_INTERLACING, TW5864_DI_EN);
	tw_writel(TW5864_MASTER_ENB_REG, TW5864_PCI_VLC_INTR_ENB);
	tw_writel(TW5864_PCI_INTR_CTL,
		  TW5864_TIMER_INTR_ENB | TW5864_PCI_MAST_ENB |
		  TW5864_MVD_VLC_MAST_ENB);

	dev->encoder_busy = 0;

	dev->irqmask |= TW5864_INTR_VLC_DONE | TW5864_INTR_TIMER;
	tw5864_irqmask_apply(dev);

	tasklet_init(&dev->tasklet, tw5864_handle_frame_task,
		     (unsigned long)dev);

	for (i = 0; i < TW5864_INPUTS; i++) {
		dev->inputs[i].root = dev;
		dev->inputs[i].input_number = i;
		ret = tw5864_video_input_init(&dev->inputs[i], video_nr[i]);
		if (ret)
			goto input_init_fail;
	}

	return 0;

dma_alloc_fail:
	for (i = 0; i < H264_BUF_CNT; i++) {
		dma_free_coherent(&dev->pci->dev, H264_VLC_BUF_SIZE,
				  dev->h264_buf[i].vlc.addr,
				  dev->h264_buf[i].vlc.dma_addr);
		dma_free_coherent(&dev->pci->dev, H264_MV_BUF_SIZE,
				  dev->h264_buf[i].mv.addr,
				  dev->h264_buf[i].mv.dma_addr);
	}

	i = TW5864_INPUTS;

input_init_fail:
	for (; i >= 0; i--)
		tw5864_video_input_fini(&dev->inputs[i]);

	tasklet_kill(&dev->tasklet);

	return ret;
}

static int tw5864_video_input_init(struct tw5864_input *input, int video_nr)
{
	int ret;
	struct v4l2_ctrl_handler *hdl = &input->hdl;

	mutex_init(&input->lock);
	spin_lock_init(&input->slock);

	/* setup video buffers queue */
	INIT_LIST_HEAD(&input->active);
	input->vidq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	input->vidq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	input->vidq.io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
	input->vidq.ops = &tw5864_video_qops;
	input->vidq.mem_ops = &vb2_dma_contig_memops;
	input->vidq.drv_priv = input;
	input->vidq.gfp_flags = __GFP_DMA32;
	input->vidq.buf_struct_size = sizeof(struct tw5864_buf);
	input->vidq.lock = &input->lock;
	input->vidq.min_buffers_needed = 12;
	ret = vb2_queue_init(&input->vidq);
	if (ret)
		goto vb2_q_init_fail;

	input->vdev = tw5864_video_template;
	input->vdev.v4l2_dev = &input->root->v4l2_dev;
	input->vdev.lock = &input->lock;
	input->vdev.queue = &input->vidq;
	video_set_drvdata(&input->vdev, input);

	/* Initialize the device control structures */
	input->alloc_ctx = vb2_dma_contig_init_ctx(&input->root->pci->dev);
	if (IS_ERR(input->alloc_ctx)) {
		ret = PTR_ERR(input->alloc_ctx);
		goto vb2_dma_contig_init_ctx_fail;
	}

	v4l2_ctrl_handler_init(hdl, 6);
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 100);
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 128);
	/* NTSC only */
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops, V4L2_CID_HUE, -128, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 255, 1, GOP_SIZE);
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 28, 51, 1, QP_VALUE);
	v4l2_ctrl_new_std_menu(hdl, &tw5864_ctrl_ops,
			       V4L2_CID_DETECT_MD_MODE,
			       V4L2_DETECT_MD_MODE_THRESHOLD_GRID, 0,
			       V4L2_DETECT_MD_MODE_DISABLED);
	v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
			  V4L2_CID_DETECT_MD_GLOBAL_THRESHOLD,
			  tw5864_md_thresholds.min, tw5864_md_thresholds.max,
			  tw5864_md_thresholds.step, tw5864_md_thresholds.def);
	input->md_threshold_grid_ctrl =
		v4l2_ctrl_new_custom(hdl, &tw5864_md_thresholds, NULL);
	if (hdl->error) {
		ret = hdl->error;
		goto v4l2_ctrl_fail;
	}
	input->vdev.ctrl_handler = hdl;
	v4l2_ctrl_handler_setup(hdl);

	input->qp = QP_VALUE;
	input->gop = GOP_SIZE;
	input->frame_interval = 1;

	ret = video_register_device(&input->vdev, VFL_TYPE_GRABBER, video_nr);
	if (ret)
		goto v4l2_ctrl_fail;

	dev_info(&input->root->pci->dev, "Registered video device %s\n",
		 video_device_node_name(&input->vdev));

	return 0;

v4l2_ctrl_fail:
	v4l2_ctrl_handler_free(hdl);
	vb2_dma_contig_cleanup_ctx(input->alloc_ctx);
vb2_dma_contig_init_ctx_fail:
	vb2_queue_release(&input->vidq);
vb2_q_init_fail:
	mutex_destroy(&input->lock);

	return ret;
}

static void tw5864_video_input_fini(struct tw5864_input *dev)
{
	video_unregister_device(&dev->vdev);
	v4l2_ctrl_handler_free(&dev->hdl);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	vb2_queue_release(&dev->vidq);
}

void tw5864_video_fini(struct tw5864_dev *dev)
{
	int i;

	tasklet_kill(&dev->tasklet);

	for (i = 0; i < TW5864_INPUTS; i++)
		tw5864_video_input_fini(&dev->inputs[i]);

	for (i = 0; i < H264_BUF_CNT; i++) {
		dma_free_coherent(&dev->pci->dev, H264_VLC_BUF_SIZE,
				  dev->h264_buf[i].vlc.addr,
				  dev->h264_buf[i].vlc.dma_addr);
		dma_free_coherent(&dev->pci->dev, H264_MV_BUF_SIZE,
				  dev->h264_buf[i].mv.addr,
				  dev->h264_buf[i].mv.dma_addr);
	}
}

void tw5864_prepare_frame_headers(struct tw5864_input *input)
{
	struct tw5864_buf *vb = input->vb;
	u8 *dst;
	unsigned long dst_size;
	size_t dst_space;
	unsigned long flags;

	u8 *sl_hdr;
	unsigned long space_before_sl_hdr;

	if (!vb) {
		spin_lock_irqsave(&input->slock, flags);
		if (list_empty(&input->active)) {
			spin_unlock_irqrestore(&input->slock, flags);
			input->vb = NULL;
			return;
		}
		vb = list_first_entry(&input->active, struct tw5864_buf, list);
		list_del(&vb->list);
		spin_unlock_irqrestore(&input->slock, flags);
	}

	dst = vb2_plane_vaddr(&vb->vb.vb2_buf, 0);
	dst_size = vb2_plane_size(&vb->vb.vb2_buf, 0);
	dst_space = dst_size;

	/*
	 * Generate H264 headers:
	 * If this is first frame, put SPS and PPS
	 */
	if (input->h264_frame_seqno_in_gop == 0)
		tw5864_h264_put_stream_header(&dst, &dst_space, input->qp,
					      input->width, input->height);

	/* Put slice header */
	sl_hdr = dst;
	space_before_sl_hdr = dst_space;
	tw5864_h264_put_slice_header(&dst, &dst_space, input->h264_idr_pic_id,
				     input->h264_frame_seqno_in_gop,
				     &input->tail_nb_bits, &input->tail);
	input->vb = vb;
	input->buf_cur_ptr = dst;
	input->buf_cur_space_left = dst_space;
}

/*
 * Returns heuristic motion detection metric value from known components of
 * hardware-provided Motion Vector Data.
 */
static unsigned int tw5864_md_metric_from_mvd(u32 mvd)
{
	/*
	 * Format of motion vector data exposed by tw5864, according to
	 * manufacturer:
	 * mv_x 10 bits
	 * mv_y 10 bits
	 * non_zero_members 8 bits
	 * mb_type 3 bits
	 * reserved 1 bit
	 *
	 * non_zero_members: number of non-zero residuals in each macro block
	 * after quantization
	 *
	 * unsigned int reserved = mvd >> 31;
	 * unsigned int mb_type = (mvd >> 28) & 0x7;
	 * unsigned int non_zero_members = (mvd >> 20) & 0xff;
	 */
	unsigned int mv_y = (mvd >> 10) & 0x3ff;
	unsigned int mv_x = mvd & 0x3ff;

	/* heuristic: */
	mv_x &= 0x0f;
	mv_y &= 0x0f;

	return mv_y + mv_x;
}

static int tw5864_is_motion_triggered(struct tw5864_h264_frame *frame)
{
	struct tw5864_input *input = frame->input;
	u32 *mv = (u32 *)frame->mv.addr;
	int i;
	int detected = 0;
	unsigned int md_cells = MD_CELLS_HOR * MD_CELLS_VERT;

#ifdef DEBUG
	/* Stats */
	unsigned int max = 0;
	unsigned int min = UINT_MAX;
	unsigned int sum = 0;
	unsigned int cnt_above_thresh = 0;
#endif

	for (i = 0; i < md_cells; i++) {
		const u16 thresh = input->md_threshold_grid_values[i];
		const unsigned int metric = tw5864_md_metric_from_mvd(mv[i]);

		if (metric > thresh)
			detected = 1;

#ifdef DEBUG
		if (metric > thresh)
			cnt_above_thresh++;
		if (metric > max)
			max = metric;
		if (metric < min)
			min = metric;
		sum += metric;
#else
		if (detected)
			break;
#endif
	}
#ifdef DEBUG
	dev_dbg(&input->root->pci->dev,
		"input %d, frame md stats: min %u, max %u, avg %u, cells above threshold: %u\n",
		input->input_number, min, max, sum / md_cells,
		cnt_above_thresh);
#endif
	return detected;
}

#ifdef MD_DUMP
static void tw5864_md_dump(struct tw5864_input *input)
{
	struct tw5864_dev *dev = input->root;
	u32 *mv = (u32 *)dev->h264_mv_buf[dev->h264_buf_index].addr;
	int offset = 0;
	int i;

	if (input->h264_frame_seqno_in_gop) {
		offset = 0;
		for (i = 0; i < MD_CELLS_VERT; i++) {
			dev_dbg(&dev->pci->dev,
				"MVD [%02d]: %08x %08x %08x %08x   %08x %08x %08x %08x   %08x %08x %08x %08x   %08x %08x %08x %08x\n",
				i, mv[offset + 0], mv[offset + 1],
				mv[offset + 2], mv[offset + 3], mv[offset + 4],
				mv[offset + 5], mv[offset + 6], mv[offset + 7],
				mv[offset + 8], mv[offset + 9], mv[offset + 10],
				mv[offset + 11], mv[offset + 12],
				mv[offset + 13], mv[offset + 14],
				mv[offset + 15]
			       );
			offset += MD_CELLS_HOR;
		}
		offset = 0;
		for (i = 0; i < MD_CELLS_VERT; i++) {
			dev_dbg(&dev->pci->dev,
				"MD heur [%02d]: % 2x % 2x % 2x % 2x   % 2x % 2x % 2x % 2x   % 2x % 2x % 2x % 2x   % 2x % 2x % 2x % 2x\n",
				i, tw5864_md_metric_from_mvd(mv[offset + 0]),
				tw5864_md_metric_from_mvd(mv[offset + 1]),
				tw5864_md_metric_from_mvd(mv[offset + 2]),
				tw5864_md_metric_from_mvd(mv[offset + 3]),
				tw5864_md_metric_from_mvd(mv[offset + 4]),
				tw5864_md_metric_from_mvd(mv[offset + 5]),
				tw5864_md_metric_from_mvd(mv[offset + 6]),
				tw5864_md_metric_from_mvd(mv[offset + 7]),
				tw5864_md_metric_from_mvd(mv[offset + 8]),
				tw5864_md_metric_from_mvd(mv[offset + 9]),
				tw5864_md_metric_from_mvd(mv[offset + 10]),
				tw5864_md_metric_from_mvd(mv[offset + 11]),
				tw5864_md_metric_from_mvd(mv[offset + 12]),
				tw5864_md_metric_from_mvd(mv[offset + 13]),
				tw5864_md_metric_from_mvd(mv[offset + 14]),
				tw5864_md_metric_from_mvd(mv[offset + 15])
			       );
			offset += MD_CELLS_HOR;
		}
	}
}
#endif

static void tw5864_handle_frame_task(unsigned long data)
{
	struct tw5864_dev *dev = (struct tw5864_dev *)data;
	unsigned long flags;
	int batch_size = H264_BUF_CNT;

	spin_lock_irqsave(&dev->slock, flags);
	while (dev->h264_buf_r_index != dev->h264_buf_w_index && batch_size--) {
		spin_unlock_irqrestore(&dev->slock, flags);
		tw5864_handle_frame(&dev->h264_buf[dev->h264_buf_r_index]);
		spin_lock_irqsave(&dev->slock, flags);

		dev->h264_buf_r_index++;
		dev->h264_buf_r_index %= H264_BUF_CNT;
	}
	spin_unlock_irqrestore(&dev->slock, flags);
}

#ifdef DEBUG
static u32 checksum(u32 *data, int len)
{
	u32 val, count_len = len;

	val = *data++;
	while (((count_len >> 2) - 1) > 0) {
		val ^= *data++;
		count_len -= 4;
	}
	val ^= htonl((len >> 2));
	return val;
}
#endif

static void tw5864_handle_frame(struct tw5864_h264_frame *frame)
{
	struct tw5864_input *input = frame->input;
	struct tw5864_dev *dev = input->root;
	struct tw5864_buf *vb;
	struct vb2_v4l2_buffer *v4l2_buf;
	int frame_len = frame->vlc_len;
	unsigned long dst_size;
	unsigned long dst_space;
	int skip_bytes = 3;
	u8 *dst = input->buf_cur_ptr;
	u8 tail_mask, vlc_mask = 0;
	int i;
	u8 vlc_first_byte = ((u8 *)(frame->vlc.addr + skip_bytes))[0];
	unsigned long flags;
	int zero_run;
	u8 *src;
	u8 *src_end;

#ifdef DEBUG
	if (frame->checksum != checksum((u32 *)frame->vlc.addr, frame_len))
		dev_err(&dev->pci->dev,
			"Checksum of encoded frame doesn't match!\n");
#endif

	spin_lock_irqsave(&input->slock, flags);
	vb = input->vb;
	input->vb = NULL;
	spin_unlock_irqrestore(&input->slock, flags);

	v4l2_buf = to_vb2_v4l2_buffer(&vb->vb.vb2_buf);

	if (!vb) { /* Gone because of disabling */
		dev_dbg(&dev->pci->dev, "vb is empty, dropping frame\n");
		return;
	}

	dst_size = vb2_plane_size(&vb->vb.vb2_buf, 0);

	dst_space = input->buf_cur_space_left;
	frame_len -= skip_bytes;
	if (WARN_ON_ONCE(dst_space < frame_len)) {
		dev_err_once(&dev->pci->dev,
			     "Left space in vb2 buffer %lu is insufficient for frame length %d, writing truncated frame\n",
			     dst_space, frame_len);
		frame_len = dst_space;
	}

	for (i = 0; i < 8 - input->tail_nb_bits; i++)
		vlc_mask |= 1 << i;
	tail_mask = (~vlc_mask) & 0xff;

	dst[0] = (input->tail & tail_mask) | (vlc_first_byte & vlc_mask);
	skip_bytes++;
	frame_len--;
	dst++;
	dst_space--;

	/* H.264 startcode emulation prevention */
	if (dst_space < frame_len * 5 / 4) {
		dev_err(&dev->pci->dev,
			"Buffer is insufficient for H.264 startcode emulation prevention, dropping frame\n");
		return;
	}
	src = frame->vlc.addr + skip_bytes;
	src_end = src + frame_len;
	zero_run = 0;
	for (; src < src_end; src++) {
		if (zero_run < 2) {
			if (*src == 0)
				++zero_run;
			else
				zero_run = 0;
		} else {
			if ((*src & ~0x03) == 0)
				*dst++ = 0x03;
			zero_run = *src == 0;
		}
		*dst++ = *src;
	}
	/* dst_space is not actual at this point */

	vb2_set_plane_payload(&vb->vb.vb2_buf, 0,
			      dst - (u8*)vb2_plane_vaddr(&vb->vb.vb2_buf, 0));

	vb->vb.vb2_buf.timestamp = frame->timestamp;
	v4l2_buf->field = V4L2_FIELD_NONE;
	v4l2_buf->sequence = input->frame_seqno - 1;

	/* Check for motion flags */
	if (input->h264_frame_seqno_in_gop /* P-frame */ &&
	    tw5864_is_motion_triggered(frame)) {
		struct v4l2_event ev = {
			.type = V4L2_EVENT_MOTION_DET,
			.u.motion_det = {
				.flags =
					V4L2_EVENT_MD_FL_HAVE_FRAME_SEQ,
				.frame_sequence =
					v4l2_buf->sequence,
			},
		};

		v4l2_event_queue(&input->vdev, &ev);
	}

	vb2_buffer_done(&vb->vb.vb2_buf, VB2_BUF_STATE_DONE);

#ifdef MD_DUMP
	tw5864_md_dump(input);
#endif
}

v4l2_std_id tw5864_get_v4l2_std(enum tw5864_vid_std std)
{
	switch (std) {
	case STD_NTSC:
		return V4L2_STD_NTSC_M;
	case STD_PAL:
		return V4L2_STD_PAL_B;
	case STD_SECAM:
		return V4L2_STD_SECAM_B;
	case STD_INVALID:
		WARN_ON_ONCE(1);
		return 0;
	}
	return 0;
}

enum tw5864_vid_std tw5864_from_v4l2_std(v4l2_std_id v4l2_std)
{
	if (v4l2_std & V4L2_STD_NTSC)
		return STD_NTSC;
	if (v4l2_std & V4L2_STD_PAL)
		return STD_PAL;
	if (v4l2_std & V4L2_STD_SECAM)
		return STD_SECAM;
	WARN_ON_ONCE(1);
	return STD_AUTO;
}

static void tw5864_tables_upload(struct tw5864_dev *dev)
{
	int i;

	tw_writel(TW5864_VLC_RD, 0x1);
	for (i = 0; i < VLC_LOOKUP_TABLE_LEN; i++) {
		tw_writel((TW5864_VLC_STREAM_MEM_START + (i << 2)),
			  encoder_vlc_lookup_table[i]);
	}
	tw_writel(TW5864_VLC_RD, 0x0);

	for (i = 0; i < QUANTIZATION_TABLE_LEN; i++) {
		tw_writel((TW5864_QUAN_TAB + (i << 2)),
			  forward_quantization_table[i]);
	}

	for (i = 0; i < QUANTIZATION_TABLE_LEN; i++) {
		tw_writel((TW5864_QUAN_TAB + (i << 2)),
			  inverse_quantization_table[i]);
	}
}
