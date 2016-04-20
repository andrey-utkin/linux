/*
 *  TW5864 driver - H.264 headers generation functions
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

#include "tw5864.h"
#include "tw5864-bs.h"

static u8 marker[] = { 0x00, 0x00, 0x00, 0x01 };

/* log2 of max GOP size, taken 8 as V4L2-advertised max GOP size is 255 */
#define i_log2_max_frame_num 8
#define i_log2_max_poc_lsb i_log2_max_frame_num

static int tw5864_h264_gen_sps_rbsp(u8 *buf, size_t size, int width, int height)
{
	struct bs bs, *s;
	const int i_mb_width = width / 16;
	const int i_mb_height = height / 16;

	s = &bs;
	bs_init(s, buf, size);
	bs_write(s, 8, 0x42 /* profile == 66, baseline */);
	bs_write(s, 8, 1 /* constraints */);
	bs_write(s, 8, 0x1E /* level */);
	bs_write_ue(s, 0 /* SPS id */);
	bs_write_ue(s, i_log2_max_frame_num - 4);
	bs_write_ue(s, 0 /* i_poc_type */);
	bs_write_ue(s, i_log2_max_poc_lsb - 4);

	bs_write_ue(s, 1 /* i_num_ref_frames */);
	bs_write(s, 1, 0 /* b_gaps_in_frame_num_value_allowed */);
	bs_write_ue(s, i_mb_width - 1);
	bs_write_ue(s, i_mb_height - 1);
	bs_write(s, 1, 1 /* b_frame_mbs_only */);
	bs_write(s, 1, 0 /* b_direct8x8_inference */);
	bs_write(s, 1, 0);
	bs_write(s, 1, 0);
	bs_rbsp_trailing(s);
	return bs_len(s);
}

static int tw5864_h264_gen_pps_rbsp(u8 *buf, size_t size, int qp)
{
	struct bs bs, *s;

	s = &bs;
	bs_init(s, buf, size);
	bs_write_ue(s, 0 /* PPS id */);
	bs_write_ue(s, 0 /* SPS id */);
	bs_write(s, 1, 0 /* b_cabac */);
	bs_write(s, 1, 0 /* b_pic_order */);
	bs_write_ue(s, (1 /* i_num_slice_groups */) - 1);
	bs_write_ue(s, (1 /* i_num_ref_idx_l0_active */) - 1);
	bs_write_ue(s, (1 /* i_num_ref_idx_l1_active */) - 1);
	bs_write(s, 1, 0 /* b_weighted_pred */);
	bs_write(s, 2, 0 /* b_weighted_bipred */);
	bs_write_se(s, qp - 26);
	bs_write_se(s, qp - 26);
	bs_write_se(s, 0 /* i_chroma_qp_index_offset */);
	bs_write(s, 1, 0 /* b_deblocking_filter_control */);
	bs_write(s, 1, 0 /* b_constrained_intra_pred */);
	bs_write(s, 1, 0 /* b_redundant_pic_cnt */);
	bs_rbsp_trailing(s);
	return bs_len(s);
}

static int tw5864_h264_gen_slice_head(u8 *buf, size_t size,
				      unsigned int idr_pic_id,
				      unsigned int frame_seqno_in_gop,
				      int *tail_nb_bits, u8 *tail)
{
	struct bs bs, *s;
	int is_i_frame = frame_seqno_in_gop == 0;
	int i_poc_lsb = frame_seqno_in_gop;

	s = &bs;
	bs_init(s, buf, size);
	bs_write_ue(s, 0 /* i_first_mb */);
	bs_write_ue(s, is_i_frame ? 2 : 5 /* slice type - I or P */);
	bs_write_ue(s, 0 /* PPS id */);
	bs_write(s, i_log2_max_frame_num, frame_seqno_in_gop);
	if (is_i_frame)
		bs_write_ue(s, idr_pic_id);

	bs_write(s, i_log2_max_poc_lsb, i_poc_lsb);

	if (!is_i_frame)
		bs_write1(s, 0 /*b_num_ref_idx_override */);

	/* ref pic list reordering */
	if (!is_i_frame)
		bs_write1(s, 0 /* b_ref_pic_list_reordering_l0 */);

	if (is_i_frame) {
		bs_write1(s, 0); /* no output of prior pics flag */
		bs_write1(s, 0); /* long term reference flag */
	} else {
		bs_write1(s, 0); /* adaptive_ref_pic_marking_mode_flag */
	}

	bs_write_se(s, 0 /* i_qp_delta */);

	if (s->i_left != 8) {
		*tail = ((s->p[0]) << s->i_left);
		*tail_nb_bits = 8 - s->i_left;
	} else {
		*tail = 0;
		*tail_nb_bits = 0;
	}

	return bs_len(s);
}

void tw5864_h264_put_stream_header(u8 **buf, size_t *space_left, int qp,
				   int width, int height)
{
	int nal_len;

	/* SPS */
	WARN_ON_ONCE(*space_left < 4);
	memcpy(*buf, marker, sizeof(marker));
	*buf += 4;
	*space_left -= 4;

	**buf = 0x67; /* SPS NAL header */
	*buf += 1;
	*space_left -= 1;

	nal_len = tw5864_h264_gen_sps_rbsp(*buf, *space_left, width, height);
	*buf += nal_len;
	*space_left -= nal_len;

	/* PPS */
	WARN_ON_ONCE(*space_left < 4);
	memcpy(*buf, marker, sizeof(marker));
	*buf += 4;
	*space_left -= 4;

	**buf = 0x68; /* PPS NAL header */
	*buf += 1;
	*space_left -= 1;

	nal_len = tw5864_h264_gen_pps_rbsp(*buf, *space_left, qp);
	*buf += nal_len;
	*space_left -= nal_len;
}

void tw5864_h264_put_slice_header(u8 **buf, size_t *space_left,
				  unsigned int idr_pic_id,
				  unsigned int frame_seqno_in_gop,
				  int *tail_nb_bits, u8 *tail)
{
	int nal_len;

	WARN_ON_ONCE(*space_left < 4);
	memcpy(*buf, marker, sizeof(marker));
	*buf += 4;
	*space_left -= 4;

	/* Frame NAL header */
	**buf = (frame_seqno_in_gop == 0) ? 0x25 : 0x21;
	*buf += 1;
	*space_left -= 1;

	nal_len = tw5864_h264_gen_slice_head(*buf, *space_left, idr_pic_id,
					     frame_seqno_in_gop, tail_nb_bits,
					     tail);
	*buf += nal_len;
	*space_left -= nal_len;
}
