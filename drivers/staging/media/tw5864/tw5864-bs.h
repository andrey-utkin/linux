/*
 *  TW5864 driver - Exp-Golomb code functions
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

struct bs {
	u8 *p_start;
	u8 *p;
	u8 *p_end;
	int i_left; /* number of available bits */
};

static inline void bs_init(struct bs *s, void *buf, int size)
{
	s->p_start = buf;
	s->p = buf;
	s->p_end = s->p + size;
	s->i_left = 8;
}

static inline int bs_pos(struct bs *s)
{
	return 8 * (s->p - s->p_start) + 8 - s->i_left;
}

static inline bool bs_eof(struct bs *s)
{
	return s->p >= s->p_end;
}

static inline int bs_len(struct bs *s)
{
	return s->p - s->p_start;
}

static inline int bs_left(struct bs *s)
{
	return 8 - s->i_left;
}

static inline void bs_direct_write(struct bs *s, u8 value)
{
	*s->p = value;
	s->p++;
	s->i_left = 8;
}

static inline void bs_write(struct bs *s, int i_count, u32 i_bits)
{
	if (s->p >= s->p_end - 4)
		return;
	while (i_count > 0) {
		if (i_count < 32)
			i_bits &= (1 << i_count) - 1;
		if (i_count < s->i_left) {
			*s->p = (*s->p << i_count) | i_bits;
			s->i_left -= i_count;
			break;
		}
		*s->p = (*s->p << s->i_left) | (i_bits >> (i_count -
							   s->i_left));
		i_count -= s->i_left;
		s->p++;
		s->i_left = 8;
	}
}

static inline void bs_write1(struct bs *s, u32 i_bit)
{
	if (s->p < s->p_end) {
		*s->p <<= 1;
		*s->p |= i_bit;
		s->i_left--;
		if (s->i_left == 0) {
			s->p++;
			s->i_left = 8;
		}
	}
}

static inline void bs_align_0(struct bs *s)
{
	if (s->i_left != 8) {
		*s->p <<= s->i_left;
		s->i_left = 8;
		s->p++;
	}
}

static inline void bs_sh_align(struct bs *s)
{
	if (s->i_left != 8) {
		*s->p <<= s->i_left;
		s->i_left = 8;
	}
}

static inline void bs_align_1(struct bs *s)
{
	if (s->i_left != 8) {
		*s->p <<= s->i_left;
		*s->p |= (1 << s->i_left) - 1;
		s->i_left = 8;
		s->p++;
	}
}

static inline void bs_align(struct bs *s)
{
	bs_align_0(s);
}

/* golomb functions */
static inline void bs_write_ue(struct bs *s, u32 val)
{
	if (val == 0) {
		bs_write1(s, 1);
	} else {
		val++;
		bs_write(s, 2 * fls(val) - 1, val);
	}
}

static inline void bs_write_se(struct bs *s, int val)
{
	bs_write_ue(s, val <= 0 ? -val * 2 : val * 2 - 1);
}

static inline void bs_write_te(struct bs *s, int x, int val)
{
	if (x == 1)
		bs_write1(s, 1 & ~val);
	else if (x > 1)
		bs_write_ue(s, val);
}

static inline void bs_rbsp_trailing(struct bs *s)
{
	bs_write1(s, 1);
	if (s->i_left != 8)
		bs_write(s, s->i_left, 0x00);
}
