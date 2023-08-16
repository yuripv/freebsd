/*-
 * Copyright (c) 2015 Conrad Meyer <cem@FreeBSD.org>
 * Copyright (c) 2005 The FreeBSD Foundation
 * Copyright (c) 1996 Larry Ewing <lewing@isc.tamu.edu>
 * Copyright (c) 1988 Kirk McKusick <mckusick@FreeBSD.org>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <dev/vt/vt.h>

const unsigned int vt_logo_sprite_width = 80;
const unsigned int vt_logo_sprite_height = 80;

const unsigned char vt_beastie_vga16[] = {
	0x16, 0x00, 0x62, 0x16, 0x88, 0x03, 0x80, 0x16, 0x00, 0x23, 0x88, 0x80,
	0x00, 0x00, 0x08, 0x88, 0x16, 0x00, 0x21, 0x08, 0x16, 0x00, 0x06, 0x88,
	0x16, 0x00, 0x20, 0x80, 0x16, 0x00, 0x06, 0x08, 0x80, 0x16, 0x00, 0x1e,
	0x08, 0x16, 0x00, 0x08, 0x88, 0x16, 0x00, 0x1e, 0x08, 0x16, 0x00, 0x06,
	0x87, 0x00, 0x08, 0x16, 0x00, 0x1e, 0x80, 0x16, 0x00, 0x06, 0x77, 0x80,
	0x00, 0x80, 0x16, 0x00, 0x1d, 0x80, 0x16, 0x00, 0x06, 0x08, 0x00, 0x00,
	0x08, 0x16, 0x00, 0x1d, 0x80, 0x16, 0x00, 0x09, 0x08, 0x16, 0x00, 0x1c,
	0x08, 0x16, 0x00, 0x0a, 0x08, 0x16, 0x00, 0x1c, 0x08, 0x16, 0x00, 0x0b,
	0x80, 0x16, 0x00, 0x1b, 0x08, 0x16, 0x00, 0x0b, 0x80, 0x16, 0x00, 0x1b,
	0x08, 0x00, 0x08, 0x70, 0x16, 0x00, 0x03, 0x77, 0x70, 0x16, 0x00, 0x03,
	0x80, 0x16, 0x00, 0x1b, 0x08, 0x00, 0x87, 0x77, 0x00, 0x00, 0x07, 0xff,
	0xf7, 0x16, 0x00, 0x03, 0x80, 0x16, 0x00, 0x1b, 0x08, 0x08, 0x77, 0xff,
	0x00, 0x00, 0x7f, 0x77, 0xf7, 0x16, 0x00, 0x03, 0x80, 0x16, 0x00, 0x1b,
	0x08, 0x08, 0x70, 0x0f, 0x80, 0x00, 0xf7, 0x08, 0x7f, 0x70, 0x00, 0x00,
	0x80, 0x16, 0x00, 0x1b, 0x08, 0x08, 0x80, 0x07, 0x80, 0x00, 0xf8, 0x00,
	0x8f, 0x70, 0x00, 0x00, 0x80, 0x16, 0x00, 0x1b, 0x08, 0x08, 0x70, 0x07,
	0x88, 0x88, 0xf8, 0x00, 0x8f, 0x70, 0x00, 0x00, 0x80, 0x16, 0x00, 0x1b,
	0x08, 0x00, 0xf0, 0x06, 0x16, 0xe6, 0x03, 0x00, 0x8f, 0x16, 0x00, 0x03,
	0x80, 0x16, 0x00, 0x1b, 0x08, 0x00, 0x77, 0x16, 0x6e, 0x05, 0x77, 0x16,
	0x00, 0x03, 0x80, 0x16, 0x00, 0x1b, 0x08, 0x00, 0x06, 0x16, 0xe6, 0x06,
	0x16, 0x00, 0x03, 0x80, 0x16, 0x00, 0x1b, 0x08, 0x00, 0x16, 0x6e, 0x07,
	0x60, 0x00, 0x00, 0x08, 0x16, 0x00, 0x1b, 0x08, 0x80, 0x16, 0xe6, 0x07,
	0x60, 0x00, 0x00, 0x08, 0x16, 0x00, 0x1b, 0x08, 0x80, 0x16, 0x6e, 0x05,
	0x66, 0x66, 0x80, 0x08, 0x00, 0x00, 0x80, 0x16, 0x00, 0x1a, 0x08, 0x80,
	0x86, 0x16, 0xe6, 0x03, 0x16, 0x66, 0x03, 0x80, 0x08, 0x78, 0x00, 0x80,
	0x16, 0x00, 0x1a, 0x08, 0x80, 0x86, 0x16, 0x66, 0x05, 0x77, 0x70, 0x00,
	0x77, 0x00, 0x08, 0x16, 0x00, 0x1a, 0x08, 0x00, 0x87, 0x16, 0x66, 0x04,
	0x77, 0x77, 0x78, 0x00, 0x88, 0x00, 0x08, 0x16, 0x00, 0x1a, 0x08, 0x00,
	0x87, 0x76, 0x66, 0x66, 0x77, 0x77, 0xff, 0xf7, 0x16, 0x00, 0x03, 0x08,
	0x16, 0x00, 0x1a, 0x80, 0x08, 0xff, 0x16, 0x77, 0x04, 0x16, 0xff, 0x03,
	0x80, 0x16, 0x00, 0x03, 0x80, 0x16, 0x00, 0x19, 0x80, 0x07, 0xff, 0x16,
	0x77, 0x03, 0x7f, 0x16, 0xff, 0x03, 0x70, 0x16, 0x00, 0x03, 0x80, 0x16,
	0x00, 0x18, 0x08, 0x00, 0x8f, 0xff, 0xf7, 0x77, 0x77, 0x16, 0xff, 0x04,
	0xf0, 0x16, 0x00, 0x03, 0x08, 0x16, 0x00, 0x18, 0x80, 0x08, 0x7f, 0x16,
	0xff, 0x08, 0xf8, 0x16, 0x00, 0x04, 0x80, 0x16, 0x00, 0x16, 0x08, 0x00,
	0x08, 0x16, 0xff, 0x09, 0xf7, 0x16, 0x00, 0x04, 0x08, 0x16, 0x00, 0x16,
	0x08, 0x00, 0x08, 0x16, 0xff, 0x0a, 0x16, 0x00, 0x05, 0x80, 0x16, 0x00,
	0x15, 0x80, 0x00, 0x87, 0x16, 0xff, 0x0a, 0x80, 0x16, 0x00, 0x04, 0x08,
	0x16, 0x00, 0x14, 0x08, 0x00, 0x00, 0x87, 0x77, 0xff, 0xf7, 0x77, 0x16,
	0xff, 0x03, 0x16, 0x77, 0x03, 0x78, 0x16, 0x00, 0x04, 0x08, 0x16, 0x00,
	0x14, 0x08, 0x00, 0x00, 0x77, 0x7f, 0xff, 0xff, 0x7f, 0x16, 0xff, 0x04,
	0x77, 0x77, 0x78, 0x00, 0x80, 0x16, 0x00, 0x03, 0x80, 0x16, 0x00, 0x13,
	0x80, 0x00, 0x00, 0x7f, 0x16, 0xff, 0x09, 0xf7, 0x77, 0x00, 0x08, 0x80,
	0x00, 0x00, 0x80, 0x16, 0x00, 0x13, 0x80, 0x80, 0x08, 0x16, 0xff, 0x0b,
	0x77, 0x80, 0x00, 0x08, 0x00, 0x00, 0x08, 0x16, 0x00, 0x12, 0x08, 0x00,
	0x80, 0x07, 0x16, 0xff, 0x0c, 0x78, 0x00, 0x08, 0x80, 0x00, 0x08, 0x16,
	0x00, 0x12, 0x08, 0x08, 0x00, 0x8f, 0x16, 0xff, 0x0c, 0xf7, 0x08, 0x80,
	0x80, 0x00, 0x08, 0x16, 0x00, 0x12, 0x16, 0x08, 0x03, 0x7f, 0x16, 0xff,
	0x0c, 0xf7, 0x08, 0x80, 0x80, 0x00, 0x00, 0x80, 0x16, 0x00, 0x11, 0x80,
	0x08, 0x07, 0x16, 0xff, 0x0e, 0x80, 0x00, 0x08, 0x00, 0x00, 0x80, 0x16,
	0x00, 0x11, 0x80, 0x80, 0x0f, 0x16, 0xff, 0x0e, 0x70, 0x00, 0x08, 0x00,
	0x00, 0x80, 0x16, 0x00, 0x10, 0x08, 0x00, 0x80, 0x8f, 0x16, 0xff, 0x0e,
	0x70, 0x00, 0x08, 0x00, 0x00, 0x80, 0x16, 0x00, 0x10, 0x08, 0x08, 0x00,
	0x7f, 0x16, 0xff, 0x0e, 0x70, 0x00, 0x08, 0x00, 0x00, 0x08, 0x16, 0x00,
	0x10, 0x80, 0x08, 0x00, 0x16, 0xff, 0x05, 0x7f, 0x16, 0xff, 0x09, 0xf0,
	0x00, 0x08, 0x00, 0x00, 0x08, 0x16, 0x00, 0x0f, 0x08, 0x00, 0x08, 0x00,
	0x16, 0xff, 0x05, 0x7f, 0x16, 0xff, 0x09, 0xf0, 0x00, 0x08, 0x00, 0x00,
	0x08, 0x16, 0x00, 0x0f, 0x08, 0x00, 0x08, 0x08, 0x16, 0xff, 0x05, 0x7f,
	0x16, 0xff, 0x09, 0xf0, 0x00, 0x08, 0x00, 0x00, 0x08, 0x16, 0x00, 0x0f,
	0x08, 0x00, 0x08, 0x08, 0x16, 0xff, 0x05, 0x7f, 0x16, 0xff, 0x09, 0xf0,
	0x00, 0x08, 0x00, 0x00, 0x08, 0x16, 0x00, 0x0f, 0x08, 0x00, 0x00, 0x88,
	0x16, 0xff, 0x05, 0x7f, 0x16, 0xff, 0x09, 0xf0, 0x00, 0x08, 0x00, 0x00,
	0x08, 0x16, 0x00, 0x0f, 0x08, 0x00, 0x00, 0x08, 0x16, 0xff, 0x05, 0x7f,
	0x16, 0xff, 0x09, 0xf0, 0x88, 0x88, 0x80, 0x00, 0x08, 0x16, 0x00, 0x0f,
	0x08, 0x06, 0xe6, 0x00, 0x8f, 0x16, 0xff, 0x04, 0x7f, 0x16, 0xff, 0x09,
	0xf8, 0x00, 0x00, 0x08, 0x80, 0x08, 0x16, 0x00, 0x10, 0x6e, 0x6e, 0x60,
	0x08, 0x16, 0xff, 0x04, 0x7f, 0x16, 0xff, 0x08, 0xe6, 0xe0, 0x16, 0x00,
	0x03, 0x88, 0x80, 0x16, 0x00, 0x0f, 0x06, 0x16, 0xe6, 0x03, 0x00, 0x8f,
	0x16, 0xff, 0x0b, 0xfe, 0x6e, 0x60, 0x16, 0x00, 0x04, 0x60, 0x16, 0x00,
	0x0f, 0x16, 0x6e, 0x04, 0x60, 0x08, 0x16, 0xff, 0x0b, 0xf6, 0xe6, 0xe0,
	0x16, 0x00, 0x03, 0x06, 0xe6, 0x16, 0x00, 0x0c, 0x06, 0x16, 0xe6, 0x06,
	0xe0, 0x00, 0x8f, 0x16, 0xff, 0x0a, 0xfe, 0x6e, 0x60, 0x16, 0x00, 0x03,
	0x0e, 0x6e, 0x16, 0x00, 0x0c, 0x16, 0x6e, 0x08, 0x00, 0x08, 0x16, 0xff,
	0x0a, 0x76, 0xe6, 0xe6, 0x16, 0x00, 0x03, 0xe6, 0xe6, 0x16, 0x00, 0x0c,
	0x16, 0xe6, 0x08, 0xe0, 0x00, 0x8f, 0x16, 0xff, 0x08, 0xf7, 0x7e, 0x16,
	0x6e, 0x07, 0x16, 0x00, 0x0c, 0x16, 0x6e, 0x08, 0x60, 0x00, 0x08, 0x16,
	0xff, 0x08, 0xf7, 0x76, 0x16, 0xe6, 0x07, 0xe0, 0x16, 0x00, 0x0b, 0x16,
	0xe6, 0x09, 0x00, 0x00, 0x0f, 0x16, 0xff, 0x07, 0xf7, 0x7e, 0x16, 0x6e,
	0x08, 0x16, 0x00, 0x0b, 0x16, 0x6e, 0x09, 0x60, 0x00, 0x0f, 0x16, 0xff,
	0x07, 0xf7, 0x76, 0x16, 0xe6, 0x08, 0xe0, 0x16, 0x00, 0x0a, 0x16, 0xe6,
	0x09, 0xe0, 0x00, 0x8f, 0x16, 0xff, 0x07, 0xf7, 0x8e, 0x16, 0x6e, 0x09,
	0x16, 0x00, 0x0a, 0x16, 0x6e, 0x0a, 0x88, 0x16, 0xff, 0x08, 0x78, 0x86,
	0x16, 0xe6, 0x0a, 0x16, 0x00, 0x09, 0x16, 0xe6, 0x0a, 0xef, 0x16, 0xff,
	0x07, 0xf7, 0x80, 0x06, 0x16, 0x6e, 0x0a, 0x16, 0x00, 0x09, 0x16, 0x6e,
	0x0b, 0x16, 0xff, 0x07, 0x78, 0x00, 0x06, 0x16, 0xe6, 0x09, 0xe0, 0x16,
	0x00, 0x09, 0x16, 0xe6, 0x0b, 0x7f, 0x16, 0xff, 0x05, 0x78, 0x80, 0x00,
	0x06, 0x16, 0x6e, 0x09, 0x16, 0x00, 0x09, 0x0e, 0x16, 0x6e, 0x0a, 0x66,
	0x67, 0x16, 0xff, 0x04, 0x78, 0x80, 0x00, 0x00, 0x86, 0x16, 0xe6, 0x08,
	0xe0, 0x16, 0x00, 0x09, 0x06, 0x16, 0xe6, 0x0b, 0x60, 0x16, 0x00, 0x08,
	0x86, 0x16, 0x6e, 0x06, 0x66, 0x60, 0x16, 0x00, 0x0a, 0x0e, 0x16, 0x6e,
	0x0a, 0x66, 0x60, 0x16, 0x00, 0x08, 0x86, 0x16, 0xe6, 0x06, 0x60, 0x16,
	0x00, 0x0c, 0x16, 0xe6, 0x0b, 0x60, 0x16, 0x00, 0x08, 0x86, 0x16, 0x6e,
	0x04, 0x66, 0x66, 0x16, 0x00, 0x0f, 0x16, 0x66, 0x03, 0x16, 0x6e, 0x05,
	0x66, 0x60, 0x00, 0x16, 0x88, 0x05, 0x80, 0x00, 0x06, 0x66, 0x16, 0xe6,
	0x03, 0x66, 0x16, 0x00, 0x12, 0x16, 0x66, 0x04, 0xe6, 0xe6, 0x66, 0x88,
	0x88, 0x16, 0x00, 0x05, 0x08, 0x88, 0x86, 0x66, 0x6e, 0x6e, 0x66, 0x60,
	0x16, 0x00, 0x14, 0x06, 0x16, 0x66, 0x04, 0x16, 0x00, 0x09, 0x06, 0x16,
	0x66, 0x04, 0x16, 0x00, 0x16, 0x06, 0x66, 0x66, 0x60, 0x16, 0x00, 0x0a,
	0x16, 0x66, 0x03, 0x60, 0x16, 0x00, 0x82
};

const unsigned char vt_beastie2_vga16[] = {
	0x16, 0x00, 0x11, 0x04, 0x16, 0x00, 0x26, 0x04, 0x44, 0x16, 0x00, 0x26,
	0x44, 0x40, 0x16, 0x00, 0x25, 0x44, 0x44, 0x16, 0x00, 0x0b, 0x44, 0x16,
	0x00, 0x19, 0x04, 0x44, 0x40, 0x16, 0x00, 0x0b, 0x04, 0x40, 0x16, 0x00,
	0x18, 0x44, 0x44, 0x40, 0x16, 0x00, 0x0b, 0x04, 0x44, 0x16, 0x00, 0x17,
	0x04, 0x44, 0x44, 0x16, 0x00, 0x0d, 0x44, 0x40, 0x16, 0x00, 0x16, 0x16,
	0x44, 0x03, 0x16, 0x00, 0x03, 0x04, 0x04, 0x16, 0x00, 0x08, 0x44, 0x44,
	0x16, 0x00, 0x16, 0x16, 0x44, 0x03, 0x00, 0x04, 0x16, 0x44, 0x05, 0x16,
	0x00, 0x06, 0x44, 0x44, 0x16, 0x00, 0x15, 0x04, 0x44, 0x44, 0x40, 0x40,
	0x16, 0x44, 0x07, 0x40, 0x16, 0x00, 0x04, 0x44, 0x44, 0x40, 0x16, 0x00,
	0x14, 0x04, 0x16, 0x44, 0x06, 0x04, 0x04, 0x16, 0x44, 0x03, 0x04, 0x16,
	0x00, 0x03, 0x04, 0x44, 0x44, 0x40, 0x16, 0x00, 0x14, 0x04, 0x16, 0x44,
	0x07, 0x40, 0x16, 0x44, 0x04, 0x40, 0x00, 0x00, 0x16, 0x44, 0x03, 0x40,
	0x16, 0x00, 0x14, 0x04, 0x16, 0x44, 0x03, 0x84, 0x16, 0x44, 0x04, 0x04,
	0x16, 0x44, 0x04, 0x04, 0x16, 0x44, 0x04, 0x40, 0x16, 0x00, 0x14, 0x04,
	0x44, 0x44, 0x0f, 0xf8, 0x44, 0x48, 0x84, 0x16, 0x44, 0x0b, 0x40, 0x16,
	0x00, 0x14, 0x04, 0x44, 0x40, 0xff, 0xf8, 0x40, 0xff, 0xff, 0x16, 0x44,
	0x0b, 0x40, 0x16, 0x00, 0x14, 0x04, 0x44, 0x0f, 0xff, 0x74, 0x47, 0xff,
	0xff, 0x74, 0x16, 0x44, 0x0a, 0x40, 0x16, 0x00, 0x14, 0x04, 0x04, 0xff,
	0xff, 0x44, 0x7f, 0xff, 0xff, 0xf4, 0x16, 0x44, 0x0a, 0x16, 0x00, 0x15,
	0x04, 0x48, 0xff, 0xf7, 0x40, 0x16, 0xff, 0x03, 0xf6, 0x16, 0x44, 0x0a,
	0x16, 0x00, 0x16, 0x4f, 0xff, 0xf8, 0x47, 0x16, 0xff, 0x03, 0xf8, 0x16,
	0x44, 0x09, 0x40, 0x16, 0x00, 0x16, 0x07, 0x07, 0xf8, 0x0f, 0x16, 0xff,
	0x03, 0xf8, 0x16, 0x44, 0x08, 0x40, 0x40, 0x16, 0x00, 0x15, 0x04, 0x77,
	0x80, 0xf4, 0x78, 0x0f, 0xff, 0xff, 0xf8, 0x16, 0x44, 0x09, 0x16, 0x00,
	0x16, 0x04, 0x8f, 0x00, 0xf0, 0x8f, 0x88, 0xff, 0xff, 0xf8, 0x16, 0x44,
	0x08, 0x16, 0x00, 0x17, 0x04, 0x00, 0x00, 0x88, 0x0f, 0x00, 0xff, 0xff,
	0xf6, 0x16, 0x44, 0x07, 0x40, 0x16, 0x00, 0x17, 0x40, 0x00, 0x00, 0x48,
	0x07, 0x00, 0xff, 0xff, 0xf4, 0x16, 0x44, 0x06, 0x40, 0x16, 0x00, 0x18,
	0x44, 0x80, 0x08, 0x48, 0x00, 0x00, 0xff, 0xff, 0xf4, 0x16, 0x44, 0x06,
	0x16, 0x00, 0x18, 0x04, 0x44, 0x40, 0x04, 0x48, 0x00, 0x00, 0xff, 0xff,
	0x84, 0x16, 0x44, 0x06, 0x40, 0x16, 0x00, 0x17, 0x44, 0x44, 0x04, 0x00,
	0x48, 0x00, 0x07, 0xff, 0xff, 0x16, 0x44, 0x07, 0x40, 0x16, 0x00, 0x17,
	0x44, 0x40, 0x16, 0x44, 0x03, 0xf7, 0xff, 0xff, 0xf0, 0x16, 0x44, 0x07,
	0x40, 0x16, 0x00, 0x16, 0x04, 0x44, 0x40, 0x44, 0x44, 0x40, 0x0f, 0xff,
	0xf7, 0x00, 0x16, 0x44, 0x07, 0x40, 0x16, 0x00, 0x14, 0x6e, 0x00, 0x04,
	0x16, 0x44, 0x05, 0x40, 0x40, 0x16, 0x44, 0x08, 0x40, 0x40, 0x16, 0x00,
	0x14, 0x0e, 0xe0, 0x00, 0x44, 0x44, 0x04, 0x16, 0x44, 0x0d, 0x16, 0x00,
	0x15, 0x06, 0x66, 0x00, 0x16, 0x44, 0x03, 0x16, 0x40, 0x03, 0x16, 0x44,
	0x09, 0x04, 0x16, 0x00, 0x13, 0x60, 0x00, 0x00, 0x06, 0x60, 0x44, 0x44,
	0x04, 0x16, 0x44, 0x0c, 0x40, 0x16, 0x00, 0x13, 0x0e, 0xe0, 0x00, 0x00,
	0xe0, 0x04, 0x44, 0x40, 0x16, 0x44, 0x0b, 0x40, 0x40, 0x16, 0x00, 0x13,
	0x06, 0xee, 0x00, 0x00, 0xe0, 0x00, 0x04, 0x16, 0x44, 0x07, 0x40, 0x40,
	0x16, 0x44, 0x04, 0x16, 0x00, 0x14, 0x06, 0x06, 0xe6, 0x00, 0xe0, 0x00,
	0x00, 0x04, 0x04, 0x16, 0x44, 0x04, 0x40, 0x16, 0x44, 0x05, 0x16, 0x00,
	0x17, 0x6e, 0x6e, 0x60, 0x16, 0x00, 0x04, 0x16, 0x44, 0x03, 0x40, 0x16,
	0x44, 0x04, 0x40, 0x40, 0x16, 0x00, 0x13, 0x68, 0x60, 0x00, 0x00, 0x06,
	0xee, 0x60, 0x16, 0x00, 0x05, 0x40, 0x40, 0x16, 0x44, 0x03, 0x04, 0x16,
	0x00, 0x16, 0x0e, 0xe0, 0x00, 0x00, 0x6e, 0xe6, 0xe6, 0x04, 0x44, 0x44,
	0x00, 0x00, 0x16, 0x44, 0x04, 0x04, 0x44, 0x40, 0x40, 0x16, 0x00, 0x15,
	0x6e, 0x66, 0x6e, 0xe6, 0x66, 0xee, 0x04, 0x44, 0x44, 0x16, 0x00, 0x03,
	0x16, 0x40, 0x03, 0x44, 0x44, 0x40, 0x16, 0x00, 0x16, 0x06, 0x6e, 0xee,
	0x68, 0x00, 0x0e, 0x64, 0x44, 0x44, 0x16, 0x00, 0x03, 0x16, 0x44, 0x06,
	0x04, 0x16, 0x00, 0x1b, 0x60, 0x44, 0x40, 0x16, 0x00, 0x03, 0x16, 0x44,
	0x06, 0x40, 0x16, 0x00, 0x1a, 0x04, 0x44, 0x40, 0x16, 0x00, 0x03, 0x04,
	0x16, 0x44, 0x06, 0x04, 0x16, 0x00, 0x1a, 0x04, 0x16, 0x44, 0x05, 0x04,
	0x16, 0x44, 0x06, 0x40, 0x16, 0x00, 0x1a, 0x04, 0x16, 0x44, 0x04, 0x04,
	0x04, 0x44, 0x04, 0x16, 0x44, 0x03, 0x04, 0x40, 0x40, 0x16, 0x00, 0x19,
	0x04, 0x44, 0x44, 0x04, 0x40, 0x44, 0x04, 0x44, 0x44, 0x04, 0x44, 0x40,
	0x44, 0x44, 0x16, 0x00, 0x1b, 0x04, 0x40, 0x44, 0x04, 0x16, 0x44, 0x04,
	0x40, 0x40, 0x44, 0x44, 0x40, 0x40, 0x16, 0x00, 0x1a, 0x04, 0x16, 0x44,
	0x03, 0x40, 0x16, 0x44, 0x08, 0x16, 0x00, 0x1c, 0x04, 0x40, 0x44, 0x04,
	0x16, 0x44, 0x07, 0x40, 0x40, 0x16, 0x00, 0x1e, 0x16, 0x44, 0x09, 0x16,
	0x00, 0x1e, 0x04, 0x16, 0x44, 0x06, 0x40, 0x44, 0x44, 0x04, 0x16, 0x00,
	0x1d, 0x04, 0x16, 0x44, 0x06, 0x40, 0x44, 0x44, 0x40, 0x16, 0x00, 0x1e,
	0x16, 0x44, 0x06, 0x40, 0x44, 0x16, 0x00, 0x20, 0x04, 0x00, 0x16, 0x44,
	0x04, 0x04, 0x00, 0x04, 0x16, 0x00, 0x1f, 0x40, 0x16, 0x44, 0x05, 0x00,
	0x04, 0x16, 0x00, 0x1f, 0x04, 0x44, 0x00, 0x04, 0x04, 0x40, 0x40, 0x04,
	0x00, 0x40, 0x40, 0x16, 0x00, 0x1d, 0x04, 0x44, 0x44, 0x00, 0x40, 0x06,
	0x6e, 0x60, 0x04, 0x16, 0x00, 0x20, 0x16, 0x44, 0x04, 0x40, 0x6e, 0xe6,
	0x00, 0x40, 0x40, 0x16, 0x00, 0x1e, 0x16, 0x44, 0x05, 0x46, 0xee, 0x60,
	0x16, 0x00, 0x20, 0x16, 0x44, 0x05, 0x04, 0x6e, 0xee, 0x04, 0x04, 0x16,
	0x00, 0x1e, 0x04, 0x16, 0x44, 0x05, 0x06, 0x6e, 0xe0, 0x00, 0x04, 0x16,
	0x00, 0x1e, 0x16, 0x44, 0x06, 0x6e, 0x64, 0x04, 0x16, 0x00, 0x1f, 0x04,
	0x04, 0x00, 0x00, 0x16, 0x04, 0x04, 0x00, 0x04, 0x16, 0x00, 0x1e, 0x04,
	0x44, 0x04, 0x04, 0x16, 0x40, 0x04, 0x44, 0x40, 0x04, 0x16, 0x00, 0x1d,
	0x44, 0x84, 0x74, 0x86, 0x87, 0x84, 0x44, 0x04, 0x00, 0x44, 0x40, 0x04,
	0x16, 0x00, 0x1c, 0x87, 0xc7, 0x40, 0x77, 0x74, 0x04, 0x04, 0x80, 0x00,
	0x00, 0x44, 0x40, 0x16, 0x00, 0x1b, 0x7c, 0x7c, 0x84, 0x74, 0x84, 0x44,
	0x84, 0x48, 0x78, 0x40, 0x00, 0x00, 0x44, 0x44, 0x04, 0x16, 0x00, 0x16,
	0x40, 0x48, 0xc8, 0xc7, 0x44, 0x40, 0x84, 0x44, 0x76, 0x04, 0x48, 0x78,
	0x16, 0x00, 0x04, 0x04, 0x40, 0x44, 0x04, 0x16, 0x00, 0x10, 0x08, 0x77,
	0xff, 0x77, 0x84, 0x44, 0x04, 0x00, 0x00, 0x46, 0x48, 0x74, 0x04, 0x44,
	0x40, 0x80, 0x40, 0x16, 0x00, 0x04, 0x04, 0x16, 0x44, 0x05, 0x40, 0x40,
	0x16, 0x00, 0x0a, 0x87, 0x16, 0xff, 0x03, 0xf7, 0x70, 0x88, 0x77, 0x77,
	0x84, 0x04, 0x44, 0x40, 0x44, 0x04, 0x44, 0x44, 0x04, 0x16, 0x00, 0x07,
	0x04, 0x16, 0x44, 0x03, 0x04, 0x40, 0x16, 0x00, 0x08, 0x07, 0x88, 0x16,
	0xff, 0x03, 0x77, 0x87, 0x16, 0xff, 0x03, 0x77, 0x84, 0x04, 0x04, 0x44,
	0x44, 0x16, 0x40, 0x03, 0x80, 0x16, 0x00, 0x09, 0x04, 0x44, 0x44, 0x40,
	0x16, 0x00, 0x07, 0x0f, 0xff, 0x16, 0x88, 0x03, 0x80, 0x7f, 0x16, 0xff,
	0x03, 0xf7, 0x78, 0x16, 0x44, 0x03, 0x04, 0x04, 0x48, 0x87, 0x80, 0x16,
	0x00, 0x0b, 0x04, 0x44, 0x16, 0x00, 0x07, 0x08, 0x16, 0xff, 0x03, 0x77,
	0x88, 0x87, 0x16, 0xff, 0x03, 0x77, 0x77, 0x16, 0x04, 0x03, 0x40, 0x48,
	0x87, 0x77, 0x16, 0x00, 0x0d, 0x44, 0x40, 0x16, 0x00, 0x07, 0x88, 0x77,
	0xf7, 0x77, 0x8f, 0x77, 0x16, 0x88, 0x05, 0x80, 0x88, 0x88, 0x87, 0x77,
	0x77, 0x80, 0x16, 0x00, 0x0d, 0x44, 0x40, 0x16, 0x00, 0x0b, 0x07, 0x16,
	0xff, 0x03, 0x7f, 0x16, 0x77, 0x06, 0x78, 0x16, 0x00, 0x0a, 0x04, 0x40,
	0x16, 0x00, 0x03, 0x44, 0x40, 0x16, 0x00, 0x0c, 0x87, 0x7f, 0xff, 0xff,
	0x16, 0x77, 0x04, 0x88, 0x80, 0x16, 0x00, 0x0a, 0x04, 0x44, 0x44, 0x00,
	0x00, 0x04, 0x44, 0x40, 0x16, 0x00, 0x0e, 0x08, 0x08, 0x88, 0x08, 0x16,
	0x00, 0x0e, 0x16, 0x44, 0x06, 0x40, 0x16, 0x00, 0x20, 0x16, 0x44, 0x06,
	0x16, 0x00, 0x21, 0x16, 0x44, 0x05, 0x40, 0x16, 0x00, 0x22, 0x40, 0x40,
	0x16, 0x00, 0x08
};

const unsigned char vt_orb_vga16[] = {
	0x16, 0x00, 0x52, 0x04, 0x04, 0x16, 0x00, 0x0c, 0x16, 0x80, 0x03, 0x88,
	0x88, 0x16, 0x80, 0x04, 0x16, 0x00, 0x0b, 0x40, 0x40, 0x16, 0x00, 0x03,
	0x04, 0x44, 0x6c, 0xcc, 0x64, 0x16, 0x00, 0x08, 0x08, 0x08, 0x88, 0x77,
	0x16, 0x7f, 0x04, 0x77, 0x78, 0x88, 0x80, 0x80, 0x16, 0x00, 0x07, 0x04,
	0x6c, 0x6c, 0x44, 0x40, 0x00, 0x00, 0x04, 0x46, 0x4c, 0x77, 0x7c, 0xcc,
	0x40, 0x16, 0x00, 0x04, 0x08, 0x08, 0x88, 0x7f, 0x16, 0xff, 0x08, 0xf7,
	0x78, 0x80, 0x80, 0x16, 0x00, 0x05, 0x4c, 0xc7, 0xc7, 0xcc, 0x44, 0x44,
	0x00, 0x00, 0x44, 0x44, 0x46, 0xcc, 0xf7, 0xfc, 0x7c, 0x60, 0x00, 0x00,
	0x08, 0x08, 0x87, 0x16, 0xff, 0x0c, 0xf7, 0x88, 0x08, 0x16, 0x00, 0x03,
	0x6c, 0x16, 0xf7, 0x03, 0xc6, 0x44, 0x44, 0x00, 0x00, 0x16, 0x44, 0x03,
	0xc7, 0x7f, 0xf7, 0x77, 0xc7, 0x60, 0x00, 0x80, 0x77, 0x16, 0xff, 0x0e,
	0xf7, 0x70, 0x80, 0x08, 0x67, 0xff, 0x7f, 0x7f, 0x7c, 0xc4, 0x44, 0x44,
	0x00, 0x00, 0x16, 0x44, 0x03, 0x4c, 0xc7, 0xff, 0xf7, 0xf7, 0x77, 0x68,
	0x8f, 0x16, 0xff, 0x09, 0x16, 0xf7, 0x03, 0x16, 0xff, 0x05, 0x88, 0x67,
	0xff, 0x7f, 0xff, 0xff, 0x7c, 0x64, 0x44, 0x44, 0x00, 0x00, 0x04, 0x16,
	0x44, 0x03, 0x6c, 0xcf, 0x16, 0x7f, 0x04, 0x16, 0xff, 0x0d, 0x16, 0xf7,
	0x03, 0xff, 0xff, 0x7e, 0x16, 0xff, 0x04, 0x7c, 0xc6, 0x16, 0x44, 0x03,
	0x00, 0x00, 0x04, 0x16, 0x44, 0x03, 0x46, 0xcc, 0x7c, 0xf7, 0x16, 0xff,
	0x0d, 0x16, 0x7f, 0x03, 0x7c, 0xc6, 0xcc, 0x7e, 0x16, 0xff, 0x04, 0xf7,
	0x7c, 0x6c, 0x16, 0x44, 0x03, 0x00, 0x00, 0x04, 0x16, 0x44, 0x03, 0x4c,
	0x4c, 0xcc, 0xcf, 0x16, 0xff, 0x0f, 0xf7, 0xc4, 0x44, 0x4c, 0x16, 0xff,
	0x04, 0xf7, 0x7c, 0xcc, 0xc4, 0x16, 0x44, 0x03, 0x00, 0x00, 0x04, 0x16,
	0x44, 0x04, 0x4c, 0xcc, 0x7f, 0x16, 0xff, 0x0e, 0xf7, 0xf7, 0x44, 0x46,
	0xcf, 0x16, 0xff, 0x03, 0xf7, 0xf7, 0x77, 0xcc, 0x16, 0x44, 0x03, 0xc0,
	0x16, 0x00, 0x03, 0x16, 0x44, 0x04, 0xc4, 0xc7, 0x16, 0xff, 0x10, 0xf7,
	0x44, 0x47, 0x16, 0xff, 0x05, 0x77, 0xcc, 0xc4, 0x44, 0x44, 0x46, 0x40,
	0x16, 0x00, 0x03, 0xc4, 0x16, 0x44, 0x03, 0x4c, 0xcf, 0x16, 0xff, 0x11,
	0x44, 0x6f, 0x16, 0xff, 0x04, 0x7f, 0x77, 0x7c, 0x16, 0x44, 0x03, 0x4c,
	0x40, 0x16, 0x00, 0x03, 0x46, 0x16, 0x44, 0x03, 0xcc, 0x16, 0xff, 0x12,
	0xc6, 0xcf, 0x16, 0xff, 0x04, 0xf7, 0x7c, 0xc4, 0x16, 0x44, 0x03, 0x8c,
	0x16, 0x00, 0x04, 0x0c, 0x44, 0x44, 0x64, 0x7f, 0x16, 0xff, 0x12, 0x77,
	0xcf, 0x16, 0xff, 0x04, 0x7f, 0xcc, 0x16, 0x44, 0x03, 0x48, 0xc4, 0x16,
	0x00, 0x04, 0x08, 0xc4, 0x44, 0x4c, 0x16, 0xff, 0x13, 0xfc, 0x7e, 0x16,
	0xff, 0x03, 0xf7, 0xfc, 0x64, 0x16, 0x44, 0x03, 0x4c, 0xc0, 0x16, 0x00,
	0x05, 0xc6, 0x44, 0xc7, 0x16, 0xff, 0x13, 0x7f, 0x77, 0x16, 0xff, 0x03,
	0xfc, 0xcc, 0x16, 0x44, 0x04, 0x67, 0x60, 0x16, 0x00, 0x05, 0x8c, 0x84,
	0x7f, 0x16, 0xff, 0x09, 0x7f, 0xef, 0xff, 0x16, 0xf7, 0x06, 0xff, 0xff,
	0x77, 0xff, 0xff, 0x7c, 0xcc, 0xc4, 0x16, 0x44, 0x03, 0x48, 0xc7, 0x40,
	0x16, 0x00, 0x05, 0x07, 0xc7, 0x16, 0xff, 0x0c, 0x7f, 0x7f, 0xef, 0xef,
	0xfe, 0xff, 0xff, 0x7f, 0x7f, 0x76, 0xcf, 0xc7, 0xcc, 0xcc, 0x16, 0x44,
	0x04, 0x8c, 0x76, 0x16, 0x00, 0x06, 0x06, 0x7f, 0x7f, 0x16, 0xff, 0x08,
	0x16, 0x7f, 0x03, 0xfe, 0xfe, 0x16, 0xf7, 0x07, 0xfc, 0x6c, 0x7c, 0xcc,
	0xc4, 0x16, 0x44, 0x04, 0x67, 0xc8, 0x16, 0x00, 0x07, 0x7f, 0x16, 0xff,
	0x08, 0x7f, 0xf7, 0xf7, 0xfe, 0x7f, 0x7f, 0x7e, 0x7e, 0x16, 0xf7, 0x06,
	0x8c, 0xcc, 0xcc, 0xc4, 0x16, 0x44, 0x03, 0x48, 0xc7, 0x60, 0x16, 0x00,
	0x06, 0x08, 0xff, 0xff, 0xf7, 0x16, 0xff, 0x06, 0x16, 0xf7, 0x03, 0x77,
	0xfc, 0xfc, 0xf7, 0x77, 0x77, 0x16, 0x7f, 0x05, 0xc8, 0xcc, 0xc4, 0x16,
	0x44, 0x04, 0x8c, 0x7c, 0x78, 0x16, 0x00, 0x06, 0x0f, 0x7f, 0xff, 0x7f,
	0x16, 0xff, 0x05, 0x7f, 0x7f, 0x77, 0x7e, 0x7e, 0xcf, 0xcf, 0xce, 0x77,
	0x16, 0x7c, 0x04, 0x16, 0x77, 0x03, 0x4c, 0x4c, 0x16, 0x44, 0x04, 0xc7,
	0x77, 0x77, 0x16, 0x00, 0x06, 0x8f, 0xff, 0xff, 0x7f, 0x7f, 0x16, 0xff,
	0x04, 0xf7, 0xf7, 0x7e, 0x77, 0x77, 0x7c, 0xec, 0xcc, 0xcc, 0x16, 0xc6,
	0x04, 0xcc, 0xc7, 0xc7, 0x76, 0x16, 0x44, 0x04, 0x48, 0xc7, 0xc7, 0xf7,
	0x80, 0x16, 0x00, 0x05, 0x7f, 0x7f, 0xf7, 0x7f, 0x16, 0xff, 0x04, 0x7f,
	0x7f, 0xcf, 0xc7, 0xce, 0xcc, 0xc6, 0x16, 0xc4, 0x03, 0x4c, 0x4c, 0x44,
	0x4c, 0x48, 0x66, 0xc8, 0xc7, 0x84, 0x16, 0x44, 0x03, 0x8c, 0x7c, 0x67,
	0x7f, 0x80, 0x16, 0x00, 0x04, 0x08, 0x7f, 0xff, 0x77, 0xf7, 0x16, 0xff,
	0x04, 0xfe, 0x77, 0x7c, 0xec, 0xec, 0xc6, 0x4c, 0x46, 0x4c, 0x4c, 0x44,
	0x44, 0x4c, 0x16, 0x44, 0x03, 0x64, 0x86, 0xc8, 0x64, 0x44, 0x44, 0xc7,
	0x7c, 0x87, 0x7f, 0x70, 0x16, 0x00, 0x04, 0x08, 0xf7, 0xff, 0xe7, 0x7f,
	0x16, 0xff, 0x03, 0xfe, 0xfc, 0xfc, 0xec, 0xcc, 0x16, 0x44, 0x0b, 0x64,
	0x46, 0x44, 0x8c, 0x86, 0x84, 0x44, 0x6c, 0x46, 0x8c, 0x77, 0xf8, 0x16,
	0x00, 0x04, 0x0f, 0x7f, 0xf7, 0xcf, 0x7f, 0x7f, 0xff, 0xff, 0xed, 0xe7,
	0xce, 0xcc, 0x64, 0x64, 0xc4, 0x16, 0x44, 0x0b, 0x46, 0x16, 0x48, 0x03,
	0x84, 0x84, 0x88, 0x88, 0x77, 0xf8, 0x16, 0x00, 0x04, 0x0f, 0xf7, 0xfe,
	0x7e, 0x7f, 0xff, 0xff, 0xef, 0x7e, 0xcc, 0xcc, 0x64, 0x16, 0x44, 0x10,
	0x48, 0x68, 0x68, 0x88, 0x48, 0x68, 0xc7, 0x7f, 0x16, 0x00, 0x04, 0x7f,
	0x7f, 0x7c, 0x7c, 0xfe, 0xff, 0xff, 0xf7, 0xec, 0x7c, 0x64, 0x4c, 0x16,
	0x44, 0x10, 0x84, 0x48, 0x48, 0x68, 0x88, 0x88, 0x67, 0xf7, 0x80, 0x16,
	0x00, 0x03, 0x7f, 0x7f, 0xc7, 0xce, 0x7f, 0xff, 0xff, 0xe7, 0xec, 0x66,
	0x4c, 0x16, 0x44, 0x11, 0x16, 0x48, 0x03, 0x88, 0x68, 0x68, 0x67, 0x7f,
	0x80, 0x16, 0x00, 0x03, 0xf7, 0x7f, 0xcc, 0xec, 0xfe, 0xff, 0xff, 0x7e,
	0xc6, 0x16, 0x44, 0x15, 0x04, 0x84, 0x88, 0x88, 0x6c, 0x7f, 0x70, 0x00,
	0x00, 0x08, 0x7f, 0x77, 0x6c, 0xc7, 0xef, 0xff, 0xf7, 0xec, 0xc4, 0x64,
	0x16, 0x44, 0x13, 0x04, 0x84, 0x04, 0x84, 0x84, 0x86, 0x7f, 0x70, 0x00,
	0x00, 0x08, 0xf7, 0x7c, 0xc6, 0xec, 0xe7, 0xef, 0xe7, 0xc6, 0x64, 0x16,
	0x44, 0x15, 0x04, 0x84, 0x06, 0x86, 0x46, 0x7f, 0x78, 0x00, 0x00, 0x08,
	0xf7, 0x7c, 0x6c, 0xcc, 0xe7, 0x77, 0xcc, 0xc6, 0xc4, 0x16, 0x44, 0x13,
	0x04, 0x04, 0x16, 0x40, 0x04, 0x46, 0x7f, 0xf0, 0x00, 0x00, 0x08, 0xf7,
	0xcc, 0x6c, 0x6e, 0xce, 0xce, 0xc6, 0x66, 0x64, 0x16, 0x44, 0x17, 0x04,
	0x04, 0x46, 0xcf, 0x78, 0x00, 0x00, 0x07, 0xf7, 0x74, 0x6c, 0x6c, 0xcc,
	0x16, 0xc6, 0x03, 0x64, 0x16, 0x44, 0x13, 0x04, 0x44, 0x40, 0x40, 0x44,
	0x40, 0x46, 0x7f, 0xf8, 0x00, 0x00, 0x08, 0xf7, 0xc6, 0x46, 0x66, 0x6c,
	0x6c, 0x66, 0x46, 0x46, 0x16, 0x44, 0x17, 0x04, 0x04, 0x44, 0xef, 0x78,
	0x00, 0x00, 0x07, 0xf7, 0xc6, 0x46, 0x4c, 0x16, 0x46, 0x03, 0x66, 0x64,
	0x16, 0x44, 0x15, 0x04, 0x04, 0x44, 0x44, 0x46, 0x7f, 0xf8, 0x00, 0x00,
	0x07, 0xf7, 0xc4, 0x16, 0x64, 0x07, 0x16, 0x44, 0x17, 0x04, 0x04, 0x44,
	0xff, 0x78, 0x00, 0x00, 0x08, 0xf7, 0xc4, 0x16, 0x46, 0x07, 0x16, 0x44,
	0x15, 0x04, 0x04, 0x44, 0x44, 0x4c, 0xef, 0xf8, 0x00, 0x00, 0x07, 0xf7,
	0xc4, 0x64, 0x16, 0x46, 0x06, 0x16, 0x44, 0x14, 0x40, 0x44, 0x44, 0x04,
	0x44, 0x4c, 0xff, 0x78, 0x00, 0x00, 0x08, 0xf7, 0x84, 0x46, 0x44, 0x16,
	0x64, 0x06, 0x16, 0x44, 0x14, 0x04, 0x04, 0x44, 0x44, 0x47, 0xef, 0xf8,
	0x00, 0x00, 0x08, 0xff, 0xc4, 0x44, 0x46, 0x44, 0x16, 0x46, 0x04, 0x16,
	0x44, 0x14, 0x04, 0x16, 0x44, 0x04, 0xce, 0xff, 0x78, 0x00, 0x00, 0x08,
	0xff, 0x84, 0x46, 0x44, 0x16, 0x64, 0x05, 0x66, 0x16, 0x44, 0x13, 0x40,
	0x40, 0x44, 0x44, 0x46, 0xcf, 0xef, 0x70, 0x16, 0x00, 0x03, 0xff, 0xc4,
	0x16, 0x44, 0x04, 0x16, 0x46, 0x04, 0x16, 0x44, 0x12, 0x40, 0x16, 0x44,
	0x04, 0x4c, 0xee, 0xff, 0x70, 0x16, 0x00, 0x03, 0x7f, 0x74, 0x44, 0x44,
	0x16, 0x64, 0x07, 0x16, 0x44, 0x12, 0x04, 0x04, 0x44, 0x44, 0x4e, 0xef,
	0xef, 0x70, 0x16, 0x00, 0x03, 0x7f, 0x76, 0x16, 0x44, 0x05, 0x16, 0x46,
	0x04, 0x16, 0x44, 0x11, 0x40, 0x16, 0x44, 0x04, 0xee, 0xfe, 0xff, 0x80,
	0x16, 0x00, 0x03, 0x8f, 0xf8, 0x16, 0x44, 0x03, 0x16, 0x64, 0x06, 0x16,
	0x44, 0x12, 0x40, 0x44, 0x44, 0x4c, 0xee, 0xef, 0xf7, 0x80, 0x16, 0x00,
	0x03, 0x8f, 0xfc, 0x16, 0x44, 0x06, 0x16, 0x46, 0x03, 0x16, 0x44, 0x11,
	0x04, 0x04, 0x44, 0x44, 0x6e, 0xef, 0xef, 0xf7, 0x16, 0x00, 0x04, 0x07,
	0xf7, 0x16, 0x44, 0x04, 0x16, 0x64, 0x03, 0x16, 0x46, 0x03, 0x16, 0x44,
	0x13, 0x46, 0xce, 0xee, 0xff, 0x77, 0x16, 0x00, 0x04, 0x08, 0xff, 0x84,
	0x16, 0x44, 0x05, 0x46, 0x44, 0x16, 0x64, 0x03, 0x16, 0x44, 0x0f, 0x04,
	0x44, 0x44, 0x4c, 0xec, 0xef, 0xef, 0xf8, 0x16, 0x00, 0x04, 0x08, 0x7f,
	0x74, 0x16, 0x44, 0x03, 0x46, 0x46, 0x44, 0x16, 0x46, 0x04, 0x16, 0x44,
	0x12, 0xcc, 0xec, 0xef, 0xff, 0x70, 0x16, 0x00, 0x05, 0x7f, 0xf4, 0x16,
	0x44, 0x05, 0x46, 0x44, 0x44, 0x16, 0x64, 0x03, 0x16, 0x44, 0x10, 0x46,
	0xc6, 0xcc, 0xf7, 0xf7, 0x70, 0x16, 0x00, 0x05, 0x8f, 0xf7, 0x04, 0x16,
	0x44, 0x05, 0x16, 0x46, 0x05, 0x16, 0x44, 0x10, 0xc4, 0xc6, 0xcc, 0xfe,
	0xf7, 0x16, 0x00, 0x06, 0x07, 0xff, 0x16, 0x44, 0x05, 0x46, 0x16, 0x44,
	0x03, 0x16, 0x64, 0x04, 0x16, 0x44, 0x0c, 0x46, 0x66, 0x66, 0x6c, 0x67,
	0x7f, 0x77, 0x16, 0x00, 0x06, 0x08, 0xff, 0x74, 0x04, 0x16, 0x44, 0x04,
	0x16, 0x46, 0x05, 0x4c, 0x46, 0x16, 0x44, 0x0c, 0x64, 0xc4, 0xc4, 0xc6,
	0xc7, 0x7f, 0x70, 0x16, 0x00, 0x07, 0x8f, 0xf6, 0x16, 0x44, 0x09, 0x16,
	0x64, 0x05, 0x16, 0x44, 0x08, 0x46, 0xc6, 0xc6, 0x66, 0x6c, 0x6c, 0x77,
	0x77, 0x80, 0x16, 0x00, 0x07, 0x87, 0xf7, 0x40, 0x16, 0x44, 0x05, 0x16,
	0x46, 0x09, 0x16, 0x44, 0x06, 0xc6, 0xc6, 0x66, 0x66, 0xc6, 0xc6, 0xc7,
	0x77, 0xf7, 0x16, 0x00, 0x08, 0x08, 0xff, 0xc4, 0x16, 0x44, 0x09, 0x16,
	0x64, 0x05, 0x66, 0x66, 0x64, 0x66, 0x6c, 0x16, 0x66, 0x03, 0xc6, 0xc6,
	0x6c, 0x6c, 0xc7, 0x77, 0x78, 0x16, 0x00, 0x09, 0x7f, 0xf6, 0x04, 0x16,
	0x44, 0x05, 0x16, 0x46, 0x08, 0x64, 0x64, 0x6c, 0x66, 0x66, 0x16, 0xc6,
	0x03, 0x66, 0x66, 0xc6, 0x6c, 0x77, 0x7f, 0x80, 0x16, 0x00, 0x09, 0x07,
	0xf7, 0x16, 0x44, 0x0a, 0x16, 0x64, 0x04, 0x66, 0xc6, 0x64, 0x64, 0x16,
	0x66, 0x04, 0xc6, 0xc6, 0x6c, 0xc7, 0xe7, 0x77, 0x16, 0x00, 0x0a, 0x08,
	0x7f, 0x74, 0x16, 0x44, 0x06, 0x16, 0x46, 0x03, 0x44, 0x16, 0x64, 0x05,
	0xc6, 0x66, 0x66, 0x16, 0xc6, 0x03, 0x66, 0x6c, 0x6c, 0xe7, 0x77, 0x70,
	0x16, 0x00, 0x0b, 0x87, 0xfc, 0x16, 0x44, 0x09, 0x64, 0x16, 0x46, 0x03,
	0x4c, 0x16, 0x46, 0x04, 0x16, 0x66, 0x03, 0xc6, 0xc6, 0xce, 0xde, 0x77,
	0x80, 0x16, 0x00, 0x0b, 0x08, 0x7f, 0xc4, 0x16, 0x44, 0x07, 0x64, 0x44,
	0x16, 0x64, 0x05, 0x16, 0x66, 0x03, 0x46, 0xc6, 0xc6, 0x66, 0x6c, 0x77,
	0xe7, 0x78, 0x16, 0x00, 0x0d, 0x87, 0xfc, 0x16, 0x44, 0x08, 0x64, 0x44,
	0x46, 0x46, 0x4c, 0x16, 0x46, 0x04, 0x16, 0x66, 0x03, 0xc6, 0xce, 0x7e,
	0xd7, 0x80, 0x16, 0x00, 0x0d, 0x08, 0x7f, 0xc4, 0x16, 0x44, 0x08, 0x16,
	0x64, 0x06, 0xc6, 0x66, 0x4c, 0x46, 0xc6, 0xce, 0x7c, 0x77, 0x78, 0x16,
	0x00, 0x0f, 0x87, 0xf7, 0x16, 0x44, 0x0a, 0x16, 0x46, 0x06, 0x66, 0x66,
	0x6c, 0xec, 0xe7, 0xe7, 0x16, 0x00, 0x11, 0x8f, 0x7c, 0x16, 0x44, 0x07,
	0x16, 0x64, 0x09, 0x6c, 0xec, 0xe7, 0xc7, 0x80, 0x16, 0x00, 0x11, 0x08,
	0x77, 0xc6, 0x16, 0x44, 0x09, 0x16, 0x46, 0x04, 0x4c, 0x6c, 0xec, 0xee,
	0xc7, 0x78, 0x16, 0x00, 0x13, 0x07, 0x77, 0xcc, 0x16, 0x44, 0x06, 0x16,
	0x64, 0x05, 0xc4, 0xc6, 0xce, 0xce, 0xec, 0x78, 0x16, 0x00, 0x15, 0x88,
	0x77, 0xc6, 0xc4, 0xc4, 0x16, 0x44, 0x07, 0xc6, 0xc6, 0x16, 0xec, 0x03,
	0x78, 0x80, 0x16, 0x00, 0x16, 0x88, 0x77, 0xcc, 0x16, 0x4c, 0x06, 0x16,
	0x6c, 0x03, 0xec, 0xec, 0xe6, 0xe8, 0x80, 0x16, 0x00, 0x18, 0x88, 0x87,
	0xcc, 0x16, 0xc6, 0x03, 0x16, 0xcc, 0x03, 0xe6, 0xec, 0xe6, 0xe6, 0x78,
	0x16, 0x00, 0x1c, 0x88, 0x87, 0x16, 0xcc, 0x06, 0xc6, 0x76, 0x88, 0x16,
	0x00, 0x1f, 0x80, 0x16, 0x88, 0x05, 0x80, 0x16, 0x00, 0x88
};
