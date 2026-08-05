/*
 * retro-overlay socket protocol — reference header, protocol version 1.
 *
 * Copyright (c) 2026 Neil Graham
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This header is intentionally MIT-licensed so it can be vendored into
 * consumers of any license (including GPLv3). PROTOCOL.md in the
 * retro-overlay crate is the normative specification; on any conflict, the
 * document wins.
 *
 * Transport: SOCK_SEQPACKET Unix socket, default path
 * /run/retro-stream/retro-overlay.sock. One message per datagram, packed
 * little-endian structs, each preceded by rovl_hdr. dma-buf fds travel only
 * in SURFACE_ADD messages via SCM_RIGHTS (one fd per message).
 *
 * Region geometry is in the encoder's 16x16-pixel macroblock grid (both
 * h264 and h265 VEPU580 sessions use 16 px OSD units). The surface pixel
 * format is an 8-bit palette index, LINEAR raster, row stride =
 * width-in-MBs * 16 bytes — NOT macroblock-tiled (empirically verified on
 * RK3588, kernel 6.1.43-15-rk2312). Palette entries are packed
 * (alpha<<24)|(v<<16)|(u<<8)|y with FULL-SWING YUV; the encoder maps them
 * to limited swing (Y_out = 16 + Y*219/255).
 */

#ifndef RETRO_OVERLAY_PROTOCOL_H
#define RETRO_OVERLAY_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROVL_PROTO_VERSION 1
#define ROVL_PROTO_VERSION_MIN 1
#define ROVL_MAGIC 0x524f564cu /* "ROVL" */
#define ROVL_SOCKET_DEFAULT "/run/retro-stream/retro-overlay.sock"
#define ROVL_MAX_MSG_SIZE 4096
#define ROVL_MAX_REGIONS 8   /* VEPU580 hardware limit */
#define ROVL_MAX_SURFACES 4  /* daemon currently rotates 3 */
#define ROVL_PALETTE_SIZE 256

enum rovl_msg_type {
  ROVL_MSG_HELLO = 1,       /* client -> daemon */
  ROVL_MSG_HELLO_ACK = 2,   /* daemon -> client */
  ROVL_MSG_ERROR = 3,       /* daemon -> client, connection closes after */
  ROVL_MSG_SINK_INFO = 4,   /* sink -> daemon: encode geometry, once + on change */
  ROVL_MSG_SURFACE_ADD = 5, /* daemon -> sink, x surface count, carries fd */
  ROVL_MSG_PALETTE = 6,     /* daemon -> sink */
  ROVL_MSG_PRESENT = 7,     /* daemon -> sink */
  ROVL_MSG_HIDE = 8,        /* daemon -> sink: show nothing (attach no side data) */
  ROVL_MSG_MENU_TOGGLE = 9, /* control -> daemon: Guide double-press detected */
  ROVL_MSG_MENU_STATE = 10, /* daemon -> every control peer: menu open/closed */
  ROVL_MSG_INPUT_STATE = 11,/* control -> daemon: pad state while menu open */
  ROVL_MSG_RELOAD = 12,     /* control -> daemon: re-read [overlay] config */
  ROVL_MSG_NOTIFY = 13,     /* control -> daemon: show a toast */
  ROVL_MSG_STATUS_GET = 14, /* control -> daemon, empty body */
  ROVL_MSG_STATUS = 15,     /* daemon -> control, JSON body */
};

enum rovl_role {
  ROVL_ROLE_SINK = 1,    /* receives surfaces/regions (Sunshine); at most one */
  ROVL_ROLE_CONTROL = 2, /* menu/notify/config control; any number */
};

enum rovl_error_code {
  ROVL_ERR_BUSY = 1,    /* a sink is already connected */
  ROVL_ERR_VERSION = 2, /* no protocol version overlap */
  ROVL_ERR_INVALID = 3, /* malformed or out-of-state message */
  ROVL_ERR_INTERNAL = 4,
};

/* NOTIFY severity */
#define ROVL_SEVERITY_INFO 0
#define ROVL_SEVERITY_WARN 1
#define ROVL_SEVERITY_ERROR 2

#if defined(__GNUC__) || defined(__clang__)
#define ROVL_PACKED __attribute__((packed))
#else
#error "define ROVL_PACKED for this compiler"
#endif

/* Every message begins with this header. hdr_flags reserved, must be 0. */
struct ROVL_PACKED rovl_hdr {
  uint16_t type; /* enum rovl_msg_type */
  uint16_t hdr_flags;
};

struct ROVL_PACKED rovl_hello {
  struct rovl_hdr hdr;
  uint32_t magic; /* ROVL_MAGIC */
  uint16_t ver_min;
  uint16_t ver_max;
  uint8_t role; /* enum rovl_role */
  uint8_t reserved[3];
  char name[32]; /* NUL-padded client identity, for daemon logs */
};

struct ROVL_PACKED rovl_hello_ack {
  struct rovl_hdr hdr;
  uint16_t version; /* the version this connection speaks */
  uint16_t reserved;
};

struct ROVL_PACKED rovl_error {
  struct rovl_hdr hdr;
  uint16_t code; /* enum rovl_error_code */
  uint16_t reserved;
};

/*
 * Sink -> daemon after HELLO_ACK, and again whenever the encode session's
 * geometry changes. The daemon (re)allocates surfaces to match and replies
 * with the full SURFACE_ADD set, PALETTE, then PRESENT or HIDE.
 */
struct ROVL_PACKED rovl_sink_info {
  struct rovl_hdr hdr;
  uint16_t width;   /* encoded frame width in pixels */
  uint16_t height;  /* encoded frame height in pixels */
  uint8_t ten_bit;  /* 1 = 10-bit (NV15/PQ) session: daemon picks PQ palette */
  uint8_t reserved[3];
};

/*
 * One dma-buf surface. Accompanied by exactly one fd via SCM_RIGHTS.
 * The surface is an 8-bit palette-index LINEAR raster sized for the full
 * frame's MB grid: stride bytes per row, height_mb*16 rows. The daemon
 * never writes a surface while it is the one referenced by the latest
 * PRESENT (triple-buffer rotation).
 */
struct ROVL_PACKED rovl_surface_add {
  struct rovl_hdr hdr;
  uint8_t index; /* 0..ROVL_MAX_SURFACES-1 */
  uint8_t reserved[3];
  uint32_t size;   /* dma-buf size in bytes */
  uint32_t stride; /* row stride in bytes = width_mb * 16 */
  uint16_t width;  /* pixels covered, = width_mb * 16 */
  uint16_t height; /* pixels covered, = height_mb * 16 */
};

/*
 * Full palette replacement. entries are (alpha<<24)|(v<<16)|(u<<8)|y,
 * full-swing YUV (see file preamble). serial increments on every change;
 * the sink passes it downstream so the encoder can apply the palette only
 * when it actually changed.
 */
struct ROVL_PACKED rovl_palette {
  struct rovl_hdr hdr;
  uint32_t serial;
  uint32_t entries[ROVL_PALETTE_SIZE];
};

struct ROVL_PACKED rovl_region {
  uint16_t start_mb_x; /* 16-px units */
  uint16_t start_mb_y;
  uint16_t num_mb_x;
  uint16_t num_mb_y;
  uint32_t buf_offset; /* bytes into the surface; 16-byte aligned */
  uint8_t enable;
  uint8_t reserved[3];
};

/*
 * Show the given regions of one surface. Replaces the previous PRESENT
 * atomically at the next encoded frame. serial increments per PRESENT.
 */
struct ROVL_PACKED rovl_present {
  struct rovl_hdr hdr;
  uint64_t serial;
  uint8_t surface_index;
  uint8_t num_region; /* 1..ROVL_MAX_REGIONS */
  uint16_t reserved;
  struct rovl_region region[ROVL_MAX_REGIONS];
};

/* Bare-header messages: HIDE, MENU_TOGGLE, RELOAD, STATUS_GET. */

struct ROVL_PACKED rovl_menu_state {
  struct rovl_hdr hdr;
  uint8_t open; /* 1 = menu visible; controller must gate pad input */
  uint8_t reserved[3];
};

/*
 * Pad state forwarded by the controller daemon while the menu is open.
 * Mirrors the Moonlight/XInput wire layout used by retro-controller's
 * ControllerState (GUIDE = 0x0400, A = 0x1000, B = 0x2000, ...).
 */
struct ROVL_PACKED rovl_input_state {
  struct rovl_hdr hdr;
  uint16_t buttons;
  uint8_t left_trigger;
  uint8_t right_trigger;
  int16_t left_stick_x;
  int16_t left_stick_y;
  int16_t right_stick_x;
  int16_t right_stick_y;
  uint32_t seq; /* monotonically increasing per send */
};

struct ROVL_PACKED rovl_notify {
  struct rovl_hdr hdr;
  uint8_t severity; /* ROVL_SEVERITY_* */
  uint8_t reserved[3];
  char text[160]; /* NUL-terminated UTF-8, truncated by the daemon to fit */
};

/*
 * ---- Encoder side-data ABI (not a socket message) ----------------------
 *
 * Payload of the AV_FRAME_DATA_RKMPP_OSD frame side data attached by the
 * Sunshine client and consumed by the ffmpeg-rockchip rkmppenc.c patch. A
 * byte-identical copy of this struct lives inside that patch (rkmppenc.h)
 * because the ffmpeg build cannot include this header — keep them in sync
 * the same way retro-dongle keeps wire.c and wire.rs in sync.
 *
 * The fd is NOT dup'd per frame: it stays owned by the Sunshine client and
 * is valid for the lifetime of the surface. The encoder imports it once per
 * distinct (fd, size) and caches the MppBuffer.
 */
struct ROVL_PACKED rovl_osd_side_data {
  int32_t fd;        /* dma-buf of the presented surface */
  uint32_t size;     /* dma-buf size in bytes */
  uint32_t num_region;
  struct ROVL_PACKED {
    uint32_t enable;
    uint32_t start_mb_x;
    uint32_t start_mb_y;
    uint32_t num_mb_x;
    uint32_t num_mb_y;
    uint32_t buf_offset;
  } region[ROVL_MAX_REGIONS];
  uint32_t palette_serial; /* apply palette below when this changes */
  uint32_t palette[ROVL_PALETTE_SIZE];
};

#ifdef __cplusplus
}
#endif

#endif /* RETRO_OVERLAY_PROTOCOL_H */
