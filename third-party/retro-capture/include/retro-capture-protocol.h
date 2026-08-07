/*
 * retro-capture socket protocol — reference header, protocol version 1.
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
 * retro-capture repository is the normative specification; on any conflict,
 * the document wins.
 *
 * Transport: SOCK_SEQPACKET Unix socket, default path
 * /run/retro-stream/retro-capture.sock. One message per datagram, packed
 * little-endian structs, each preceded by rcap_hdr. dma-buf fds travel only
 * in POOL_ADD messages via SCM_RIGHTS (one fd per message).
 */

#ifndef RETRO_CAPTURE_PROTOCOL_H
#define RETRO_CAPTURE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RCAP_PROTO_VERSION 3
#define RCAP_PROTO_VERSION_MIN 1 /* daemon still speaks v1 to old consumers */
#define RCAP_MAGIC 0x52434150u /* "RCAP" */
#define RCAP_SOCKET_DEFAULT "/run/retro-stream/retro-capture.sock"
#define RCAP_MAX_MSG_SIZE 4096

enum rcap_msg_type {
  RCAP_MSG_HELLO = 1,      /* client -> daemon */
  RCAP_MSG_HELLO_ACK = 2,  /* daemon -> client */
  RCAP_MSG_ERROR = 3,      /* daemon -> client, connection closes after */
  RCAP_MSG_SETUP = 4,      /* consumer -> daemon, once */
  RCAP_MSG_STREAM_INFO = 5,/* daemon -> consumer, once */
  RCAP_MSG_POOL_ADD = 6,   /* daemon -> consumer, x pool_count, carries fd */
  RCAP_MSG_FRAME = 7,      /* daemon -> consumer */
  RCAP_MSG_RELEASE = 8,    /* consumer -> daemon */
  RCAP_MSG_STATUS_GET = 9, /* status client -> daemon, empty body */
  RCAP_MSG_STATUS = 10,    /* daemon -> status client, JSON body */
  /* v2 additions: live shader control, no daemon restart. */
  RCAP_MSG_SHADER_SET = 11, /* status client -> daemon (v2 connections only) */
  RCAP_MSG_SHADER_ACK = 12, /* daemon -> status client */
  /* v3 additions: live HDMI-TX display-passthrough control. */
  RCAP_MSG_DISPLAY_SET = 13, /* status client -> daemon (v3 connections only) */
  RCAP_MSG_DISPLAY_ACK = 14, /* daemon -> status client */
};

enum rcap_role {
  RCAP_ROLE_CONSUMER = 1, /* receives the frame stream; at most one */
  RCAP_ROLE_STATUS = 2,   /* read-only JSON status; any number */
};

enum rcap_error_code {
  RCAP_ERR_BUSY = 1,      /* a consumer is already connected */
  RCAP_ERR_VERSION = 2,   /* no protocol version overlap */
  RCAP_ERR_BAD_SETUP = 3, /* unacceptable SETUP parameters */
  RCAP_ERR_INTERNAL = 4,
};

/* FRAME flags */
#define RCAP_FRAME_FRESH (1u << 0)       /* newly captured content */
#define RCAP_FRAME_HELD (1u << 1)        /* cadence re-emission of last good */
#define RCAP_FRAME_BLACK (1u << 2)       /* pre-first-frame black */
#define RCAP_FRAME_SIGNAL_LOST (1u << 3) /* input unlocked (informational) */

#if defined(__GNUC__) || defined(__clang__)
#define RCAP_PACKED __attribute__((packed))
#else
#error "define RCAP_PACKED for this compiler"
#endif

/* Every message begins with this header. hdr_flags reserved, must be 0. */
struct RCAP_PACKED rcap_hdr {
  uint16_t type; /* enum rcap_msg_type */
  uint16_t hdr_flags;
};

struct RCAP_PACKED rcap_hello {
  struct rcap_hdr hdr;
  uint32_t magic; /* RCAP_MAGIC */
  uint16_t ver_min;
  uint16_t ver_max;
  uint8_t role; /* enum rcap_role */
  uint8_t reserved[3];
  char name[32]; /* NUL-padded client identity, for daemon logs */
};

struct RCAP_PACKED rcap_hello_ack {
  struct rcap_hdr hdr;
  uint16_t version; /* the version this connection speaks */
  uint16_t reserved;
};

struct RCAP_PACKED rcap_error {
  struct rcap_hdr hdr;
  uint16_t code; /* enum rcap_error_code */
  uint16_t reserved;
};

/* Zeros mean "daemon default". STREAM_INFO is authoritative; parameters are
 * immutable for the connection (the fixed-output contract). */
struct RCAP_PACKED rcap_setup {
  struct rcap_hdr hdr;
  uint32_t width;
  uint32_t height;
  uint32_t fps_num;
  uint32_t fps_den;
};

/* v2 length-extension of SETUP: a consumer MAY send this longer message to
 * request a pool pixel format for the session. A short (original) SETUP is
 * equivalent to fourcc 0. fourcc 0 or DRM_FORMAT_NV12 requests the 8-bit
 * NV12 pool (a 10-bit capture is downconverted); DRM_FORMAT_NV15 requests
 * the compact 10-bit pool and is served only while the daemon is capturing
 * 10-bit — otherwise the daemon falls back to NV12. STREAM_INFO remains
 * authoritative: consumers MUST verify its fourcc matches what the session
 * requires and reconnect/fail rather than assume. */
struct RCAP_PACKED rcap_setup2 {
  struct rcap_setup base;
  uint32_t fourcc;   /* requested pool format; 0 = DRM_FORMAT_NV12 */
  uint32_t reserved; /* must be 0 */
};

struct RCAP_PACKED rcap_stream_info {
  struct rcap_hdr hdr;
  uint32_t width;
  uint32_t height;
  uint32_t fps_num;
  uint32_t fps_den;
  uint32_t fourcc;   /* DRM_FORMAT_NV12, or DRM_FORMAT_NV15 (Rockchip compact
                      * 10-bit 4:2:0) when the daemon runs a 10-bit session.
                      * Consumers MUST honor this rather than assume NV12. */
  uint64_t modifier; /* DRM_FORMAT_MOD_LINEAR */
  uint32_t stride_y; /* bytes (NV12: 16-aligned pixel count; NV15: 64-aligned
                      * compact byte stride, ALIGN(width*10/8, 320)) */
  uint32_t stride_uv;
  uint32_t offset_y;
  uint32_t offset_uv;
  uint32_t buffer_size; /* total bytes per pool buffer */
  uint32_t pool_count;  /* number of POOL_ADD messages that follow */
};

/* Carries exactly one dma-buf fd via SCM_RIGHTS. The consumer owns the fd;
 * dma-heap buffers stay valid even if the daemon exits while fds are held. */
struct RCAP_PACKED rcap_pool_add {
  struct rcap_hdr hdr;
  uint32_t index; /* 0 .. pool_count-1 */
  uint32_t reserved;
};

/* FRESH frames are sent immediately on processing completion; HELD/BLACK
 * frames are sent by the cadence watchdog so a frame arrives every output
 * period. A FRAME referencing an index the consumer does not hold creates a
 * lease; re-references (held re-emissions) do not. One RELEASE per lease.
 * All timestamps are CLOCK_MONOTONIC nanoseconds; HELD re-emissions keep the
 * original dqbuf_ns/process_done_ns and update send_ns. */
struct RCAP_PACKED rcap_frame {
  struct rcap_hdr hdr;
  uint32_t index;
  uint32_t flags; /* RCAP_FRAME_* */
  uint64_t serial; /* monotonic per FRAME message; gaps allowed */
  uint64_t dqbuf_ns;
  uint64_t process_done_ns;
  uint64_t send_ns;
};

struct RCAP_PACKED rcap_release {
  struct rcap_hdr hdr;
  uint32_t index;
  uint32_t reserved;
  uint64_t serial; /* serial of the FRAME that created the lease */
};

struct RCAP_PACKED rcap_status_get {
  struct rcap_hdr hdr;
};

/* RCAP_MSG_STATUS body is UTF-8 JSON following rcap_hdr, NUL-terminated
 * within the datagram. See PROTOCOL.md §3.10 for the field list; fields are
 * additive without a version bump. */

/* ---- v2: live shader control ---- */

enum rcap_shader_effect {
  RCAP_SHADER_DISABLED = 0,
  RCAP_SHADER_CRT_BASIC = 1,
};

/* Applies the shader state live (next frame, no restart, no effect on the
 * consumer session or pool). Sent by a STATUS-role client on a connection
 * speaking version >= 2; the daemon clamps out-of-range values and replies
 * with SHADER_ACK. crt_width/crt_height describe the emulated CRT raster the
 * capture is downscaled to before the CRT pass. crt_height 0 follows the
 * source; crt_width 0 with a set height derives the width from the displayed
 * aspect ratio (854 for a 480-line 16:9 picture, 640 under a forced 4:3).
 * Floats are IEEE-754 binary32, little-endian like every other field. */
struct RCAP_PACKED rcap_shader_set {
  struct rcap_hdr hdr;
  uint8_t effect; /* enum rcap_shader_effect */
  uint8_t fade;   /* 0/1: fade scanlines out near 1:1 vertical scale */
  uint16_t crt_width;
  uint16_t crt_height;
  uint16_t reserved;
  float scan;   /* 0..1  scanline darkness */
  float sharp;  /* 0..1  beam sharpness */
  float mask;   /* 0..1  aperture-grille strength */
  float bright; /* 0..0.5 brightness compensation */
  float soft;   /* 0..1  horizontal softness */
  float pitch;  /* 2..4  grille pitch in output pixels */
};

struct RCAP_PACKED rcap_shader_ack {
  struct rcap_hdr hdr;
  uint16_t ok; /* 1 = accepted (post-clamp), 0 = rejected */
  uint16_t reserved;
};

/* ---- v3: live HDMI-TX display passthrough control ---- */

/* Enables/disables the daemon's local display sink (capture frames scanned
 * out to the board's HDMI-TX) live — next frame, no restart, no effect on
 * the consumer session or pool. Sent by a STATUS-role client on a connection
 * speaking version >= 3. The sink must have been armed at daemon start
 * (RETRO_CAPTURE_DISPLAY=hdmi); when it was not, the daemon replies
 * available=0 and the enable request is a no-op. */
struct RCAP_PACKED rcap_display_set {
  struct rcap_hdr hdr;
  uint8_t enable; /* 1 = scan out frames, 0 = blank the plane + drop HDR */
  uint8_t reserved[3];
};

struct RCAP_PACKED rcap_display_ack {
  struct rcap_hdr hdr;
  uint8_t ok;        /* 1 = request queued */
  uint8_t available; /* 1 = the display sink is armed in this daemon run */
  uint16_t reserved;
};

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RETRO_CAPTURE_PROTOCOL_H */
