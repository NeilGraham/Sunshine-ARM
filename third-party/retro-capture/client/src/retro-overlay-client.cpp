/*
 * retro-overlay-client — see include/retro-overlay-client.h.
 *
 * Copyright (c) 2026 Neil Graham
 * SPDX-License-Identifier: MIT
 *
 * Migrated out of the Sunshine fork 2026-08-07 (was
 * src/platform/linux/retro-overlay.cpp). Behaviour is unchanged; the only
 * edits are BOOST_LOG -> the log callback, and returning the OSD struct to the
 * caller instead of attaching AVFrame side data here.
 */
#include "retro-overlay-client.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace retro::overlay {

  namespace {

    log_fn g_logger = nullptr;

    void logf(log_level level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

    void logf(log_level level, const char *fmt, ...) {
      if (!g_logger) {
        return;
      }
      char buf[512];
      va_list ap;
      va_start(ap, fmt);
      std::vsnprintf(buf, sizeof(buf), fmt, ap);
      va_end(ap);
      g_logger(level, buf);
    }

    constexpr auto reconnect_pace = 2s;

    struct surface_t {
      int fd = -1;
      std::uint32_t size = 0;
      std::uint32_t stride = 0;
    };

    struct shared_state_t {
      std::mutex mtx;
      surface_t surfaces[ROVL_MAX_SURFACES];
      std::uint32_t palette[ROVL_PALETTE_SIZE] {};
      std::uint32_t palette_serial = 0;
      rovl_present present {};
      bool visible = false;

      // Geometry the encode thread last reported; the socket thread sends
      // SINK_INFO whenever it drifts from what the daemon was told.
      std::atomic<std::uint32_t> want_geometry {0};  // (w << 16) | h
      std::atomic<bool> want_ten_bit {false};

      void drop_surfaces_locked() {
        for (auto &s : surfaces) {
          if (s.fd >= 0) {
            close(s.fd);
          }
          s = surface_t {};
        }
        visible = false;
      }
    };

    shared_state_t state;

    ssize_t send_msg(int sock, const void *msg, std::size_t len) {
      return send(sock, msg, len, MSG_NOSIGNAL);
    }

    int connect_daemon() {
      const char *path = getenv("RETRO_OVERLAY_SOCKET");
      if (!path) {
        path = ROVL_SOCKET_DEFAULT;
      }

      int sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
      if (sock < 0) {
        return -1;
      }

      // Bound every blocking call: a wedged daemon must not stall this
      // thread's shutdown-or-resync checks. 1 s, like the capture client.
      timeval tv {1, 0};
      setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

      sockaddr_un addr {};
      addr.sun_family = AF_UNIX;
      std::strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
      if (connect(sock, (sockaddr *) &addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
      }

      rovl_hello hello {};
      hello.hdr = {ROVL_MSG_HELLO, 0};
      hello.magic = ROVL_MAGIC;
      hello.ver_min = ROVL_PROTO_VERSION;
      hello.ver_max = ROVL_PROTO_VERSION;
      hello.role = ROVL_ROLE_SINK;
      std::strncpy(hello.name, "sunshine", sizeof(hello.name) - 1);
      if (send_msg(sock, &hello, sizeof(hello)) != (ssize_t) sizeof(hello)) {
        close(sock);
        return -1;
      }

      std::uint8_t buf[ROVL_MAX_MSG_SIZE];
      ssize_t n = recv(sock, buf, sizeof(buf), 0);
      if (n < (ssize_t) sizeof(rovl_hello_ack) ||
          ((rovl_hdr *) buf)->type != ROVL_MSG_HELLO_ACK) {
        close(sock);
        return -1;
      }
      return sock;
    }

    bool send_sink_info(int sock) {
      std::uint32_t geo = state.want_geometry.load();
      if (!geo) {
        return true;  // no encode session reported yet
      }
      rovl_sink_info info {};
      info.hdr = {ROVL_MSG_SINK_INFO, 0};
      info.width = geo >> 16;
      info.height = geo & 0xffff;
      info.ten_bit = state.want_ten_bit.load() ? 1 : 0;
      return send_msg(sock, &info, sizeof(info)) == (ssize_t) sizeof(info);
    }

    // Receive one datagram; SURFACE_ADD arrives with one fd via SCM_RIGHTS.
    ssize_t recv_msg(int sock, void *buf, std::size_t len, int &fd_out) {
      fd_out = -1;

      iovec iov {buf, len};
      alignas(cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))];
      msghdr mh {};
      mh.msg_iov = &iov;
      mh.msg_iovlen = 1;
      mh.msg_control = cmsg_buf;
      mh.msg_controllen = sizeof(cmsg_buf);

      ssize_t n = recvmsg(sock, &mh, MSG_CMSG_CLOEXEC);
      if (n <= 0) {
        return n;
      }

      for (auto *c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS) {
          std::memcpy(&fd_out, CMSG_DATA(c), sizeof(int));
        }
      }
      return n;
    }

    void handle_msg(const std::uint8_t *buf, std::size_t len, int fd) {
      auto type = ((const rovl_hdr *) buf)->type;

      std::lock_guard lg {state.mtx};
      switch (type) {
        case ROVL_MSG_SURFACE_ADD: {
          if (len < sizeof(rovl_surface_add) || fd < 0) {
            break;
          }
          auto *m = (const rovl_surface_add *) buf;
          if (m->index >= ROVL_MAX_SURFACES) {
            break;
          }
          auto &s = state.surfaces[m->index];
          if (s.fd >= 0) {
            close(s.fd);
          }
          s.fd = fd;
          s.size = m->size;
          s.stride = m->stride;
          fd = -1;  // ownership taken
          break;
        }
        case ROVL_MSG_PALETTE: {
          if (len < sizeof(rovl_palette)) {
            break;
          }
          auto *m = (const rovl_palette *) buf;
          std::memcpy(state.palette, m->entries, sizeof(state.palette));
          state.palette_serial = m->serial;
          break;
        }
        case ROVL_MSG_PRESENT: {
          if (len < sizeof(rovl_present)) {
            break;
          }
          auto *m = (const rovl_present *) buf;
          if (m->surface_index >= ROVL_MAX_SURFACES ||
              state.surfaces[m->surface_index].fd < 0 ||
              m->num_region == 0 || m->num_region > ROVL_MAX_REGIONS) {
            logf(log_warning, "retro-overlay: rejected PRESENT serial=%llu surface=%d regions=%d",
                 (unsigned long long) m->serial, (int) m->surface_index, (int) m->num_region);
            break;
          }
          state.present = *m;
          state.visible = true;
          break;
        }
        case ROVL_MSG_HIDE:
          state.visible = false;
          break;
        default:
          break;  // control-plane or future messages: ignore
      }

      if (fd >= 0) {
        close(fd);  // fd on an unexpected message
      }
    }

    void client_loop() {
      bool logged_connected = false;

      while (true) {
        int sock = connect_daemon();
        if (sock < 0) {
          if (logged_connected) {
            logf(log_info, "retro-overlay: daemon gone, overlay hidden");
            logged_connected = false;
          }
          {
            std::lock_guard lg {state.mtx};
            state.drop_surfaces_locked();
          }
          std::this_thread::sleep_for(reconnect_pace);
          continue;
        }

        logf(log_info, "retro-overlay: connected");
        logged_connected = true;

        std::uint32_t sent_geometry = 0;
        bool sent_ten_bit = false;

        while (true) {
          // (Re)announce encode geometry when it drifts.
          std::uint32_t geo = state.want_geometry.load();
          bool ten = state.want_ten_bit.load();
          if (geo && (geo != sent_geometry || ten != sent_ten_bit)) {
            if (!send_sink_info(sock)) {
              break;
            }
            sent_geometry = geo;
            sent_ten_bit = ten;
          }

          std::uint8_t buf[ROVL_MAX_MSG_SIZE];
          int fd = -1;
          ssize_t n = recv_msg(sock, buf, sizeof(buf), fd);
          if (n == 0) {
            break;  // daemon closed
          }
          if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
              continue;  // timeout tick: loop to re-check geometry
            }
            break;
          }
          if (n < (ssize_t) sizeof(rovl_hdr)) {
            if (fd >= 0) {
              close(fd);
            }
            continue;
          }
          handle_msg(buf, n, fd);
        }

        close(sock);
        {
          std::lock_guard lg {state.mtx};
          state.drop_surfaces_locked();
        }
        std::this_thread::sleep_for(reconnect_pace);
      }
    }

    void ensure_thread() {
      static std::once_flag once;
      std::call_once(once, []() {
        std::thread {client_loop}.detach();
      });
    }

  }  // namespace

  void set_logger(log_fn fn) {
    g_logger = fn;
  }

  void report_encode_geometry(int width, int height, bool ten_bit) {
    ensure_thread();
    state.want_geometry.store(((std::uint32_t) width << 16) | (std::uint32_t) height);
    state.want_ten_bit.store(ten_bit);
  }

  bool current_osd(rovl_osd_side_data &out) {
    // Logs on TRANSITIONS only — the caller runs this per encoded frame.
    static std::atomic<int> logged_state {-1};

    std::lock_guard lg {state.mtx};
    const int now_state = state.visible ? 1 : 0;
    if (logged_state.exchange(now_state) != now_state) {
      logf(log_info, "retro-overlay: encode side data %s (serial %llu)",
           now_state ? "ATTACHING" : "detached", (unsigned long long) state.present.serial);
    }
    if (!state.visible) {
      return false;
    }

    const auto &surf = state.surfaces[state.present.surface_index];
    if (surf.fd < 0) {
      return false;
    }

    std::memset(&out, 0, sizeof(out));
    out.fd = surf.fd;
    out.size = surf.size;
    out.num_region = state.present.num_region;
    for (std::uint32_t i = 0; i < out.num_region; ++i) {
      const auto &r = state.present.region[i];
      out.region[i].enable = r.enable;
      out.region[i].start_mb_x = r.start_mb_x;
      out.region[i].start_mb_y = r.start_mb_y;
      out.region[i].num_mb_x = r.num_mb_x;
      out.region[i].num_mb_y = r.num_mb_y;
      out.region[i].buf_offset = r.buf_offset;
    }
    out.palette_serial = state.palette_serial;
    std::memcpy(out.palette, state.palette, sizeof(out.palette));
    return true;
  }

}  // namespace retro::overlay
