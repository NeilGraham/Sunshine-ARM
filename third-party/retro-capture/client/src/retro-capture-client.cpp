/*
 * retro-capture-client — see include/retro-capture-client.h.
 *
 * Copyright (c) 2026 Neil Graham
 * SPDX-License-Identifier: MIT
 *
 * Migrated out of the Sunshine fork 2026-08-07. The lease bookkeeping below is
 * the subtle part and the comments explaining it are carried over verbatim,
 * because each one records a bug that was actually hit.
 */
#include "retro-capture-client.h"

#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "retro-capture-protocol.h"

namespace retro::capture {

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

    std::string fourcc_str(std::uint32_t fourcc) {
      const char s[5] = {
        (char) (fourcc & 0xff),
        (char) ((fourcc >> 8) & 0xff),
        (char) ((fourcc >> 16) & 0xff),
        (char) ((fourcc >> 24) & 0xff),
        0,
      };
      return s;
    }
  }  // namespace

  void set_logger(log_fn fn) {
    g_logger = fn;
  }

  namespace {
    std::uint64_t now_ns() {
      timespec ts {};
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (std::uint64_t) ts.tv_sec * 1000000000ull + ts.tv_nsec;
    }
  }  // namespace

  struct client::impl {
    int sock {-1};
    stream_info sinfo {};
    std::vector<int> pool_fds;
    std::vector<bool> leased;
    int cur_index {-1};
    frame_observer observer {nullptr};
    void *observer_user {nullptr};
    std::chrono::steady_clock::time_point last_reconnect_at {};

    ~impl() {
      for (auto fd : pool_fds) {
        if (fd >= 0) {
          ::close(fd);
        }
      }
      if (sock >= 0) {
        ::close(sock);
      }
    }

    bool send_msg(const void *msg, std::size_t len) {
      ssize_t n;
      do {
        n = send(sock, msg, len, MSG_NOSIGNAL);
      } while (n < 0 && errno == EINTR);
      return n == (ssize_t) len;
    }

    ssize_t recv_timeout(void *buf, std::size_t len, int timeout_ms) {
      pollfd pfd {sock, POLLIN, 0};
      if (poll(&pfd, 1, timeout_ms) <= 0) {
        return -1;
      }
      return recv(sock, buf, len, 0);
    }

    void mark_dead(const char *why) {
      if (sock >= 0) {
        logf(log_warning, "retro-capture: %s; holding last frame and retrying connect", why);
        ::close(sock);
        sock = -1;
      }
    }

    void release_lease(int index) {
      if (index < 0 || index >= (int) leased.size() || !leased[index]) {
        return;
      }
      rcap_release rel {{RCAP_MSG_RELEASE, 0}, (std::uint32_t) index, 0, 0};
      if (send_msg(&rel, sizeof(rel))) {
        leased[index] = false;
      }
    }
  };

  client::client():
      p(std::make_unique<impl>()) {}

  client::~client() = default;
  client::client(client &&) noexcept = default;
  client &client::operator=(client &&) noexcept = default;

  const char *client::socket_path() {
    const char *s = std::getenv("RETRO_CAPTURE_SOCKET");
    return (s && *s) ? s : RCAP_SOCKET_DEFAULT;
  }

  bool client::alive() const {
    return p->sock >= 0;
  }

  const stream_info &client::info() const {
    return p->sinfo;
  }

  std::size_t client::pool_size() const {
    return p->pool_fds.size();
  }

  int client::pool_fd(std::size_t index) const {
    return index < p->pool_fds.size() ? p->pool_fds[index] : -1;
  }

  int client::fd() const {
    return p->sock;
  }

  void client::set_frame_observer(frame_observer fn, void *user) {
    p->observer = fn;
    p->observer_user = user;
  }

  std::unique_ptr<client> client::connect(int width, int height, std::uint32_t want_fourcc,
                                          const char *name) {
    std::unique_ptr<client> c {new client()};
    auto &s = *c->p;

    s.sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (s.sock < 0) {
      return nullptr;
    }
    sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, socket_path(), sizeof(addr.sun_path) - 1);
    if (::connect(s.sock, (sockaddr *) &addr, sizeof(addr)) < 0) {
      logf(log_info, "retro-capture: no daemon at %s (%s); consumer should fall back",
           socket_path(), strerror(errno));
      return nullptr;
    }

    rcap_hello hello {};
    hello.hdr = {RCAP_MSG_HELLO, 0};
    hello.magic = RCAP_MAGIC;
    hello.ver_min = RCAP_PROTO_VERSION;
    hello.ver_max = RCAP_PROTO_VERSION;
    hello.role = RCAP_ROLE_CONSUMER;
    std::strncpy(hello.name, name && *name ? name : "consumer", sizeof(hello.name) - 1);
    if (!s.send_msg(&hello, sizeof(hello))) {
      return nullptr;
    }

    std::uint8_t buf[RCAP_MAX_MSG_SIZE];
    if (s.recv_timeout(buf, sizeof(buf), 3000) < (ssize_t) sizeof(rcap_hello_ack) ||
        ((rcap_hdr *) buf)->type != RCAP_MSG_HELLO_ACK) {
      logf(log_warning, "retro-capture: daemon refused HELLO");
      return nullptr;
    }

    // The consumer's encoder was created for one pool format; an 8-bit session
    // must get NV12 even while the daemon captures 10-bit (the daemon
    // downconverts). SETUP v2's length extension carries the request; a zero
    // want_fourcc sends the plain v1 SETUP instead ("serve me whatever you
    // have") — the test-consumer oracle's mode, never Sunshine's.
    rcap_setup2 setup {};
    setup.base.hdr = {RCAP_MSG_SETUP, 0};
    setup.base.width = width;
    setup.base.height = height;
    setup.base.fps_num = 0;  // daemon default cadence; the consumer paces itself
    setup.base.fps_den = 0;
    setup.fourcc = want_fourcc;
    if (!s.send_msg(&setup, want_fourcc ? sizeof(setup) : sizeof(setup.base))) {
      return nullptr;
    }

    // STREAM_INFO, then pool_count POOL_ADD messages carrying one fd each.
    rcap_stream_info raw {};
    bool have_info = false;
    while (!have_info || s.pool_fds.size() < raw.pool_count) {
      iovec iov {buf, sizeof(buf)};
      char cbuf[CMSG_SPACE(sizeof(int))];
      msghdr mh {};
      mh.msg_iov = &iov;
      mh.msg_iovlen = 1;
      mh.msg_control = cbuf;
      mh.msg_controllen = sizeof(cbuf);
      pollfd pfd {s.sock, POLLIN, 0};
      if (poll(&pfd, 1, 3000) <= 0) {
        logf(log_warning, "retro-capture: timeout waiting for stream setup");
        return nullptr;
      }
      ssize_t n = recvmsg(s.sock, &mh, 0);
      if (n < (ssize_t) sizeof(rcap_hdr)) {
        return nullptr;
      }
      auto *hdr = (rcap_hdr *) buf;
      if (hdr->type == RCAP_MSG_STREAM_INFO && n >= (ssize_t) sizeof(rcap_stream_info)) {
        raw = *(rcap_stream_info *) buf;
        have_info = true;
      } else if (hdr->type == RCAP_MSG_POOL_ADD && n >= (ssize_t) sizeof(rcap_pool_add)) {
        int fd = -1;
        for (cmsghdr *cm = CMSG_FIRSTHDR(&mh); cm; cm = CMSG_NXTHDR(&mh, cm)) {
          if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) {
            std::memcpy(&fd, CMSG_DATA(cm), sizeof(int));
          }
        }
        auto *add = (rcap_pool_add *) buf;
        if (fd < 0 || add->index != s.pool_fds.size()) {
          logf(log_warning, "retro-capture: bad POOL_ADD");
          return nullptr;
        }
        s.pool_fds.push_back(fd);
      } else if (hdr->type == RCAP_MSG_ERROR) {
        logf(log_warning, "retro-capture: daemon error %u", ((rcap_error *) buf)->code);
        return nullptr;
      }
    }

    // A zero width/height request means "daemon's configured output" and
    // adopts whatever STREAM_INFO says (oracle mode, as above).
    if (width > 0 && ((int) raw.width != width || (int) raw.height != height)) {
      logf(log_warning, "retro-capture: daemon stream %ux%u != requested %dx%d",
           raw.width, raw.height, width, height);
      return nullptr;
    }

    // STREAM_INFO is authoritative: the consumer configured itself for the
    // requested layout, so a different pool format (old daemon, or NV15 asked
    // for while the capture just left 10-bit) is unusable.
    if (want_fourcc && raw.fourcc != want_fourcc) {
      logf(log_warning, "retro-capture: daemon served pool format %s but this session needs %s",
           fourcc_str(raw.fourcc).c_str(), fourcc_str(want_fourcc).c_str());
      return nullptr;
    }

    s.sinfo = stream_info {
      raw.width, raw.height, raw.fourcc,
      raw.stride_y, raw.stride_uv, raw.offset_y, raw.offset_uv,
      raw.buffer_size, raw.modifier, raw.pool_count
    };
    s.leased.assign(s.pool_fds.size(), false);

    logf(log_info, "retro-capture: connected — %ux%u %s pool of %u (stride %u, zero-copy dma-buf)",
         raw.width, raw.height, fourcc_str(raw.fourcc).c_str(), raw.pool_count, raw.stride_y);
    return c;
  }

  frame client::next() {
    auto &s = *p;
    const int previous = s.cur_index;

    if (s.sock < 0) {
      return frame {s.cur_index, 0, false};
    }

    std::uint8_t buf[RCAP_MAX_MSG_SIZE];
    int newest = -1;
    std::uint32_t newest_flags = 0;
    for (;;) {
      ssize_t n = recv(s.sock, buf, sizeof(buf), MSG_DONTWAIT);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        s.mark_dead("recv error");
        break;
      }
      if (n == 0) {
        s.mark_dead("daemon closed");
        break;
      }
      auto *hdr = (rcap_hdr *) buf;
      if (hdr->type == RCAP_MSG_FRAME && n >= (ssize_t) sizeof(rcap_frame)) {
        auto *f = (rcap_frame *) buf;
        if (s.observer) {
          frame_telemetry t {f->serial, f->index, f->flags,
                             f->dqbuf_ns, f->process_done_ns, f->send_ns, now_ns()};
          s.observer(t, s.observer_user);
        }
        if (f->index < s.pool_fds.size()) {
          if (!s.leased[f->index]) {
            s.leased[f->index] = true;  // lease created by first reference
          }
          // A frame superseded within this same drain must be released here:
          // it is neither cur_index nor the drain's winner, so the post-loop
          // release below never sees it, and the daemon cannot re-send an
          // index it still considers leased — each occurrence would
          // permanently shrink the 4-buffer pool.
          if (newest >= 0 && newest != (int) f->index && newest != s.cur_index) {
            s.release_lease(newest);
          }
          newest = (int) f->index;
          newest_flags = f->flags;
        }
      }
      // (ERROR after streaming started ends in EOF; nothing else expected.)
    }

    if (newest >= 0) {
      if (s.cur_index != newest) {
        // Superseded: hand the buffer back so the daemon can write into it.
        s.release_lease(s.cur_index);
      }
      s.cur_index = newest;
    }

    return frame {s.cur_index, newest_flags, s.cur_index != previous};
  }

  void client::maybe_reconnect(int width, int height, std::uint32_t want_fourcc,
                               const char *name) {
    if (p->sock >= 0) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - p->last_reconnect_at < std::chrono::seconds(1)) {
      return;
    }
    p->last_reconnect_at = now;

    auto fresh = connect(width, height, want_fourcc, name);
    if (fresh) {
      logf(log_info, "retro-capture: reconnected");
      fresh->p->observer = p->observer;
      fresh->p->observer_user = p->observer_user;
      p = std::move(fresh->p);
    }
  }

}  // namespace retro::capture
