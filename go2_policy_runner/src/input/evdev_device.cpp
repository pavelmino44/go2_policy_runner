// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include "go2_policy_runner/input/evdev_device.hpp"

#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>

namespace go2_policy_runner::input
{

namespace
{
constexpr std::size_t kBitsPerLong = sizeof(unsigned long) * 8;  // NOLINT(runtime/int)
template<std::size_t N>
using BitArray = std::array<unsigned long, (N + kBitsPerLong - 1) / kBitsPerLong>;  // NOLINT

template<std::size_t N>
bool test_bit(const BitArray<N> & bits, std::size_t i)
{
  return i < N && ((bits[i / kBitsPerLong] >> (i % kBitsPerLong)) & 1ul);
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

BitArray<KEY_CNT> key_bits(int fd)
{
  BitArray<KEY_CNT> b{};
  ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(b)), b.data());
  return b;
}

BitArray<ABS_CNT> abs_bits(int fd)
{
  BitArray<ABS_CNT> b{};
  ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(b)), b.data());
  return b;
}
}  // namespace

EvdevDevice::EvdevDevice(const std::string & path)
: path_(path)
{
  fd_ = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    const int err = errno;
    std::string msg = "cannot open " + path + ": " + std::strerror(err);
    if (err == EACCES) {
      msg += " (add the user to the 'input' group: sudo usermod -aG input $USER, then log in again)";
    }
    throw std::runtime_error(msg);
  }
  std::array<char, 256> name{};
  if (ioctl(fd_, EVIOCGNAME(name.size() - 1), name.data()) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(path + " is not an input event device");
  }
  name_ = name.data();
}

EvdevDevice::~EvdevDevice()
{
  if (fd_ >= 0) {::close(fd_);}
}

bool EvdevDevice::is(DeviceKind kind) const
{
  const auto keys = key_bits(fd_);
  if (kind == DeviceKind::kGamepad) {
    const auto abs = abs_bits(fd_);
    // the DualShock 4 motion sensor and touchpad nodes have no BTN_SOUTH
    return test_bit<KEY_CNT>(keys, BTN_SOUTH) && test_bit<ABS_CNT>(abs, ABS_X) &&
           test_bit<ABS_CNT>(abs, ABS_Y);
  }
  // a real keyboard, not a power button / media remote / mouse
  for (int k : {KEY_A, KEY_Z, KEY_1, KEY_SPACE, KEY_ENTER, KEY_LEFTSHIFT}) {
    if (!test_bit<KEY_CNT>(keys, static_cast<std::size_t>(k))) {return false;}
  }
  return true;
}

AxisRange EvdevDevice::axis_range(std::uint16_t code) const
{
  input_absinfo info{};
  if (ioctl(fd_, EVIOCGABS(code), &info) < 0) {return AxisRange{};}
  return AxisRange{info.minimum, info.maximum};
}

bool EvdevDevice::grab(bool on)
{
  return ioctl(fd_, EVIOCGRAB, on ? 1 : 0) == 0;
}

std::vector<Event> EvdevDevice::snapshot() const
{
  std::vector<Event> out;
  BitArray<KEY_CNT> state{};
  if (ioctl(fd_, EVIOCGKEY(sizeof(state)), state.data()) >= 0) {
    const auto supported = key_bits(fd_);
    for (std::size_t k = 0; k < KEY_CNT; ++k) {
      if (test_bit<KEY_CNT>(supported, k)) {
        out.push_back({EV_KEY, static_cast<std::uint16_t>(k), test_bit<KEY_CNT>(state, k) ? 1 : 0});
      }
    }
  }
  const auto abs = abs_bits(fd_);
  for (std::size_t a = 0; a < ABS_CNT; ++a) {
    if (!test_bit<ABS_CNT>(abs, a)) {continue;}
    input_absinfo info{};
    if (ioctl(fd_, EVIOCGABS(a), &info) >= 0) {
      out.push_back({EV_ABS, static_cast<std::uint16_t>(a), info.value});
    }
  }
  return out;
}

bool EvdevDevice::read(std::vector<Event> & out)
{
  std::array<input_event, 64> buf;
  while (true) {
    const ssize_t n = ::read(fd_, buf.data(), sizeof(buf));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {return true;}
      if (errno == EINTR) {continue;}
      return false;  // ENODEV: unplugged
    }
    if (n == 0) {return false;}
    const std::size_t count = static_cast<std::size_t>(n) / sizeof(input_event);
    for (std::size_t i = 0; i < count; ++i) {
      const auto & ev = buf[i];
      if (ev.type == EV_SYN && ev.code == SYN_DROPPED) {
        dropping_ = true;  // discard everything up to the next SYN_REPORT, then resync
        continue;
      }
      if (dropping_) {
        if (ev.type == EV_SYN && ev.code == SYN_REPORT) {
          dropping_ = false;
          out.push_back({EV_SYN, SYN_DROPPED, 0});
          const auto s = snapshot();
          out.insert(out.end(), s.begin(), s.end());
        }
        continue;
      }
      if (ev.type == EV_KEY || ev.type == EV_ABS) {
        out.push_back({ev.type, ev.code, ev.value});
      }
    }
  }
}

std::vector<DeviceInfo> find_devices(
  DeviceKind kind, const std::string & name_filter, bool * permission_denied)
{
  namespace fs = std::filesystem;
  std::vector<DeviceInfo> found;
  if (permission_denied) {*permission_denied = false;}
  std::error_code ec;
  std::vector<std::string> paths;
  for (const auto & entry : fs::directory_iterator("/dev/input", ec)) {
    const auto name = entry.path().filename().string();
    if (name.rfind("event", 0) == 0) {paths.push_back(entry.path().string());}
  }
  // event2 < event10
  std::sort(paths.begin(), paths.end(), [](const std::string & a, const std::string & b) {
      return a.size() != b.size() ? a.size() < b.size() : a < b;
    });
  const auto filter = lower(name_filter);
  for (const auto & p : paths) {
    if (::access(p.c_str(), R_OK) != 0) {
      if (errno == EACCES && permission_denied) {*permission_denied = true;}
      continue;
    }
    try {
      EvdevDevice dev(p);
      if (!dev.is(kind)) {continue;}
      if (!filter.empty() && lower(dev.name()).find(filter) == std::string::npos) {continue;}
      found.push_back({p, dev.name()});
    } catch (const std::exception &) {
      // vanished or not an event device
    }
  }
  return found;
}

}  // namespace go2_policy_runner::input
