#include "robot_hardware/serial_backend.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace robot_hardware {
namespace {

speed_t BaudToTermios(const int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

bool ConfigureSerialPort(const int fd, const int baud, std::string* error) {
  termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }
  cfmakeraw(&tty);
  const speed_t speed = BaudToTermios(baud);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }
  return true;
}

}  // namespace

SerialBackend::SerialBackend(ChassisBackendConfig config) : config_(std::move(config)) {}

SerialBackend::~SerialBackend() { Close(); }

std::string SerialBackend::Name() const { return "serial"; }

bool SerialBackend::Open(std::string* error) {
  Close();
  fd_ = open(config_.serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    if (error != nullptr) {
      *error = "failed to open serial backend device '" + config_.serial_device +
               "': " + std::strerror(errno);
    }
    return false;
  }
  std::string config_error;
  if (!ConfigureSerialPort(fd_, config_.serial_baud, &config_error)) {
    if (error != nullptr) {
      *error = "failed to configure serial backend device '" + config_.serial_device +
               "': " + config_error;
    }
    Close();
    return false;
  }
  return true;
}

void SerialBackend::Close() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool SerialBackend::IsOpen() const { return fd_ >= 0; }

bool SerialBackend::WriteCommand(const ChassisCommand& command, std::string* error) {
  if (!IsOpen()) {
    if (error != nullptr) {
      *error = "serial backend is not open";
    }
    return false;
  }
  const auto binary_packet = EncodeMickSpeedCommand(command);
  const std::string text_packet = EncodeCommand(command);
  const char* data = config_.protocol == "mick_binary"
                         ? reinterpret_cast<const char*>(binary_packet.data())
                         : text_packet.data();
  const std::size_t size = config_.protocol == "mick_binary" ? binary_packet.size()
                                                             : text_packet.size();
  const ssize_t written = write(fd_, data, size);
  if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }
  return written >= 0 || errno == EAGAIN || errno == EWOULDBLOCK;
}

std::optional<std::string> SerialBackend::Read(std::string* error) {
  if (!IsOpen()) {
    return std::nullopt;
  }
  char buffer[512];
  const ssize_t received = read(fd_, buffer, sizeof(buffer));
  if (received < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK && error != nullptr) {
      *error = std::strerror(errno);
    }
    return std::nullopt;
  }
  if (received == 0) {
    return std::nullopt;
  }
  return std::string(buffer, static_cast<std::size_t>(received));
}

}  // namespace robot_hardware
