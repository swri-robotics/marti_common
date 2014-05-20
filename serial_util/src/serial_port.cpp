// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <serial_util/serial_port.h>

#include <errno.h> // NOLINT
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <linux/serial.h>
#include <cstring>

#include <boost/lexical_cast.hpp>

namespace serial_util
{
  SerialConfig::SerialConfig() :
      baud(B115200),
      data_bits(8),
      stop_bits(1),
      parity(NO_PARITY),
      flow_control(false),
      low_latency_mode(false)
  {
  }

  SerialConfig::SerialConfig(
      int32_t baud,
      int32_t data_bits,
      int32_t stop_bits,
      Parity parity,
      bool flow_control,
      bool low_latency_mode) :
          baud(baud),
          data_bits(data_bits),
          stop_bits(stop_bits),
          parity(parity),
          flow_control(flow_control),
          low_latency_mode(low_latency_mode)
  {
  }

  SerialPort::SerialPort() :
    fd_(-1),
    error_msg_("")
  {
  }

  SerialPort::~SerialPort()
  {
    Close();
  }

  void SerialPort::Close()
  {
    if (fd_ < 0)
      return;

    close(fd_);
    fd_ = -1;
  }

  bool SerialPort::Open(const std::string &device, SerialConfig config)
  {
    Close();

    int32_t baud = ParseBaudRate(config.baud);
    if (baud == -1)
    {
      error_msg_ = "Invalid baud rate: " + boost::lexical_cast<std::string>(config.baud);
      return false;
    }

    if (config.stop_bits != 1 && config.stop_bits != 2)
    {
      error_msg_ = "Invalid stop bits: " + boost::lexical_cast<std::string>(config.stop_bits);
      return false;
    }

    if (config.data_bits != 7 && config.stop_bits != 8)
    {
      error_msg_ = "Invalid data bits: " + boost::lexical_cast<std::string>(config.data_bits);
      return false;
    }

    if (config.parity != SerialConfig::NO_PARITY &&
        config.parity != SerialConfig::EVEN_PARITY &&
        config.parity != SerialConfig::ODD_PARITY)
    {
      error_msg_ = "Invalid parity mode.";
      return false;
    }

    fd_ = open(device.c_str(), O_RDONLY);
    if (fd_ == -1)
    {
      error_msg_ = "Error opening serial port <" + device + ">: " + strerror(errno);
      return false;
    }

    struct termios term;
    if (tcgetattr(fd_, &term) < 0)
    {
      error_msg_ = "Unable to set serial attributes <" + device + ">: " + strerror(errno);
      Close();
      return false;
    }

    cfmakeraw(&term);

    if (config.stop_bits == 2)
    {
      term.c_cflag |= CSTOPB;
    }
    else
    {
      term.c_cflag &= ~CSTOPB;
    }

    if (config.parity == SerialConfig::EVEN_PARITY)
    {
      term.c_cflag |= PARENB;
      term.c_cflag &= ~PARODD;
    }
    else if (config.parity == SerialConfig::ODD_PARITY)
    {
      term.c_cflag |= PARENB;
      term.c_cflag |= PARODD;
    }
    else
    {
      term.c_cflag &= ~PARENB;
      term.c_cflag &= ~PARODD;
    }

    if (config.data_bits == 8)
    {
      term.c_cflag &= ~CSIZE;
      term.c_cflag |= CS8;
    }
    else
    {
      term.c_cflag &= ~CSIZE;
      term.c_cflag |= CS7;
    }

    if (cfsetspeed(&term, config.baud) < 0)
    {
      error_msg_ = "Invalid baud rate: " + boost::lexical_cast<std::string>(config.baud);
      Close();
      return false;
    }

    if (tcsetattr(fd_, TCSAFLUSH, &term) < 0)
    {
      error_msg_ = "Unable to set serial port attributes <" + device + ">: " + strerror(errno);
      Close();
      return false;
    }

    if (!SetLowLatencyMode(config.low_latency_mode))
    {
      Close();
      return false;
    }

    return true;
  }

  bool SerialPort::SetLowLatencyMode(bool enabled)
  {
    if (fd_ < 0)
    {
      error_msg_ = "Device not open.";
      return false;
    }

    struct serial_struct serial_info;

    if (ioctl(fd_, TIOCGSERIAL, &serial_info) < 0)
    {
      error_msg_ = "Failed to set low latency mode.  Cannot get serial configuration: " + std::string(strerror(errno));
      return false;
    }

    if (enabled)
    {
      serial_info.flags |= ASYNC_LOW_LATENCY;
    }
    else
    {
      serial_info.flags &= ~ASYNC_LOW_LATENCY;
    }

    if (ioctl(fd_, TIOCSSERIAL, &serial_info) < 0)
    {
      error_msg_ = "Failed to set low latency mode.  Cannot set serial configuration: " + std::string(strerror(errno));
      return false;
    }

    return true;
  }

  int32_t SerialPort::ParseBaudRate(int32_t baud)
  {
    int32_t value = -1;

    if (baud == B50 || baud == 50)
      value = B50;
    else if (baud == B75 || baud == 75)
      value = B75;
    else if (baud == B110 || baud == 110)
      value = B110;
    else if (baud == B134 || baud == 134)
      value = B134;
    else if (baud == B150 || baud == 150)
      value = B150;
    else if (baud == B200 || baud == 200)
      value = B200;
    else if (baud == B300 || baud == 300)
      value = B300;
    else if (baud == B600 || baud == 600)
      value = B600;
    else if (baud == B1200 || baud == 1200)
      value = B1200;
    else if (baud == B1800 || baud == 1800)
      value = B1800;
    else if (baud == B2400 || baud == 2400)
      value = B2400;
    else if (baud == B4800 || baud == 4800)
      value = B4800;
    else if (baud == B9600 || baud == 9600)
      value = B9600;
    else if (baud == B19200 || baud == 19200)
      value = B19200;
    else if (baud == B38400 || baud == 38400)
      value = B38400;
    else if (baud == B57600 || baud == 57600)
      value = B57600;
    else if (baud == B115200 || baud == 115200)
      value = B115200;
    else if (baud == B230400 || baud == 230400)
      value = B230400;

    return value;
  }

  SerialPort::Result SerialPort::ReadBytes(std::vector<uint8_t>& output, size_t max_bytes, int32_t timeout)
  {
    if (fd_ < 0)
    {
      error_msg_ = "Device not open.";
      return ERROR;
    }

    struct pollfd fds[1];
    fds[0].fd = fd_;
    fds[0].events = POLLIN;

    int poll_return = poll(fds, 1, timeout);
    if (poll_return == 0)
    {
      error_msg_ = "Timed out while waiting for data.";
      return TIMEOUT;
    }
    else if (poll_return < 0)
    {
      int error_num = errno;
      switch (error_num)
      {
        case EINTR:
          return INTERRUPTED;
        default:
          error_msg_ = "Error polling serial port: " + std::string(strerror(errno));
          return ERROR;
      }
    }

    size_t to_read = max_bytes;
    if (to_read <= 0)
    {
      int bytes;
      ioctl(fd_, FIONREAD, &bytes);
      to_read = static_cast<size_t>(bytes);
    }

    size_t output_size = output.size();
    output.resize(output_size + to_read);

    int result = read(fd_, output.data() + output_size, to_read);

    if (result > 0)
    {
      output.resize(output_size + result);
    }
    else
    {
      output.resize(output_size);
    }

    if (result > 0)
    {
      return SUCCESS;
    }
    else if (result == 0)
    {
      return INTERRUPTED;
    }
    else
    {
      int error_num = errno;
      switch (error_num)
      {
        case EINTR:
          return INTERRUPTED;
          break;
        default:
          error_msg_ = "Error reading serial port: " + std::string(strerror(errno));
          return ERROR;
      }
    }
  }
}
