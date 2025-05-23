// *****************************************************************************
//
// Copyright (c) 2014-2025, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <swri_serial_util/serial_port.h>

#include <errno.h> // NOLINT
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <linux/serial.h>
#include <cstring>

namespace swri_serial_util
{
  SerialConfig::SerialConfig() :
      baud(B115200),
      data_bits(8),
      stop_bits(1),
      parity(NO_PARITY),
      flow_control(false),
      low_latency_mode(false),
      writable(false)
  {
  }

  SerialConfig::SerialConfig(
      int32_t baud,
      int32_t data_bits,
      int32_t stop_bits,
      Parity parity,
      bool flow_control,
      bool low_latency_mode,
      bool writable) :
          baud(baud),
          data_bits(data_bits),
          stop_bits(stop_bits),
          parity(parity),
          flow_control(flow_control),
          low_latency_mode(low_latency_mode),
          writable(writable)
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
      error_msg_ = "Invalid baud rate: " + std::to_string(config.baud);
      return false;
    }

    if (config.stop_bits != 1 && config.stop_bits != 2)
    {
      error_msg_ = "Invalid stop bits: " + std::to_string(config.stop_bits);
      return false;
    }

    if (config.data_bits != 7 && config.data_bits != 8)
    {
      error_msg_ = "Invalid data bits: " + std::to_string(config.data_bits);
      return false;
    }

    if (config.parity != SerialConfig::NO_PARITY &&
        config.parity != SerialConfig::EVEN_PARITY &&
        config.parity != SerialConfig::ODD_PARITY)
    {
      error_msg_ = "Invalid parity mode.";
      return false;
    }

    fd_ = open(device.c_str(), config.writable ? O_RDWR : O_RDONLY);
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
      error_msg_ = "Invalid baud rate: " + std::to_string(config.baud);
      Close();
      return false;
    }

    if (tcsetattr(fd_, TCSAFLUSH, &term) < 0)
    {
      error_msg_ = "Unable to set serial port attributes <" + device + ">: " + strerror(errno);
      Close();
      return false;
    }

    if (config.low_latency_mode && !SetLowLatencyMode())
    {
      Close();
      return false;
    }

    return true;
  }

  bool SerialPort::SetLowLatencyMode()
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

    serial_info.flags |= ASYNC_LOW_LATENCY;

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
    {
      value = B50;
    }
    else if (baud == B75 || baud == 75)
    {
      value = B75;
    }
    else if (baud == B110 || baud == 110)
    {
      value = B110;
    }
    else if (baud == B134 || baud == 134)
    {
      value = B134;
    }
    else if (baud == B150 || baud == 150)
    {
      value = B150;
    }
    else if (baud == B200 || baud == 200)
    {
      value = B200;
    }
    else if (baud == B300 || baud == 300)
    {
      value = B300;
    }
    else if (baud == B600 || baud == 600)
    {
      value = B600;
    }
    else if (baud == B1200 || baud == 1200)
    {
      value = B1200;
    }
    else if (baud == B1800 || baud == 1800)
    {
      value = B1800;
    }
    else if (baud == B2400 || baud == 2400)
    {
      value = B2400;
    }
    else if (baud == B4800 || baud == 4800)
    {
      value = B4800;
    }
    else if (baud == B9600 || baud == 9600)
    {
      value = B9600;
    }
    else if (baud == B19200 || baud == 19200)
    {
      value = B19200;
    }
    else if (baud == B38400 || baud == 38400)
    {
      value = B38400;
    }
    else if (baud == B57600 || baud == 57600)
    {
      value = B57600;
    }
    else if (baud == B115200 || baud == 115200)
    {
      value = B115200;
    }
    else if (baud == B230400 || baud == 230400)
    {
      value = B230400;
    }
    else if (baud == B460800 || baud == 460800)
    {
      value = B460800;
    }
    else if (baud == B576000 || baud == 576000)
    {
      value = B576000;
    }
    else if (baud == B921600 || baud == 921600)
    {
      value = B921600;
    }
    else if (baud == B1000000 || baud == 1000000)
    {
      value = B1000000;
    }
    else if (baud == B1152000 || baud == 1152000)
    {
      value = B1152000;
    }
    else if (baud == B1500000 || baud == 1500000)
    {
      value = B1500000;
    }
    else if (baud == B2000000 || baud == 2000000)
    {
      value = B2000000;
    }
    else if (baud == B2500000 || baud == 2500000)
    {
      value = B2500000;
    }
    else if (baud == B3000000 || baud == 3000000)
    {
      value = B3000000;
    }
    else if (baud == B3500000 || baud == 3500000)
    {
      value = B3500000;
    }
    else if (baud == B4000000 || baud == 4000000)
    {
      value = B4000000;
    }

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
      if (bytes < 0)
      {
        error_msg_ = "Error getting number of available bytes from serial port: " + std::string(strerror(errno));
        return ERROR;
      }
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

  int32_t SerialPort::Write(const std::vector<uint8_t>& input)
  {
    return write(fd_, input.data(), input.size());
  }
}
