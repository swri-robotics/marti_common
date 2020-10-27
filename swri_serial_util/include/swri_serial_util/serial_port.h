// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#ifndef SERIAL_UTIL_SERIAL_PORT_H_
#define SERIAL_UTIL_SERIAL_PORT_H_

#include <cstdint>

#include <string>
#include <vector>

namespace swri_serial_util
{
  /**
   * Structure defining serial port configuration parameters.
   */
  struct SerialConfig
  {
    enum Parity
    {
      NO_PARITY,
      EVEN_PARITY,
      ODD_PARITY
    };

    /**
     * Default constructor.
     *
     * baud = 115200
     * parity = NO_PARITY
     * flow control = false
     * data bits = 8
     * stop bits = 1
     * low latency mode = false
     */
    SerialConfig();

    SerialConfig(
        int32_t baud,
        int32_t data_bits,
        int32_t stop_bits,
        Parity parity,
        bool flow_control,
        bool low_latency_mode,
        bool writable);

    int32_t baud;
    int32_t data_bits;
    int32_t stop_bits;
    Parity parity;
    bool flow_control;
    bool low_latency_mode;
    bool writable;
  };

  class SerialPort
  {
   public:

    enum Result
    {
      SUCCESS,
      TIMEOUT,
      INTERRUPTED,
      ERROR
    };

    int fd_;
    std::string error_msg_;

    /**
     * Constructor.
     */
    SerialPort();

    /**
     * Destructor.
     *
     * Closes serial port if open.
     */
    ~SerialPort();

    /**
     * Open and configure the serial port.
     *
     * The default configuration is:
     *   baud = 115200
     *   parity = NO_PARITY
     *   flow control = false
     *   data bits = 8
     *   stop bits = 1
     *   low latency mode = false
     *
     * @param[in]  device  The OS path of the device.
     * @param[in]  config  The port configuration settings.
     */
    virtual bool Open(const std::string &device, SerialConfig config = SerialConfig());

    /**
     * Close the serial port.
     */
    virtual void Close();

    /**
     * Read bytes from the serial port.
     *
     * Appends up to max_bytes into the provided vector.  If max_bytes is 0,
     * it reads all available bytes.
     *
     * @param[out]  output     The output buffer for bytes read in.
     * @param[in]   max_bytes  The maximum number of bytes to read.  If set
     *                         to 0, all available bytes are read.
     * @param[in]   timeout    The maximum time to block in milliseconds
     *
     * @returns Read result (SUCCESS, TIMEOUT, INTERRUPTED, or ERROR).
     */
    virtual Result ReadBytes(std::vector<uint8_t>& output, size_t max_bytes, int32_t timeout);

    virtual int32_t Write(const std::vector<uint8_t>& input);

    /**
     * Get the most recent error message.
     */
    std::string ErrorMsg() const { return error_msg_; }

   private:
    /**
     * Attempts to put serial port in low latency mode.
     */
    bool SetLowLatencyMode();

    /**
     * Parses integer and enumerated baud rates into enumerated baud rates.
     *
     * @param[in]  baud  The baud rate (either integer or enumerated)
     *
     * @returns  The enumerated baud rate or -1 if invalid.
     */
    int32_t ParseBaudRate(int32_t baud);
  };
}

#endif  // SERIAL_UTIL_SERIAL_PORT_H_
