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

#ifndef SERIAL_UTIL_SERIAL_PORT_H_
#define SERIAL_UTIL_SERIAL_PORT_H_

#include <stdint.h>

#include <string>
#include <vector>

namespace serial_util
{

  struct SerialConfig
  {
    enum Parity
    {
      NO_PARITY,
      EVEN_PARITY,
      ODD_PARITY
    };

    SerialConfig();

    SerialConfig(
        int32_t baud,
        int32_t data_bits,
        int32_t stop_bits,
        Parity parity,
        bool flow_control,
        bool low_latency_mode);

    int32_t baud;
    int32_t data_bits;
    int32_t stop_bits;
    Parity parity;
    bool flow_control;
    bool low_latency_mode;
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

    SerialPort();
    ~SerialPort();

    /**
     * Open and configure the serial port.
     *
     * The default configuration is:
     *   baud = 115200
     *   parity = false
     *   flow control = false
     *   data bits = 8
     *   stop bits = 1
     *   low latency mode = false
     */
    bool Open(const std::string &device, SerialConfig config = SerialConfig());

    /**
     * Close the serial port.
     */
    void Close();

    /**
     * Read bytes from the serial port.
     *
     * Appends up to max_bytes into the provided vector.  If max_bytes
     * is 0, it reads all available bytes.
     */
    Result ReadBytes(std::vector<uint8_t>* output, size_t max_bytes, int32_t timeout);

    std::string ErrorMsg() const { return error_msg_; }

   private:
    /**
     * Attempts to put serial port in low latency mode.
     */
    bool SetLowLatencyMode(bool enabled = true);

    int32_t ParseBaudRate(int32_t baud);
  };
}

#endif  // SERIAL_UTIL_SERIAL_PORT_H_
