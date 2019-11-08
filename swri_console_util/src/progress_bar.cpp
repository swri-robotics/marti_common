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

#include <swri_console_util/progress_bar.h>

#include <ctime>
#include <iomanip>
#include <sstream>

namespace swri_console_util
{
  ProgressBar::ProgressBar() :
      paused_(false),
      percent_complete_(0),
      start_time_(clock_.now()),
      paused_time_(0)
  {
    SetupTerminal();
  }

  ProgressBar::~ProgressBar()
  {
    RestoreTerminal();
  }

  void ProgressBar::SetStartTime(const rclcpp::Time& start_time)
  {
    start_time_ = start_time;
  }

  void ProgressBar::SetProgress(double percent_complete)
  {
    percent_complete_ = percent_complete;
  }

  void ProgressBar::PrintTime()
  {
    rclcpp::Time current_time = clock_.now();
    rclcpp::Duration elapsed = (current_time - start_time_) - paused_time_;

    if (percent_complete_ > 0)
    {
      rclcpp::Duration time_left =
        (elapsed * (1.0 / percent_complete_)) - elapsed;

      if (paused_)
      {
        printf("\r [PAUSED]  %.2f%% Complete,   Elapsed: %s   Estimated Remaining: %s  \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.seconds()).c_str(),
            GetTimeString(time_left.seconds()).c_str());
      }
      else
      {
        printf("\r [RUNNING] %.2f%% Complete,   Elapsed: %s   Estimated Remaining: %s  \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.seconds()).c_str(),
            GetTimeString(time_left.seconds()).c_str());
      }
    }
    else
    {
      if (paused_)
      {
        printf("\r [PAUSED]  %.2f%% Complete,   Elapsed: %s                                   \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.seconds()).c_str());
      }
      else
      {
        printf("\r [RUNNING] %.2f%% Complete,   Elapsed: %s                                   \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.seconds()).c_str());
      }
    }

    fflush(stdout);
  }

  void ProgressBar::CheckForPause()
  {
    rclcpp::Time start_pause = clock_.now();
    do
    {
      bool charsleftorpaused = true;
      while (charsleftorpaused && rclcpp::ok())
      {
        switch (ReadCharFromStdin())
        {
          case ' ':
            paused_ = !paused_;

            if (paused_)
            {
              PrintTime();
            }
            charsleftorpaused = paused_;
            break;
          case EOF:
            charsleftorpaused = paused_;
            break;
        }
      }
    }
    while (paused_ && rclcpp::ok());

    paused_time_ = paused_time_ + (clock_.now() - start_pause);

    PrintTime();
  }

  char ProgressBar::ReadCharFromStdin()
  {
    fd_set testfd = stdin_fdset_;

    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;

    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
    {
      return EOF;
    }

    return getc(stdin);
  }

  void ProgressBar::SetupTerminal()
  {
    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for
                                   // non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);

    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
  }

  void ProgressBar::RestoreTerminal()
  {
    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);
  }

  std::string ProgressBar::GetTimeString(double seconds)
  {
    int days = static_cast<int>(seconds / 86400.0);
    seconds -= days * 86400.0;

    int hours = static_cast<int>(seconds / 3600.0);
    seconds -= hours * 3600.0;

    int minutes = static_cast<int>(seconds / 60.0);
    seconds -= minutes * 60.0;

    std::string time;
    std::string unit;
    if (days > 0)
    {
      time += IntToString(days, 2) + ":";
      unit = "d";
    }

    if (hours > 0 || !time.empty())
    {
      if (time.empty())
      {
        unit = "h";
      }

      time += IntToString(hours, 2) + ":";
    }

    if (minutes > 0 || !time.empty())
    {
      if (time.empty())
      {
        unit = "m";
      }

      time += IntToString(minutes, 2) + ":";
    }

    if (time.empty())
    {
      unit = "s";
    }

    time += IntToString(seconds, 2) + unit;

    return time;
  }

  std::string ProgressBar::IntToString(int64_t i, int width)
  {
    std::stringstream ss;
    std::string s;
    ss << std::setfill('0');
    ss << std::setw(width);
    ss << i;
    s = ss.str();
    return s;
  }
}
