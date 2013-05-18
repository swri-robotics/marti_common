// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <console_util/progress_bar.h>

#include <ctime>
#include <sstream>

namespace console_util
{
  ProgressBar::ProgressBar() :
      paused_(false),
      percent_complete_(0),
      start_time_(ros::WallTime::now()),
      paused_time_(0)
  {
    SetupTerminal();
  }

  ProgressBar::~ProgressBar()
  {
    RestoreTerminal();
  }

  void ProgressBar::SetStartTime(const ros::WallTime& start_time)
  {
    start_time_ = start_time;
  }

  void ProgressBar::SetProgress(double percent_complete)
  {
    percent_complete_ = percent_complete;
  }

  void ProgressBar::PrintTime()
  {
    ros::WallTime current_time = ros::WallTime::now();
    ros::WallDuration elapsed = (current_time - start_time_) - paused_time_;

    if (percent_complete_ > 0)
    {
      ros::WallDuration time_left = (elapsed * (1.0 / percent_complete_)) - elapsed;

      if (paused_)
      {
        printf("\r [PAUSED]  %.2f%% Complete,   Elapsed: %s   Estimated Remaining: %s  \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.toSec()).c_str(),
            GetTimeString(time_left.toSec()).c_str());
      }
      else
      {
        printf("\r [RUNNING] %.2f%% Complete,   Elapsed: %s   Estimated Remaining: %s  \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.toSec()).c_str(),
            GetTimeString(time_left.toSec()).c_str());
      }
    }
    else
    {
      if (paused_)
      {
        printf("\r [PAUSED]  %.2f%% Complete,   Elapsed: %s                                   \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.toSec()).c_str());
      }
      else
      {
        printf("\r [RUNNING] %.2f%% Complete,   Elapsed: %s                                   \r",
            percent_complete_ * 100.0,
            GetTimeString(elapsed.toSec()).c_str());
      }
    }

    fflush(stdout);
  }

  void ProgressBar::CheckForPause()
  {
    ros::WallTime start_pause = ros::WallTime::now();
    do
    {
      bool charsleftorpaused = true;
      while (charsleftorpaused && ros::ok())
      {
        switch (ReadCharFromStdin())
        {
          case ' ':
            paused_ = !paused_;

            if (paused_)
            {
              PrintTime();
            }
          case EOF:
            charsleftorpaused = paused_;
        }
      }
    }
    while (paused_ && ros::ok());

    paused_time_ += (ros::WallTime::now() - start_pause);

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
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
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
