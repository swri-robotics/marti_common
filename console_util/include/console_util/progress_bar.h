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

#ifndef CONSOLE_UTIL_PROGRESS_BAR_H_
#define CONSOLE_UTIL_PROGRESS_BAR_H_

#include <termios.h>
#include <string>

#include <ros/ros.h>

namespace console_util
{
  class ProgressBar
  {
  public:
    ProgressBar();
    ~ProgressBar();

    void SetStartTime(const ros::WallTime& start_time);
    void SetProgress(double percent_complete);
    void PrintTime();
    void CheckForPause();
    char ReadCharFromStdin();

    static std::string GetTimeString(double seconds);
    static std::string IntToString(int64_t i, int width = 0);

  private:
    void SetupTerminal();
    void RestoreTerminal();

    bool paused_;

    double percent_complete_;

    ros::WallTime start_time_;
    ros::WallDuration paused_time_;

    termios orig_flags_;
    fd_set  stdin_fdset_;
    int     maxfd_;
  };
}

#endif  // CONSOLE_UTIL_PROGRESS_BAR_H_
