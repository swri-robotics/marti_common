// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#ifndef SWRI_ROSCPP_H_
#define SWRI_ROSCPP_H_

namespace swri
{
class Timer;
class TimerImpl
{
 protected:
  ros::Timer timer_;
  ros::Duration desired_period_;

  int ticks_;

  ros::WallTime tick_begin_wall_;
  ros::Time tick_begin_normal_;

  ros::Duration total_periods_;
  ros::Duration min_period_;
  ros::Duration max_period_;

  ros::WallDuration total_durations_;
  ros::WallDuration min_duration_;
  ros::WallDuration max_duration_;

  void tickBegin()
  {
    tick_begin_wall_ = ros::WallTime::now();

    ros::Time now = ros::Time::now();
    if (ticks_ > 0) {
      ros::Duration period = now - tick_begin_normal_;
      total_periods_ += period;

      if (ticks_ == 1) {
        min_period_ = period;
        max_period_ = period;
      } else {
        min_period_ = std::min(min_period_, period);
        max_period_ = std::max(max_period_, period);
      }
    }
    tick_begin_normal_ = now;
  }

  void tickEnd()
  {
    ros::WallTime end_time_ = ros::WallTime::now();
    ros::WallDuration duration = end_time_ - tick_begin_wall_;
    total_durations_ += duration;
    if (ticks_ == 0) {
      min_duration_ = duration;
      max_duration_ = duration;
    } else {
      min_duration_ = std::min(min_duration_, duration);
      max_duration_ = std::max(max_duration_, duration);
    }
    ticks_++;
  }

 public:
  TimerImpl()
  {
    resetStatistics();
  }

  ros::Duration desiredPeriod() const
  {
    return desired_period_;
  }
  
  void resetStatistics()
  {
    ticks_ = 0;
  }

  // The number of times the timer callback has been called.
  size_t ticks() const
  {
    return ticks_;
  }

  double meanFrequencyHz() const
  {
    if (ticks_ < 2) {
      return 0.0;
    } else {
      return 1e9 / meanPeriod().toNSec();
    }
  }
  
  ros::Duration meanPeriod() const
  {
    if (ticks_ < 2) {
      return ros::DURATION_MAX;
    } else {
      return ros::Duration(total_periods_.toSec() / (ticks_ - 1));
    }
  }
  
  ros::Duration minPeriod() const
  {
    if (ticks_ < 2) {
      return ros::DURATION_MAX;
    } else {
      return min_period_;
    }
  }
  
  ros::Duration maxPeriod() const
  {
    if (ticks_ < 2) {
      return ros::DURATION_MAX;
    } else {
      return max_period_;
    }
  }
  
  ros::WallDuration meanDuration() const
  {
    if (ticks_ == 0) {
      return ros::WallDuration(0.0);
    } else {
      return ros::WallDuration(total_durations_.toSec() / ticks_);
    }
  }
  
  ros::WallDuration minDuration() const
  {
    if (ticks_ == 0) {
      return ros::WallDuration(0.0);
    } else {
      return min_duration_;
    }
  }
  
  ros::WallDuration maxDuration() const
  {
    if (ticks_ == 0) {
      return ros::WallDuration(0.0);
    } else {
      return max_duration_;
    }
  }
};  // class TimerImpl

template<class T>
class TypedTimerImpl : public TimerImpl
{
  T *obj_;
  void (T::*callback_)(const ros::TimerEvent &);

 public:
  TypedTimerImpl(
    ros::NodeHandle &nh,
    ros::Duration period,
    void(T::*callback)(const ros::TimerEvent&),
    T *obj)
  {
    callback_ = callback;
    obj_ = obj;

    desired_period_ = period;
    timer_ = nh.createTimer(period,
                            &TypedTimerImpl::handleTimer,
                            this);
  }
 
  void handleTimer(const ros::TimerEvent &event)
  {
    tickBegin();
    (obj_->*callback_)(event);
    tickEnd();
  }
};  // class TypedTimerImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_H_
