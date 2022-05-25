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
#ifndef SWRI_ROSCPP_TIMER_IMPL_H_
#define SWRI_ROSCPP_TIMER_IMPL_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <chrono>

namespace swri
{
class Timer;
class TimerImpl
{
 protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration desired_period_ = rclcpp::Duration(0, 0);

  int ticks_;

  typedef std::chrono::nanoseconds WallDuration;
  typedef std::chrono::system_clock::time_point WallTime;

  WallTime tick_begin_wall_;
  rclcpp::Time tick_begin_normal_;

  rclcpp::Duration total_periods_ = rclcpp::Duration(std::chrono::nanoseconds::zero());
  rclcpp::Duration min_period_ = rclcpp::Duration(std::chrono::nanoseconds::zero());
  rclcpp::Duration max_period_ = rclcpp::Duration(std::chrono::nanoseconds::zero());

  WallDuration total_durations_;
  WallDuration min_duration_;
  WallDuration max_duration_;

  void tickBegin()
  {
    tick_begin_wall_ = std::chrono::system_clock::now();

    rclcpp::Time now = rclcpp::Clock().now();
    if (ticks_ > 0) {
      rclcpp::Duration period = now - tick_begin_normal_;
      total_periods_ = total_periods_ + period;

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
    WallTime end_time_ = std::chrono::system_clock::now();
    WallDuration duration = end_time_ - tick_begin_wall_;
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

  rclcpp::Duration desiredPeriod() const
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
      return 1e9 / meanPeriod().nanoseconds();
    }
  }
  
  rclcpp::Duration meanPeriod() const
  {
    if (ticks_ < 2) {
      return rclcpp::Duration::max();
    } else {
      return rclcpp::Duration::from_seconds(total_periods_.seconds() / (ticks_ - 1));
    }
  }
  
  rclcpp::Duration minPeriod() const
  {
    if (ticks_ < 2) {
      return rclcpp::Duration::max();
    } else {
      return min_period_;
    }
  }
  
  rclcpp::Duration maxPeriod() const
  {
    if (ticks_ < 2) {
      return rclcpp::Duration::max();
    } else {
      return max_period_;
    }
  }

  WallDuration meanDuration() const
  {
    if (ticks_ == 0) {
      return WallDuration(0);
    } else {
      return WallDuration(total_durations_.count() / 100000000 / ticks_);
    }
  }

  WallDuration minDuration() const
  {
    if (ticks_ == 0) {
      return WallDuration(0);
    } else {
      return min_duration_;
    }
  }

  WallDuration maxDuration() const
  {
    if (ticks_ == 0) {
      return WallDuration(0);
    } else {
      return max_duration_;
    }
  }
};  // class TimerImpl

template<class T>
class TypedTimerImpl : public TimerImpl
{
  T *obj_;
  void (T::*callback_)();

 public:
  TypedTimerImpl(
    rclcpp::Node &nh,
    rclcpp::Duration period,
    void(T::*callback)(),
    T *obj)
  {
    callback_ = callback;
    obj_ = obj;

    desired_period_ = period;
    timer_ = nh.create_wall_timer(std::chrono::nanoseconds(period.nanoseconds()),
                            std::bind(&TypedTimerImpl::handleTimer,
                            this));
  }
 
  void handleTimer()
  {
    tickBegin();
    (obj_->*callback_)();
    tickEnd();
  }
};  // class TypedTimerImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_TIMER_IMPL_H_
