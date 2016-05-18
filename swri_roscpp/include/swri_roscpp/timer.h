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
#ifndef SWRI_ROSCPP_TIMER_H_
#define SWRI_ROSCPP_TIMER_H_

#include <ros/node_handle.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <swri_roscpp/timer_impl.h>

namespace swri
{
class Timer
{
 private:
  boost::shared_ptr<TimerImpl> impl_;

 public:
  Timer();

  template<class T>
  Timer(ros::NodeHandle &nh,
        ros::Duration period,
        void(T::*callback)(const ros::TimerEvent&),
        T *obj);

  Timer& operator=(const Timer &other);

  ros::Duration desiredPeriod() const;
  double desiredFrequency() const;
  
  void resetStatistics();

  // The number of times the timer callback has been called.
  size_t ticks() const;

  // Frequency/Period of the timer callbacks.
  double meanFrequencyHz() const;
  ros::Duration meanPeriod() const;
  ros::Duration minPeriod() const;
  ros::Duration maxPeriod() const;
  double meanPeriodMilliseconds() const;
  double minPeriodMilliseconds() const;
  double maxPeriodMilliseconds() const;

  // Average duration of the timer callback.  These are tracked as
  // wall durations because they are typically used as a rough profile
  // of how long the callback takes to execute which is independent of
  // simulated time.
  ros::WallDuration meanDuration() const;
  ros::WallDuration minDuration() const;
  ros::WallDuration maxDuration() const;
  double meanDurationMicroseconds() const;
  double minDurationMicroseconds() const;
  double maxDurationMicroseconds() const;

  enum DIAGNOSTIC_FLAGS {
    DIAG_COUNT    = 1 << 0,
    DIAG_RATE     = 1 << 1,
    DIAG_DURATION = 1 << 2,

    DIAG_ALL      = ~0,
    DIAG_MOST     = DIAG_ALL ^ DIAG_COUNT
  };

  void appendDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status,
                         const std::string &name,
                         const int flags);
};  // class Timer


inline
Timer::Timer()
{
  // Setup an empty implementation so that we can assume impl_ is
  // non-null and avoid a lot of unnecessary NULL checks.
  impl_ = boost::shared_ptr<TimerImpl>(new TimerImpl());
}

template<class T>
inline
Timer::Timer(ros::NodeHandle &nh,
             ros::Duration period,
             void(T::*callback)(const ros::TimerEvent&),
             T *obj)
{
  impl_ = boost::shared_ptr<TimerImpl>(
    new TypedTimerImpl<T>(nh, period, callback, obj));
}

inline
Timer& Timer::operator=(const Timer &other)
{
  impl_ = other.impl_;
  return *this;
}

inline
ros::Duration Timer::desiredPeriod() const
{
  return impl_->desiredPeriod();
}

inline
double Timer::desiredFrequency() const
{
  return 1.0 / desiredPeriod().toSec();
}

inline
void Timer::resetStatistics()
{
  return impl_->resetStatistics();
}

inline
size_t Timer::ticks() const
{
  return impl_->ticks();
}

inline
double Timer::meanFrequencyHz() const
{
  return impl_->meanFrequencyHz();
}

inline
ros::Duration Timer::meanPeriod() const
{
  return impl_->meanPeriod();
}

inline
ros::Duration Timer::minPeriod() const
{
  return impl_->minPeriod();
}

inline
ros::Duration Timer::maxPeriod() const
{
  return impl_->maxPeriod();
}

inline
double Timer::meanPeriodMilliseconds() const
{
  return impl_->meanPeriod().toNSec() / 1000000.0;
}

inline
double Timer::minPeriodMilliseconds() const
{
  return impl_->minPeriod().toNSec() / 1000000.0;
}

inline
double Timer::maxPeriodMilliseconds() const
{
  return impl_->maxPeriod().toNSec() / 1000000.0;
}

inline
ros::WallDuration Timer::meanDuration() const
{
  return impl_->meanDuration();
}

inline
ros::WallDuration Timer::minDuration() const
{
  return impl_->minDuration();
}

inline
ros::WallDuration Timer::maxDuration() const
{
  return impl_->maxDuration();
}

inline
double Timer::meanDurationMicroseconds() const
{
  return impl_->meanDuration().toNSec() / 1000.0;
}

inline
double Timer::minDurationMicroseconds() const
{
  return impl_->minDuration().toNSec() / 1000.0;
}

inline
double Timer::maxDurationMicroseconds() const
{
  return impl_->maxDuration().toNSec() / 1000.0;
}



inline
void Timer::appendDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status,
                              const std::string &name,
                              int flags)
{
  // Alias a type for easier access to DiagnosticStatus enumerations.
  // typedef diagnostic_msgs::DiagnosticStatus DS;

  // todo(anyone): Add an optional limit to set a warning or error
  // some critical measurement crosses as threshold.  Big ones are
  // probably frequency dropping too low and duration getting too
  // large.

  if (flags & DIAG_COUNT) {
    status.addf(name + " ticks", "%d", ticks());
  }

  if (flags & DIAG_RATE) {
    if (ticks() < 2) {
    status.add(
      name + " rates",
      "min period: N/A ms, mean frequency: N/A hz, max period: N/A ms");
    } else {
      status.addf(
        name + " rates",
        "min period: %.2f ms, mean frequency: %.2f hz, max period: %.2f ms",
        minPeriodMilliseconds(),
        meanFrequencyHz(),
        maxPeriodMilliseconds());
    }
  }

  if (flags & DIAG_DURATION) {
    if (ticks() < 1) {
      status.add(name + " duration",
                 "min: N/A us, mean: N/A us, max: N/A us");
    } else {
      status.addf(name + " duration",
                  "min: %.2f us, mean: %.2f us, max: %.2f us",
                  minDurationMicroseconds(),
                  meanDurationMicroseconds(),
                  maxDurationMicroseconds());
    }
  }
}
}  // namespace swri
#endif  // SWRI_ROSCPP_TIMER_H_
