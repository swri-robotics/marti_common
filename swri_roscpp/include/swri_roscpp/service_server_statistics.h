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
#ifndef SWRI_ROSCPP_SERVICE_SERVER_STATISTICS_H_
#define SWRI_ROSCPP_SERVICE_SERVER_STATISTICS_H_

#include <chrono>

namespace swri
{
class ServiceServerImpl;
class ServiceServerStatistics
{
  int servings_;
  int succeeded_;
  int failed_;
  bool last_failed_;

  std::chrono::nanoseconds total_time_;
  std::chrono::nanoseconds min_time_;
  std::chrono::nanoseconds max_time_;

  void merge(bool success, const std::chrono::nanoseconds &runtime);
  friend class ServiceServerImpl;

 public:
  ServiceServerStatistics() { reset(); }
  void reset();

  int servings() const { return servings_; }
  int succeeded() const { return succeeded_; }
  int failed() const { return failed_; }
  bool lastFailed() const { return last_failed_; }

  std::chrono::nanoseconds meanTime() const;
  std::chrono::nanoseconds minTime() const { return min_time_; }
  std::chrono::nanoseconds maxTime() const { return max_time_; }
};  // struct ServiceServerStatistics


inline
void ServiceServerStatistics::reset()
{
  servings_ = 0;
  succeeded_ = 0;
  failed_ = 0;
  last_failed_ = false;
  total_time_ = std::chrono::nanoseconds(0);
  min_time_ = std::chrono::nanoseconds(0);
  max_time_ = std::chrono::nanoseconds(0);
}

inline
std::chrono::nanoseconds ServiceServerStatistics::meanTime() const
{
  if (servings_ == 0) {
    return std::chrono::nanoseconds(0);
  } else {
    return std::chrono::nanoseconds(total_time_.count() / 1000000000 / servings_);
  }
}

inline
void ServiceServerStatistics::merge(
  bool success, const std::chrono::nanoseconds &runtime)
{
  servings_++;
  if (success) {
    succeeded_++;
    last_failed_ = false;
  } else {
    failed_++;
    last_failed_ = true;
  }

  if (servings_ == 1) {
    total_time_ = runtime;
    min_time_ = runtime;
    max_time_ = runtime;
  } else {
    total_time_ += runtime;
    min_time_ = std::min(min_time_, runtime);
    max_time_ = std::max(max_time_, runtime);
  }
}
}  // namespace swri
#endif  // SWRI_ROSCPP_SERVICE_SERVER_STATISTICS_H_
