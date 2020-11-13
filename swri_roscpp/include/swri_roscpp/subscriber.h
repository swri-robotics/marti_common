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
#ifndef SWRI_ROSCPP_SUBSCRIBER_H_
#define SWRI_ROSCPP_SUBSCRIBER_H_


#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <swri_roscpp/subscriber_impl.h>

namespace swri
{
// This is an extended interface to the rclcpp::Subscriber class.
//
// This is an extended interface to the rclcpp::Subscriber class that
// provides a little more default functionality and instrumentation.
//
// - Prints information when the subscription is created (to RCLCPP_INFO)
//   about the unmapped and mapped topic names.
//
// - Maintains some statistics about the messages (counts, latency, period)
//
// - Implements timeout logic and timeout counting.
//
// This implementation provides two interfaces.  It supports the
// traditional ROS callback interface (though it is deliberately
// limited to callbacks that take a *ConstPtr& type), and an interface
// where the message is simply assigned to *ConstPtr variable that is
// specified at creation.  The second interface replaces the previous
// swri_roscpp::LatchedSubscriber and allows you to avoid writing
// trivial callback functions.

class Subscriber
{
 private:
  // The implementation is kept in a separate class to maintain clean
  // copy/move semantics with ROS (since the ROS subscriber binds to a
  // single address for the callback) and to allow us to hide the
  // template arguments so the message type doesn't have to be
  // specified when the subscriber is declared.
  std::shared_ptr<SubscriberImpl> impl_;

 public:
  Subscriber();

  // Using class method callback.
  template<class M , class T >
  Subscriber(rclcpp::Node &nh,
             const std::string &topic,
             uint32_t queue_size,
             void(T::*fp)(const std::shared_ptr< M const > &),
             T *obj,
             const rclcpp::QoS& transport_hints = rclcpp::QoS(1));

  // Using a boost function callback.
  template<class M>
  Subscriber(rclcpp::Node &nh,
             const std::string &topic,
             uint32_t queue_size,
             const std::function<void(const std::shared_ptr<M const> &)> &callback,
             const rclcpp::QoS& transport_hints = rclcpp::QoS(1));

  // This is an alternate interface that stores a received message in
  // a variable without calling a user-defined callback function.
  // This is useful for cases where you just want to store a message
  // for usage later and avoids having to write a trivial callback
  // function.
  template<class M>
  Subscriber(rclcpp::Node &nh,
             const std::string &topic,
             std::shared_ptr< M const > *dest,
             const rclcpp::QoS& transport_hints = rclcpp::QoS(1));
  
  Subscriber& operator=(const Subscriber &other);

  // Reset all statistics, including message and timeout counts.
  void resetStatistics();

  // Returns the unmapped topic name that was provided when the
  // subscriber was created.
  const std::string& unmappedTopic() const;
  // Returns the number of publishers that are connected to this
  // subscriber.
  int numPublishers() const;

  // How many messages have been received.
  int messageCount() const;

  // Age of the most recent message (difference between now and the
  // header stamp (or time message was received for messages that
  // don't have headers).
  rclcpp::Duration age(const rclcpp::Time &now = rclcpp::Time(0,0,RCL_ROS_TIME)) const;
  double ageSeconds(const rclcpp::Time &now = rclcpp::Time(0,0,RCL_ROS_TIME)) const;
  double ageMilliseconds(const rclcpp::Time &now = rclcpp::Time(0,0,RCL_ROS_TIME)) const;

  // Average latency (time difference between the time stamp and when
  // the message is received). These will be useless for message types
  // that do not have a header.
  rclcpp::Duration meanLatency() const;
  rclcpp::Duration minLatency() const;
  rclcpp::Duration maxLatency() const;
  double meanLatencyMicroseconds() const;
  double minLatencyMicroseconds() const;
  double maxLatencyMicroseconds() const;

  // Frequency/Period in terms of when the message was received (not
  // the header stamp).
  double meanFrequencyHz() const;
  rclcpp::Duration meanPeriod() const;
  rclcpp::Duration minPeriod() const;
  rclcpp::Duration maxPeriod() const;
  double meanPeriodMilliseconds() const;
  double minPeriodMilliseconds() const;
  double maxPeriodMilliseconds() const;

  // Provide a negative value to disable the timeout (default is -1).
  void setTimeout(const rclcpp::Duration &time_out);
  void setTimeout(const double time_out);
  
  // Block/unblock timeouts from occuring.  This allows you to
  // temporarily block timeouts (for example, if a message is not
  // expected in a particular mode).  Returns the current state
  bool blockTimeouts(bool block);

  // Return true if the subscriber is currently blocking timeouts from
  // occurring.
  bool timeoutsBlocked() const;

  // Determine if the timeout is currently enabled.
  bool timeoutEnabled() const;
  // Read the current timeout setting.
  rclcpp::Duration timeout() const;
  double timeoutMilliseconds() const;

  // Determine if the topic is in a timed out state.
  bool inTimeout();
  // How many times the topic has been in a timeout state.
  int timeoutCount();

  // These flags determine which values are added to a diagnostic
  // status by the appendDiagnostics method.
  enum DIAGNOSTIC_FLAGS {
    DIAG_CONNECTION = 1 << 0,  // Include topic names, publisher counts
    DIAG_MSG_COUNT  = 1 << 1,  // Include message count
    DIAG_TIMEOUT    = 1 << 2,  // Include timeout counts if applicable
    DIAG_LATENCY    = 1 << 3,  // Include latency information
    DIAG_RATE       = 1 << 4,  // Include rate information

    DIAG_ALL        = ~0,       // Abbreviation to include all information
    DIAG_MOST       = DIAG_ALL ^ DIAG_CONNECTION
    // Abbreviation to include everything except connection info.
  };

  // This is a convenience method to add information about this
  // subscription to a diagnostic status in a standard way.
  //
  // The status is merged with a warning if no messages have been
  // received or if timeouts have occurred.  The status is merged with
  // an error if the subscription is in an active timeout status.
  //
  // The flags parameter determines which values are added to the
  // status' key/value pairs.  This should be a bitwise combination of
  // the values defined in DIAGNOSTIC_FLAGS.
  void appendDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status,
                         const std::string &name,
                         const int flags);
};  // class Subscriber

inline
Subscriber::Subscriber()
{
  // Setup an empty implementation so that we can assume impl_ is
  // non-null and avoid a lot of unnecessary NULL checks.
  impl_ = std::make_shared<SubscriberImpl>();
}

template<class M , class T >
inline
Subscriber::Subscriber(rclcpp::Node &nh,
                       const std::string &topic,
                       uint32_t queue_size,
                       void(T::*fp)(const std::shared_ptr< M const > &),
                       T *obj,
                       const rclcpp::QoS& transport_hints)
{
  impl_ = std::shared_ptr<SubscriberImpl>(
    new TypedSubscriberImpl<M,T>(
      nh, topic, queue_size, fp, obj, transport_hints));
}

template<class M>
inline
Subscriber::Subscriber(rclcpp::Node &nh,
                       const std::string &topic,
                       uint32_t queue_size,
                       const std::function<void(const std::shared_ptr<M const> &)> &callback,
                       const rclcpp::QoS& transport_hints)
{
  impl_ = std::shared_ptr<SubscriberImpl>(
    new BindSubscriberImpl<M>(
      nh, topic, queue_size, callback, transport_hints));
}

template<class M>
inline
Subscriber::Subscriber(rclcpp::Node &nh,
                       const std::string &topic,
                       std::shared_ptr< M const > *dest,
                       const rclcpp::QoS& transport_hints)
{
  impl_ = std::shared_ptr<SubscriberImpl>(
    new StorageSubscriberImpl<M>(
      nh, topic, dest, transport_hints));
}

inline
Subscriber& Subscriber::operator=(const Subscriber &other)
{
  // If we have some non-default parameters and the other class
  // doesn't, we preserve our parameters across assignment.
  //
  // This is to support the use case where we read ROS parameters, set
  // them in the subscriber, and then create the subscriptions.
  // Otherwise, the user has to read the timeouts, save them
  // temporarily, and then set them after the subscription is set up.
  //
  // This could cause confusing behavior if you are moving subscribers
  // around a lot, but I've never seen that kind of use case in any
  // ROS code.

  rclcpp::Duration new_timeout = other.impl_->timeout();
  if (impl_->timeoutEnabled() && !other.impl_->timeoutEnabled()) {
    new_timeout = impl_->timeout();
  }

  impl_ = other.impl_;
  impl_->setTimeout(new_timeout);

  return *this;
}

inline
const std::string& Subscriber::unmappedTopic() const
{
  return impl_->unmappedTopic();
}

inline
int Subscriber::numPublishers() const
{
  return impl_->numPublishers();
}

inline
void Subscriber::resetStatistics()
{
  impl_->resetStatistics();
}

inline
int Subscriber::messageCount() const
{
  return impl_->messageCount();
}

inline
rclcpp::Duration Subscriber::age(const rclcpp::Time &now) const
{
  return impl_->age(now);
}

inline
double Subscriber::ageSeconds(const rclcpp::Time &now) const
{
  return age(now).seconds();
}

inline
double Subscriber::ageMilliseconds(const rclcpp::Time &now) const
{
  return age(now).nanoseconds() / 1000000.0;
}

inline
rclcpp::Duration Subscriber::meanLatency() const
{
  return impl_->meanLatency();
}

inline
rclcpp::Duration Subscriber::minLatency() const
{
  return impl_->minLatency();
}

inline
rclcpp::Duration Subscriber::maxLatency() const
{
  return impl_->maxLatency();
}

inline
double Subscriber::meanLatencyMicroseconds() const
{
  return meanLatency().nanoseconds() / 1000.0;
}

inline
double Subscriber::minLatencyMicroseconds() const
{
  return minLatency().nanoseconds() / 1000.0;
}

inline
double Subscriber::maxLatencyMicroseconds() const
{
  return maxLatency().nanoseconds() / 1000.0;
}

inline
rclcpp::Duration Subscriber::meanPeriod() const
{
  return impl_->meanPeriod();
}

inline
double Subscriber::meanFrequencyHz() const
{
  return impl_->meanFrequencyHz();
}

inline
rclcpp::Duration Subscriber::minPeriod() const
{
  return impl_->minPeriod();
}

inline
rclcpp::Duration Subscriber::maxPeriod() const
{
  return impl_->maxPeriod();
}

inline
double Subscriber::meanPeriodMilliseconds() const
{
  return meanPeriod().nanoseconds() / 1000000.0;
}

inline
double Subscriber::minPeriodMilliseconds() const
{
  return minPeriod().nanoseconds() / 1000000.0;
}

inline
double Subscriber::maxPeriodMilliseconds() const
{
  return maxPeriod().nanoseconds() / 1000000.0;
}

inline
void Subscriber::setTimeout(const rclcpp::Duration &time_out)
{
  impl_->setTimeout(time_out);
}

inline
void Subscriber::setTimeout(const double time_out)
{
  setTimeout(std::chrono::duration<double>(time_out));
}

inline
bool Subscriber::blockTimeouts(bool block)
{
  return impl_->blockTimeouts(block);
}

inline
bool Subscriber::timeoutsBlocked() const
{
  return impl_->timeoutsBlocked();
}

inline
rclcpp::Duration Subscriber::timeout() const
{
  return impl_->timeout();
}

inline
bool Subscriber::timeoutEnabled() const
{
  return impl_->timeoutEnabled();
}

inline
double Subscriber::timeoutMilliseconds() const
{
  return impl_->timeout().nanoseconds() / 1.0e6;
}

inline
bool Subscriber::inTimeout()
{
  return impl_->inTimeout();
}

inline
int Subscriber::timeoutCount()
{
  return impl_->timeoutCount();
}

inline
void Subscriber::appendDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status,
                                   const std::string &name,
                                   int flags)
{
  // Alias a type for easier access to DiagnosticStatus enumerations.
  typedef diagnostic_msgs::msg::DiagnosticStatus DS;

  // We are considering no messages seen as a warning because this is
  // a normal condition during start up, but should be resolved once
  // the system is running.  On the other hand, timeouts are typically
  // an unexpected condition that occur when a publisher dies or
  // communications is getting blocked.  Active timeouts affect
  // operation so are treated as an error condition.  If a timeout is
  // not active, but has occurred, we report this as a warning so that
  // we know something has been wrong and needs to be investigated.

  if (!messageCount()) {
    status.mergeSummaryf(DS::WARN, "Waiting for %s messages.", name.c_str());
  } else if (inTimeout()) {
    status.mergeSummaryf(DS::ERROR, "%s timeout.", name.c_str());
  } else if (timeoutCount()) {
    status.mergeSummaryf(DS::WARN, "%s timeouts have occurred.", name.c_str());
  }

  if (flags & DIAG_CONNECTION) {
    status.addf(name + "topic name", "%s", unmappedTopic().c_str());
    status.addf(name + " publishers", "%d", numPublishers());
  }

  if (flags & DIAG_MSG_COUNT) {
    status.add(name + " message count", messageCount());
  }

  if ((flags & DIAG_TIMEOUT) && timeoutEnabled()) {
    status.addf(name + " timeouts count",
                "%d ( > %.2f ms)",
                timeoutCount(),
                timeoutMilliseconds());
  }

  if (flags & DIAG_LATENCY) {
    if (messageCount() < 1) {
      status.add(name + " latency",
                 "min: N/A us, mean: N/A us, max: N/A us");
    } else {
      status.addf(name + " latency",
                  "min: %.2f us, mean: %.2f us, max: %.2f us",
                  minLatencyMicroseconds(),
                  meanLatencyMicroseconds(),
                  maxLatencyMicroseconds());
    }
  }

  if (flags & DIAG_RATE) {
    if (messageCount() < 2) {
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
}
}  // namespace swri
#endif  // SWRI_ROSCPP_SUBSCRIBER_H_
