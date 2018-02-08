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
#ifndef SWRI_ROSCPP_SUBSCRIBER_IMPL_H_
#define SWRI_ROSCPP_SUBSCRIBER_IMPL_H_

#include <std_msgs/Header.h>
#include <ros/subscriber.h>

namespace swri
{
class Subscriber;
class SubscriberImpl
{
 protected:
  ros::Subscriber sub_;
  std::string unmapped_topic_;
  std::string mapped_topic_;

  int message_count_;

  ros::Time last_header_stamp_;
  ros::Time last_receive_time_;

  ros::Duration total_latency_;
  ros::Duration min_latency_;
  ros::Duration max_latency_;

  ros::Duration total_periods_;
  ros::Duration min_period_;
  ros::Duration max_period_;

  ros::Duration timeout_;
  bool in_timeout_;
  int timeout_count_;
  bool blocking_timeout_;

  void processHeader(const ros::Time &stamp)
  {
    ros::Time now = ros::Time::now();

    // Check for timeouts so that we can correctly increment the
    // timeout count.
    checkTimeout(now);

    message_count_++;

    if (!stamp.isZero()) {
      ros::Duration latency = now - stamp;
      if (message_count_ == 1) {
        min_latency_ = latency;
        max_latency_ = latency;
        total_latency_ = latency;
      } else {
        min_latency_ = std::min(min_latency_, latency);
        max_latency_ = std::max(max_latency_, latency);
        total_latency_ += latency;
      }
    }

    if (message_count_ > 1) {
      ros::Duration period = now - last_receive_time_;
      if (message_count_ == 2) {
        min_period_ = period;
        max_period_ = period;
        total_periods_ = period;
      } else if (message_count_ > 2) {
        min_period_ = std::min(min_period_, period);
        max_period_ = std::max(max_period_, period);
        total_periods_ += period;
      }
    }

    // Reset the timeout condition to false.
    in_timeout_ = false;

    last_receive_time_ = now;
    last_header_stamp_ = stamp;
  }

  void checkTimeout(const ros::Time &now)
  {
    if (blocking_timeout_) {
      return;
    }

    if (in_timeout_ || timeout_ <= ros::Duration(0.0)) {
      return;
    }

    if (message_count_ == 0) {
      return;
    }

    if (age(now) > timeout_) {
      in_timeout_ = true;
      timeout_count_++;
    }
  }


 public:
  SubscriberImpl() :
    unmapped_topic_("N/A"),
    mapped_topic_("N/A"),
    message_count_(0),
    timeout_(-1.0),
    in_timeout_(false),
    timeout_count_(0),
    blocking_timeout_(false)
  {
    resetStatistics();
  }

  const std::string& mappedTopic() const
  {
    return mapped_topic_;
  }

  const std::string& unmappedTopic() const
  {
    return unmapped_topic_;
  }

  int numPublishers() const
  {
    return sub_.getNumPublishers();
  }

  void resetStatistics()
  {
    message_count_ = 0;
    in_timeout_ = false;
    timeout_count_ = 0;
  }

  int messageCount() const
  {
    return message_count_;
  }

  ros::Duration age(const ros::Time &now) const
  {
    if (message_count_ < 1) {
      return ros::DURATION_MAX;
    } else if (last_header_stamp_.isValid()) {
      return now - last_header_stamp_;
    } else {
      // If we've received messages but they don't have valid stamps, we can't
      // actually determine the age, so just return an empty duration.
      return ros::Duration(0.0);
    }
  }

  ros::Duration meanLatency() const
  {
    if (message_count_ < 1) {
      return ros::DURATION_MAX;
    } else {
      return ros::Duration(total_latency_.toSec() / message_count_);
    }
  }

  ros::Duration minLatency() const
  {
    if (message_count_ < 1) {
      return ros::DURATION_MAX;
    } else {
      return min_latency_;
    }
  }

  ros::Duration maxLatency() const
  {
    if (message_count_ < 1) {
      return ros::DURATION_MAX;
    } else {
      return max_latency_;
    }
  }

  double meanFrequencyHz() const
  {
    if (message_count_ < 2) {
      return 0.0;
    } else {
      return 1e9 / meanPeriod().toNSec();
    }
  }

  ros::Duration meanPeriod() const
  {
    if (message_count_ < 2) {
      return ros::DURATION_MAX;
    } else {
      return ros::Duration(total_periods_.toSec() / (message_count_ - 1));
    }
  }

  ros::Duration minPeriod() const
  {
    if (message_count_ < 2) {
      return ros::DURATION_MAX;
    } else {
      return min_period_;
    }
  }

  ros::Duration maxPeriod() const
  {
    if (message_count_ < 2) {
      return ros::DURATION_MAX;
    } else {
      return max_period_;
    }
  }

  void setTimeout(const ros::Duration &time_out)
  {
    timeout_ = time_out;
    in_timeout_ = false;
    timeout_count_ = 0;
  }

  bool blockTimeouts(bool block) {
    if (block) {
      in_timeout_ = false;
    }

    bool old_block = blocking_timeout_;
    blocking_timeout_ = block;
    return old_block;
  }

  bool timeoutsBlocked() const {
    return blocking_timeout_;
  }

  ros::Duration timeout() const
  {
    return timeout_;
  }

  bool timeoutEnabled() const
  {
    return timeout_ > ros::Duration(0.0);
  }

  bool inTimeout()
  {
    checkTimeout(ros::Time::now());
    return in_timeout_;
  }

  int timeoutCount()
  {
    checkTimeout(ros::Time::now());
    return timeout_count_;
  }
};  // class SubscriberImpl

struct TrueType
{
  static const bool value = true;
};

template<class M , class T>
class TypedSubscriberImpl : public SubscriberImpl
{
  T *obj_;
  void (T::*callback_)(const boost::shared_ptr< M const > &);

 public:
  TypedSubscriberImpl(
    ros::NodeHandle &nh,
    const std::string &topic,
    uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr< M const > &),
    T *obj,
    const ros::TransportHints &transport_hints)
  {
    unmapped_topic_ = topic;
    mapped_topic_ = nh.resolveName(topic);

    if (unmapped_topic_ == mapped_topic_) {
      ROS_INFO("Subscribing to '%s'.", mapped_topic_.c_str());
    } else {
      ROS_INFO("Subscribing to '%s' at '%s'.",
               unmapped_topic_.c_str(),
               mapped_topic_.c_str());
    }

    callback_ = fp;
    obj_ = obj;

    sub_ = nh.subscribe(mapped_topic_, queue_size,
                        &TypedSubscriberImpl::handleMessage<M>,
                        this,
                        transport_hints);
  }

  // Handler for messages with headers
  template <class M2>
  typename boost::enable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const boost::shared_ptr< M const> &msg)
  {
    processHeader(msg->header.stamp);
    (obj_->*callback_)(msg);
  }

  // Handler for messages without headers
  template <class M2>
  typename boost::disable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const boost::shared_ptr< M const> &msg)
  {
    processHeader(ros::Time::now());
    (obj_->*callback_)(msg);
  }
};  // class TypedSubscriberImpl

template<class M>
class BindSubscriberImpl : public SubscriberImpl
{
  boost::function<void(const boost::shared_ptr< M const> &)> callback_;
  

 public:
  BindSubscriberImpl(
    ros::NodeHandle &nh,
    const std::string &topic,
    uint32_t queue_size,
    const boost::function<void(const boost::shared_ptr< M const> &)> &callback,
    const ros::TransportHints &transport_hints)
  {
    unmapped_topic_ = topic;
    mapped_topic_ = nh.resolveName(topic);

    if (unmapped_topic_ == mapped_topic_) {
      ROS_INFO("Subscribing to '%s'.", mapped_topic_.c_str());
    } else {
      ROS_INFO("Subscribing to '%s' at '%s'.",
               unmapped_topic_.c_str(),
               mapped_topic_.c_str());
    }

    callback_ = callback;

    sub_ = nh.subscribe(mapped_topic_, queue_size,
                        &BindSubscriberImpl::handleMessage<M>,
                        this,
                        transport_hints);
  }

  // Handler for messages with headers
  template <class M2>
  typename boost::enable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const boost::shared_ptr< M const> &msg)
  {
    processHeader(msg->header.stamp);
    callback_(msg);
  }

  // Handler for messages without headers
  template <class M2>
  typename boost::disable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const boost::shared_ptr< M const> &msg)
  {
    processHeader(ros::Time::now());
    callback_(msg);
  }
};  // class BindSubscriberImpl

template<class M>
class StorageSubscriberImpl : public SubscriberImpl
{
  boost::shared_ptr< M const > *dest_;

 public:
  StorageSubscriberImpl(
    ros::NodeHandle &nh,
    const std::string &topic,
    boost::shared_ptr< M const > *dest,
    const ros::TransportHints &transport_hints)
  {
    unmapped_topic_ = topic;
    mapped_topic_ = nh.resolveName(topic);

    if (unmapped_topic_ == mapped_topic_) {
      ROS_INFO("Subscribing to '%s'.", mapped_topic_.c_str());
    } else {
      ROS_INFO("Subscribing to '%s' at '%s'.",
               unmapped_topic_.c_str(),
               mapped_topic_.c_str());
    }

    dest_ = dest;

    sub_ = nh.subscribe(mapped_topic_, 2,
                        &StorageSubscriberImpl::handleMessage<M>,
                        this,
                        transport_hints);
  }

  // Handler for messages with headers
  template <class M2>
  typename boost::enable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const boost::shared_ptr< M const> &msg)
  {
    processHeader(msg->header.stamp);
    *dest_ = msg;
  }

  // Handler for messages without headers
  template <class M2>
  typename boost::disable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const boost::shared_ptr< M const> &msg)
  {
    processHeader(ros::Time::now());
    *dest_ = msg;
  }
};  // class StorageSubscriberImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_SUBSCRIBER_IMPL_H_
