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

#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/utility/enable_if.hpp>

namespace swri
{

  class Subscriber;
  class SubscriberImpl
  {
  public:
    rclcpp::Node* nh_;

  protected:
    rclcpp::SubscriptionBase::SharedPtr sub_;
    std::string unmapped_topic_;

    int message_count_;

    rclcpp::Time last_header_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_receive_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    rclcpp::Duration total_latency_ = rclcpp::Duration(0);
    rclcpp::Duration min_latency_ = rclcpp::Duration::max();
    rclcpp::Duration max_latency_ = rclcpp::Duration(0);

    rclcpp::Duration total_periods_ = rclcpp::Duration::max();
    rclcpp::Duration min_period_ = rclcpp::Duration::max();
    rclcpp::Duration max_period_ = rclcpp::Duration(0);

    rclcpp::Duration timeout_ = rclcpp::Duration(0, 0);
    bool in_timeout_;
    int timeout_count_;
    bool blocking_timeout_;

    void processHeader(const rclcpp::Time &stamp)
    {
      rclcpp::Time now = nh_->now();

      // Check for timeouts so that we can correctly increment the
      // timeout count.
      checkTimeout(now);

      message_count_++;

      if (stamp.nanoseconds() != 0) {
        rclcpp::Duration latency = now - stamp;
        if (message_count_ == 1) {
          min_latency_ = latency;
          max_latency_ = latency;
          total_latency_ = latency;
        } else {
          min_latency_ = std::min(min_latency_, latency);
          max_latency_ = std::max(max_latency_, latency);
          total_latency_ = total_latency_ + latency;
        }
      }

      if (message_count_ > 1) {
        rclcpp::Duration period = now - last_receive_time_;
        if (message_count_ == 2) {
          min_period_ = period;
          max_period_ = period;
          total_periods_ = period;
        } else if (message_count_ > 2) {
          min_period_ = std::min(min_period_, period);
          max_period_ = std::max(max_period_, period);
          total_periods_ = total_periods_ + period;
        }
      }

      // Reset the timeout condition to false.
      in_timeout_ = false;

      last_receive_time_ = now;
      last_header_stamp_ = stamp;
    }

    void checkTimeout(const rclcpp::Time &now)
    {
      if (blocking_timeout_) {
        return;
      }

      if (in_timeout_ || timeout_ <= rclcpp::Duration(0, 0)) {
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
    SubscriberImpl()
    {
      unmapped_topic_ = "N/A";
      timeout_ = rclcpp::Duration(-1,0);
      blocking_timeout_ = false;
      resetStatistics();
    }

    /*const std::string& mappedTopic() const
    {
      return mapped_topic_;
    }*/

    const std::string& unmappedTopic() const
    {
      return unmapped_topic_;
    }

    int numPublishers() const
    {
      return 0;//sub_.getNumPublishers();
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

    rclcpp::Duration age(const rclcpp::Time &now) const
    {
      if (message_count_ < 1) {
        return rclcpp::Duration::max();
      } else if (now == rclcpp::Time(0,0,RCL_ROS_TIME)) {
        return nh_->now() - last_header_stamp_;
      } else {
        return now - last_header_stamp_;
      }
    }

    rclcpp::Duration meanLatency() const
    {
      if (message_count_ < 1) {
        return rclcpp::Duration::max();
      } else {
        return rclcpp::Duration(total_latency_.seconds() / message_count_);
      }
    }

    rclcpp::Duration minLatency() const
    {
      if (message_count_ < 1) {
        return rclcpp::Duration::max();
      } else {
        return min_latency_;
      }
    }

    rclcpp::Duration maxLatency() const
    {
      if (message_count_ < 1) {
        return rclcpp::Duration::max();
      } else {
        return max_latency_;
      }
    }

    double meanFrequencyHz() const
    {
      if (message_count_ < 2) {
        return 0.0;
      } else {
        return 1e9 / meanPeriod().nanoseconds();
      }
    }

    rclcpp::Duration meanPeriod() const
    {
      if (message_count_ < 2) {
        return rclcpp::Duration::max();
      } else {
        return rclcpp::Duration(total_periods_.seconds() / (message_count_ -1));
      }
    }

    rclcpp::Duration minPeriod() const
    {
      if (message_count_ < 2) {
        return rclcpp::Duration::max();
      } else {
        return min_period_;
      }
    }

    rclcpp::Duration maxPeriod() const
    {
      if (message_count_ < 2) {
        return rclcpp::Duration::max();
      } else {
        return max_period_;
      }
    }

    void setTimeout(const rclcpp::Duration &time_out)
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

    rclcpp::Duration timeout() const
    {
      return timeout_;
    }

    bool timeoutEnabled() const
    {
      return timeout_ > rclcpp::Duration(0, 0);
    }

    bool inTimeout()
    {
      checkTimeout(nh_->now());
      return in_timeout_;
    }

    int timeoutCount()
    {
      checkTimeout(nh_->now());
      return timeout_count_;
    }
  };  // class SubscriberImpl

#include <type_traits>

// This comes from cppreference
  template<typename... Ts> struct make_void { typedef void type;};
  template<typename... Ts> using void_t = typename make_void<Ts...>::type;

// primary template handles types that have no ::aMember
  template< class T, class = void_t<> >
  struct has_header : std::false_type { };

// specialization recognizes types that do have a ::aMember
  template< class T >
  struct has_header<T, void_t<decltype( T::header )>> : std::true_type { };

  template<class M , class T>
  class TypedSubscriberImpl : public SubscriberImpl
  {
    T *obj_;
    void (T::*callback_)(const std::shared_ptr< const M > &);

  public:
    TypedSubscriberImpl(
        rclcpp::Node& nh,
        const std::string &topic,
        uint32_t queue_size,
        void(T::*fp)(const std::shared_ptr< M const > &),
        T *obj,
        const rclcpp::QoS& transport_hints)
    {
      unmapped_topic_ = topic;
      // mapped_topic_ = nh->ResolveName(topic);
      // nh_ = nh->nh_;
      nh_ = &nh;

      RCLCPP_INFO(nh_->get_logger(), "Subscribing to '%s'.", unmapped_topic_.c_str());

      callback_ = fp;
      obj_ = obj;
      //transport_hints.depth = queue_size;

      rclcpp::QoS hints = transport_hints;
      hints.keep_last(queue_size);

      sub_ = nh_->create_subscription<M>(unmapped_topic_,
                                         hints,
                                         std::bind(&TypedSubscriberImpl::handleMessage<M>,
                                                   this, std::placeholders::_1)
                                         );
    }

    // Handler for messages with headers
    template <class M2 = M>
    typename std::enable_if<(bool)has_header<M2>(), void>::type
    handleMessage(const std::shared_ptr< M > msg)
    {
      processHeader(msg->header.stamp);
      (obj_->*callback_)(msg);
    }

    // Handler for messages without headers
    template <class M2 = M>
    typename std::enable_if< !(bool)has_header<M2>(), void>::type
    handleMessage(const std::shared_ptr< M > msg)
    {
      processHeader(nh_->now());
      (obj_->*callback_)(msg);
    }
  };  // class TypedSubscriberImpl

  template<class M>
  class BindSubscriberImpl : public SubscriberImpl
  {
    std::function<void(const std::shared_ptr< const M > )> callback_;


  public:
    BindSubscriberImpl(
        rclcpp::Node& nh,
        const std::string &topic,
        uint32_t queue_size,
        const std::function<void(const std::shared_ptr< const M > )> &callback,
        const rclcpp::QoS& transport_hints)
    {
      unmapped_topic_ = topic;
      //mapped_topic_ = nh->ResolveName(topic);
      nh_ = &nh;

      RCLCPP_INFO(nh_->get_logger(), "Subscribing to '%s'.", unmapped_topic_.c_str());

      callback_ = callback;

      rclcpp::QoS hints = transport_hints;
      hints.keep_last(queue_size);
      sub_ = nh_->create_subscription<M>(unmapped_topic_,
                                         hints,
                                         std::bind(&BindSubscriberImpl::handleMessage<M>,
                                                   this, std::placeholders::_1)
                                        );
    }

    // Handler for messages with headers
    template <class M2 = M>
    typename std::enable_if< (bool)has_header<M2>(), void>::type
    handleMessage(const std::shared_ptr< const M > msg)
    {
      processHeader(msg->header.stamp);
      callback_(msg);
    }

    // Handler for messages without headers
    template <class M2 = M>
    typename std::enable_if< !(bool)has_header<M2>(), void>::type
    handleMessage(const std::shared_ptr< const M > msg)
    {
      processHeader(nh_->now());
      callback_(msg);
    }
  };  // class BindSubscriberImpl

  template<class M>
  class StorageSubscriberImpl : public SubscriberImpl
  {
    std::shared_ptr< const M > *dest_;

  public:
    StorageSubscriberImpl(
        rclcpp::Node& nh,
        const std::string &topic,
        std::shared_ptr< const M > *dest,
        const rclcpp::QoS& transport_hints)
    {
      unmapped_topic_ = topic;
      //mapped_topic_ = nh->ResolveName(topic);
      nh_ = &nh;

      RCLCPP_INFO(nh_->get_logger(), "Subscribing to '%s'.", unmapped_topic_.c_str());

      dest_ = dest;
      sub_ = nh_->create_subscription<M>(unmapped_topic_,
                                         transport_hints,
                                         std::bind(&StorageSubscriberImpl::handleMessage<M>,
                                                   this, std::placeholders::_1)
                                         );
    }

    // Handler for messages with headers
    template <class M2 = M>
    typename std::enable_if< (bool)has_header<M2>(), void>::type
    handleMessage(const std::shared_ptr< const M > msg)
    {
      processHeader(msg->header.stamp);
      *dest_ = msg;
    }

    // Handler for messages without headers
    template <class M2 = M>
    typename std::enable_if< !(bool)has_header<M2>(), void>::type
    handleMessage(const std::shared_ptr< const M > msg)
    {
      processHeader(nh_->now());
      *dest_ = msg;
    }
  };  // class StorageSubscriberImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_SUBSCRIBER_IMPL_H_
