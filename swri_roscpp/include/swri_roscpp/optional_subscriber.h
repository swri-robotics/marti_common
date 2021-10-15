// *****************************************************************************
//
// Copyright (c) 2021, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROSCPP_OPTIONAL_SUBSCRIBER_H_
#define SWRI_ROSCPP_OPTIONAL_SUBSCRIBER_H_

#include <ros/node_handle.h>

#include <swri_roscpp/subscriber.h>

// This is a wrapper that lets you bring a swri::Subscriber up and down with a simple function call

namespace swri
{

class OptionalSubscriberImplRoot
{
public:
  virtual ~OptionalSubscriberImplRoot()
  {

  }

  virtual swri::Subscriber subscribe() = 0;
};

template<class M, class T>
class OptionalSubscriberImpl: public OptionalSubscriberImplRoot
{
  ros::NodeHandle nh_;
  ros::TransportHints transport_hints_;
  std::string name_;
  uint32_t queue_size_;

  void(T::*callback_)(const boost::shared_ptr<M const> &);
  T* obj_;

public:
  void initialize(ros::NodeHandle &nh,
              const std::string &name,
              uint32_t queue_size,
              void(T::*cb)(const boost::shared_ptr<M const> &),
              T *obj,
              const ros::TransportHints &transport_hints)
  {
    nh_ = nh;
    name_ = name;
    queue_size_ = queue_size;
    callback_ = cb;
    obj_ = obj;
    transport_hints_ = transport_hints;
  }

  virtual swri::Subscriber subscribe()
  {
    return swri::Subscriber(nh_, name_, queue_size_, callback_, obj_, transport_hints_);
  }
};

class OptionalSubscriber
{
  swri::Subscriber sub_;
  boost::shared_ptr<OptionalSubscriberImplRoot> impl_;
  
public:

  OptionalSubscriber() {}

  template<class T, class M>
  OptionalSubscriber(ros::NodeHandle& nh, std::string topic, uint32_t queue_size,
             void(T::*fp)(const boost::shared_ptr< M const > &),
             T *obj,
             const ros::TransportHints &transport_hints=ros::TransportHints())
  {
    OptionalSubscriberImpl<M, T>* ptr = new OptionalSubscriberImpl<M, T>();
    ptr->initialize(nh, topic, queue_size, fp, obj, transport_hints);
    impl_ = boost::shared_ptr<OptionalSubscriberImplRoot>(ptr);
  }

  inline void subscribe()
  {
    if (!impl_)
    {
      ROS_ERROR("Called subscribe on uninitialized optional subscriber");
      return;
    }
    sub_ = impl_->subscribe();
  }

  inline int numPublishers()
  {
    return sub_.numPublishers();
  }

  void shutdown()
  {
    sub_ = swri::Subscriber();
  }
};

}  // namespace swri
#endif  // SWRI_ROSCPP_OPTIONAL_SUBSCRIBER_H_
