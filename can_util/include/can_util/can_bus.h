// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef CAN_UTIL_CAN_BUS_H_
#define CAN_UTIL_CAN_BUS_H_

#include <string>
#include <list>

#include <boost/bind.hpp>

#include <ros/this_node.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <marti_can_msgs/CanFrame.h>

namespace can_util
{
/* @brief Emulate a CAN bus over a ROS topic.
 *
 * This class allows a node to use a single topic to transmit and
 * receive can messages.  When a message is transmitted, it sets
 * header.frame_id to the node name.  When a message is received, it
 * filters out any messages with the same name to prevent the node
 * from receiving messages that it sent out.
 *
 * This scheme is not as robust as saving outgoing messages to a
 * buffer and filtering them for equality, but it is faster, uses less
 * memory, and has the side effect that we can tell from where each
 * frame originated.  It also makes this class thread-safe without
 * needing locks.
 */
class CanBus
{
 public:
  template<class T>
  void Initialize(
    const ros::NodeHandle &node_handle,
    const std::string& can_topic,
    uint32_t queue_size,
    void (T::*fp)(const marti_can_msgs::CanFrame &),
    T *obj);

  void Shutdown();

  void Publish(const marti_can_msgs::CanFrame &msg) const;

 protected:
  bool EchoedMessage(const marti_can_msgs::CanFrame &msg);
  void CanFrameCallback(const marti_can_msgs::CanFrame &msg);  

 private:
  ros::Subscriber can_frame_sub_;
  ros::Publisher can_frame_pub_;
  boost::function<void(const marti_can_msgs::CanFrame &msg)> callback_fn_;
  std::string node_name_;
  bool timestamp_messages_;
};

template<class T>
inline void CanBus::Initialize(
  const ros::NodeHandle &node_handle,
  const std::string &can_topic,
  uint32_t queue_size,
  void (T::*fp)(const marti_can_msgs::CanFrame &),
  T *obj)
{
  ros::NodeHandle nh = node_handle;
  can_frame_sub_ = nh.subscribe(can_topic,
                                queue_size,
                                &CanBus::CanFrameCallback,
                                this);

  can_frame_pub_ = nh.advertise<marti_can_msgs::CanFrame>(can_topic,
                                                          queue_size);

  callback_fn_ = boost::bind(fp, obj, _1);
  node_name_ = ros::this_node::getName();
}

inline void CanBus::Shutdown()
{
  can_frame_sub_.shutdown();
  can_frame_pub_.shutdown();
}

inline void CanBus::Publish(const marti_can_msgs::CanFrame &msg) const
{
  marti_can_msgs::CanFrame modified_msg = msg;
  modified_msg.header.frame_id = node_name_;
  if (timestamp_messages_)
    modified_msg.header.stamp = ros::Time::now();
  can_frame_pub_.publish(modified_msg);
}

inline bool CanBus::EchoedMessage(const marti_can_msgs::CanFrame &msg)
{
  if (msg.header.frame_id == node_name_)
    return true;
  return false;
}

inline void CanBus::CanFrameCallback(const marti_can_msgs::CanFrame &msg)
{
  if (!EchoedMessage(msg))
    callback_fn_(msg);
}
}  // namespace can_util
#endif  // CAN_UTIL_CAN_BUS_H_
