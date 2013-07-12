#ifndef CAN_UTIL_CAN_BUS_H_
#define CAN_UTIL_CAN_BUS_H_

#include <string>
#include <list>

#include <boost/bind.hpp>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <marti_can_msgs/CanFrame.h>

namespace sumet_util
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

  void Publish(const marti_can_msgs::CanFrame &msg);

 protected:
  bool EchoedMessage(const marti_can_msgs::CanFrame &msg);
  void CanFrameCallback(const marti_can_msgs::CanFrame &msg);  

 private:
  ros::Subscriber can_frame_sub_;
  ros::Publisher can_frame_pub_;
  boost::function<void(const marti_can_msgs::CanFrame &msg)> callback_fn_;
  std::string node_name_;
};

template<class T>
void CanBus::Initialize(
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

void CanBus::Shutdown()
{
  can_frame_sub_.shutdown();
  can_frame_pub_.shutdown();
}

void CanBus::Publish(const marti_can_msgs::CanFrame &msg)
{
  marti_can_msgs::CanFrame modified_msg = msg;
  modified_msg.header.frame_id = node_name_;
  can_frame_pub_.publish(modified_msg);
}

bool CanBus::EchoedMessage(const marti_can_msgs::CanFrame &msg)
{
  if (msg.header.frame_id == node_name_)
    return true;
  return false;
}

void CanBus::CanFrameCallback(const marti_can_msgs::CanFrame &msg)
{
  if (!EchoedMessage(msg))
    callback_fn_(msg);
}
}  // namespace sumet_util
#endif  // CAN_UTIL_CAN_BUS_H_
