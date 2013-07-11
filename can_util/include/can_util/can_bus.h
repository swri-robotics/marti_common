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
 * receive can messages.  It keeps track of outgoing messages and
 * filters incoming messages to prevent the node from receiving
 * messages that it sent out.
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
  void CanFrameCallback(const marti_can_msgs::CanFrame &msg);

 private:
  ros::Subscriber can_frame_sub_;
  ros::Publisher can_frame_pub_;
  std::list<marti_can_msgs::CanFrame> outgoing_msgs_;
  boost::function<void(const marti_can_msgs::CanFrame &msg)> callback_fn_;
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
}

void CanBus::Shutdown()
{
  can_frame_sub_.shutdown();
  can_frame_pub_.shutdown();
  outgoing_msgs_.clear();
}

void CanBus::Publish(const marti_can_msgs::CanFrame &msg)
{
  outgoing_msgs_.push_back(msg);
  can_frame_pub_.publish(msg);
}

void CanBus::CanFrameCallback(const marti_can_msgs::CanFrame &msg)
{
  for (std::list<marti_can_msgs::CanFrame>::iterator
         it = outgoing_msgs_.begin();
       it != outgoing_msgs_.end();
       ++it)
  {
    // These are ordered to test the most likely differing fields
    // first.  We don't test against header.seq since ros::Publisher
    // modifies this field.
    if (msg.header.stamp != it->header.stamp)
      break;
    if (msg.ID != it->ID)
      break;
    if (msg.data != it->data)
      break;
    if (msg.data_length != it->data_length)
      break;
    if (msg.header.frame_id != it->header.frame_id)
      break;
    if (msg.msg_type != it->msg_type)
      break;

    // This message is equal to an outgoing message so it will not be
    // passed on.  We also assume that earlier messages in the queue
    // were not received and delete those too. This could result in a
    // transmitted message being received again, but eliminates the
    // buffer from growing too large as long as we get occaisional
    // matches.
    outgoing_msgs_.erase(outgoing_msgs_.begin(), ++it);
    return;
  }
  callback_fn_(msg);
}
}  // namespace sumet_util
#endif  // CAN_UTIL_CAN_BUS_H_
