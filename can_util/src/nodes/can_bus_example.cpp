#include <ros/ros.h>
#include <can_util/can_bus.h>

namespace mcm = marti_can_msgs;

/* This is an example of using the CanBus object to send and receive
 * messages over a CAN topic.
 *
 * Echo the can topic:
 *   rostopic echo /can_bus
 *
 * You should see no messages
 *
 * In a new terminal, launch one instance:
 *   rosrun can_util can_bus_example 200
 *
 * You will start to see messages echoed at 1.0 Hz on the /can_bus
 * topic, but you will see no output from the can_bus_example because
 * it is not receiving those messages.
 *
 * In a new terminal, launch a second instance:
 *    rosrun can_util can_bus_example 300
 *
 * You will see two messages per second echoed by rostopic.  You
 * should see about one message per second from each can_bus_example
 * because they can see the messages published by each other, but not
 * themselves.
 */

class CanBusExampleNode
{
 public:
  CanBusExampleNode()
    :
    can_id_(0)
  {
  }

  void Initialize(int can_id)
  {
    ros::NodeHandle nh;
    can_bus_.Initialize(nh, "/can_bus", 1000,
                       &CanBusExampleNode::CanFrameCallback,
                       this);

    timer_ = nh.createTimer(ros::Duration(1.0),
                            &CanBusExampleNode::TimerCallback,
                            this);

    can_id_ = can_id;
  }

  void CanFrameCallback(const mcm::CanFrame &msg)
  {
    ROS_WARN("Received message with ID %d:", msg.ID);
    ROS_WARN("  header.seq: %d", msg.header.seq);
    ROS_WARN("  header.stamp: %f", msg.header.stamp.toSec());
  }

  void TimerCallback(const ros::TimerEvent &e)
  {
    mcm::CanFrame msg;
    msg.ID = can_id_;
    msg.data_length = 8;
    for (uint8_t i = 0; i < msg.data_length; i++)
      msg.data[0] = i;

    can_bus_.Publish(msg);
  }

 private:
  ros::Timer timer_;
  sumet_util::CanBus can_bus_;
  int can_id_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "canbus_example",
            ros::init_options::AnonymousName);

  if (argc != 2)
  {
    ROS_ERROR("Usage: %s <can_id>", argv[0]);
    return -1;
  }

  ros::NodeHandle nh;
  CanBusExampleNode node;
  node.Initialize(atoi(argv[1]));
  ros::spin();
  return 0;
}

