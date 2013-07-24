import roslib
roslib.load_manifest('can_util')
import rospy

import marti_can_msgs as mcm
import marti_can_msgs.msg

class CanBus(object):
    def __init__(self, topic_name, callback):
        self.can_pub = rospy.Publisher(topic_name, mcm.msg.CanFrame)
        self.can_sub = rospy.Subscriber(topic_name, mcm.msg.CanFrame,
                                        self.can_frame_callback)
        self.ext_callback = callback
        
    def publish(self, can_frame):
        can_frame.header.frame_id = rospy.get_name()
        self.can_pub.publish(can_frame)

    def can_frame_callback(self, can_frame):
        if can_frame.header.frame_id == rospy.get_name():
            return
        self.ext_callback(can_frame)
        
