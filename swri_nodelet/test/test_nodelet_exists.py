#! /usr/bin/env python

import rospy
import rostest
from std_msgs.msg import Int32
import sys
from time import sleep
import unittest

PKG = 'swri_nodelet'
NAME = 'test_nodelet_exists'

class TestNodeletExists(unittest.TestCase):
    
    def callback(self, msg):
        self.msg = msg
    
    def test_init(self):
        ''' 
        Test that the test nodelet exists by waiting for the message it
        publishes.
        '''
        self.msg = None
        sub = rospy.Subscriber('numbers', Int32, self.callback)
        while self.msg is None:
            sleep(0.1)
        sub.unregister()
        self.assertEquals(self.msg.data, 1337)
        
if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestNodeletExists, sys.argv)
