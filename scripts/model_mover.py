#!/usr/bin/env python 


import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import time
import numpy as np
import tf

def main():
    rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = 'sensor_0'
    state_msg.pose.position.x = 1.0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.5
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 1.0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        angle = 0.0
        while(1):
                angle = angle%628
                quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,angle+3.14)
                state_msg.pose.orientation.x = quaternion[0]
                state_msg.pose.orientation.y = quaternion[1]
                state_msg.pose.orientation.z = quaternion[2]
                state_msg.pose.orientation.w = quaternion[3]
                state_msg.pose.position.x = 3.0*np.cos(angle)
                state_msg.pose.position.y = 3.0*np.sin(angle)
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )
                time.sleep(0.015)
                angle += 0.01

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass