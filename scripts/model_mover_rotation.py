#!/usr/bin/env python 


import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
import time
import numpy as np
import tf


class Mover(object):

    def callback(self,data):
            print data.value
            print data.pose.orientation.x , self.quaternion[0],self.quaternion[1] , data.pose.orientation.y,self.quaternion[2], data.pose.orientation.z,self.quaternion[3], data.pose.orientation.w
            if (np.round(data.pose.orientation.x,3) == np.round(self.quaternion[0],3))&(np.round(self.quaternion[1],3) == np.round(data.pose.orientation.y,3))&(np.round(self.quaternion[2],3) == np.round(data.pose.orientation.z,3))&(np.round(self.quaternion[3],3) == np.round(data.pose.orientation.w,3)):
                self.i+=1
                f = open(self.filename, "a")
                f.write("{},{}\n".format(self.angle,data.value))
                f.close()
                self.angle += 0.01

    def __init__(self):

        self.i = 0
        self.quaternion = [0.0,0.0,0.0,0.0]
        self.angle = 0
        self.filename = "/home/tom/Desktop/rotate.csv"
        f = open(self.filename, "w")
        f.close()
        rospy.init_node('set_pose')
        rospy.Subscriber("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, self.callback)


        state_msg = ModelState()
        state_msg.model_name = 'sensor_0'
        state_msg.pose.position.x = -1.0
        state_msg.pose.position.y = -0.5
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.angle = 0.0
            while(self.angle < 6.28):
                self.quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,self.angle-0.785398)
                state_msg.pose.orientation.x = self.quaternion[0]
                state_msg.pose.orientation.y = self.quaternion[1]
                state_msg.pose.orientation.z = self.quaternion[2]
                state_msg.pose.orientation.w = self.quaternion[3]

                #state_msg.pose.position.x = 3.0*np.cos(angle)
                #state_msg.pose.position.y = 3.0*np.sin(angle)
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )
            
                rospy.wait_for_message("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, timeout=None)
                
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        m = Mover()
    except rospy.ROSInterruptException:
        pass