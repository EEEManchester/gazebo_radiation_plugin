#!/usr/bin/env python 


import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, DeleteModel
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import numpy as np
import tf
import os
import subprocess



class Box(object):
    def __init__(self,x):
        self.name = "unit_box_" + str(x)
        self.state = False
        self.x = (x+1.0)*2
        self.y = 1.0

    def get_position(self):
        if self.state:
            self.y = 0.0
        else:
            self.y = 1.0
        return self.name,self.x,self.y


class Mover(object):

    def callback(self,data):
           
            p = subprocess.Popen(["gz stats -p -d 1"], stdout=subprocess.PIPE,stderr=subprocess.PIPE, shell=True)
            out, err = p.communicate()
            out = out.split("\n")
            out =out[1:]
            #print out
            rtf = []
            for i in out[:-1]:  
                rtf.append(float(i.split(',')[0]))

            #print rtf

               
            f = open(self.filename, "a")
            f.write("{},{},{},{}\n".format(self.sensor_count+1,self.source_count,self.box_count,rtf))
            f.close()
            self.box_count +=1
            
            if self.box_count == self.nboxes+1:
                self.source_count+=1
                self.source_count = self.source_count%(self.nsources+1)   
                if self.source_count ==0:
                    for i in range(1,self.nsources+1):
                        resp = self.delete_model("source_{}".format(i))
                        #time.sleep(0.1)
                else:
                    cmd = "rosrun gazebo_ros spawn_model -file '/home/tom/ROS/custom_gazebo_sensor/catkin_ws/src/gazebo_radiation_plugin/sdf/radiation_source.sdf' -sdf -model source_{} -x {} -z 0.5".format(self.source_count,25)
                    os.system(cmd)
                    #time.sleep(0.1)

                if self.source_count == 0:
                    self.sensor_count +=1
                    print "sensor count" ,self.sensor_count , self.nsensors
                    if self.sensor_count == self.nsensors:
    
                        rospy.signal_shutdown("finished") 
                    else:
                        cmd = "rosrun gazebo_ros spawn_model -file '/home/tom/ROS/custom_gazebo_sensor/catkin_ws/src/gazebo_radiation_plugin/sdf/radiation_sensor_alt.sdf' -sdf -model sensor_{} -x {} -z 0.5".format(self.sensor_count,0.0)
                        os.system(cmd)
                        #time.sleep(0.1)


                print self.sensor_count,self.source_count

                  



            self.box_count = self.box_count%(self.nboxes+1)
            self.box_array = np.zeros(self.nboxes)
            self.box_array[:self.box_count] = 1

            print self.box_array

            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            for i in range(0,len(self.box_array)):
                self.boxes[i].state = bool(self.box_array[i])
                self.state_msg.model_name,self.state_msg.pose.position.x,self.state_msg.pose.position.y = self.boxes[i].get_position()
                
                resp = self.set_state( self.state_msg )

            #time.sleep(1)


    def __init__(self,nb,so,se):

        self.nboxes = nb
        self.nsources = so
        self.nsensors = se
        self.filename = "/home/tom/ROS/custom_gazebo_sensor/catkin_ws/src/gazebo_radiation_plugin/data/test.csv"
        #f = open(self.filename, "w")
        #f.close()
        rospy.init_node('sensor_source_mover')
        rospy.Subscriber("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, self.callback)
        self.pos_pub = rospy.Publisher("set_pose",PoseWithCovarianceStamped, queue_size=1)
        self.frame = "map"
        self.boxes = []
        for i in range(0,self.nboxes):
            self.boxes.append(Box(i))
        self.box_count = 0
        self.box_array = np.zeros(self.nboxes)
        self.source_count = 0
        self.source_array = np.zeros(self.nboxes)
        self.sensor_count = 0


        self.state_msg = ModelState()
        self.state_msg.model_name = ''
        self.state_msg.pose.position.x = 0.0
        self.state_msg.pose.position.y = 0
        self.state_msg.pose.position.z = 0.01
        self.state_msg.pose.orientation.x = 0
        self.state_msg.pose.orientation.y = 0
        self.state_msg.pose.orientation.z = 0
        self.state_msg.pose.orientation.w = 1.0
        

        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        rospy.wait_for_service('/gazebo/set_model_state')

if __name__ == '__main__':
    try:
        m = Mover(10,10,10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass