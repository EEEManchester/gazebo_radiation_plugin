#!/usr/bin/env python 


import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, DeleteModel, GetModelState
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import numpy as np
import tf
import os
import subprocess
import xml.etree.ElementTree as ET
from random import shuffle

class Mover(object):

    def callback(self,data):

            print "callback", self.model_count,self.source_count,self.sensor_count
           
            
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
            f.write("{},{},{},{}\n".format(self.sensor_count+1,self.source_count+1,self.model_count,rtf))
            f.close()
            self.model_count +=1
            
            
            if self.model_count == len(self.models_without_radiation):
                self.model_count = 0
                self.source_count+=1
                self.source_count = self.source_count%len(self.models_with_radiation) 
                self.reset_world()
                self.set_item(self.models_with_radiation[self.source_count])

                if self.source_count == 0:
                    for i in range(0,10):
                        self.sensor_count +=1
                        if self.sensor_count == self.nsensors:
        
                            rospy.signal_shutdown("finished") 
                        else:
                            cmd = "rosrun gazebo_ros spawn_model -file '/home/tom/ROS/custom_gazebo_sensor/catkin_ws/src/gazebo_radiation_plugin/sdf/radiation_sensor_alt.sdf' -sdf -model sensor_{} -x {} -y {} -z 0.5".format(self.sensor_count,(-5.0+(self.sensor_count%5)),(-1*(int(self.sensor_count/5))))
                            os.system(cmd)
                            #time.sleep(0.1)

            self.set_item(self.models_without_radiation[self.model_count])
                

                  



            

            #time.sleep(1)


    def process_infile(self):
        world_file = ET.parse(self.infile)
        root = ET.parse(self.infile).getroot()
        world = root.find('world')
        models = world.findall('model')
      
        moveable_models = []
        radioactive_models = []
        for i in models:
            if i.find('model') != None:
                if "radiation" in str(i.find('model').attrib):
                    self.models_with_radiation.append(i)
            
            elif ("ground" not in i.get('name')) & ("wall" not in i.get('name')) :
               self.models_without_radiation.append(i)
                    

        shuffle(self.models_without_radiation)
        shuffle(self.models_with_radiation)

        self.reset_world()
        self.set_item(self.models_with_radiation[0])
        


    def reset_world(self):

        for i in range(self.source_count,len(self.models_with_radiation)):
                [x,y,z,r,p,yaw] =  self.models_with_radiation[i].find('pose').text.split()        
                quaternion = tf.transformations.quaternion_from_euler(float(r), float(p), float(yaw))
                state_msg = ModelState()
                state_msg.model_name = self.models_with_radiation[i].get("name")
                state_msg.pose.position.x = float(x) +150.0
                state_msg.pose.position.y = float(y)
                state_msg.pose.position.z = float(z)
                state_msg.pose.orientation.x = quaternion[0]
                state_msg.pose.orientation.y = quaternion[1]
                state_msg.pose.orientation.z = quaternion[2]
                state_msg.pose.orientation.w = quaternion[3]
                state_msg.reference_frame = "map"
               
                resp = self.set_state( state_msg )
                time.sleep(.05)

        for i in self.models_without_radiation:
                [x,y,z,r,p,yaw] =  i.find('pose').text.split()
                quaternion = tf.transformations.quaternion_from_euler(float(r), float(p), float(yaw))
                state_msg = ModelState()
                state_msg.model_name = i.get("name")
                state_msg.pose.position.x = float(x) +150.0
                state_msg.pose.position.y = float(y)
                state_msg.pose.position.z = float(z)
                state_msg.pose.orientation.x = quaternion[0]
                state_msg.pose.orientation.y = quaternion[1]
                state_msg.pose.orientation.z = quaternion[2]
                state_msg.pose.orientation.w = quaternion[3]
                state_msg.reference_frame = "map"             
                resp = self.set_state( state_msg )
                time.sleep(.05)
        print "world reset"


    def set_item(self,i):

        [x,y,z,r,p,yaw] =  i.find('pose').text.split()
        quaternion = tf.transformations.quaternion_from_euler(float(r), float(p), float(yaw))
        state_msg = ModelState()
        state_msg.model_name = i.get("name")
        state_msg.pose.position.x = float(x) 
        state_msg.pose.position.y = float(y)
        state_msg.pose.position.z = float(z)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        state_msg.reference_frame = "map"             
        resp = self.set_state( state_msg )
        time.sleep(.05)



  
            

    def __init__(self, nsensors):

        self.nsources = 10
        self.nsensors = nsensors
        self.filename = "/home/tom/ROS/custom_gazebo_sensor/catkin_ws/src/gazebo_radiation_plugin/data/test.csv"
        rospy.init_node('sensor_source_mover')
        rospy.Subscriber("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, self.callback)
        self.infile = rospy.get_param(file,"/home/tom/ROS/custom_gazebo_sensor/catkin_ws/src/gazebo_radiation_plugin/custom_models/barrel_store_custom/updated_barrel_store_custom_10_sources.world")
        self.pos_pub = rospy.Publisher("set_pose",PoseWithCovarianceStamped, queue_size=1)
        self.frame = "map"
        self.models_with_radiation = []
        self.models_without_radiation = []
        self.source_count = 0
        self.sensor_count = 0
        self.model_count = 0

        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        


        self.process_infile()  

        rospy.wait_for_service('/gazebo/set_model_state')

if __name__ == '__main__':
    try:
        m = Mover(100)
        while not rospy.is_shutdown():
            rospy.wait_for_service('/gazebo/set_model_state')
    except rospy.ROSInterruptException:
        pass