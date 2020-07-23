#!/usr/bin/env python 

import rospy
import rospkg
import os

rospack = rospkg.RosPack()

sources_folder = rospack.get_path("gazebo_radiation_plugins")+"/example_radiation_sources/"

sources = rospy.get_param("sources")


for i in sources: 
    s = sources_folder+i
    os.system("rosrun gazebo_ros spawn_model -file {}.sdf -sdf -model {}".format(s,i))
    
