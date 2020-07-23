#!/usr/bin/env python 

import rospy
import rospkg
import os

rospack = rospkg.RosPack()

sources_folder = rospack.get_path("gazebo_radiation_plugins")+"/custom_models/reactor_room/"

for file in os.listdir(sources_folder):
    if file.endswith(".sdf"):
        s = sources_folder+file
        os.system("rosrun gazebo_ros spawn_model -file {} -sdf -model {}".format(s,file[:-4]))
    
