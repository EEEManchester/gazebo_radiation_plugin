#!/usr/bin/env python 

import sys
import os
import subprocess
import rospy

from gazebo_radiation_plugins.srv import MassYamlLoader



class GenWorlds(object):

    def __init__(self):
        rospy.init_node('MassYamlLoader')
        s = rospy.Service('mass_yaml_loader', MassYamlLoader, self.run)




    def run(self,req):
        folder = req.folder
        if folder[-1] != '/':
            folder+='/'

        for file in os.listdir(folder):
            cmd = "rosparam load {}{}".format(folder,file)
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            (output, err) = p.communicate()  
            #This makes the wait possible
            p_status = p.wait()

if __name__ == "__main__":
    x = GenWorlds()
    rospy.spin()