#!/usr/bin/env python 


import sys
import os
import rospy

from gazebo_radiation_plugins.srv import GenWorldsFromModels



class GenWorlds(object):

    def __init__(self):
        rospy.init_node('GenWorlds')
        s = rospy.Service('generate_worlds_from_dae_models', GenWorldsFromModels, self.run)




    def run(self,req):
        folder = req.folder
        if folder[-1] != '/':
            folder+='/'

        m = folder.split('/')[-2]


        
        out = open(folder+req.model_filename,'w')

        temp = open(folder+"template/template_start.sdf", "r")
        for line in temp:
            out.write(line)

        for file in os.listdir(folder+'models/'):
            f = file[:-4]
            keywords = {'fname':f,'model_folder_name': m}
            
            if file.endswith(".dae"):
                temp = open(folder+"template/template_middle.sdf", "r")
                for line in temp:
                    out.write(line.format('',**keywords))

        temp = open(folder+"template/template_end.sdf", "r")
        for line in temp:
            out.write(line.format(file))
        

if __name__ == "__main__":
    x = GenWorlds()
    rospy.spin()