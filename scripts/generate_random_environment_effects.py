#!/usr/bin/env python 


import sys
import os
import rospy
import random


from gazebo_radiation_plugins.srv import GenRandomEnvironmentalEffects



class GenEnvEffects(object):

    def __init__(self):
        rospy.init_node('GenWorlds')
        s = rospy.Service('generate_random_environemtal_effects', GenRandomEnvironmentalEffects, self.run)


    def gen_kwargs(self,bend,rust,rad,req):
        all_models = set(bend+rust+rad)
        keywords = []
        count = 0
        for i in all_models:
            if i in rust:
                rust_factor = random.uniform(0.0,req.max_rust_factor)
            else:
                rust_factor = 0.0
            if i in bend:
                bend_factor = random.uniform(0.0,req.max_bend_factor)
            else:
                bend_factor = 0.0
            if i in rad:
                rad_source = "source_{}".format(str(count))
                count+=1
            else:
                rad_source = "None"

            keywords.append({'model_name':i.strip(),'radiation_source': rad_source,'bend_factor': bend_factor,'rust_factor': rust_factor})
        return keywords

    def run(self,req):
        folder = req.folder
        if folder[-1] != '/':
            folder+='/'

        models_folder = folder+"models/"
        yaml_folder = folder+"yamls/"

        rad_models = []
        rust_models = []
        bend_models = []

        if os.path.exists(models_folder+'all_models.txt'):
            f = open(models_folder+'all_models.txt','r')
            for line in f:
                rad_models.append(line)

        if os.path.exists(models_folder+'bendable_models.txt'):
            f = open(models_folder+'bendable_models.txt','r')
            for line in f:
                rust_models.append(line)

        if os.path.exists(models_folder+'rustable_models.txt'):
            f = open(models_folder+'rustable_models.txt','r')
            for line in f:
                bend_models.append(line)

        if req.number_of_radiation_sources > len(rad_models):
            req.number_of_radiation_sources = len(rad_models)
        if req.number_of_rusting_models > len(rust_models):
            req.number_of_rusting_models = len(rust_models)
        if req.number_of_bending_models > len(bend_models):
            req.number_of_bending_models = len(bend_models)
        

        selected_rad_models = []
        selected_rust_models = []
        selected_bend_models = []

        for i in range(0,req.number_of_radiation_sources):
            choice = random.choice(rad_models)
            selected_rad_models.append(choice)
            rad_models.remove(choice)
        for i in range(0,req.number_of_rusting_models):
            choice = random.choice(rust_models)
            selected_rust_models.append(choice)
            rust_models.remove(choice)
        for i in range(0,req.number_of_bending_models):
            choice = random.choice(bend_models)
            selected_bend_models.append(choice)
            bend_models.remove(choice)

        keywords = self.gen_kwargs(selected_bend_models,selected_rust_models,selected_rad_models,req)


        out = open(yaml_folder+"subset.yaml",'w')
        out.write("model_subset:\n")

        for k in keywords:
            temp = open(folder+"template/subset_template.yaml", "r")
            for line in temp:
                out.write(line.format('',**k))
        

if __name__ == "__main__":
    x = GenEnvEffects()
    rospy.spin()