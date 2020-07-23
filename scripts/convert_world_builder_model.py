#!/usr/bin/env python 

import sys
import os
import subprocess
import rospy
import random
import cv2
import numpy as np
import xml.etree.ElementTree as ET
from datetime import datetime
from collada import *
from gazebo_radiation_plugins.srv import ConvertWorldBuilderModel



class ConvertWorlds(object):

    def __init__(self):
        rospy.init_node('ConvertWorldBuilderModel')
        s = rospy.Service('convert_world_builder_model', ConvertWorldBuilderModel, self.run)



    def run_cmd(self,cmd):
        #print cmd
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        (output, err) = p.communicate()
        p_status = p.wait()

    def backup(self,model_folder):
        if os.path.isdir(model_folder):
            if os.path.isdir(model_folder+"folder_backups") == False:
                cmd = "mkdir "+model_folder+"folder_backups/"
                self.run_cmd(cmd)                
            now = datetime.now().strftime("%Y%m%d%H%M%S")
            cmd = "mkdir "+model_folder+"folder_backups/{}/".format(now)
            self.run_cmd(cmd)
            cmd = "cp -r {} {} ".format(model_folder,model_folder+"folder_backups/{}/".format(now))
            self.run_cmd(cmd)
        else:
            print "making ",model_folder
            os.mkdir(model_folder)


    def run(self,req):
        gazebo_model_folder = req.gazebo_model_path
        if gazebo_model_folder[-1] != '/':
            print  gazebo_model_folder[-1]
            gazebo_model_folder+='/'
        last_bit = req.world_builder_file.split('/')[-1].split('.')[0] +'/'
        model_folder = gazebo_model_folder + last_bit

        self.backup(model_folder)
        
        world_builder_folder = req.world_builder_file[:-len(req.world_builder_file.split('/')[-1])]
        
        if os.path.isfile(req.noise_file) == True:
            cmd = "rosparam load "+req.noise_file
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            (output, err) = p.communicate()  
            p_status = p.wait()
        if rospy.has_param('models_with_noise'):
            add_noise_to_models = rospy.get_param('models_with_noise/')
        else:
            add_noise_to_models = []
        print add_noise_to_models

        cmd = "cp {} {}".format(world_builder_folder+'*',model_folder)
        #print cmd
        self.run_cmd(cmd)

        cmd = "cp -r {} {}".format(gazebo_model_folder+"template/*",model_folder)
        self.run_cmd(cmd)

        model_file = model_folder + req.world_builder_file.split('/')[-1]
        tree = ET.parse(model_file)
        root = tree.getroot()
        models = root.findall('world/model')
        all_models = []
        rustable_models = []
        bendable_models = []
        for model in models:
            all_models.append(model.get('name')+'\n')
            f_name = model.get('name')
            links = model.findall('link')
            for link in links:
                col_uris = link.findall('collision/geometry/mesh/uri')
                vis_uris = link.findall('visual/geometry/mesh/uri')
                uris = col_uris + vis_uris
                for i in uris:
                    bendable_models.append(model.get('name')+'\n')
                    cmd = "cp {} {}".format(i.text,model_folder+"/models/{}.dae".format(f_name))
                    self.run_cmd(cmd)
                    i.text = "model://{}models/{}.dae".format(last_bit,f_name)
                    dae_file = model_folder+"/models/"+i.text.split('/')[-1]
                    for item in add_noise_to_models:
                        if item in f_name:
                            self.addNoiseToModel(dae_file,add_noise_to_models[item]['pose'])
                    ET.register_namespace('', "http://www.collada.org/2005/11/COLLADASchema")
                    dae_tree =  ET.parse(dae_file)
                    dae_root = dae_tree.getroot()
                    ns = dae_root.tag.split('}')[0] +'}'
                    material_xml = dae_root.find("{}library_images/{}image/{}init_from".format(ns,ns,ns))
                    if material_xml == None:
                        rospy.logwarn("No texture found. Follow the video tutorial on how to add textures using blender")
                    else:  
                        rustable_models.append(model.get('name')+'\n')          
                        material = material_xml.text
                        if (material.startswith("../"))|( material.startswith("/")):
                            if material.startswith("../"):
                                material_path = os.path.realpath(model_folder+"models/" + material)  
                                print material_path 
                            elif material.startswith("/"):
                                material_path = material
                            cmd = "cp {} {}/materials/textures/{}.png".format(material_path,model_folder,f_name)   
                            self.run_cmd(cmd) 
                            for item in add_noise_to_models:
                                if item in f_name:
                                    self.addNoiseToTexture("{}/materials/textures/{}.png".format(model_folder,f_name),add_noise_to_models[item]['colour'])
                            material_xml.text = "../materials/textures/{}.png".format(f_name) 
                            dae_tree.write(dae_file)
                        else:
                            rospy.logwarn("Texture image path is none standard. Skipping as to not break anything. Please fix or update textures manually. ")
        
        tree.write(model_file)


        rustable_models = set(rustable_models)
        bendable_models = set(bendable_models)
        with open(model_folder+"/models/all_models.txt", 'w') as f:
             for item in all_models:
                f.write(item)
        with open(model_folder+"/models/bendable_models.txt", 'w') as f:
             for item in bendable_models:
                f.write(item)
        with open(model_folder+"/models/rustable_models.txt", 'w') as f:
             for item in rustable_models:
                f.write(item)




    def addNoiseToTexture(self,f,noise):

        img = cv2.imread(f,cv2.IMREAD_UNCHANGED)
        noise_img = np.zeros((img.shape[0],img.shape[1],img.shape[2]))
        if int(noise["b"]*255) > 0:
            noise_img[:,:,0] = np.random.normal(0, int(noise["b"]*255), (img.shape[0],img.shape[1])) 
        if int(noise["g"]*255) > 0:
            noise_img[:,:,1] = np.random.normal(0, int(noise["g"]*255), (img.shape[0],img.shape[1])) 
        if int(noise["r"]*255) > 0:
            noise_img[:,:,2] = np.random.normal(0, int(noise["r"]*255), (img.shape[0],img.shape[1])) 

        noise_img += img
        cv2.imwrite(f,noise_img)

    def addNoiseToModel(self,dae,noise):
        
        mesh = Collada(dae)
        geom = mesh.geometries[0]
        triset = geom.primitives[0]
        output = []
        x_noise_factor = float(noise['x'])
        y_noise_factor = float(noise['y'])
        z_noise_factor = float(noise['z'])
        for i in range(0,len(triset.vertex[triset.vertex_index])):
            tri_pointset = []
            for j in range(0,len(triset.vertex[triset.vertex_index][i])):
                point = [triset.vertex[triset.vertex_index][i][j][0],triset.vertex[triset.vertex_index][i][j][1],(triset.vertex[triset.vertex_index][i][j][2])]
                point[0] = random.gauss(point[0],x_noise_factor*point[0])
                point[1] = random.gauss(point[1],y_noise_factor*point[1])
                point[2] = random.gauss(point[2],z_noise_factor*point[2])
                tri_pointset.append(point)
            output.append(tri_pointset)

        mesh.geometries[0].primitives[0].vertex[triset.vertex_index] = np.asarray(output)

        mesh.write(dae)       


if __name__ == "__main__":
    x = ConvertWorlds()
    rospy.spin()