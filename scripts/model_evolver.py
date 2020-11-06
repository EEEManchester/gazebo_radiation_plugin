#!/usr/bin/env python 

from collada import *
import numpy as np
import sys 
import os
import rospy
import shutil
import cv2
import copy
import random
from datetime import datetime
from gazebo_radiation_plugins.srv import EnvironmentEvolver,EnvironmentEvolverResponse

class Evolver(object):

    def __init__(self):
        rospy.init_node('Evolver')
        s = rospy.Service('environment_evolver', EnvironmentEvolver, self.run_multiple)


    def bend(self,p,c,f,h):
        x = p[0] -c[0]
        y = p[1] -c[1]
        z = p[2] -c[2]

        ratio = f*np.cos(z*3.14/h)

        if (f > 0.0) & (ratio < 0.0):
            ratio = 0.0

        if (f < 0.0) & (ratio > 0.0):
            ratio = 0.0

        rx = p[0]+(x*(ratio))
        ry = p[1]+(y*(ratio))

        #print [p[0],p[1],p[2]],[rx,ry,p[2]]
        return [rx,ry,p[2]]

    def check_centre_and_height(self,centre,height,triset):

        if (centre == None) | (height == None):
            zs = []
            xs = []
            ys =[]
            for i in triset.vertex[triset.vertex_index]:
                for j in i:
                    xs.append(j[0])
                    ys.append(j[1])
                    zs.append(j[2])
            minZ = min(zs)
            maxZ = max(zs)
            if height == None:
                height = maxZ - minZ
            if centre == None:
                #centre = [np.mean(xs),np.mean(ys),np.mean(zs)]
                centre = [(min(xs)+max(xs))/2.0,(min(ys)+max(ys))/2.0,(min(zs)+max(zs))/2.0]
        return centre,height


    def run_multiple(self,req):
        self.models_to_run = rospy.get_param("/models")
        self.now = datetime.now() # current date and time
        count = 0
        for i in  self.models_to_run:
            print self.models_to_run[i]
            self.run(i,count)
            count +=1
            


    def run(self,model,count):

        infolder = self.models_to_run[model]['world_path']
        if infolder[-1] !='/':
            infolder+='/'
        dae = infolder+self.models_to_run[model]['dae']
        if dae == 'None':
            dae = None
        material = infolder+self.models_to_run[model]['material']

        backup = (self.models_to_run[model]['world_path']+'backups/{}_{}/'.format(self.now.strftime("%Y%m%d%H%M%S"),count)).rstrip()
        backup_check =  os.path.isdir(self.models_to_run[model]['world_path']+'backups/original/')
        os.makedirs(os.path.dirname(backup))

        shutil.copytree(infolder+'models',backup+'/models')
        shutil.copytree(infolder+'materials',backup+'/materials')
        if backup_check == False:
            print "backing up original"
            original = (self.models_to_run[model]['world_path']+'/backups/original/')
            shutil.copytree(infolder+'models',original+'/models')
            shutil.copytree(infolder+'materials',original+'/materials')
                    
        if self.models_to_run[model]['bend_factor']:
            bend_factor = self.models_to_run[model]['bend_factor']
        else:
            bend_factor = 0.0
        if self.models_to_run[model]['rust_factor']:
            rust_factor = self.models_to_run[model]['rust_factor']
        else:
            rust_factor = 0.0

        if self.models_to_run[model]['centre']:
            centre = [self.models_to_run[model]['centre']['x'],self.models_to_run[model]['centre']['y'],self.models_to_run[model]['centre']['z']]
        else:
            centre = None
        if self.models_to_run[model]['height']:
            height = self.models_to_run[model]['height']
            if height == 'None':
                height = None
        else:
            height = None


       
        
        if (bend_factor != 0.0) | (dae == None):
            mesh = Collada(dae)
            geom = mesh.geometries[0]
            triset = geom.primitives[0]
            centre,height = self.check_centre_and_height(centre,height,triset)
            self.alter_model(mesh,dae,triset,centre,bend_factor,height)
        if rust_factor != 0.0:
            self.alter_texture(material,rust_factor)

        return EnvironmentEvolverResponse(True)
        


    def alter_model(self,mesh,dae,triset,centre,bend_factor,height):
        output = []
        for i in range(0,len(triset.vertex[triset.vertex_index])):
            tri_pointset = []
            for j in range(0,len(triset.vertex[triset.vertex_index][i])):
                point = [triset.vertex[triset.vertex_index][i][j][0],triset.vertex[triset.vertex_index][i][j][1],(triset.vertex[triset.vertex_index][i][j][2])]
                #tri_pointset.append(point)
                tri_pointset.append(self.bend(point,centre,bend_factor,height))
            output.append(tri_pointset)

        mesh.geometries[0].primitives[0].vertex[triset.vertex_index] = np.asarray(output)

        mesh.write(dae)

    def alter_texture(self,texture,rust_factor):
        img = cv2.imread(texture,cv2.IMREAD_UNCHANGED)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        rows,cols,channels = img.shape
        rust_mask  = np.zeros((rows,cols), np.uint8)

        rusted = False
        for i in range(rows):
            for j in range(cols):
                if (img_hsv[i,j][0] >10) & (img_hsv[i,j][0] <30)& (img_hsv[i,j][1] > 150) & (img_hsv[i,j][2] < 150) :
                    rusted = True
                    break
            if rusted == True:
                break
        print rusted
        if rusted == False:
            i = int(random.uniform(0.1,0.9)*rows)
            j = int(random.uniform(0.1,0.9)*cols)
            rust_mask[i,j] = 255
 

        else:
            for i in range(rows):
                for j in range(cols):
                    if (img_hsv[i,j][0] >=10) & (img_hsv[i,j][0] <=30)& (img_hsv[i,j][1] >= 150) & (img_hsv[i,j][2] <= 150) :
                        rust_mask[i,j] = 255
            


        rust_mask_original = copy.copy(rust_mask)
        boundary = cv2.Canny(rust_mask,100,200)

        boundary_list = []
        for i in range(rows):
            for j in range(cols):
                if boundary[i,j] == 255:
                    boundary_list.append([i,j])

        i = 1
        while i < int(cols*rows*rust_factor):
            idx = random.randint(0,len(boundary_list)-1)
            open_neighbours = []
            for a in range (-1,2):
                for b in range(-1,2):
                    pixel = [boundary_list[idx][0]+a,boundary_list[idx][1]+b]
                    if pixel[0] < 0:
                        pixel[0] = rows-1
                    elif pixel[0] >=rows:
                        pixel[0] = 0 
                    if pixel[1] < 0:
                        pixel[1] = cols-1
                    elif pixel[1] >=cols:
                        pixel[1] = 0 
                    if rust_mask[pixel[0],pixel[1]]== 0:
                        open_neighbours.append(pixel)
            if len(open_neighbours) > 0:
                p = random.choice(open_neighbours)
                boundary[p[0],p[1]] = 255
                rust_mask[p[0],p[1]] = 255
                open_neighbours.remove(p)
                i+=1
                 
            del boundary_list[idx]
            boundary_list += open_neighbours
            

        count = 0
        for i in range(rows):
            for j in range(cols):
                if rust_mask[i][j] == 255:
                    img_hsv[i][j] = [random.randint(10,30),random.randint(150,255),random.randint(0,150)]


        img = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)    
        kernel = np.ones((5,5),np.float32)/25    
        img = cv2.filter2D(img,-1,kernel)

        cv2.imwrite(texture,img)

        cv2.imshow('img',img) 
        cv2.imshow('rust_mask_original',rust_mask_original)       
        cv2.imshow('rust_mask',rust_mask)
        cv2.imshow('boundary',boundary)
        cv2.waitKey(0)
        cv2.destroyAllWindows()




        


if __name__ == "__main__":
    x = Evolver()
    rospy.spin()