#!/usr/bin/env python 


import sys
import subprocess
import os
import numpy as np
import cv2
import rospy
from collada import *
from datetime import datetime
import xml.etree.ElementTree as ET
from gazebo_radiation_plugins.srv import GenYamlsFromWorld


#        backup_check =  os.path.isdir(self.models_to_run[model]['path']+model+'/backups/original/')
#        os.makedirs(os.path.dirname(backup))


class GenYamls(object):

    def __init__(self):
        rospy.init_node('GenYamls')
        s = rospy.Service('generate_yamls_from_world', GenYamlsFromWorld, self.run)
        self.folder = ""

    def run(self,req):
        self.folder = req.folder
        if self.folder[-1] != '/':
            self.folder+='/'

                
        gen_all = False        
        if req.model_subset_filename == '':
            pass
            #think about how to do a run on all 
        else:
            gen_all = False
            cmd = "rosparam load "+self.folder+req.model_subset_filename
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            (output, err) = p.communicate()  
            p_status = p.wait()
            models_to_gen = rospy.get_param('/model_subset')
            rospy.loginfo("models to generate:")
            for m in models_to_gen:
                rospy.loginfo(m)
            if rospy.has_param('/sources'):
                sources = rospy.get_param('/sources')
            else:
                rospy.logwarn("if you wish to add none standard radiaiton sources to your models, please load the sources onto the param server")

        #now = datetime.now().strftime("%Y%m%d%H%M%S")
        #cmd = "cp " + self.folder+req.world_filename + " "+ self.folder+"backup_"+now+"_"+req.world_filename
        #os.system(cmd)


        tree = ET.parse(self.folder+req.world_filename)
        root = tree.getroot()
        for model in root.iter('model'):
            for i in models_to_gen:
                if i == model.attrib['name']:
                    dae = self.findDAE(model) 

                    if dae:
                        material = self.getDAE_material(dae)
                    else:
                        if "gen_material" in models_to_gen[i]:
                            print "here ", models_to_gen[i]
                            gen_material = bool(models_to_gen[i]["gen_material"])
                        else:
                            gen_material = False
                        material = self.getModel_material(model,gen_material,model.attrib['name'])
                        """
                        if material_added:
                            mat = model.find('./link/visual/material')
                            child = ET.SubElement(mat, "script")
                            script = model.find('./link/visual/material/script')
                            child = ET.SubElement(script, "uri")
                            child.set("")
                            child = ET.SubElement(script, "name")
                            child.set("default")
                        """

                    keywords = self.genKeywords(i,self.folder,models_to_gen[i],dae,material)                    
                    temp = open(self.folder+"template/template.yaml", "r")   
                    outfile =  open(self.folder+"yamls/{}.yaml".format(i), "w")  
                    for line in temp:
                        outfile.write(line.format('',**keywords))

                    if "radiation_source_name" in models_to_gen[i]:
                        if ((models_to_gen[i]["radiation_source_name"] != None) & (models_to_gen[i]["radiation_source_name"] != "")& (models_to_gen[i]["radiation_source_name"] != "None")):
                            s = models_to_gen[i]["radiation_source_name"]
                            t = sources[models_to_gen[i]["radiation_source_name"]]["type"]
                            if "radiation_position_offset" in models_to_gen[i]:
                                offset = [models_to_gen[i]["radiation_position_offset"]["x"],models_to_gen[i]["radiation_position_offset"]["y"],models_to_gen[i]["radiation_position_offset"]["z"]]
                            else:
                                offset = [0.0,0.0,0.0]
                            transform = self.findTransform(dae, model.attrib['name'],offset)                   
                            source_xml = self.genRadiationSources(i,s,t,transform)
                            model.append(source_xml)
                            #child = ET.SubElement(model, source_xml)
                            #child.set("model",source_xml)
        
        tree.write(self.folder+"updated_"+req.world_filename)

                    

    def findTransform(self,dae,name,offset):
        if dae == None:
            return str(offset[0]) + " " +str(offset[1]) + " "+str(offset[2]) + " 0 0 0"
        mesh = Collada(self.folder+dae)
        xml_tree = ET.parse(self.folder+dae)
        xroot = xml_tree.getroot()
        ns = xroot.tag.split('}')[0] +'}'
        node = xroot.find("{}library_visual_scenes/{}visual_scene/{}node".format(ns,ns,ns))
        if node.get('id') == name:
            matrix = node.find("{}matrix".format(ns)).text
            matrix = np.asarray(matrix.split(),dtype=float).reshape((4, 4))
            rotation = matrix[0:3,0:3]
            translation = matrix[0:3,3]
        else:
            rotation = None
            translation = [0.0,0.0,0.0]

        
        geom = mesh.geometries[0]
        triset = geom.primitives[0]
        zs = []
        xs = []
        ys =[]
        for i in triset.vertex[triset.vertex_index]:
            for j in i:
                xs.append(j[0])
                ys.append(j[1])
                zs.append(j[2])


        centre = [(min(xs)+max(xs))/2.0,(min(ys)+max(ys))/2.0,(min(zs)+max(zs))/2.0]

        if rotation != None:
            centre = rotation.dot(centre)
        centre += translation + offset

        return  str(centre[0])+ " "+str(centre[1])+ " "+str(centre[2])+ " 0 0 0"

    

    def genRadiationSources(self,name,source,typ,transform):
        temp = open(self.folder+"template/radiation_source_template.sdf", "r")
        template = ""
        keywords = {'name':name,'source':source,'type':typ,'transform':transform}
        for line in temp:
            template += line.format('',**keywords)
        return ET.fromstring(template)



    def getModel_material(self,model,gen_material,name):
        
        materials = model.findall('./link/visual/material/')
        
        for i in materials:
            
            if i.tag == 'script':
                for child in i:
                    if child.tag == 'uri':
                        uri = child.text.split(self.folder.split('/')[-2]+'/')[1]
                    elif child.tag == 'name':
                        name = child.text
                
                found = False
                script_file = open(self.folder+uri, "r")
                for line in script_file:
                    if name in line:
                        found = True
                    if (found == True)&("texture " in line):
                        return "materials"+line.split("..")[1]
                
                return None

        if gen_material:
            colour = [0.0,0.0,0.0,1.0]
            for i in materials:
                if i.tag == 'diffuse':
                    colour = i.text.split(" ")
            image = np.zeros((600, 600, 4), np.uint8)
            image[:] = [int(float(colour[2])*255),int(float(colour[1])*255),int(float(colour[0])*255),int(float(colour[3])*255)]
            cv2.imwrite(self.folder+"materials/textures/{}.png".format(name),image)
            rospy.logwarn("""material generated!!! Now add the orge script to the sdf/world file. The material can be found: """,self.folder+"materials/textures/{}.png".format(name))
            return "materials/textures/{}.png".format(name)
        
        return None



    def getDAE_material(self,dae):

        """
        load material or print out no texture found follow video tutorial on how to add textures
        if material is loaded then check file paths and change the one in the dae as appropriate
        """

        xml_tree = ET.parse(self.folder+dae)
        xroot = xml_tree.getroot()
        ns = xroot.tag.split('}')[0] +'}'
        material_xml = xroot.find("{}library_images/{}image/{}init_from".format(ns,ns,ns))
        if material_xml == None:
            rospy.logwarn("No texture found. Follow the video tutorial on how to add textures using blender")
            return None
        else:
            material = material_xml.text
            mat = material.split(self.folder)
            if len(mat) == 1:
                if mat[0].startswith("../"):
                    mat[0] = mat[0][3:]
                material = mat[0]
            elif len(mat) == 2:
                if os.path.isfile(self.folder+mat[1]):
                    material = mat[1]

            return material
            
                

    def findDAE(self,model):
        geom = model.findall('./link/visual/geometry/')
        tags = []
        for x in geom:
            tags.append(x.tag)
        if 'mesh' in tags:
            daes = model.findall('./link/visual/geometry/mesh/uri')
            for d in daes:
                try:
                    dae = d.text.split(self.folder.split('/')[-2]+'/')[1]                  
                    return dae
                except:
                   pass
        else:
            dae = None
        return dae
        

    def genKeywords(self,name,folder,kws,dae,material):
        if 'bend_factor' in kws:
            bf = kws['bend_factor']
        else:
            bf = 0.0
        if 'rust_factor' in kws:
            rf = kws['rust_factor']
        else:
            rf = 0.0

        return {'model_name': name+':','path':folder,'dae':dae,'material':material,'bend_factor':bf,'rust_factor':rf}


if __name__ == "__main__":
    x = GenYamls()
    rospy.spin()