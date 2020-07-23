import bpy
import sys
from os import listdir,remove
from os.path import isfile, join

if len(sys.argv) > 1:
    if sys.argv[1][-1] != '/':
        sys.argv[1] += '/'
        mypath = sys.argv[1]
    else:
        mypath = '/home/tom/Desktop/daes/'

def export_all_dae(exportFolder):
    objects = bpy.data.objects
    for object in objects:
        bpy.ops.object.select_all(action='DESELECT')
        object.select_set(state=True)
        exportName = exportFolder + object.name + '.dae'
        bpy.ops.wm.collada_export(filepath=exportName,selected=True)
        

bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')    
export_all_dae(mypath)

onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

for i in onlyfiles:
    with open(mypath+i) as f:
        if '<mesh>' in f.read():
            print(mypath+i, "true")
        else:
            remove(mypath+i)
