import sys
import os

a = sys.argv[1].split('/')[-2] 
out = open(sys.argv[1][:-len(a)-1]+"world.sdf",'w')

temp = open(sys.argv[2]+"template_start.sdf", "r")
for line in temp:
    out.write(line.format(file))

dae_files = []
for file in os.listdir(sys.argv[1]):
    if file.endswith(".dae"):
        temp = open(sys.argv[2]+"template_middle.sdf", "r")
        for line in temp:
            out.write(line.format(file[:-4]))

temp = open(sys.argv[2]+"template_end.sdf", "r")
for line in temp:
    out.write(line.format(file))


