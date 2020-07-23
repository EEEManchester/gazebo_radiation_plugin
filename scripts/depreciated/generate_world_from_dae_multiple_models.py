import sys
import os



dae_files = []
for file in os.listdir(sys.argv[1]):
    if file.endswith(".dae"):
        a = sys.argv[1].split('/')[-2] 
        out = open(sys.argv[1][:-len(a)-1]+file[:-3]+"sdf",'w')
        temp = open(sys.argv[2], "r")
        for line in temp:
            out.write(line.format(file))



