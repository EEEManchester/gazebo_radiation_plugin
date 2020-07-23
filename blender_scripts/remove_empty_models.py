import sys
from os import listdir,remove
from os.path import isfile, join


mypath = sys.argv[1]

if sys.argv[1][-1] != '/':
    sys.argv[1] += '/'

onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

for i in onlyfiles:
    with open(sys.argv[1]+i) as f:
        if '<mesh>' in f.read():
            print(sys.argv[1]+i, "true")
        else:
            remove(sys.argv[1]+i)