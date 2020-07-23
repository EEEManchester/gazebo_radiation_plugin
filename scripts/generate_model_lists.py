import sys
from os import listdir,remove
from os.path import isfile, join


mypath = sys.argv[1]

if sys.argv[1][-1] != '/':
    sys.argv[1] += '/'

onlyfiles = [f for f in listdir(mypath) if (isfile(join(mypath, f))&(".dae" in f))]
bendable_files = []
rustable_files = []
for i in onlyfiles:
    with open(mypath+i) as f:
        if '<mesh>' in f.read():
            bendable_files.append(i)
        if '<library_images>' in f.read():
            rustable_files.append(i)


rustable_models = set(rustable_files)
bendable_models = set(bendable_files)
with open(mypath + "all_models.txt", 'w') as f:
    for item in onlyfiles:
        f.write(item.split('.')[0]+'\n')
with open(mypath + "bendable_models.txt", 'w') as f:
    for item in bendable_models:
        f.write(item.split('.')[0]+'\n')
with open(mypath + "rustable_models.txt", 'w') as f:
    for item in rustable_models:
        f.write(item.split('.')[0]+'\n')
    
