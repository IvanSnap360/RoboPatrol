import sys
import os
import glob

src_dir = "/home/robopatrol/RoboPatrol/src"

dirs = os.listdir(src_dir)
for dir in dirs:
    file_dir = glob.glob("{}/{}/src/*.py".format(src_dir, dir))
    for f in file_dir:
        file = open(f, "r")
        lines = file.readlines()
        file.close()
        lines[0] = "#! /usr/bin/env python3 \n"
        file = open(f, "w")
        file.writelines(lines)
        file.close()
