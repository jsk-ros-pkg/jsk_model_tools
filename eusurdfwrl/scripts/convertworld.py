#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('filename')
parser.add_argument('outfilename')
args = parser.parse_args()
filename = args.filename
outfilename = args.outfilename

objs = dict()

import rospkg
rospack = rospkg.RosPack()
eusurdfwrl_dir = rospack.get_path("eusurdfwrl")

from scipy.spatial.transform import Rotation as R

import xml.etree.ElementTree as ET
tree = ET.parse(filename)
root = tree.getroot()
for inc in root[0]:
    if inc.tag == "include":
        name=""
        uri=""
        pose="0 0 0 0 0 0"
        for child in inc:
            if child.tag == "name":
                name = child.text
            if child.tag == "uri":
                uri = child.text
            if child.tag == "pose":
                pose = child.text
        if name == "" or uri == "":
            continue
        obj = dict()
        obj["name"] = name
        obj["file"] = eusurdfwrl_dir + "/models/" + uri.replace("model://","") + "/" + uri.replace("model://","") + ".wrl"
        obj["static"] = True
        obj["static_joint"] = True
        pose_array = pose.split()
        obj["translation"] = [float(pose_array[0]), float(pose_array[1]), float(pose_array[2])]
        r = R.from_euler('xyz', [float(pose_array[3]), float(pose_array[4]), float(pose_array[5])])
        obj["rotation"] = r.as_dcm().tolist()
        objs[name] = obj

import yaml
f = open(outfilename, "w")
f.write(yaml.dump(objs))
f.close()

