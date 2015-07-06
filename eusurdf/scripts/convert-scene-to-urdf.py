#! /usr/bin/env python

import sys
import os
import os.path
import commands
import re

urdfmodel_dir_path = commands.getoutput('rospack find eusurdf') + "/models/"
eusmodel_dir_path = commands.getoutput('rospack find euslisp') + "/jskeus/eus/models/"
eusmodel_file_path_list = commands.getoutput('ls ' + eusmodel_dir_path + '*.l -1')

for eusmodel_file_path in eusmodel_file_path_list.split('\n'):
    eusmodel_file_name = os.path.basename(eusmodel_file_path)
    regexp_search_res = re.compile("(.*)\-scene\.l").search(eusmodel_file_name)
    if regexp_search_res:
        scene_name = regexp_search_res.group(1)
        print "[convert-scene-to-urdf] converting %s %s" % (eusmodel_file_name, scene_name)
        os.system('rosrun roseus roseus "(progn (load \\"package://eusurdf/euslisp/eus2urdf.l\\") (generate-room-models \\"%s\\") (exit))"' % scene_name)
os.system('touch %s/GENERATED' % urdfmodel_dir_path)
