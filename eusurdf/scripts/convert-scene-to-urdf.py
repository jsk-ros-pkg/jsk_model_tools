#! /usr/bin/env python

import sys
import os
import os.path
import commands
import re

eusurdf_package_path = sys.argv[1]
collada_to_urdf_exe_path = sys.argv[2]
roseus_exe_path = sys.argv[3]
urdfmodel_dir_path = eusurdf_package_path + "/models/"
eusmodel_dir_path = commands.getoutput('rospack find euslisp') + "/jskeus/eus/models/"
eusmodel_file_path_list = commands.getoutput('ls ' + eusmodel_dir_path + '*.l -1')

for eusmodel_file_path in eusmodel_file_path_list.split('\n'):
    eusmodel_file_name = os.path.basename(eusmodel_file_path)
    regexp_search_res = re.compile("(.*)\-scene\.l").search(eusmodel_file_name)
    if regexp_search_res:
        scene_name = regexp_search_res.group(1)
        print "[convert-scene-to-urdf] converting %s %s" % (eusmodel_file_name, scene_name)
        os.system('%s "(progn (load \\"%s/euslisp/convert-eus-to-urdf.l\\") (generate-room-models \\"%s\\" :eusurdf-package-path \\"%s\\" :collada-to-urdf-exe-path \\"%s\\") (exit))"' % (roseus_exe_path, eusurdf_package_path, scene_name, eusurdf_package_path, collada_to_urdf_exe_path))
os.system('touch %s/GENERATED' % urdfmodel_dir_path)
