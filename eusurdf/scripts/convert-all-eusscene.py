#! /usr/bin/env python

import sys
import os
import os.path
try:
    import commands
except ImportError: #py3k
    import subprocess as commands # python3 / subprocess supports getoutput
import re


def convert_all_eusscene ():

    urdfmodel_dir_path = commands.getoutput('rospack find eusurdf') + "/models/"
    eusmodel_dir_path = commands.getoutput('rospack find euslisp') + "/jskeus/eus/models/"
    eusmodel_file_path_list = commands.getoutput('ls ' + eusmodel_dir_path + '*.l -1')

    for eusmodel_file_path in eusmodel_file_path_list.split('\n'):
        eusmodel_file_name = os.path.basename(eusmodel_file_path)
        regexp_search_res = re.compile("(.*)\-scene\.l").search(eusmodel_file_name)
        if regexp_search_res:
            print("[convert-all-eusmodel] converting %s" % eusmodel_file_name)
            eusmodel_function_name = regexp_search_res.group(1)
            os.system('mkdir -p %s' % urdfmodel_dir_path+eusmodel_function_name+"-scene")
            os.system('rosrun roseus roseus "(progn (load \\"package://pr2eus_moveit/euslisp/eus2scene.l\\") (load \\"package://euslisp/jskeus/eus/models/%s\\") (generate-scene-file-from-eusscene (%s) \\"%s\\") (exit))"' % (eusmodel_file_name, eusmodel_function_name, urdfmodel_dir_path+eusmodel_function_name+"-scene/model.scene"))

if __name__ == '__main__':
    convert_all_eusscene()
