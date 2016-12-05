#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from __future__ import print_function
import sys
import os


def errorf(s):
    print(s, file=sys.stderr)
    exit(1)

def urdf2srdl(eusurdf_dir_path, model_name, srdl_path, urdf2srdl_bin):
    xacro_path = os.path.join(eusurdf_dir_path, "worlds", "%s.urdf.xacro" % model_name)
    urdf_path = os.path.join("/tmp", "%s.urdf" % model_name)
    uri = "http://knowrob.org/kb/%s#" % model_name

    try:
        os.makedirs(os.path.dirname(srdl_path))
    except OSError:
        pass

    # xacro
    os.system("rosrun xacro xacro.py %s > %s" % (xacro_path, urdf_path))

    os.system("%s -u %s -s %s -i %s" % (urdf2srdl_bin, urdf_path, srdl_path, uri))

    # TODO: texture model
    os.system("sed -i -e \"s@model://@package://eusurdf/models/@g\" %s" % (srdl_path))

if __name__ == '__main__':
    try:
        if len(sys.argv) != 5:
            errorf("%s eusurdf_dir_path model_name srdl_path urdf2srdl_bin" % __file__)
        eusurdf_dir_path = sys.argv[1]
        model_name = sys.argv[2]
        srdl_path = sys.argv[3]
        urdf2srdl_bin = sys.argv[4]
        urdf2srdl(eusurdf_dir_path, model_name, srdl_path, urdf2srdl_bin)
    except Exception as e:
        errorf("Error: %s" % str(e))
