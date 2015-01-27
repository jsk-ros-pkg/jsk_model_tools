#!/usr/bin/env python

from xml.dom import minidom
import sys
import tf
from tf.transformations import *
def usage():
    print "Usage:"
    print "  remove_sensor_from_urdf.py link_name input_urdf output_urdf"

def removeLink(xdoc, link_name):
    robot = xdoc.getElementsByTagName("robot")[0]
    # creating child joint
    links = xdoc.getElementsByTagName("link")
    # remove <link>
    for link in links:
        if link.getAttribute("name") == link_name:
            link.parentNode.removeChild(link)
    # remove gazebo_reference
    for gazebo in xdoc.getElementsByTagName("gazebo"):
        if gazebo.getAttribute("reference") == link_name:
            gazebo.parentNode.removeChild(gazebo)
    return xdoc

def updateURDF(link_name, input_file, output_file):
    xdoc = minidom.parse(input_file)
    xdoc = removeLink(xdoc, link_name)
    with open(output_file, "w") as f:
        f.write(xdoc.toprettyxml())


# argument
# link_name output_file

def main(argv):
    if len(argv) == 3:
        link_name = argv[0]
        input_file = argv[1]
        output_file = argv[2]
        updateURDF(link_name, input_file, output_file)
    else:
        usage()
        exit(1)

if __name__ == "__main__":
    main(sys.argv[1:])
