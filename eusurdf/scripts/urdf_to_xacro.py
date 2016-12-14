#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import lxml.etree
import lxml.builder
import os
import sys

class InvalidURDFException(Exception):
    pass


class URDF2XACRO(object):
    def __init__(self, urdf_path, xacro_path):
        self.root = lxml.etree.parse(urdf_path,
                                     parser=lxml.etree.XMLParser(remove_blank_text=True))
        self.xacro_path = xacro_path
    def find_root_link(self):
        links = self.root.xpath("//robot/link")
        if len(links) == 0:
            raise InvalidURDFException("No link found in model")
        elif len(links) == 1:
            return links[0]
        else:
            link_names = [l.get("name") for l in links]
            joints = self.root.xpath("//robot/joint")
            for j in joints:
                child_link_name = j.find("child").get("link")
                if child_link_name in link_names:
                    link_names.remove(child_link_name)
            if len(link_names) != 1:
                raise InvalidURDFException("Links are not connected with joints: %s" % link_names)
            return self.root.xpath("//robot/link[@name='%s']" % link_names[0])[0]
    def add_namespace(self, ns):
        ns_str = "${%s}_" % ns

        links = self.root.xpath("//robot/link")
        for l in links:
            l.set("name", ns_str + l.get("name"))

        gazebo = self.root.xpath("//robot/gazebo")
        for g in gazebo:
            if g.get("reference") is not None:
                g.set("reference", ns_str + g.get("reference"))

        joints = self.root.xpath("//robot/joint")
        for j in joints:
            j.set("name", ns_str + j.get("name"))
            j_parent = j.find("parent")
            j_parent.set("link", ns_str +  j_parent.get("link"))
            j_child = j.find("child")
            j_child.set("link", ns_str + j_child.get("link"))

        transmissions = self.root.xpath("//robot/transmission")
        for t in transmissions:
            t.set("name", ns_str + t.get("name"))
            t_joint = t.find("joint")
            t_joint.set("name", ns_str + t_joint.get("name"))
            t_actuator = t.find("actuator")
            t_actuator.set("name", ns_str + t_actuator.get("name"))

    def inject_macro(self):
        self.add_namespace("name")
        root_link = self.find_root_link()
        robot = self.root.getroot()
        robot_name = robot.get("name")
        first_joint = robot.find("joint")
        virtual_joint = lxml.etree.Element("joint",
                                           attrib={"name": "${name}_root_parent_joint", "type": "fixed"})
        virtual_joint.append(lxml.etree.Element("parent", attrib={"link": "${parent}"}))
        virtual_joint.append(lxml.etree.Element("child", attrib={"link": root_link.get("name")}))
        virtual_joint.append(lxml.etree.Element("insert_block", attrib={"name": "origin"}))
        if first_joint is not None:
            first_joint.addprevious(virtual_joint)
        else:
            robot.append(virtual_joint)
        macro = lxml.etree.Element("macro",
                                   attrib={"name": robot_name, "params": "name parent *origin"})
        for e in robot.getchildren():
            macro.append(e)
        for e in robot.getchildren():
            robot.remove(e)
        robot.append(macro)

    def save(self):
        out_path = os.path.abspath(self.xacro_path)
        if not os.path.exists(os.path.dirname(out_path)):
            os.makedirs(os.path.dirname(out_path))
        xmlstring = lxml.etree.tostring(self.root,
                                        encoding="utf-8",
                                        xml_declaration=True,
                                        pretty_print=True,
                                        with_comments=True)
        with open(out_path, "w") as f:
            f.write(xmlstring)
        print "saved to %s" % out_path
    def convert(self):
        self.inject_macro()
        self.save()

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description="xacrify urdf file")
    p.add_argument("urdf", type=str, help="path to input urdf file")
    p.add_argument("xacro", type=str, help="path to output xacro file")
    args = p.parse_args()

    c = URDF2XACRO(args.urdf, args.xacro)
    c.convert()
