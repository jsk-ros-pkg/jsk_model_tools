#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from __future__ import print_function
import sys
import os
import copy
import lxml.etree as E


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

    # URDF2SRDL
    os.system("%s -u %s -s %s -i %s" % (urdf2srdl_bin, urdf_path, srdl_path, uri))

    # TODO: texture model
    os.system("sed -i -e \"s@model://@package://eusurdf/models/@g\" %s" % (srdl_path))

    doctype_string = ""
    with open(srdl_path, "r") as f:
        xmlstr = f.read()
        header_end = xmlstr.find("<rdf:RDF")
        if header_end != -1:
            doctype_string = xmlstr[0:header_end]

    tree = E.parse(srdl_path, parser=E.XMLParser(remove_blank_text=True))
    nsmap = tree.getroot().nsmap
    ns_robot = nsmap.pop(None)
    robot_name = "%s%s_robot1" % (ns_robot, model_name)

    # add knowrob_common ontology to imports
    owl_ontology = tree.xpath("//rdf:RDF/owl:Ontology", namespaces=nsmap)[0]
    owl_ontology.attrib["{%s}about" % nsmap["rdf"]] += ".owl"
    owl_imports = E.SubElement(tree.getroot(), "{%s}imports" % nsmap["owl"])
    owl_imports.attrib["{%s}resource" % nsmap["rdf"]] = "package://knowrob_common/owl/knowrob.owl"
    owl_ontology.append(owl_imports)

    # set type SemanticEnvironmentMap
    owl_inst = tree.xpath("//rdf:RDF/owl:NamedIndividual[@rdf:about='%s']" % robot_name, namespaces=nsmap)[0]
    for t in owl_inst.findall("{%s}type" % nsmap["rdf"], namespaces=nsmap):
        t.getparent().remove(t)
    sem = E.SubElement(tree.getroot(), "{%s}type" % nsmap["rdf"])
    sem.attrib["{%s}resource" % nsmap["rdf"]] = "%sSemanticEnvironmentMap" % nsmap["knowrob"]
    owl_inst.append(sem)

    # add tags for publishing markers to each link and joint
    objs = []
    for urdf_obj in ["FixedUrdfJoint", "RevoluteUrdfJoint", "PrismaticUrdfJoint", "UrdfLink"]:
        xpath = "//rdf:RDF/owl:NamedIndividual[rdf:type[@rdf:resource='%s%s']]" % (nsmap["srdl2-comp"], urdf_obj)
        objs += tree.xpath(xpath, namespaces=nsmap)
    for elem in objs:
        # add knowrob:describedInMap
        d = E.SubElement(tree.getroot(), "{%s}describedInMap" % nsmap["knowrob"])
        d.attrib["{%s}resource" % nsmap["rdf"]] = robot_name
        elem.append(d)

        # add knowrob:hasVisual
        mesh = elem.find("{%s}mesh_filename" % nsmap["srdl2-comp"], namespaces=nsmap)
        if mesh is None:
            d = E.SubElement(tree.getroot(), "{%s}hasVisual" % nsmap["knowrob"])
            d.attrib["{%s}datatype" % nsmap["rdf"]] = "%sboolean" % nsmap["xsd"]
            d.text = "false"
            elem.append(d)
        else:
            mesh_filename = mesh.text
            p = E.SubElement(tree.getroot(), "{%s}pathToCadModel" % nsmap["knowrob"])
            p.attrib["{%s}datatype" % nsmap["rdf"]] = "%sstring" % nsmap["xsd"]
            p.text = mesh_filename
            elem.append(p)

        for sj in elem.findall("{%s}succeedingJoint" % nsmap["srdl2-comp"], namespaces=nsmap):
            e = E.SubElement(tree.getroot(), "{%s}subComponent" % nsmap["srdl2-comp"])
            e.attrib["{%s}resource" % nsmap["rdf"]] = sj.attrib["{%s}resource" % nsmap["rdf"]]
            elem.append(e)
        for sl in elem.findall("{%s}succeedingLink" % nsmap["srdl2-comp"], namespaces=nsmap):
            e = E.SubElement(tree.getroot(), "{%s}subComponent" % nsmap["srdl2-comp"])
            e.attrib["{%s}resource" % nsmap["rdf"]] = sl.attrib["{%s}resource" % nsmap["rdf"]]
            elem.append(e)

    # readdress reference of proprioceptions from joint to link
    preps = tree.xpath("//rdf:RDF/owl:NamedIndividual[rdf:type[@rdf:resource='%sProprioception']]" % nsmap["knowrob"], namespaces=nsmap)
    for prep in preps:
        acted_on = prep.find("{%s}objectActedOn" % nsmap["knowrob"], namespaces=nsmap)
        joint_name = acted_on.attrib["{%s}resource" % nsmap["rdf"]]
        joint = tree.xpath("//rdf:RDF/owl:NamedIndividual[@rdf:about='%s']" % joint_name, namespaces=nsmap)[0]
        j_subcomp = joint.find("{%s}subComponent" % nsmap["srdl2-comp"], namespaces=nsmap)
        link_name = j_subcomp.attrib["{%s}resource" % nsmap["rdf"]]
        acted_on.attrib["{%s}resource" % nsmap["rdf"]] = link_name

    xml_strings = E.tostring(tree, encoding="utf-8", xml_declaration=False,
                             pretty_print=True, with_comments=True).split(os.linesep)
    replace_ns = False
    for i, s in enumerate(xml_strings):
        if s.find("SRDL2 Robot Capabilities") != -1:
            replace_ns = True
        if replace_ns:
            nsmap.update({"robot": ns_robot})
            for nskey, nsval in nsmap.items():
                s = s.replace(nsval, "&%s;" % nskey)
            xml_strings[i] = s
    with open(srdl_path, "w") as f:
        f.write(doctype_string)
        f.write(os.linesep.join(xml_strings))


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
