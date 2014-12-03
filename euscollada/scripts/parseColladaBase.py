#!/usr/bin/env python

import sys, os
from xml.dom.minidom import parse, parseString
import xml.dom

reload(sys)
sys.setdefaultencoding('utf-8')

#### >>> copied from xacro/src/xacro.py
# Better pretty printing of xml
# Taken from http://ronrothman.com/public/leftbraned/xml-dom-minidom-toprettyxml-and-silly-whitespace/
def fixed_writexml(self, writer, indent="", addindent="", newl=""):
    # indent = current indentation
    # addindent = indentation to add to higher levels
    # newl = newline string
    writer.write(indent+"<" + self.tagName)

    attrs = self._get_attributes()
    a_names = attrs.keys()
    a_names.sort()

    for a_name in a_names:
        writer.write(" %s=\"" % a_name)
        xml.dom.minidom._write_data(writer, attrs[a_name].value)
        writer.write("\"")
    if self.childNodes:
        if len(self.childNodes) == 1 \
          and self.childNodes[0].nodeType == xml.dom.minidom.Node.TEXT_NODE:
            writer.write(">")
            self.childNodes[0].writexml(writer, "", "", "")
            writer.write("</%s>%s" % (self.tagName, newl))
            return
        writer.write(">%s"%(newl))
        for node in self.childNodes:
            if node.nodeType is not xml.dom.minidom.Node.TEXT_NODE: # 3:
                node.writexml(writer,indent+addindent,addindent,newl) 
                #node.writexml(writer,indent+addindent,addindent,newl)
        writer.write("%s</%s>%s" % (indent,self.tagName,newl))
    else:
        writer.write("/>%s"%(newl))
# replace minidom's function with ours
xml.dom.minidom.Element.writexml = fixed_writexml
#### <<< copied from xacro/src/xacro.py

def search_tagname_id (el, tagname, idname, attr = 'id'):
    ret = None
    for ee in el.getElementsByTagName(tagname):
        if ee.hasAttribute(attr):
            if ee.getAttribute(attr) == idname:
                ret = ee
                break
    return ret

class parseXmlBase:
    doc = None

    def readColladaFile(self, filename):
        self.doc = xml.dom.minidom.parse(filename)

    def getDocumentString(self, indent = '\t'):
        if self.doc:
            return self.doc.toprettyxml(indent = indent)

    def writeDocument(self, strm, indent = '\t'):
        dstr = self.getDocumentString(indent = indent)
        if dstr:
            strm.write(dstr)
        else:
            sys.stderr.write('no document to write!\n')

    def ListToString (self, lst):
        ret = '%s'%lst.pop(0)
        for l in lst:
            ret = ret + ' %s'%l
        return ret

    def StringToList (self, strin):
        ret = []
        for r in strin.split():
            ret.append(float(r))
        return ret

#class parseURDFBase(parseXmlBase):

class parseColladaBase(parseXmlBase):
    def searchLinkid (self, linkname):
        tmp_link = ''
        for f in self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("technique_common")[0].getElementsByTagName("link"):
            if f.getAttribute("name") == linkname:
                tmp_link = f.getAttribute("sid")
        kmodel_id = self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("kinematics_model")[0].getAttribute("id")
        if tmp_link == '':
            sys.stderr.write('link name: %s was not found!\n'%linkname)
        return '%s/%s'%(kmodel_id,tmp_link)

    def searchRootLink (self):
        return

class parseColladaSensor(parseColladaBase):
    library_sensors_node = None
    target_articulated_system = None

    sensor_id = 1
    force_sensor_id = 0
    gyro_sensor_id = 0
    acc_sensor_id = 0
    cam_sensor_id = 0

    def init (self, filename):
        self.readColladaFile(filename)

        plst = self.doc.getElementsByTagName('extra')
        for p in plst:
            if p.hasAttribute('id') and p.getAttribute('id') == 'sensors' and p.hasAttribute('type') and p.getAttribute('type') == 'library_sensors':
                self.library_sensors_node = p

        plst = self.doc.getElementsByTagName('articulated_system')
        for p in plst:
            if p.hasAttribute('id') and p.getAttribute('id').find('_motion') != -1:
                self.target_articulated_system = p

        return self.library_sensors_node != None and self.target_articulated_system != None

    def add_manipulator (self, name, origin, tip, translate = None, rotate = None):
        if self.target_articulated_system == None:
            return
        if isinstance(translate, list):
            translate = self.ListToString(translate)
        if isinstance(rotate, list):
            rotate = self.ListToString(rotate)

        ### add manipulator to articulated system
        ex = self.doc.createElement('extra')
        ex.setAttribute('name', name)
        ex.setAttribute('type', 'manipulator')

        tec = self.doc.createElement('technique')
        tec.setAttribute('profile', 'OpenRAVE')

        ##
        tmp = self.doc.createElement('frame_origin')
        tmp.setAttribute('link', self.searchLinkid(origin))
        tec.appendChild(tmp)

        tmp = self.doc.createElement('frame_tip')
        tmp.setAttribute('link', self.searchLinkid(tip))
        #
        tl = self.doc.createElement('translate')
        if translate == None:
            tl.appendChild(self.doc.createTextNode('0 0 0'))
        else:
            tl.appendChild(self.doc.createTextNode(translate))

        ro = self.doc.createElement('rotate')
        if rotate == None:
            ro.appendChild(self.doc.createTextNode('1 0 0 0'))
        else:
            ro.appendChild(self.doc.createTextNode(rotate))
        tmp.appendChild(tl)
        tmp.appendChild(ro)
        #
        tec.appendChild(tmp)
        ex.appendChild(tec)
        self.target_articulated_system.appendChild(ex)

    def add_sensor (self, name, parent_link, sensor_type, translate = None, rotate = None):
        if self.library_sensors_node == None or self.target_articulated_system == None:
            return
        ### add sensor to articulated system
        ex = self.doc.createElement('extra')
        ex.setAttribute('name', name)
        ex.setAttribute('type', 'attach_sensor')

        tec = self.doc.createElement('technique')
        tec.setAttribute('profile', 'OpenRAVE')

        tmp = self.doc.createElement('instance_sensor')
        tmp.setAttribute('url', '#sensor%d' % self.sensor_id)
        self.sensor_id = self.sensor_id + 1
        tec.appendChild(tmp)

        tmp = self.doc.createElement('frame_origin')
        tmp_parent_link = ''
        for f in self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("technique_common")[0].getElementsByTagName("link"):
            if f.getAttribute("name") == parent_link:
                tmp_parent_link = f.getAttribute("sid")
        kmodel_id = self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("kinematics_model")[0].getAttribute("id")
        if tmp_parent_link == '':
            sys.stderr.write('parent_link: %s was not found!\n'%parent_link)

        tmp.setAttribute('link', '%s/%s'%(kmodel_id,tmp_parent_link))

        tl = self.doc.createElement('translate')
        if translate == None:
            tl.appendChild(self.doc.createTextNode('0 0 0'))
        else:
            tl.appendChild(self.doc.createTextNode(translate))

        ro = self.doc.createElement('rotate')
        if rotate == None:
            ro.appendChild(self.doc.createTextNode('1 0 0 0'))
        else:
            ro.appendChild(self.doc.createTextNode(rotate))

        tmp.appendChild(tl)
        tmp.appendChild(ro)

        tec.appendChild(tmp)
        ex.appendChild(tec)
        self.target_articulated_system.appendChild(ex)

        ### add to library_sensors
        plst = self.library_sensors_node.getElementsByTagName('technique')
        targetNode = None
        for p in plst:
            if p.hasAttribute('profile') and p.getAttribute('profile') == 'OpenRAVE':
                targetNode = p

        if targetNode != None:
            sen = self.doc.createElement('sensor')
            sen.setAttribute('id', 'sensor%d' % (self.sensor_id - 1))

            if sensor_type == 'force' or sensor_type == 'base_force6d':
                sen.setAttribute('type', 'base_force6d')
                sen.setAttribute('sid', '%d'%self.force_sensor_id)
                self.force_sensor_id = self.force_sensor_id + 1
                lf = self.doc.createElement('load_range_force')
                lf.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                lt = self.doc.createElement('load_range_torque')
                lt.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                sen.appendChild(lf)
                sen.appendChild(lt)
            elif sensor_type == 'acceleration' or sensor_type == 'base_imu':
                sen.setAttribute('type', 'base_imu')
                sen.setAttribute('sid', '%d'%self.acc_sensor_id)
                self.acc_sensor_id = self.acc_sensor_id + 1
                max_acc = self.doc.createElement('max_acceleration')
                max_acc.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                sen.appendChild(max_acc)
            elif sensor_type == 'gyro':
                sen.setAttribute('type', 'base_imu')
                sen.setAttribute('sid', '%d'%self.gyro_sensor_id)
                self.gyro_sensor_id = self.gyro_sensor_id + 1
                max_ang = self.doc.createElement('max_angular_velocity')
                max_ang.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                sen.appendChild(max_ang)
            elif sensor_type == 'camera' or sensor_type == 'base_pinhole_camera':
                sen.setAttribute('type', 'base_pinhole_camera')
                sen.setAttribute('sid', '%d'%self.cam_sensor_id)
                self.cam_sensor_id = self.cam_sensor_id + 1
                #img_dim = self.doc.createElement('image_dimensions')
                #img_dim.appendChild()
                #sen.appendChild(img_dim)
                #child_elem = self.doc.createElement('format')
                #child_elem = self.doc.createElement('measurement_time')
                #child_elem = self.doc.createElement('intrinsic')
                #child_elem = self.doc.createElement('focal_length')
                #child_elem = self.doc.createElement('distortion_model')

            targetNode.appendChild(sen)

class replaceLibraryNode(parseColladaBase):

    def init(self, filename):
        self.readColladaFile(filename)

        llst = self.doc.getElementsByTagName('library_nodes')
        if len(llst) == 0:
            sys.stderr.write('This file does not have library_nodes. Do nothing.\n')
            return False

        nlst = self.doc.getElementsByTagName('instance_node')
        if len(nlst) > 1:
            sys.stderr.write('instance node is more than 1.\n')
            return False

        i_node = nlst[0]
        a_url = None
        if(i_node.hasAttribute('url')):
            a_url = i_node.getAttribute('url')

        if not(a_url):
            sys.stderr.write('instance node does not have url\n')
            return False

        a_url = a_url[1:] ## remove first #

        nd = None
        for ll in llst:
            nd = search_tagname_id (ll, 'node', a_url)
            if nd:
                break

        if not(nd):
            sys.stderr.write('node which has id as %s not found\n'%a_url)
            return False

        p_node = i_node.parentNode
        p_node.replaceChild (nd, i_node)
        #p_node.removeChild (i_node)
        #p_node.appendChild(nd.cloneNode(True))

        return True
