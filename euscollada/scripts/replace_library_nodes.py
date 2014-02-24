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

def initialize_parse_collada ():
    global doc

    argvs = sys.argv
    argc = len(argvs)
    if argc > 1:
        fname = argvs[1]
    else:
        print 'usage: %s intput_filename > outputfile'
        exit(0)

    doc = xml.dom.minidom.parse(fname)

    llst = doc.getElementsByTagName('library_nodes')
    if len(llst) == 0:
        print 'This file does not have library_nodes. Do nothing.'
        exit(-1)

    nlst = doc.getElementsByTagName('instance_node')
    if len(nlst) > 1:
        print 'instance node is more than 1.'
        exit(0)

    i_node = nlst[0]
    a_url = None
    if(i_node.hasAttribute('url')):
        a_url = i_node.getAttribute('url')

    if not(a_url):
        print 'instance node does not have url'
        exit(0)

    a_url = a_url[1:] ## remove first #

    nd = None
    for ll in llst:
        nd = search_tagname_id (ll, 'node', a_url)
        if nd:
            break

    if not(nd):
        print 'node which has id as %s not found'%a_url
        exit(0)

    p_node = i_node.parentNode
    p_node.replaceChild (nd, i_node)
    #p_node.removeChild (i_node)
    #p_node.appendChild(nd.cloneNode(True))

def search_tagname_id (el, tagname, idname):
    ret = None
    for ee in el.getElementsByTagName(tagname):
        if ee.hasAttribute('id'):
            if ee.getAttribute('id') == idname:
                ret = ee
                break
    return ret

if __name__ == '__main__':
    initialize_parse_collada()
    sys.stdout.write(doc.toprettyxml(indent = '\t'))
