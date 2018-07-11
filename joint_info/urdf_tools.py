# coding: utf-8

"""
 [summary]
 Color should be added at the gen_link
[description]

Returns:
    [type] -- [description]
"""


from joints import joints_dict
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom

def mm2m(dct):
    for i in dct:
        xyz = dct[i]['xyz']
        dct[i]['xyz'] = [round(_ / 1000.0, 6) for _ in xyz]
    return dct

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

class Link:
    def __init__(self, name, xyz, repo):
        self.name = name
        self.xyz = xyz
        self.xml = None
        self.repo = repo
        
    def gen_link_xml(self):
        """generate the xml strings of links
        """
        self.xyz = [-_ for _ in self.xyz]  # reverse the sign of xyz
        
        link = Element('link')
        link.attrib = {'name':self.name}
        
        # visual
        visual = SubElement(link, 'visual')
        origin_v = SubElement(visual, 'origin')
        origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.attrib = {'filename':'package://' + self.repo + self.name + '_m-binary.stl'}
        material = SubElement(visual, 'material')
        material.attrib = {'name':'silver'}
        color = SubElement(material, 'color')
        color.attrib = {'rgba':'1 0 0 1'}
        
        # collision
        collision = SubElement(link, 'collision')
        origin_c = SubElement(collision, 'origin')
        origin_c.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.attrib = {'filename':'package://' + self.repo + self.name + '_m-binary.stl'}

        # print("\n".join(prettify(link).split("\n")[1:]))
        self.xml = "\n".join(prettify(link).split("\n")[1:])

def link_gen(dct, repo, link_dict, file_name):
    """This generates link urdf. Link_dict will be used in joint_gen.
    """
    with open(file_name, mode='a') as f:
        # about base_link
        link = Link(name='base_link', xyz=[0,0,0], repo=repo)
        link.gen_link_xml()
        f.write(link.xml)
        link_dict[link.name] = link.xyz
        # others
        for joint in dct:
            link = Link(name=dct[joint]['child'], xyz=dct[joint]['xyz'], repo=repo)
            link.gen_link_xml()
            f.write(link.xml)
            link_dict[link.name] = link.xyz
            
class Joint:
    def __init__(self, name, xyz, axis, parent, child, type_='continuous'):
        self.name = name
        self.type = type_
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.axis = axis
        self.xml = None
    
    def gen_joint_xml(self):
        """generate the xml strings of joints
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        axis = SubElement(joint, 'axis')
        axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        
        # print("\n".join(prettify(joint).split("\n")[1:]))
        self.xml = "\n".join(prettify(joint).split("\n")[1:])


def joint_gen(dct, repo, link_dict, file_name):
    with open(file_name, mode='a') as f:
        for j in dct:
            parent = dct[j]['parent']
            child = dct[j]['child']
            xyz = [round(p-c, 6) for p, c in                    zip(link_dict[parent], link_dict[child])]  # xyz = paret - child
            joint = Joint(name=j, xyz=xyz, axis=dct[j]['axis'],                          parent=parent, child=child)
            joint.gen_joint_xml()
            f.write(joint.xml)