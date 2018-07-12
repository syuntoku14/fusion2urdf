#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os.path
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom

# length unit is 'cm' and inertial unit is 'kg/cm^2'
# Maybe if the name has space, this will cause some error.

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


class Link:
    def __init__(self, name, xyz, repo, mass, inertia):
        self.name = name
        self.xyz = xyz
        self.xml = None
        self.repo = repo
        self.mass = mass
        self.inertia = inertia
        
    def gen_link_xml(self):
        """generate the xml strings of links
        """
        self.xyz = [-_ for _ in self.xyz]  # reverse the sign of xyz
        
        link = Element('link')
        link.attrib = {'name':self.name}
        
        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value':str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = \
            {'ixx':str(self.inertia[0]), 'iyy':str(self.inertia[1]),\
            'izz':str(self.inertia[2]), 'ixy':str(self.inertia[3]),\
            'iyz':str(self.inertia[4]), 'ixz':str(self.inertia[5])}        
        
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


def link_gen(dct, repo, link_dict, file_name, inertial_dict):
    """This generates link urdf. Link_dict will be used in joint_gen.
    """
    with open(file_name, mode='a') as f:
        # about base_link
        link = Link(name='base_link', xyz=[0,0,0], repo=repo,\
            mass=inertial_dict['base_link']['mass'],
            inertia=inertial_dict['base_link']['inertia'])
        link.gen_link_xml()
        f.write(link.xml)
        link_dict[link.name] = link.xyz
        # others
        for joint in dct:
            name = dct[joint]['child']
            link = Link(name=name, xyz=dct[joint]['xyz'],\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia=inertial_dict[name]['inertia'])
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
            xyz = [round(p-c, 6) for p, c in \
                zip(link_dict[parent], link_dict[child])]  # xyz = paret - child
            joint = Joint(name=j, xyz=xyz, axis=dct[j]['axis'],\
                parent=parent, child=child)
            joint.gen_joint_xml()
            f.write(joint.xml)


def set_joints_dict(root, joints_dict):
    msg = ''
    
    for joint in root.joints:
        joint_dict = {}

        joint_dict['axis'] = [round(i / 100.0, 6) for i in \
            joint.jointMotion.rotationAxisVector.asArray()]  # converted to meter
        joint_dict['parent'] = joint.occurrenceTwo.name[:-2]
        joint_dict['child'] = joint.occurrenceOne.name[:-2]
        try:
            joint_dict['xyz'] = [round(i / 100.0, 6) for i in \
            joint.geometryOrOriginOne.origin.asArray()]  # converted to meter
        except:
            try:
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in \
                joint.geometryOrOriginTwo.origin.asArray()]  # converted to meter
            except:
                msg = joint.name + " doesn't have joint origin. Please set it and run Again."
                continue
        msg = 'Successfully generated URDF file'
        joints_dict[joint.name] = joint_dict
    return msg

def set_components_dict(root, components, components_dict):
        # Get component properties.            
        for component in components:
            # Skip the root component.
            component_dict = {}
            if root == component:
                continue
            prop = component.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy);
            component_dict['mass'] = round(prop.mass, 6)  #kg
            component_dict['inertia'] = [round(i / 10000.0, 6) for i in \
                prop.getXYZMomentsOfInertia()[1:]]  #kg m^2
            components_dict[component.name] = component_dict
        
        
def gen_urdf(joints_dict, links_dict, inertial_dict, package_name, save_dir, robot_name):
    file_name = save_dir + '/' + robot_name + '.urdf'  # the name of urdf file
    repo = package_name + '/bin_stl/'  # the repository of binary stl files
    print(repo)
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n'.format(robot_name))

    link_gen(joints_dict, repo, links_dict, file_name, inertial_dict)
    joint_gen(joints_dict, repo, links_dict, file_name)

    with open(file_name, mode='a') as f:
        f.write('</robot>')
        
        
def file_dialog(ui):     
        # Set styles of folder dialog.
        folderDlg = ui.createFolderDialog()
        folderDlg.title = 'Fusion Folder Dialog' 
        
        # Show folder dialog
        dlgResult = folderDlg.showDialog()
        if dlgResult == adsk.core.DialogResults.DialogOK:
            return folderDlg.folder
        return False
        

def export_stl(design, save_dir, components):
        # create a single exportManager instance
        exportMgr = design.exportManager
        
        # get the script location
        try: os.mkdir(save_dir + '/mm_stl')
        except: pass
    
        scriptDir = save_dir + '/mm_stl'  
        
        # export the occurrence one by one in the component to a specified file
        for component in components:
            allOccu = component.allOccurrences
            for occ in allOccu:
                fileName = scriptDir + "/" + occ.component.name
                
                # create stl exportOptions
                stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName, )
                stlExportOptions.sendToPrintUtility = False
                stlExportOptions.isBinaryFormat = False
                exportMgr.execute(stlExportOptions)


def run(context):
    ui = None
    msg = ''
    
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return
        
        components = design.allComponents
        root = design.rootComponent  # root component
        package_name = 'fusion2urdf'
        
        # set the names        
        robot_name = root.name.split()[0]
        save_dir = file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        # Generate STl files
        export_stl(design, save_dir, components)        
        
        # Generate URDF
        # Get joint information. All joints are related to root. 
        joints_dict = {}
        msg = set_joints_dict(root, joints_dict)
        
        # Get link information.
        inertial_dict = {}
        set_components_dict(root, components, inertial_dict)
        
        links_dict = {}
        gen_urdf(joints_dict, links_dict, inertial_dict, package_name, save_dir, robot_name)
        ui.messageBox(msg, title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))