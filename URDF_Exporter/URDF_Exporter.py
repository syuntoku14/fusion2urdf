#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os.path
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom

# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.

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
       
def set_inertial_dict(root, inertial_dict, msg):
    # Get component properties.      
    allOccs = root.occurrences
    for occs in allOccs:
        # Skip the root component.
        occs_dict = {}
        prop = occs.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        occs_dict['mass'] = round(prop.mass, 6)  #kg
        occs_dict['inertia'] = [round(i / 10000.0, 6) for i in \
        prop.getXYZMomentsOfInertia()[1:]]  #kg m^2
        if occs.component.name == 'base_link':
            inertial_dict['base_link'] = occs_dict
        else:
            inertial_dict[occs.name.replace(':', '__')] = occs_dict
        if ' ' in occs.name or '(' in occs.name or ')' in occs.name:
            msg = 'A space or parenthesis are detected in the name of ' + occs.name + '. Please remove them and run again.'
            break
    return msg


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
                zip(link_dict[parent], link_dict[child])]  # xyz = parent - child
            joint = Joint(name=j, xyz=xyz, axis=dct[j]['axis'],\
                parent=parent, child=child)
            joint.gen_joint_xml()
            f.write(joint.xml)


def set_joints_dict(root, joints_dict, msg):
    for joint in root.joints:
        joint_dict = {}

        joint_dict['axis'] = [round(i / 100.0, 6) for i in \
            joint.jointMotion.rotationAxisVector.asArray()]  # converted to meter
        if joint.occurrenceTwo.component.name == 'base_link':
            joint_dict['parent'] = 'base_link'
        else:
            joint_dict['parent'] = joint.occurrenceTwo.name.replace(':', '__')
        joint_dict['child'] = joint.occurrenceOne.name.replace(':', '__')
        
        try:
            joint_dict['xyz'] = [round(i / 100.0, 6) for i in \
            joint.geometryOrOriginOne.origin.asArray()]  # converted to meter
        except:
            try:
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in \
                joint.geometryOrOriginTwo.origin.asArray()]  # converted to meter
            except:
                msg = joint.name + " doesn't have joint origin. Please set it and run again."
                break
        if ' ' in joint.name or '(' in joint.name or ')' in joint.name:
            msg = 'A space or parenthesis are detected in the name of ' + joint.name + '. Please remove spaces and run again.'
            break
        joints_dict[joint.name] = joint_dict
    return msg


def gen_urdf(joints_dict, links_dict, inertial_dict, package_name, save_dir, robot_name):
    file_name = save_dir + '/' + robot_name + '.urdf'  # the name of urdf file
    repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
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
        

def copy_body(allOccs, old_occs):
    # copy the old occs to new component
    bodies = old_occs.bRepBodies
    transform = adsk.core.Matrix3D.create()
    old_component_name = old_occs.component.name
    
    # Create new components from occs
    # This support even when a component has some occses. 
    same_occs = []  
    for occs in allOccs:
        if occs.component.name == old_component_name:
            same_occs.append(occs)
    for occs in same_occs:
        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            old_occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = occs.name.replace(':', '__')
        new_occs = allOccs[-1]
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)
    old_occs.component.name = 'old_component'


def copy_occs(root):    
    # duplicate all the components
    allOccs = root.occurrences
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)


def export_stl(design, save_dir, components):
        # create a single exportManager instance
        exportMgr = design.exportManager
        # get the script location
        try: os.mkdir(save_dir + '/mm_stl')
        except: pass
        scriptDir = save_dir + '/mm_stl'  
        # export the occurrence one by one in the component to a specified file
        for component in components:
            if 'old' in component.name:
                continue
            allOccus = component.allOccurrences
            for occ in allOccus:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name              
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = False
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + 'has something wrong.')
                    


def run(context):
    ui = None
    success_msg = 'Successfully generated URDF file'
    msg = success_msg
    
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 
        components = design.allComponents

        # set the names        
        package_name = 'fusion2urdf'
        robot_name = root.name.split()[0]
        save_dir = file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        save_dir = save_dir + '/' + robot_name
        try: os.mkdir(save_dir)
        except: pass     
        
        # Generate URDF
        # Get joint information. All joints are related to root. 
        joints_dict = {}
        msg = set_joints_dict(root, joints_dict, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        print('joint_ok')    
        # Get inertial information.
        inertial_dict = {}
        msg = set_inertial_dict(root, inertial_dict, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0
        print('inertial_ok')
        
        links_dict = {}
        gen_urdf(joints_dict, links_dict, inertial_dict, package_name, save_dir, robot_name)

        # Generate STl files        
        copy_occs(root)
        export_stl(design, save_dir, components)   
        
        ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))