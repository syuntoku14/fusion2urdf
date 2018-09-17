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
    """
    Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


# ---------------------------------------------
# Class and Functions for Links


class Link:
    """
    Class to manage the information of the links
    """
    def __init__(self, name, xyz, repo, mass, inertia):
        self.name = name
        self.xyz = xyz
        self.link_xml = None
        self.repo = repo
        self.mass = mass
        self.inertia = inertia
        
    def gen_link_xml(self):
        """
        Generate the link_xml and hold it by link_xml
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
        self.link_xml = "\n".join(prettify(link).split("\n")[1:])


def write_link_urdf(dct, repo, link_dict, file_name, inertial_dict):
    """
    Write link information in urdf file.
    In this function, Link_dict is changed for write_joint_tran_urdf.
    """
    with open(file_name, mode='a') as f:
        # for base_link
        link = Link(name='base_link', xyz=[0,0,0], repo=repo,\
            mass=inertial_dict['base_link']['mass'],
            inertia=inertial_dict['base_link']['inertia'])
        link.gen_link_xml()
        f.write(link.link_xml)
        link_dict[link.name] = link.xyz
        # others
        for joint in dct:
            name = dct[joint]['child']
            link = Link(name=name, xyz=dct[joint]['xyz'],\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia=inertial_dict[name]['inertia'])
            link.gen_link_xml()
            f.write(link.link_xml)
            link_dict[link.name] = link.xyz

       
def set_inertial_dict(root, inertial_dict, msg):
    """
    inertial_dict holds the information of mass and inertia.
    """
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

# ---------------------------------------------
# Class and Functions for Joints and Transmission

class Joint:
    """
    Class to manage the information of the links
    """
    def __init__(self, name, xyz, axis, parent, child, type_='continuous'):
        self.name = name
        self.type = type_
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.axis = axis
        self.joint_xml = None
        self.tran_xml = None
    
    def gen_joint_xml(self):
        """
        Generate the joint_xml and hold it by link_xml
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
        self.joint_xml = "\n".join(prettify(joint).split("\n")[1:])

    def gen_transmission_xml(self):
        """
        Generate the tran_xml and hold it by tran_xml
        mechanicalTransmission:1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}
        
        type_ = SubElement(tran, 'type')
        type_.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'PositionJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'PositionJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'
        
        self.tran_xml = "\n".join(prettify(tran).split("\n")[1:])
            
            
def write_joint_tran_urdf(dct, repo, link_dict, file_name):
    """
    Write joint and transmission information in urdf file.
    """
    with open(file_name, mode='a') as f:
        for j in dct:
            parent = dct[j]['parent']
            child = dct[j]['child']
            xyz = [round(p-c, 6) for p, c in \
                zip(link_dict[parent], link_dict[child])]  # xyz = parent - child
            joint = Joint(name=j, xyz=xyz, axis=dct[j]['axis'],\
                parent=parent, child=child)
            joint.gen_joint_xml()
            joint.gen_transmission_xml()
            f.write(joint.joint_xml)
            f.write(joint.tran_xml)
        

def set_joints_dict(root, joints_dict, msg):
    """
    joints_dict holds 'parent', 'axis' and 'xyz' information of the joints.
    """
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


def write_urdf(joints_dict, links_dict, inertial_dict, package_name, save_dir, robot_name):
    file_name = save_dir + '/' + robot_name + '.urdf'  # the name of urdf file
    repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
    print(repo)
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n'.format(robot_name))

    write_link_urdf(joints_dict, repo, links_dict, file_name, inertial_dict)
    write_joint_tran_urdf(joints_dict, repo, links_dict, file_name)

    with open(file_name, mode='a') as f:
        # gazebo plugin
        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('</robot>\n')
        

def write_launch(robot_name, save_dir):
    launch = Element('launch')
    param = SubElement(launch, 'param')
    param.attrib = {'name':'robot_description', 'textfile':'$(find fusion2urdf)/{}/{}.urdf'.format(robot_name, robot_name)}
    include_ =  SubElement(launch, 'include')
    include_.attrib = {'file':'$(find gazebo_ros)/launch/empty_world.launch'}        
    node = SubElement(launch, 'node')
    node.attrib = {'name':'spawn_urdf', 'pkg':'gazebo_ros', 'type':'spawn_model',\
                    'args':'-param robot_description -urdf -model {}'.format(robot_name)}
    
    controller_name = robot_name + '_controller'
    rosparam = SubElement(launch, 'rosparam')
    rosparam.attrib = {'file':'$(find fusion2urdf)/launch/' + controller_name + '.yaml',
                       'command':'load'}
    node_controller = SubElement(launch, 'node')
    node_controller.attrib = {'name':'controller_spawner', 'pkg':'controller_manager', 'type':'spawner',\
                    'args':'{}'.format(controller_name)}
    
    launch_xml = "\n".join(prettify(launch).split("\n")[1:])        
    
    file_name = save_dir + '/' + robot_name + '.launch'    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)
        

def write_yaml(robot_name, save_dir, joint_dict):
    controller_name = robot_name + '_controller'
    file_name = save_dir + '/' + controller_name + '.yaml'
    with open(file_name, 'w') as f:
        f.write(controller_name + ':\n')
        f.write('  type: "position_controllers/JointTrajectoryController"\n')
        f.write('  joints: \n')
        for joint in joint_dict:
            f.write('    - ' + joint +'\n')
        

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
        write_urdf(joints_dict, links_dict, inertial_dict, package_name, save_dir, robot_name)
        write_launch(robot_name, save_dir)
        write_yaml(robot_name, save_dir, joints_dict)
        # Generate STl files        
        copy_occs(root)
        export_stl(design, save_dir, components)   
        
        ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))