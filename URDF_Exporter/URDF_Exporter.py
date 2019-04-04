#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os.path
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100

# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

# --------------------
# utilities

def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element
    
    
    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def file_dialog(ui):     
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def copy_occs(root):    
    """    
    duplicate all the components
    """    
    def copy_body(allOccs, old_occs):
        """    
        copy the old occs to new component
        """
        
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
        
    allOccs = root.occurrences
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)


def export_stl(design, save_dir, components):  
    """
    export stl files into "sace_dir/"
    
    
    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """
          
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


# --------------------
# Class and Functions for Links


class Link:
    """
    Keep and manage the information of a link
    
    
    Attributes
    ----------
    name: str
        name of the link
    xyz: [x, y, z]
        coordinate for the visual and collision
    center_of_mass: [x, y, z]
        coordinate for the center of mass
    link_xml: str
        generated xml describing about the link
    repo: str
        the name of the repository to save the xml file
    mass: float
        mass of the link
    inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
        tensor of the inertia
    """
    def __init__(self, name, xyz, center_of_mass, repo, mass, inertia_tensor):
        """
        Parameters
        ----------
        name: str
            name of the link
        xyz: [x, y, z]
            coordinate for the visual and collision
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        repo: str
            the name of the repository to save the xml file
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        """
        self.name = name
        # xyz for visual
        self.xyz = [-_ for _ in xyz]  # reverse the sign of xyz
        # xyz for center of mass
        self.center_of_mass = center_of_mass
        self.link_xml = None
        self.repo = repo
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        
    def gen_link_xml(self):
        """
        Generate the link_xml and hold it by self.link_xml
        """
        
        link = Element('link')
        link.attrib = {'name':self.name}
        
        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.center_of_mass]), 'rpy':'0 0 0'}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value':str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = \
            {'ixx':str(self.inertia_tensor[0]), 'iyy':str(self.inertia_tensor[1]),\
            'izz':str(self.inertia_tensor[2]), 'ixy':str(self.inertia_tensor[3]),\
            'iyz':str(self.inertia_tensor[4]), 'ixz':str(self.inertia_tensor[5])}        
        
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

       
def set_inertial_dict(root, inertial_dict, msg):
    """
    inertial_dict holds the information of mass, inertia and center_of_mass
    
    
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    inertial_dict: vacant dict
        inertial_dict will hold the information of the each inertia
    msg: str
        Tell the status
        
    
    Returns
    ----------
    msg: str
        Tell the status
    """
    # Get component properties.      
    allOccs = root.occurrences
    for occs in allOccs:
        # Skip the root component.
        occs_dict = {}
        prop = occs.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        mass = round(prop.mass, 6)  #kg
        center_of_mass = [round(_ / 100.0, 6) for _ in prop.centerOfMass.asArray()]
        occs_dict['center_of_mass'] = center_of_mass
        inertia_world = [i / 10000.0 for i in \
            prop.getXYZMomentsOfInertia()[1:]]  #kg m^2
        # xx yy zz xy xz yz(default)
        inertia_world[4], inertia_world[5] = inertia_world[5], inertia_world[4]
        occs_dict['mass'] = mass
        occs_dict['inertia'] = origin2center_of_mass(inertia_world, center_of_mass, mass)  
        
        if occs.component.name == 'base_link':
            inertial_dict['base_link'] = occs_dict
        else:
            inertial_dict[occs.name.replace(':', '__')] = occs_dict
        if ' ' in occs.name or '(' in occs.name or ')' in occs.name:
            msg = 'A space or parenthesis are detected in the name of ' + occs.name + '. Please remove them and run again.'
            break
    return msg

# --------------------
# Class and Functions for Joints and Transmission

class Joint:
    """
    Keep and manage the information of a joint
    
    
    Attributes
    ----------
    name: str
        name of the joint
    type: str
        type of the joint(ex: rev)
    xyz: [x, y, z]
        coordinate of the joint
    axis: [x, y, z]
        coordinate of axis of the joint
    parent: str
        parent link
    child: str
        child link
    joint_xml: str
        generated xml describing about the joint
    tran_xml: str
        generated xml describing about the transmission
    """
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def gen_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        # print("\n".join(prettify(joint).split("\n")[1:]))
        self.joint_xml = "\n".join(prettify(joint).split("\n")[1:])

    def gen_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
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


def set_joints_dict(root, joints_dict, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints
    
    
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    joints_dict: vacant dict
        joints_dict will hold the information of the each joints
    msg: str
        Tell the status
        
    
    Returns
    ----------
    msg: str
        Tell the status
    """
    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf
    
    for joint in root.joints:
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type
        
        # swhich by the type of the joint
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0
        if joint_type == 'revolute':
            joint_dict['axis'] = [round(i / 100.0, 6) for i in \
                joint.jointMotion.rotationAxisVector.asArray()]  # converted to meter
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'
        elif joint_type == 'prismatic':
            joint_dict['axis'] = [round(i / 100.0, 6) for i in \
                joint.jointMotion.slideDirectionVector.asArray()]  # converted to meter
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue/100, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue/100, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
        elif joint_type == 'fixed':
            pass
        
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
                if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                    data = joint.geometryOrOriginTwo.geometry.origin.asArray()
                else:
                    data = joint.geometryOrOriginTwo.origin.asArray()
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
            except:
                msg = joint.name + " doesn't have joint origin. Please set it and run again."
                break
        if ' ' in joint.name or '(' in joint.name or ')' in joint.name:
            msg = 'A space or parenthesis are detected in the name of ' + joint.name + '. Please remove spaces and run again.'
            break
        joints_dict[joint.name] = joint_dict
        print(joint_dict)
    return msg
            

# --------------------
# Function for wrting  

def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict):
    """
    Write links information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial
    
    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link(name='base_link', xyz=[0,0,0], 
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'])
        links_xyz_dict[link.name] = link.xyz
        link.gen_link_xml()
        f.write(link.link_xml)

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = \
                [ i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            link = Link(name=name, xyz=joints_dict[joint]['xyz'],\
                center_of_mass=center_of_mass,\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia_tensor=inertial_dict[name]['inertia'])
            links_xyz_dict[link.name] = link.xyz            
            link.gen_link_xml()
            f.write(link.link_xml)


def write_joint_tran_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """
    Write joints and transmission information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """
    
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()
                
            joint = Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            joint.gen_joint_xml()
            joint.gen_transmission_xml()
            f.write(joint.joint_xml)
            f.write(joint.tran_xml)


def write_gazebo_plugin_and_endtag(file_name):
    """
    Write about gazebo_plugin and the </robot> tag at the end of the urdf
    
    
    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode='a') as f:
        # gazebo plugin
        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('</robot>\n')
        

def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, save_dir, robot_name):
    file_name = save_dir + '/' + robot_name + '.urdf'  # the name of urdf file
    repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n'.format(robot_name))

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict)
    write_joint_tran_urdf(joints_dict, repo, links_xyz_dict, file_name)
    write_gazebo_plugin_and_endtag(file_name)


def write_gazebo_launch(robot_name, save_dir):
    """
    write gazebo launch file "save_dir/launch/robot_name_gazebo.launch"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """
    
    try: os.mkdir(save_dir + '/launch')
    except: pass     
    
    launch = Element('launch')
    param = SubElement(launch, 'param')
    param.attrib = {'name':'robot_description', 'textfile':'$(find fusion2urdf)/{}/{}.urdf'.format(robot_name, robot_name)}

    include_ =  SubElement(launch, 'include')
    include_.attrib = {'file':'$(find gazebo_ros)/launch/empty_world.launch'}        
    
    number_of_args = 5
    args = [None for i in range(number_of_args)]
    args_name_value_pairs = [['paused', 'false'], ['use_sim_time', 'true'],
                             ['gui', 'true'], ['headless', 'false'], 
                             ['debug', 'false']]
                             
    for i, arg in enumerate(args):
        arg = SubElement(include_, 'arg')
        arg.attrib = {'name' : args_name_value_pairs[i][0] , 
        'value' : args_name_value_pairs[i][1]}

    node = SubElement(launch, 'node')
    node.attrib = {'name':'spawn_urdf', 'pkg':'gazebo_ros', 'type':'spawn_model',\
                    'args':'-param robot_description -urdf -model {}'.format(robot_name)}
    
    launch_xml = "\n".join(prettify(launch).split("\n")[1:])        
    
    file_name = save_dir + '/launch/' + robot_name + '_gazebo.launch'    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)


def write_control_launch(robot_name, save_dir, joints_dict):
    """
    write control launch file "save_dir/launch/robot_name_control.launch"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    
    try: os.mkdir(save_dir + '/launch')
    except: pass     
    
    launch = Element('launch')

    controller_name = robot_name + '_controller'
    rosparam = SubElement(launch, 'rosparam')
    rosparam.attrib = {'file':'$(find fusion2urdf)/launch/' + controller_name + '.yaml',
                       'command':'load'}
                       
    controller_args_str = ' '.join([joint + '_position_controller' for joint in joints_dict]) \
                            + ' joint_state_controller'
    node_controller = SubElement(launch, 'node')
    node_controller.attrib = {'name':'controller_spawner', 'pkg':'controller_manager', 'type':'spawner',\
                    'respawn':'false', 'output':'screen', 'ns':robot_name,\
                    'args':'{}'.format(controller_args_str)}
    
    node_publisher = SubElement(launch, 'node')
    node_publisher.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher',\
                    'type':'robot_state_publisher', 'respawn':'false', 'output':'screen'}
    remap = SubElement(node_publisher, 'remap')
    remap.attrib = {'from':'/joint_states',\
                    'to':'/' + robot_name + '/joint_states'}
    
    launch_xml = "\n".join(prettify(launch).split("\n")[1:])        
    
    file_name = save_dir + '/launch/' + robot_name + '_control.launch'    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)
        

def write_yaml(robot_name, save_dir, joints_dict):
    """
    write yaml file "save_dir/launch/robot_name_controller.yaml"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass 

    controller_name = robot_name + '_controller'
    file_name = save_dir + '/launch/' + controller_name + '.yaml'
    with open(file_name, 'w') as f:
        f.write(controller_name + ':\n')
        # joint_state_controller
        f.write('  # Publish all joint states -----------------------------------\n')
        f.write('  joint_state_controller:\n')
        f.write('    type: joint_state_controller/JointStateController\n')  
        f.write('    publish_rate: 50\n\n')
        # position_controllers
        f.write('  # Position Controllers --------------------------------------\n')
        for joint in joints_dict:
            f.write('    ' + joint + '_position_controller:\n')
            f.write('      type: effort_controllers/JointPositionController\n')
            f.write('      joint: '+ joint + '\n')
            f.write('      pid: {p: 100.0, i: 0.01, d: 10.0}\n')


def run(context):
    ui = None
    success_msg = 'Successfully generated URDF file'
    msg = success_msg
    
    try:
        # --------------------
        # initialize
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
        
        # --------------------
        # set dictionaries
        
        # Generate joints_dict. All joints are related to root. 
        joints_dict = {}
        msg = set_joints_dict(root, joints_dict, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        print('joint_ok')    
        
        # Generate inertial_dict
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
        
        links_xyz_dict = {}
        
        # --------------------
        # Generate URDF
        write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, save_dir, robot_name)
        write_gazebo_launch(robot_name, save_dir)
        write_control_launch(robot_name, save_dir, joints_dict)
        write_yaml(robot_name, save_dir, joints_dict)
        
        # Generate STl files        
        copy_occs(root)
        export_stl(design, save_dir, components)   
        
        ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
