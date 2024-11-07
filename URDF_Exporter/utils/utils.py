# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""
import sys
import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
# from distutils.dir_util import copy_tree
# from shutil import copytree as copy_tree
import fileinput

import shutil
import os


def copy_tree(src, dst):
    # 如果目标目录不存在，创建目标目录
    if not os.path.exists(dst):
        os.makedirs(dst)

    # 遍历源目录中的文件和文件夹
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            # 递归复制子目录
            copy_tree(s, d)
        else:
            # 复制文件
            shutil.copy2(s, d)


# 使用示例
# copy_tree('source_folder', 'destination_folder')


def copy_occs(root):
    """
    duplicate all the components
    """
    def copy_body(allOccs, occs):
        """
        copy the old occs to new component
        """

        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()

        # Create new components from occs
        # This support even when a component has some occses.

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = re.sub('[ :()]', '_', occs.name)
        new_occs = allOccs.item((allOccs.count-1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)

    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'


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
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'
    # export the occurrence one by one in the component to a specified file
    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + 'has something wrong.')


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

def create_package(package_name, save_dir, package_dir):
    try: os.mkdir(save_dir + '/launch')
    except: pass

    try: os.mkdir(save_dir + '/urdf')
    except: pass

    try: os.mkdir(save_dir + '/config')
    except: pass

    try: os.mkdir(save_dir + '/' +package_name)
    except: pass
    with open(os.path.join(save_dir, package_name, '__init__.py'), 'w'):
        pass

    try: os.mkdir(save_dir + '/resource')
    except: pass
    with open(os.path.join(save_dir, 'resource', package_name), 'w'):
        pass

    try: os.mkdir(save_dir + '/test')
    except: pass

    copy_tree(package_dir, save_dir)

def update_setup_py(save_dir, package_name):
    file_name = save_dir + '/setup.py'

    for line in fileinput.input(file_name, inplace=True):
        if "package_name = 'fusion2urdf_ros2'" in line:
            sys.stdout.write("package_name = '" + package_name + "'\n")
        else:
            sys.stdout.write(line)

def update_setup_cfg(save_dir, package_name):
    file_name = save_dir + '/setup.cfg'

    for line in fileinput.input(file_name, inplace=True):
        if "script-dir" in line:
            sys.stdout.write("script-dir=$base/lib/" + package_name + "\n")
        elif "install-scripts" in line:
            sys.stdout.write("install-scripts=$base/lib/" + package_name + "\n")
        else:
            sys.stdout.write(line)

def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("<name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
