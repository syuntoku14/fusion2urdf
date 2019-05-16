# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom


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
                new_occs.component.name = re.sub('[ :()]', '_', occs.name)
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
                # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
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

