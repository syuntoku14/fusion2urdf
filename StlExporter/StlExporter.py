#Author-syuntoku14
#Description-Export all components as stl format

import adsk.core, adsk.fusion, traceback
import os


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
            if 'Component' in component.name:
                continue
            
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
    success_msg = 'Successfully Exported Stl files'
    msg = success_msg
    
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'StlExporter'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return
        
        components = design.allComponents        
        root = design.rootComponent
        robot_name = root.name.split()[0]
        save_dir = file_dialog(ui) + '/' + robot_name
        if save_dir == False:
            ui.messageBox('StlExporter was canceled', title)
            return 0

        # Generate STl files
        export_stl(design, save_dir, components)        
        
        ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))