#Author-syuntoku14

import adsk.core, adsk.fusion, traceback

# Maybe copying and pasting is banned 
# Root component name can not be changed but others can.

def copy_body(allOccs, old_comp):
    bodies = old_comp.bRepBodies
    transform = adsk.core.Matrix3D.create()    
    occs = allOccs.addNewComponent(transform)  # this create new occs
    old_comp.name, occs.component.name = occs.component.name, old_comp.name  #swap the name
    occs = allOccs[-1]
    for i in range(bodies.count):
        body = bodies.item(i)
        body.copyToComponent(occs)
    

def copy_component(components, allOccs, root):    
    # duplicate all the components
    coppied = []
    for component in components:
        name = component.name
        if component == root or ('Component' in name) or (name in coppied):
            continue
        if component.bRepBodies.count > 0:
            copy_body(allOccs, component)
            coppied.append(name)
    #print(coppied)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # origin
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        root = design.rootComponent
        allOccs = root.occurrences
        components = design.allComponents
        
        copy_component(components, allOccs, root)
        #delete_old_component(root, components)  # I couldn't succeed to delete needless components.  
        
    except:
        if ui:

            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            

#
#def delete_old_component(allOccs, components):
#    componentsToDelete_name = []
#    for component in components:   
#        if 'Component' in component.name:
#            print(component.name)            
#            componentsToDelete_name.append(component.name)
#    
#    for name in componentsToDelete_name:
#        for occs in allOccs:
#            if name in occs.component.name:
#                occs.deleteMe()
#        