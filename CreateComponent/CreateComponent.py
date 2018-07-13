import adsk.core, adsk.fusion, traceback
from copy import deepcopy
# Maybe copying and pasting is banned 
# Root component name can not be changed but others can.

def copy_component(allOccs, old_comp):
    bodies = old_comp.bRepBodies
    transform = adsk.core.Matrix3D.create()    
    occs = allOccs.addNewComponent(transform)  # this create new occs
    print(occs.name, old_comp.name)
    old_comp.name, occs.component.name = occs.component.name, old_comp.name  #swap the name
    
    occs = allOccs[-1]
    for i in range(bodies.count):
        body = bodies.item(i)
        body.copyToComponent(occs)
    
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
        
        # duplicate all the components
        coppied = []
        for component in components:
            name = component.name
            if component == root or ('Component' in name) or (name in coppied):
                continue
            if component.bRepBodies.count > 0:
                copy_component(allOccs, component)
                coppied.append(name)
        print(coppied)
    except:
        if ui:

            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))