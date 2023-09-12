from dataclasses import make_dataclass, dataclass
import caddee.api as cd
import re


def process_string(input_string):
    # Remove special characters (except underscores) and replace capital letters with lowercase
    cleaned_string = re.sub(r'[^a-zA-Z0-9_]', '', input_string).lower()
    
    # Convert CamelCase to snake_case
    snake_case_string = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', cleaned_string)
    
    return snake_case_string


class Comps:
    def __init__(self, system_rep : cd.SystemRepresentation, **kwargs) -> None:
        spatial_rep = system_rep.spatial_representation
        
        for key, value in kwargs.items():
            
            comp_name = key
            primitive_names = list(spatial_rep.get_geometry_primitives(search_names=[value]).keys())
            component = cd.Component(name=comp_name,  spatial_representation=spatial_rep, primitive_names=primitive_names)
            
            system_rep.add_component(component=component)
            
            setattr(self, key, component)
        



def add_components(vsp_component_names : list, system_representation : cd.SystemRepresentation):
    """
    Method to add components to the system representation.

    parameters:
    -------------
        vsp_component_names :  list of strings with the names of 
            the components specified in the OpenVSP file
        
        system_representation : instantiated SystemRepresentation object
    
    returns:
    -------------
        data class containing all components that can be used for 
        projections or FFD
    """

    spatial_rep = system_representation.spatial_representation
    # TODO: check whether vsp_component_names actually exist
    fields = []

    @dataclass
    class Comps:
        # Loop over OpenVSP component names
        for comp_name in vsp_component_names:
            primitive_names = list(spatial_rep.get_geometry_primitives(search_names=[comp_name]).keys())
            component = cd.Component(name=comp_name,  spatial_representation=spatial_rep, primitive_names=primitive_names)
            
            system_representation.add_component(component=component)
            
            # append components to fields list (for data class)
            mod_comp_name = process_string(comp_name)
            fields.append(
                (mod_comp_name, cd.Component, component)
            )

    comp_data_class = make_dataclass(
        cls_name='comps_data_class',
        fields=fields,
    )

    return comp_data_class()


if __name__ == '__main__':
    print(process_string('EngineGroup_10'))

    from caddee import IMPORTS_FILES_FOLDER

    lpc_rep = cd.SystemRepresentation()
    lpc_param = cd.SystemParameterization(system_representation=lpc_rep)

    file_name = IMPORTS_FILES_FOLDER / 'LPC_final_custom_blades.stp'
    spatial_rep = lpc_rep.spatial_representation
    spatial_rep.import_file(file_name=file_name)
    spatial_rep.refit_geometry(file_name=file_name)

    comps = Comps(
        system_rep=lpc_rep,
        fuselage='Fuselage_***.main',
        nose_weight='EngineGroup_10',
        wing='Wing',
        htail='Tail_1',
        vtail='Tail_2',
    )

    print(comps.fuselage.parameters['name'])



    # vsp_comp_names = [
    #     'Fuselage_***.main',
    #     'EngineGroup_10',
    #     'Wing',
    #     'Tail_1',
    #     'Tail_2',
    #     'Rotor-9-disk',
    #     'Rotor_9_blades, 0',
    #     'Rotor_9_blades, 1',
    #     'Rotor_9_blades, 2',
    #     'Rotor_9_blades, 3',
    #     'Rotor_9_Hub',
    #     'Rotor_2_disk',

    # ]

    # comps = add_components(vsp_component_names=vsp_comp_names, system_representation=lpc_rep)
    # comps.

    
