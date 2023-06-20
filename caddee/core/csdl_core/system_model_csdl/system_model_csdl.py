from csdl import Model
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.core.caddee_core.system_model.system_model import SystemModel
from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.constant_mass_properties_csdl import ConstantMassPropertiesCSDL

from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization


class SystemModelCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('system_model', types=SystemModel)
        self.parameters.declare('system_representation', types=SystemRepresentation, allow_none=False)
        self.parameters.declare('system_param', default=None, types=SystemParameterization, allow_none=True)

        # self.parameters.declare('meshes', default=None, allow_none=True)
        # establish a pattern where the pure python objects corresponding to 
        # csdl object are declared as parameters (including composition)
        
        # Additionally, here we require system_configuaration in order to 
        # have access to the connections between models 
        self.parameters.declare('psa_connections', default=None, types=list, allow_none=True)

        # NOTE: previously we had pointers down below for containment relationships
        # However, these are not necessary on the pure csdl side        

    def define(self):
        system_model = self.parameters['system_model']
        system_model_connections = system_model.connections_list
        psa_connections = self.parameters['psa_connections']
        system_config = self.parameters['system_representation']
        system_param = self.parameters['system_param']

        # sizing group & mass properties
        sizing_group = system_model.sizing_group
        if sizing_group:
            self.add_module(sizing_group._assemble_csdl(), 'sizing_group')
            if sizing_group.connections_list:
                self._connect(connections=sizing_group.connections_list)
            self.add_module(ConstantMassPropertiesCSDL(sizing_group=sizing_group), 'constant_mass_properties')
            self.connect_sizing_to_mass_properties(sizing_group=sizing_group)
            
        # design scenario
        design_scenario_dictionary = system_model.design_scenario_dictionary
        for name, design_scenario in design_scenario_dictionary.items():
            if psa_connections:
                design_scenario._psa_connections_list = psa_connections
            if system_model_connections:
                design_scenario._system_model_connections_list = system_model_connections
            if system_param:
                self.add_module(design_scenario._assemble_csdl_modules(
                    system_config=system_config,
                    sizing_group=sizing_group,
                    system_param=system_param,
                ), name, promotes=['system_representation_geometry'])
            else:
                self.add_module(design_scenario._assemble_csdl_modules(
                    system_config=system_config,
                    sizing_group=sizing_group,
                    system_param=system_param,
                ), name, promotes=[])
            if sizing_group:
                self.connect('constant_mass_properties.m_total_constant', f'{name}.total_mass_properties.m_total_constant')
                self.connect('constant_mass_properties.cgx_total_constant', f'{name}.total_mass_properties.cgx_total_constant')
                self.connect('constant_mass_properties.cgy_total_constant', f'{name}.total_mass_properties.cgy_total_constant')
                self.connect('constant_mass_properties.cgz_total_constant', f'{name}.total_mass_properties.cgz_total_constant')
                self.connect('constant_mass_properties.ixx_total_constant', f'{name}.total_mass_properties.ixx_total_constant')
                self.connect('constant_mass_properties.iyy_total_constant', f'{name}.total_mass_properties.iyy_total_constant')
                self.connect('constant_mass_properties.izz_total_constant', f'{name}.total_mass_properties.izz_total_constant')
                self.connect('constant_mass_properties.ixz_total_constant', f'{name}.total_mass_properties.ixz_total_constant')
        if system_model_connections:
            self._connect(system_model_connections)
          
    
