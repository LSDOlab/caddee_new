from csdl import Model
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.core.caddee_core.system_model.system_model import SystemModel
from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation

from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization


class SystemModelCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('system_model', types=SystemModel)
        self.parameters.declare('system_representation', types=SystemRepresentation, allow_none=True)
        self.parameters.declare('system_param', default=None, types=SystemParameterization, allow_none=True)        


        # NOTE: previously we had pointers down below for containment relationships
        # However, these are not necessary on the pure csdl side        

    def define(self):
        system_model = self.parameters['system_model']
        system_config = self.parameters['system_representation']
        system_param = self.parameters['system_param']

        # Add m3l models 
        m3l_models = system_model.m3l_models

        if m3l_models:
            for m3l_model_name, m3l_model in m3l_models.items():
                csdl_model = m3l_model.assemble_csdl()
                self.add_module(csdl_model, m3l_model_name)

        # print(system_model.design_scenario_dictionary)
        # design scenario
        design_scenario_dictionary = system_model.design_scenario_dictionary
        for name, design_scenario in design_scenario_dictionary.items():
            if system_param:
                self.add_module(design_scenario._assemble_csdl(
                    system_config=system_config,
                    system_param=system_param,
                ), name, promotes=[])
            else:
                self.add_module(design_scenario._assemble_csdl(
                    system_config=system_config,
                    system_param=system_param,
                ), name, promotes=[])
    
