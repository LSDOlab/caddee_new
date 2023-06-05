from csdl import Model 
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.caddee_core.system_model.sizing_group.sizing_group import SizingGroup


class SizingGroupCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('sizing_group', types=SizingGroup)

    def define(self):
        sizing_group = self.parameters['sizing_group']

        sizing_models_dict = sizing_group.models_dictionary

        for sizing_module_name, sizing_module in sizing_models_dict.items():
            self.add_module(sizing_module._assemble_csdl(), sizing_module_name, promotes=[])

        # self.add_csdl_sizing_models(sizing_models_dict)
