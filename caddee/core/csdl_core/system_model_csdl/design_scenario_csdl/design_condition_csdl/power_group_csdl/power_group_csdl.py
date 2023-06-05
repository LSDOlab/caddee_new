from csdl import Model 
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.caddee_core.system_model.design_scenario.design_condition.power_group.power_group import PowerGroup


class PowerGroupCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('power_group', types=PowerGroup)

    def define(self):
        power_group = self.parameters['power_group']

        power_model_names_list = power_group._all_models_names_list
        power_models_list = power_group._all_models_list
        power_models_num_nodes = power_group._num_nodes

        counter = 0
        for pwr_model in power_models_list:
            csdl_model_name = power_model_names_list[counter]
            if hasattr(pwr_model, 'num_nodes') is False:
                raise Exception(f"Power model '{csdl_model_name}' does not have a 'num_nodes' attribute. All solvers integrated into CADDEE need a 'num_nodes' attribute.")
            else:
                pwr_model.num_nodes = power_models_num_nodes
            csdl_model = pwr_model._assemble_csdl()
            counter += 1
            self.add_module(csdl_model, csdl_model_name, promotes=[])