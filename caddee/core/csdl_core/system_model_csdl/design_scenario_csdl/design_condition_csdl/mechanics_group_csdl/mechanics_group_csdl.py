from csdl import Model 
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_group import MechanicsGroup


class MechanicsGroupCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('mechanics_group', types=MechanicsGroup)

    def define(self):
        mech_group = self.parameters['mechanics_group']

        mech_model_names_list = mech_group._all_models_names_list
        mech_models_list = mech_group._all_models_list
        mech_models_num_nodes = mech_group._num_nodes
        


        counter = 0
        for mech_model in mech_models_list:
            csdl_model_name = mech_model_names_list[counter]
            if hasattr(mech_model, 'num_nodes') is False:
                raise Exception(f"Mechanics model '{csdl_model_name}' does not have a 'num_nodes' attribute. All solvers integrated into CADDEE need a 'num_nodes' attribute.")
            else:
                mech_model.num_nodes = mech_models_num_nodes
            #print('mech_model.num_nodes', mech_model.num_nodes)
            csdl_model = mech_model._assemble_csdl()
            counter += 1
            self.add_module(csdl_model, csdl_model_name, promotes=[])
            # print(csdl_model.module_declared_vars)

        # exit()
        a = self.declare_variable(name='a', val=1.)
        b = a * 5.
        self.register_output(name='b', var=b)
        # self.add_csdl_models(mech_models_list, mech_model_names_list, mech_models_num_nodes)
