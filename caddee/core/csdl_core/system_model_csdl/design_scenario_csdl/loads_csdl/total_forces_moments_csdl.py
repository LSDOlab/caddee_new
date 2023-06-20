import csdl 
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_group import MechanicsGroup
from caddee.utils.base_model_csdl import BaseModelCSDL


class TotalForcesMomentsCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('mech_group', types=MechanicsGroup)
        self.parameters.declare('num_nodes')

    def define(self):
        mech_group = self.parameters['mech_group']
        num_nodes = self.parameters['num_nodes']

        models = mech_group._all_models_list
        model_names = mech_group._all_models_names_list

        Fx_total = self.register_module_input('Fx', val=0, shape=(num_nodes, 1)) 
        Fy_total = self.register_module_input('Fy', val=0, shape=(num_nodes, 1)) * 0
        Fz_total = self.register_module_input('Fz', val=0, shape=(num_nodes, 1)) * 0
        Mx_total = self.register_module_input('Mx', val=0, shape=(num_nodes, 1)) * 0
        My_total = self.register_module_input('My', val=0, shape=(num_nodes, 1)) * 0
        Mz_total = self.register_module_input('Mz', val=0, shape=(num_nodes, 1)) * 0

        for model_name in model_names:
            #if model_name == 'wing_linear_beam':
            #    pass
            #else:

                F_model = self.register_module_input(f'{model_name}_F', shape=(num_nodes, 3),val=0)
                M_model = self.register_module_input(f'{model_name}_M', shape=(num_nodes, 3),val=0)

                Fx_model = F_model[:, 0]
                Fy_model = F_model[:, 1]
                Fz_model = F_model[:, 2]

                Mx_model = M_model[:, 0]
                My_model = M_model[:, 1]
                Mz_model = M_model[:, 2]

                Fx_total = Fx_total + Fx_model
                Fy_total = Fy_total + Fy_model
                Fz_total = Fz_total + Fz_model
                
                Mx_total = Mx_total + Mx_model
                My_total = My_total + My_model
                Mz_total = Mz_total + Mz_model

        F_inertial = self.register_module_input('F_inertial', shape=(num_nodes, 3))
        M_inertial = self.register_module_input('M_inertial', shape=(num_nodes, 3))

        Fx_inertial = F_inertial[:, 0]
        Fy_inertial = F_inertial[:, 1]
        Fz_inertial = F_inertial[:, 2]

        Mx_inertial = M_inertial[:, 0]
        My_inertial = M_inertial[:, 1]
        Mz_inertial = M_inertial[:, 2]

        Fx_total = Fx_total + Fx_inertial
        Fy_total = Fy_total + Fy_inertial
        Fz_total = Fz_total + Fz_inertial
        
        Mx_total = Mx_total + Mx_inertial
        My_total = My_total + My_inertial
        Mz_total = Mz_total + Mz_inertial

        self.register_module_output('Fx_total', Fx_total)
        self.register_module_output('Fy_total', Fy_total)
        self.register_module_output('Fz_total', Fz_total)
        self.register_module_output('Mx_total', Mx_total)
        self.register_module_output('My_total', My_total)
        self.register_module_output('Mz_total', Mz_total)
