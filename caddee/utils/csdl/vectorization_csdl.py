from caddee.utils.base_model_csdl import BaseModelCSDL
from csdl import GraphRepresentation


class VectorizationCSDL(BaseModelCSDL):
    def initialize(self): 
        self.parameters.declare('mechanics_group')
        self.parameters.declare('nonmechanics_group')
        self.parameters.declare('power_group')

    def define(self):
        mechanics_group = self.parameters['mechanics_group']
        mech_modules = mechanics_group._all_models_list
        mech_modules_names = mechanics_group._all_models_names_list
        df = mechanics_group._condition_solver_frame
        num_nodes = df.shape[1]
        conditions_list = df.columns.tolist()
        vectorized_vars = []
        counter = 0
        for mech_module in mech_modules:
            module_name =  mech_modules_names[counter]  
            selection_list = df.loc[[module_name]].values[0]
            mesh = mech_module.parameters['mesh']

            if mech_module.parameters.__contains__('component'):
                comp = mech_module.parameters['component']
                if comp:
                    comp_name = comp.parameters['name']
                    comp_vars = comp.parameters['component_vars']
            else:
                comp_vars = []
            mech_module.model_selection = selection_list
            mech_module_csdl = mech_module._assemble_csdl()
            # print('\n')
            # print('module', mech_module_csdl)
            # print(module_name)
            GraphRepresentation(mech_module_csdl)
            module_vars = {**mech_module_csdl.module_inputs, **mech_module_csdl.module_declared_vars}
            # print('\n')
            # print(module_name)
            # print(module_vars)
            for var_name, var_info in module_vars.items():
                if var_info['vectorized'] is True:     
                    if var_name not in vectorized_vars:
                        if var_name in comp_vars:
                            name = f'{var_name}_{comp_name}'
                            vectorized_vars.append(name)
                            vect_var = self.create_output(name=name, shape=(num_nodes, 1), val=0)
                            for i in range(num_nodes):
                                if selection_list[i] == 1:
                                    vect_var[i] = self.declare_variable(f'{conditions_list[i]}_{name}')
                        else:
                            vectorized_vars.append(var_name)
                            # print('var_name-----------', var_name)
                            vect_var = self.create_output(name=var_name, shape=(num_nodes, 1), val=0)
                            for i in range(num_nodes):
                                vect_var[i] = self.declare_variable(f'{conditions_list[i]}_{var_name}')
                    else:
                        pass
                else:
                    pass
            counter += 1

        ac_states = ['u', 'v', 'w', 'p', 'q', 'phi', 'theta', 'psi', 'r', 'x', 'y', 'z']
        missing_states = [state for state in ac_states if state not in vectorized_vars]
        if missing_states:
            for state in missing_states:
                vect_var = self.create_output(name=state, shape=(num_nodes, 1), val=0)
                for i in range(num_nodes):
                    vect_var[i] = self.declare_variable(f'{conditions_list[i]}_{state}')



        
        # self.vectorize_inputs(mechanics_group)
        # self.vectorize_inputs(power_group)