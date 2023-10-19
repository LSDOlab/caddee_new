import csdl


class StabilityAdapterModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('arguments', types=dict)
        self.parameters.declare('num_nodes', types=int)
        self.parameters.declare('neglect', types=list, default=[])
    
    def define(self):
        args = self.parameters['arguments']
        num_nodes = self.parameters['num_nodes']
        ac_states = ['u', 'v', 'w', 'p', 'q', 'r', 'theta', 'phi', 'psi', 'x', 'y', 'z']
        special_cases = self.parameters['neglect']
        ['chord_profile', 'chord_cp', 'twist_profile','twist_cp', 'propeller_radius']
        for key, value in args.items():
            if key in ac_states:
                csdl_var = self.declare_variable(key, shape=(num_nodes * 9, ))
                self.register_output(name=f'{key}_exp', var=csdl_var * 1)
            elif key in special_cases:
                csdl_var = self.declare_variable(key, shape=value.shape)
                self.register_output(name=f'{key}_exp', var=csdl_var * 1)
            else:
                csdl_var = self.declare_variable(key, shape=value.shape)
                if len(value.shape) == 1 and value.shape[0] == 1:
                    # print(key, value.shape)
                    csdl_var_exp = csdl.expand(csdl_var, shape=(num_nodes * 9, ))
                    self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) == 1 and value.shape[0] != 1:
                    # print(key, (9, ) + value.shape)
                    csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(9, ) + value.shape, indices='i->ji'), new_shape=(9, value.shape[0]))
                    self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) == 2:
                    if num_nodes == value.shape[0]:
                        csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(9, ) + value.shape, indices='ij->kij'), new_shape=(9*num_nodes, value.shape[1]))
                        self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                    elif num_nodes == value.shape[1]:
                        csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(9, ) + value.shape, indices='ij->kij'), new_shape=(9*num_nodes, value.shape[0]))
                        self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) > 2:
                    raise NotImplementedError
                

def make_csdl_model(solver_csdl_model, stability_flag=False, neglect=[]):
    if stability_flag==False:
        return solver_csdl_model
    
    else:
        csdl_model = StabilityAdapterModelCSDL(

        )