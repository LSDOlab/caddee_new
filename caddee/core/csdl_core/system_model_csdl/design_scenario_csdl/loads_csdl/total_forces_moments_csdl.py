import csdl 
import m3l


class TotalForcesMoments(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('num_nodes', types=int, default=1)
        self.parameters.declare('name', types=str, default='total_forces_moments_model')
        self.parameters.declare('nested_connection_name', types=bool, default=False)
        self.parameters.declare('not_nested_vars', types=list, default=['F_inertial', 'M_inertial'])
        self._stability_flag = False
        # super().initialize(kwargs=kwargs)

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self):
        if self._stability_flag:
            num_nodes = self.parameters['num_nodes'] * 13
        else:
            num_nodes = self.parameters['num_nodes']

        csdl_model = TotalForcesMomentsCSDL(
            num_nodes=num_nodes,
            forces_names=self.forces_names,
            moments_names=self.moments_names, 
        )

        return csdl_model
    
    def evaluate(self, *args, stability=False):

        if stability:
            self._stability_flag = stability
            self.parameters['nested_connection_name'] = True
            num_nodes = self.parameters['num_nodes'] * 13
        else:
            num_nodes = self.parameters['num_nodes']
        self.forces_names = []
        self.moments_names = []
        self.arguments = dict()
        for arg in args:
            arg_name = arg.name
            arg_model_name = arg.operation.name
            if arg_name == 'F':
                self.forces_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            
            elif arg_name == f'{arg.operation.name}.F':
                self.forces_names.append(f"{arg_name}")
                self.arguments[f"{arg_name}"] = arg
            
            elif arg_name == 'M':
                self.moments_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            
            elif arg_name == f'{arg.operation.name}.M':
                self.moments_names.append(f"{arg_name}")
                self.arguments[f"{arg_name}"] = arg
            
            elif arg_name == 'F_inertial':
                self.forces_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            
            elif arg_name == 'M_inertial':
                self.moments_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            else:
                raise Exception(f"Inputs to total forces/moments model must be either 'F', 'M', 'F_inertial', or 'M_inertial'. Received {arg_name}")


        total_forces = m3l.Variable(name='total_forces', shape=(num_nodes, 3), operation=self)
        total_moments = m3l.Variable(name='total_moments', shape=(num_nodes, 3), operation=self)

        return total_forces, total_moments



class TotalForcesMomentsCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes')
        self.parameters.declare('forces_names')
        self.parameters.declare('moments_names')
        self.parameters.declare('stability_flag', types=bool, default=False)

    def define(self):
        num_nodes = self.parameters['num_nodes']
        forces_names = self.parameters['forces_names']
        moments_names = self.parameters['moments_names']

        # print(forces_names)
        # print(moments_names)
        
        F_total = self.create_input('F_total', val=0, shape=(num_nodes, 3))
        M_total = self.create_input('M_total', val=0, shape=(num_nodes, 3))


        for i in range(len(forces_names)):
            forces_name = forces_names[i]
            F_model = self.declare_variable(forces_name, shape=(num_nodes, 3), val=0)
            F_total = F_total + F_model

        for j in range(len(moments_names)):
            moments_name = moments_names[j]
            M_model = self.declare_variable(moments_name, shape=(num_nodes, 3), val=0)
            M_total = M_total + M_model

        # 
        self.register_output('total_forces', F_total)
        self.register_output('total_moments', M_total)
