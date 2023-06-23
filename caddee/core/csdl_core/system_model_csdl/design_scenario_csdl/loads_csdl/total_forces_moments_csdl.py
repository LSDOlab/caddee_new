import csdl 
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_group import MechanicsGroup
from caddee.utils.base_model_csdl import BaseModelCSDL
import m3l


class TotalForcesMomentsM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.num_nodes = 1

    def compute(self, forces_names, moments_names):
        return TotalForcesMomentsCSDL(
            num_nodes=self.num_nodes,
            forces_names=forces_names,
            moments_names=moments_names, 
        )
    
    def evaluate(self, *args):
        forces_names = []
        moments_names = []
        arguments = dict()
        for arg in args:
            arg_name = arg.name
            arg_model_name = arg.operation.name
            if arg_name == 'F':
                forces_names.append(f"{arg_model_name}.{arg_name}")
                arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == 'M':
                moments_names.append(f"{arg_model_name}.{arg_name}")
                arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == 'F_inertial':
                moments_names.append(f"{arg_model_name}.{arg_name}")
                arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == 'M_inertial':
                moments_names.append(f"{arg_model_name}.{arg_name}")
                arguments[f"{arg_model_name}.{arg_name}"] = arg
            else:
                raise Exception(f"Inputs to total forces/moments model must be either 'F', 'M', 'F_inertial', or 'M_inertial'. Received {arg_name}")

        operation_csdl = self.compute(forces_names=forces_names, moments_names=moments_names)

        total_forces_moments_operation = m3l.CSDLOperation(name='total_forces_moments', arguments=arguments, operation_csdl=operation_csdl)
        total_forces = m3l.Variable(name='total_forces', shape=(self.num_nodes, 3), operation=total_forces_moments_operation)
        total_moments = m3l.Variable(name='total_moments', shape=(self.num_nodes, 3), operation=total_forces_moments_operation)

        return total_forces, total_moments



class TotalForcesMomentsCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('num_nodes')
        self.parameters.declare('forces_names')
        self.parameters.declare('moments_names')

    def define(self):
        num_nodes = self.parameters['num_nodes']
        forces_names = self.parameters['forces_names']
        moments_names = self.parameters['moments_names']

        F_total = self.register_module_input('F_total', val=0, shape=(num_nodes, 3))
        M_total = self.register_module_input('F_total', val=0, shape=(num_nodes, 3))


        for i in range(len(forces_names)):
                forces_name = forces_names[i]
                moments_name = moments_names[i]
                F_model = self.register_module_input(forces_name, shape=(num_nodes, 3), val=0)
                M_model = self.register_module_input(moments_name, shape=(num_nodes, 3), val=0)

                F_total = F_total + F_model
                M_total = M_total + M_model

        self.register_module_output('total_forces', F_total)
        self.register_module_output('total_moments', M_total)
