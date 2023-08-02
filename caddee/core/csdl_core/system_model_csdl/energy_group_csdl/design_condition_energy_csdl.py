import numpy as np
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import csdl

class EnergyModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('design_condition_name')
        self.parameters.declare('argument_names')

    def define(self):
        dc_name = self.parameters['design_condition_name']
        arg_names = self.parameters['argument_names']
        num_args = len(arg_names)

        t = self.register_module_input('time')

        power_per_comp = self.register_module_output('power_per_comp', shape=(num_args,))
        for i in range(num_args):
            power_per_comp[i] = self.register_module_input(arg_names[i], shape=(1,)) * 1.

        total_power = csdl.sum(power_per_comp)
        self.register_module_output(f'{dc_name}_energy', t*total_power)