import numpy as np

import csdl

class EnergyModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('argument_names')

    def define(self):
        arg_names = self.parameters['argument_names']
        num_args = len(arg_names)

        t = self.declare_variable('time')

        power_per_comp = self.create_output('power_per_comp', shape=(num_args,))
        for i in range(num_args):
            power_per_comp[i] = self.declare_variable(arg_names[i], shape=(1,)) * 1.

        total_power = csdl.sum(power_per_comp)
        self.register_output(f'energy', t*total_power)