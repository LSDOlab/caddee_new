import numpy as np


import csdl

class TotalEnergyModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('argument_names')

    def define(self):
        arg_names = self.parameters['argument_names']
        energy_per_segment = self.create_output('energy_per_segment', shape=(len(arg_names,)))
        for i, name in enumerate(arg_names):
            energy_per_segment[i] = self.declare_variable(name, shape=(1,)) * 1.
        
        self.print_var(energy_per_segment)

        total_energy = csdl.sum(energy_per_segment)
        self.register_output('total_energy_consumption', total_energy)