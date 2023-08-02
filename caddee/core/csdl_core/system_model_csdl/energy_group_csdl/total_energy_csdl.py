import numpy as np

from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import csdl

class TotalEnergyModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('argument_names')

    def define(self):
        arg_names = self.parameters['argument_names']

        energy_per_segment = self.register_module_output('energy_per_segment', shape=(len(arg_names,)))
        for i, name in enumerate(arg_names):
            energy_per_segment[i] = self.register_module_input(name, shape=(1,)) * 1.

        self.register_module_output('total_energy_consumption', csdl.sum(energy_per_segment))