import numpy as np
import m3l
from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.total_energy_csdl import TotalEnergyModelCSDL

class TotalEnergyModelM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str)

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self):
        model = TotalEnergyModelCSDL(
            argument_names=self.argument_names
        )
        return model

    def evaluate(self, *args) -> tuple:

        self.arguments = {}
        self.argument_names = []
        for i, arg in enumerate(args):
            self.arguments[f'energy_{i}'] = arg
            self.argument_names.append(f'energy_{i}') 

        total_energy_consumption = m3l.Variable(
            name='total_energy_consumption', 
            shape=(1,),
            operation=self
        )

        return total_energy_consumption