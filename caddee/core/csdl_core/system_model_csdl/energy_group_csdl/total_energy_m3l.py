import numpy as np
import m3l
from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.total_energy_csdl import TotalEnergyModelCSDL

class TotalEnergyModelM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        pass

    def compute(self):
        model = TotalEnergyModelCSDL(
            argument_names=self.argument_names
        )
        return model

    def evaluate(self, *args) -> tuple:

        self.name = 'total_energy_consumption_model'
        self.arguments = {}
        self.argument_names = []
        for arg in args:
            self.arguments[arg.name] = arg
            self.argument_names.append(arg.name) 

        total_energy_consumption = m3l.Variable(
            'total_energy_consumption', 
            shape=(1,),
            operation=self
        )

        return total_energy_consumption