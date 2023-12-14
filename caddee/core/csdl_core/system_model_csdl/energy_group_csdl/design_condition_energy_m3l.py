import numpy as np
import m3l

from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.design_condition_energy_csdl import EnergyModelCSDL

class EnergyModelM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str)

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self):
        model = EnergyModelCSDL(
            argument_names=self.arg_names
        )
        return model

    def evaluate(self, motor_outputs, ac_states) -> tuple:

        self.arguments = {}
        self.arg_names = []
        for i, arg in enumerate(motor_outputs):
            self.arguments[f'power_{i}'] = arg.input_power
            self.arg_names.append(f'power_{i}')

        self.arguments['time'] = ac_states.time
        
        design_condition_energy = m3l.Variable(
            name=f'energy',
            shape=(1,),
            operation=self
        )

        return design_condition_energy