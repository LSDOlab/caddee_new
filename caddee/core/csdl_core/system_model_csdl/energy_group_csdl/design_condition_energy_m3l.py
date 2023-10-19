import numpy as np
import m3l

from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.design_condition_energy_csdl import EnergyModelCSDL

class EnergyModelM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        pass

    def compute(self):
        model = EnergyModelCSDL(
            design_condition_name=self.dc_name,
            argument_names=self.arg_names
        )
        return model

    def evaluate(self, *args, ac_states=None, design_condition=None) -> tuple:

        self.arguments = {}
        self.arg_names = []
        for i, arg in enumerate(args):
            self.arguments[arg.name + f'_{i+1}'] = arg
            self.arg_names.append(arg.name + f'_{i+1}')

        self.arguments['time'] = ac_states['time']
        # exit()

        if design_condition is None:
            raise ValueError('Need to provide design condition')
        self.dc_name = design_condition.parameters['name']
        self.name = f'{self.dc_name}_energy_model'

        design_condition_energy = m3l.Variable(
            f'{self.dc_name}_energy',
            shape=(1,),
            operation=self
        )

        return design_condition_energy