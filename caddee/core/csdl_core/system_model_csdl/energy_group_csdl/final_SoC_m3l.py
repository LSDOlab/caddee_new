import numpy as np
import m3l

from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.final_SoC_csdl import SoCModelCSDL

class SOCModelM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('battery_energy_density')
        self.parameters.declare('name', types=str)
        self.mission_multiplier = 1.

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self):
        model = SoCModelCSDL(
            battery_energy_density=self.battery_energy_density,
            mission_multiplier=self.mission_multiplier,
        )
        return model

    def evaluate(self,
                 battery_mass: m3l.Variable = None,
                 total_energy_consumption: m3l.Variable=None, mission_multiplier=1.) -> m3l.Variable:
        self.battery_energy_density = self.parameters['battery_energy_density']

        self.arguments = {}
        self.arguments['battery_mass'] = battery_mass
        self.arguments['total_energy_consumption'] = total_energy_consumption
        self.mission_multiplier = mission_multiplier
        finalSOC = m3l.Variable(
            name='finalSoC',
            shape=(1,),
            operation=self
        )

        return finalSOC