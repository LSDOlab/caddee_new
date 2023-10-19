import numpy as np
import csdl


class SoCModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('battery_energy_density') # in W*h/kg
        self.parameters.declare('mission_multiplier', default=1.)

    def define(self):
        battery_energy_density = self.declare_variable('battery_energy_density', self.parameters['battery_energy_density'] * 3600) # in W*s/kg
        mission_multiplier = self.parameters['mission_multiplier']

        m_batt = self.declare_variable('battery_mass', shape=(1,)) # kg
        E_used = self.declare_variable('total_energy_consumption', shape=(1,)) * mission_multiplier # J

        E_available = m_batt * battery_energy_density
        SoC = (E_available-E_used)/E_available

        self.register_output('finalSoC', SoC)
        self.print_var(SoC)