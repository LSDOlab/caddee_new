import numpy as np

from lsdo_modules.module_csdl.module_csdl import ModuleCSDL

class SoCModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('battery_energy_density') # in W*h/kg
    
    def define(self):
        battery_energy_density = self.declare_variable('battery_energy_density', self.parameters['battery_energy_density'] * 3600) # in W*s/kg

        m_batt = self.register_module_input('battery_mass', shape=(1,)) # kg
        E_used = self.register_module_input('total_energy_consumption', shape=(1,)) # J

        E_available = m_batt * battery_energy_density
        SoC = (E_available-E_used)/E_available

        self.register_module_output('finalSoC', SoC)