from csdl import Model
import numpy as np
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL


class SimpleBatterySizingCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('name', default='simple_battery_sizing', types=str)
        self.parameters.declare('battery_packaging_fraction', default=0.1, types=float)
        # Assume battery packaging is ~10% 
    
    def define(self):
        shape = (1, )
        battery_packaging_frac = self.parameters['battery_packaging_fraction']
        
        batt_mass = self.register_module_input('battery_mass', shape=shape, val=800)
        batt_energy_dens = self.register_module_input('battery_energy_density', shape=shape, val=400)
        
        battery_position = self.register_module_input('battery_position', shape=(3, ), val=np.array([3., 0, 0.5]))

        # self.register_module_output('battery_pack_mass', batt_mass * (1 + battery_packaging_frac)) 
        self.register_module_output('total_energy', batt_energy_dens * 3600 * batt_mass)

        self.register_module_output('battery_pack_mass', batt_mass * (1 + battery_packaging_frac))
        self.register_module_output('mass', batt_mass * (1 + battery_packaging_frac))
        self.register_module_output('cgx', battery_position[0] * 1)
        self.register_module_output('cgy', battery_position[1] * 1)
        self.register_module_output('cgz', battery_position[2] * 1)
        self.create_input('ixx', val=0)
        self.create_input('iyy', val=0)
        self.create_input('izz', val=0)
        self.create_input('ixz', val=0)