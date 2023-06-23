from csdl import Model
import numpy as np
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import csdl


class SimpleBatterySizingCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('name', default='simple_battery_sizing', types=str)
        self.parameters.declare('battery_packaging_fraction', default=0.1, types=float)
        # Assume battery packaging is ~10% 
    
    def define(self):
        shape = (1, )
        battery_packaging_frac = self.parameters['battery_packaging_fraction']
        
        batt_mass = self.register_module_input('battery_mass', shape=shape, val=800, computed_upstream=False)
        battery_position = self.register_module_input('battery_position', shape=(3, ), val=np.array([3., 0, 0.5]), computed_upstream=False)
        batt_energy_dens = self.register_module_input('battery_energy_density', shape=shape, val=400, computed_upstream=False)
        
        x = battery_position[0]
        y = battery_position[1]
        z = battery_position[2]
        
        self.register_module_output('mass', batt_mass * (1 + battery_packaging_frac))
        self.register_module_output('cg_vector', battery_position * 1)

        ixx = batt_mass * (y**2 + z**2)
        ixy = -batt_mass * (x * y)
        ixz = -batt_mass * (x * z)
        iyx = ixy * 1
        iyy = batt_mass * (x**2 + z**2)
        iyz = -batt_mass * (y * z)
        izx = ixz * 1
        izy = iyz * 1
        izz = batt_mass * (x**2  + y**2)

        inertia_tensor = self.register_module_output('inertia_tensor', shape=(3, 3), val=0)
        inertia_tensor[0, 0] = csdl.reshape(ixx, (1, 1))
        inertia_tensor[0, 1] = csdl.reshape(ixy, (1, 1))
        inertia_tensor[0, 2] = csdl.reshape(ixz, (1, 1))
        inertia_tensor[1, 0] = csdl.reshape(iyx, (1, 1))
        inertia_tensor[1, 1] = csdl.reshape(iyy, (1, 1))
        inertia_tensor[1, 2] = csdl.reshape(iyz, (1, 1))
        inertia_tensor[2, 0] = csdl.reshape(izx, (1, 1))
        inertia_tensor[2, 1] = csdl.reshape(izy, (1, 1))
        inertia_tensor[2, 2] = csdl.reshape(izz, (1, 1))

        
        
        self.register_module_output('total_energy', batt_energy_dens * 3600 * batt_mass)

        self.register_module_output('battery_pack_mass', batt_mass * (1 + battery_packaging_frac))
        
        self.register_module_output('cgx', battery_position[0] * 1)
        self.register_module_output('cgy', battery_position[1] * 1)
        self.register_module_output('cgz', battery_position[2] * 1)
        self.create_input('ixx', val=0)
        self.create_input('iyy', val=0)
        self.create_input('izz', val=0)
        self.create_input('ixz', val=0)