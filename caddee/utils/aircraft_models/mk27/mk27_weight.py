from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import m3l
import csdl
import numpy as np


class MK27MassProperties(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)

    def compute(self):
        csdl_model = MK27MassPropertiesCSDL()
        return csdl_model

    def evaluate(self, design_condition=None):
        self.arguments = {}
        if design_condition:
            self.name = f"{design_condition.parameters['name']}_mk27_weight"
        else:
            self.name = 'mk27_weight'

        mass = m3l.Variable(name='mass', shape=(1,), operation=self)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3,), operation=self)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=self)

        return mass, cg_vector, inertia_tensor


class MK27MassPropertiesCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('name', default='mk27_mass_properties', types=str)

    def define(self):
        ft2m = 0.3048
        lbs2kg = 0.453592

        total_mass = self.register_module_input('total_mass_input', val=92.*lbs2kg)
        self.register_module_output(name='mass', var=total_mass*1)

        inertia_tensor = self.register_module_input('inertia_tensor_input', shape=(3,3))
        self.register_module_output('inertia_tensor', inertia_tensor*1)

        cg = self.register_module_input('cg_vector_input', val=np.array([3., 0., 0.])*ft2m)
        self.register_module_output('cg_vector', cg*1)
        return
