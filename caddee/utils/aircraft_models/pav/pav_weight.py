from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import m3l
import csdl


class PavMassProperties(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)

    def compute(self):
        csdl_model = PavMassPropertiesCSDL()
        return csdl_model

    def evaluate(self, design_condition=None):
        self.arguments = {}
        if design_condition:
            self.name = f"{design_condition.parameters['name']}_pav_weight"
        else:
            self.name = 'pav_weight'

        mass = m3l.Variable(name='mass', shape=(1,), operation=self)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3,), operation=self)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=self)

        return mass, cg_vector, inertia_tensor


class PavMassPropertiesCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('name', default='PavMP', types=str)

    def define(self):
        shape = (1,)

        ft2m = 0.3048
        lbs2kg = 0.453592

        ar = self.register_module_input('wing_AR', shape=shape, val=7.25751)

        # Random junk computations. The value is specified
        m = 1764.*lbs2kg + (1.2 * ar) * 0
        Ixx = 1000 + (0.34343 * ar) * 0
        Iyy = 2000 + (1.9 * ar) * 0
        Izz = 3000 + (1.7 * ar) * 0
        Ixz = 0. + (0.3 * ar) * 0

        cgx = 10.619*ft2m + (0.21 * ar) * 0
        cgy = 0. + (0.2343 * ar) * 0
        cgz = 0.706*ft2m + (0.2212 * ar) * 0

        self.register_module_output(
            name='mass',
            var=m)

        inertia_tensor = self.register_module_output('inertia_tensor', shape=(3, 3), val=0)
        inertia_tensor[0, 0] = csdl.reshape(Ixx, (1, 1))
        inertia_tensor[0, 2] = csdl.reshape(Ixz, (1, 1))
        inertia_tensor[1, 1] = csdl.reshape(Iyy, (1, 1))
        inertia_tensor[2, 0] = csdl.reshape(Ixz, (1, 1))
        inertia_tensor[2, 2] = csdl.reshape(Izz, (1, 1))

        cg_vector = self.register_module_output('cg_vector', shape=(3,), val=0)
        cg_vector[0] = cgx
        cg_vector[1] = cgy
        cg_vector[2] = cgz
        return
