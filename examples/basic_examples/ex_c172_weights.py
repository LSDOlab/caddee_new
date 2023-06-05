from caddee.caddee_core.system_model.sizing_group.sizing_models.sizing_model import SizingModel
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL

class C172MassProperties(SizingModel):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)

    def _assemble_csdl(self):
        csdl_model = C172MassPropertiesCSDL()
        return csdl_model



class C172MassPropertiesCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('name', default='C172MP', types=str)

    def define(self):
        shape = (1,)

        area = self.register_module_input('wing_area', shape=shape, units='m^2', val=210.)
        ar = self.register_module_input('wing_AR', shape=shape, val=13.)

        # Random junk computations. The value is specified
        m = 1043.2616 + (1.2 * area + 0.6 * ar) * 0
        Ixx = 1285.3154166 + (0.34343 * area + 2121 * ar) * 0
        Iyy = 1824.9309607 + (1.9 * area + 0.1 * ar) * 0
        Izz = 2666.89390765 + (1.7 * area + 0.8 * ar) * 0
        Ixz = 0. + (0.3 * area + 456 * ar) * 0

        cgx = 4.5 + (0.21 * area + 312312 * ar) * 0
        cgy = (0.2343 * area + 321 * ar) * 0
        cgz = 5. + (0.2212 * area + 454 * ar) * 0

        self.register_module_output(
            name='mass',
            var=m)
        self.register_module_output(
            name='cgx',
            var=cgx)
        self.register_module_output(
            name='cgy',
            var=cgy)
        self.register_module_output(
            name='cgz',
            var=cgz)
        self.register_module_output(
            name='ixx',
            var=Ixx)
        self.register_module_output(
            name='iyy',
            var=Iyy)
        self.register_module_output(
            name='izz',
            var=Izz)
        self.register_module_output(
            name='ixz',
            var=Ixz)
        return


if __name__ == "__main__":
    from python_csdl_backend import Simulator
    c172_sizing_model = C172MassProperties()
    csdl_model = c172_sizing_model._assemble_csdl()
    sim = Simulator(csdl_model, analytics=True, display_scripts=True)
    sim.run()
    print(sim['mass'])
