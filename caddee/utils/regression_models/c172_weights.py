##ex all
import m3l
import csdl
from dataclasses import dataclass


@dataclass
class MassProperties:
    """
    Simple container class for standard mass properties. 
    
    Solver developer should follow this naming convention:

    Parameters
    ----------
    mass : m3l.Variabl
        The mass of the vehicle
   
    cg : m3l.Variable
        The center of gravity vector (w.r.t the nose of the vehicle)
    
    inertia_tensor : m3l.Variable
        The full 3x3 inertia tensor (w.r.t the global reference frame)
    """
    mass : m3l.Variable = None
    cg_vector : m3l.Variable = None
    inertia_tensor : m3l.Variable = None




class C172MassProperties(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('name', types=str, default='C172_mass_properties')

    def compute(self):
        csdl_model = C172MassPropertiesCSDL()

        return csdl_model
    

    def evaluate(self) -> MassProperties:
        self.arguments = {}

        mass = m3l.Variable(name='mass', shape=(1, ), operation=self)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3, ), operation=self)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=self)

        mass_properties = MassProperties(
            mass=mass,
            cg_vector=cg_vector,
            inertia_tensor=inertia_tensor,
        )

        return mass_properties

    # def _assemble_csdl(self):
    #     csdl_model = C172MassPropertiesCSDL()
    #     return csdl_model



class C172MassPropertiesCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('name', default='C172MP', types=str)

    def define(self):
        shape = (1,)

        area = self.declare_variable('wing_area', shape=shape, units='m^2', val=210.)
        ar = self.declare_variable('wing_AR', shape=shape, val=13.)

        # Random junk computations. The value is specified
        m = 1043.2616 + (1.2 * area + 0.6 * ar) * 0
        Ixx = 1285.3154166 + (0.34343 * area + 2121 * ar) * 0
        Iyy = 1824.9309607 + (1.9 * area + 0.1 * ar) * 0
        Izz = 2666.89390765 + (1.7 * area + 0.8 * ar) * 0
        Ixz = 0. + (0.3 * area + 456 * ar) * 0

        cgx = 4.5 + (0.21 * area + 312312 * ar) * 0
        cgy = (0.2343 * area + 321 * ar) * 0
        cgz = 5. + (0.2212 * area + 454 * ar) * 0

        self.register_output(
            name='mass',
            var=m)        
        
        inertia_tensor = self.create_output('inertia_tensor', shape=(3, 3), val=0)
        inertia_tensor[0, 0] = csdl.reshape(Ixx, (1, 1))
        inertia_tensor[0, 2] = csdl.reshape(Ixz, (1, 1))
        inertia_tensor[1, 1] = csdl.reshape(Iyy, (1, 1))
        inertia_tensor[2, 0] = csdl.reshape(Ixz, (1, 1))
        inertia_tensor[2, 2] = csdl.reshape(Izz, (1, 1))
        
        cg_vector = self.create_output('cg_vector', shape=(3, ), val=0)
        cg_vector[0] = cgx * 0.3024
        cg_vector[1] = cgy * 0.3024
        cg_vector[2] = cgz * 0.3024



if __name__ == "__main__":
    from python_csdl_backend import Simulator
    c172_sizing_model = C172MassProperties()
    csdl_model = c172_sizing_model._assemble_csdl()
    sim = Simulator(csdl_model, analytics=True, display_scripts=True)
    sim.run()
    print(sim['mass'])
