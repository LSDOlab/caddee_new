import m3l
import csdl
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.simple_battery_sizing_csdl import SimpleBatterySizingCSDL
from caddee.utils.helper_classes import MassProperties


class SimpleBatterySizing(m3l.ExplicitOperation):
    def initialize(self, kwargs): 
        self.parameters.declare('name', types=str)
    
    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self) -> csdl.Model:
        return SimpleBatterySizingCSDL()
    
    def evaluate(self, battery_mass : m3l.Variable, battery_position : m3l.Variable, battery_energy_density : m3l.Variable) -> MassProperties:        
        self.arguments = {}

        self.arguments['battery_mass'] = battery_mass
        self.arguments['battery_position'] = battery_position
        self.arguments['battery_energy_density'] = battery_energy_density


        mass = m3l.Variable(name='mass', shape=(1, ), operation=self)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3, ), operation=self)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=self)

        outputs = MassProperties(
            mass=mass,
            cg_vector=cg_vector,
            inertia_tensor=inertia_tensor,
        )

        return outputs