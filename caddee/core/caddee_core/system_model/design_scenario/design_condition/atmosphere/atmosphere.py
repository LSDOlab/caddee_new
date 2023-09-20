import m3l
from dataclasses import dataclass


@dataclass
class AtmosphericProperties:
    """
    Container data class for atmospheric variables 
    """
    density : m3l.Variable = None
    temperature : m3l.Variable = None
    pressure : m3l.Variable = None
    dynamic_viscosity : m3l.Variable = None
    speed_of_sound : m3l.Variable = None


class Atmosphere(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.num_nodes = 1
        super().initialize(kwargs=kwargs)

    def compute(self):
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.atmosphere_csdl.atmosphere_csdl import SimpleAtmosphereCSDL
        csdl_model = SimpleAtmosphereCSDL(
            atmosphere_model=self,
        )

        return csdl_model

    def evaluate(self, altitude : m3l.Variable) -> AtmosphericProperties:
        """
        Returns data class containing atmospheric properties.


        Parameters
        ----------
        altitude : m3l Variable
            The altitude at which the atmospheric properties are to be evaluated
        """

        name = self.parameters['name']

        self.arguments = {}
        self.arguments['altitude'] = altitude


        rho = m3l.Variable(name='density', shape=(self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(self.num_nodes, ), operation=self)

        atmosphere = AtmosphericProperties(
            density=rho,
            dynamic_viscosity=mu,
            pressure=pressure,
            speed_of_sound=a,
            temperature=temp,
        )
        
        return atmosphere


