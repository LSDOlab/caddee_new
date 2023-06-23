import m3l
import csdl
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.simple_battery_sizing_csdl import SimpleBatterySizingCSDL


class SimpleBatterySizingM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs): pass

    def compute(self) -> csdl.Model:
        return SimpleBatterySizingCSDL(module=self)
    
    def evaluate(self):
        operation_csdl = self.compute()
        arguments = {}

        battery_sizing_opration = m3l.CSDLOperation(name='battery_sizing', arguments=arguments, operation_csdl=operation_csdl)
        mass = m3l.Variable(name='mass', shape=(1, ), operation=battery_sizing_opration)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3, ), operation=battery_sizing_opration)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=battery_sizing_opration)

        return mass, cg_vector, inertia_tensor