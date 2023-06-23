import m3l 
import csdl
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_csdl import M4RegressionsCSDL


class M4RegressionsM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs): pass

    def compute(self) -> csdl.Model:
        return M4RegressionsCSDL()
    
    def evaluate(self,  battery_mass):
        operation_csdl = self.compute()
        arguments = {
            'battery_mass' : battery_mass
        }

        m4_regression_operation = m3l.CSDLOperation(name='m4_regression', arguments=arguments, operation_csdl=operation_csdl)

        mass = m3l.Variable(name='mass', shape=(1, ), operation=m4_regression_operation)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3, ), operation=m4_regression_operation)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=m4_regression_operation)

        return mass, cg_vector, inertia_tensor


    