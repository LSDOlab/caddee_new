import m3l 
import csdl
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_csdl_full import M4RegressionsCSDL
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_csdl_minus_wing import M4RegressionsMinusWingCSDL


class M4RegressionsM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs): 
        # self.parameters.declare('fully_empirical', types=bool, default=True)
        self.parameters.declare('exclude_wing', types=bool, default=False)


    def compute(self) -> csdl.Model:
        exclude_wing = self.parameters['exclude_wing']
        if exclude_wing is True:
            csdl_model = M4RegressionsMinusWingCSDL()
        else:
            csdl_model = M4RegressionsCSDL()
        
        return csdl_model
    
    def evaluate(self,  battery_mass):
        # operation_csdl = self.compute()

        self.name = 'm4_regression'

        self.arguments = {
            'battery_mass' : battery_mass
        }

        mass = m3l.Variable(name='mass', shape=(1, ), operation=self)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3, ), operation=self)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=self)

        return mass, cg_vector, inertia_tensor


    