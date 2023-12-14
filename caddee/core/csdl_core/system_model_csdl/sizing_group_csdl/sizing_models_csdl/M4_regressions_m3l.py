import m3l 
import csdl
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_csdl_full import M4RegressionsCSDL
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_csdl_minus_wing import M4RegressionsMinusWingCSDL
from caddee.utils.helper_classes import MassProperties


class M4Regressions(m3l.ExplicitOperation):
    def initialize(self, kwargs): 
        self.parameters.declare('name', types=str)
        self.parameters.declare('exclude_wing', types=bool, default=False)

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self) -> csdl.Model:
        exclude_wing = self.parameters['exclude_wing']
        if exclude_wing is True:
            csdl_model = M4RegressionsMinusWingCSDL()
        else:
            csdl_model = M4RegressionsCSDL()
        
        return csdl_model
    
    def evaluate(self,  battery_mass, fuselage_length, tail_area, fin_area, wing_area, wing_AR):
        self.arguments = {
            'battery_mass' : battery_mass,
            'fuselage_length' : fuselage_length,
            'tail_area' : tail_area,
            'fin_area' : fin_area,
            'wing_area' : wing_area,
            'wing_AR' : wing_AR,
        }

        mass = m3l.Variable(name='mass', shape=(1, ), operation=self)
        cg_vector = m3l.Variable(name='cg_vector', shape=(3, ), operation=self)
        inertia_tensor = m3l.Variable(name='inertia_tensor', shape=(3, 3), operation=self)

        outputs = MassProperties(
            mass=mass,
            cg_vector=cg_vector,
            inertia_tensor=inertia_tensor,
        )

        return outputs


    