import m3l
import csdl


class ConstantPowerDensityMotorM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('component', default=None, allow_none=True)
        self.parameters.declare('pwoer_density', default=3000)

    def compute(self) -> csdl.Model:
        from caddee.core.csdl_core.system_model_csdl.submodels.power_density_based_motor_analysis_csdl import ConstantPowerDensityMotor
        
        component = self.parameters['component']
        power_density = self.parameters['power_density']
        
        csdl_model = ConstantPowerDensityMotor(
            component=component,
            power_density=power_density,
        )  

        return csdl_model

    def evaluate(self, rotor_torque, design_condition) -> tuple:
        component = self.parameters['component']
        if component:
            component_name = component.parameters['name']
        else: 
            component_name = 'motor'

        if design_condition:
            dc_name = design_condition.parameters['name']
            self.name = f"{dc_name}_{component_name}_power_density_motor_model"
        else:
            self.name = f"{component_name}_power_density_motor_model"

        self.arguments = {}
        self.arguments['rotor_torque'] = rotor_torque

        motor_power = m3l.Variable(name='motor_power', shape=(1, ), operation=self)
        net_power = m3l.Variable(name='net_power', shape=(1, ), operation=self)

        return motor_power, net_power
