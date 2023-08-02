import csdl
import numpy as np


class ConstantPowerDensityMotor(csdl.Model):
    def initialize(self):
        self.parameters.declare('power_density', default=3000)
        self.parameters.declare('component', default=None, allow_none=True)

    def define(self):
        power_density = self.parameters['power_density']
        comp = self.parameters['component']
        if comp:
            prefix = comp.parameters['name']
        else:
            prefix = ''

        motor_mass = self.declare_variable('motor_mass', shape=(1, ))
        rotor_torque = self.declare_variable('rotor_torque', shape=(1, ))
        rpm = self.declare_variable('rotor_rpm', shape=(1, ))
        rotor_power = rotor_torque * (rpm / 60)

        power_available = power_density * motor_mass
        net_power = power_available - rotor_power

        self.register_output('net_power', net_power)
        self.register_output('motor_power', rotor_power * 1)