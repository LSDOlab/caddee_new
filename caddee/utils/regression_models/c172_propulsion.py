##skip
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import \
    MechanicsModel
import csdl
import numpy as np
import m3l


class C172PropulsionModel(m3l.ExplicitOperation):

    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)
        self.num_nodes = 1

    def compute(self) -> csdl.Model:
        return C172PropulsionModelCSDL(
            module=self,
            # thrust_vector=np.array([1, 0, 0]),  # Cruise and Climb
            thrust_vector=np.array([0, 0, -1]),  # Hover
        )

    def evaluate(self, ac_states):
        operation_csdl = self.compute()
        arguments = ac_states

        c172_prop_operation = m3l.CSDLOperation(name='c172_prop_model', arguments=arguments,
                                                operation_csdl=operation_csdl)
        forces = m3l.Variable(name='F', shape=(self.num_nodes, 3), operation=c172_prop_operation)
        moments = m3l.Variable(name='M', shape=(self.num_nodes, 3), operation=c172_prop_operation)

        return forces, moments


class C172PropulsionModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare(name='name', default='propulsion')
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('thrust_vector', default=np.array([-1, 0, 0]), types=np.ndarray)
        return

    def define(self):
        name = self.parameters['name']
        num_nodes = self.parameters['num_nodes']
        thrust_vector = self.parameters['thrust_vector']

        # Inputs constant across conditions (segments)
        prop_radius = self.register_module_input(name='propeller_radius', shape=(1,), units='m',
                                                 computed_upstream=False)
        ref_pt = self.register_module_input(name='ref_pt', shape=(3,), units='m', computed_upstream=False)
        thrust_origin = self.register_module_input(name='thrust_origin', shape=(3,), units='m', computed_upstream=False)

        # Inputs changing across conditions (segments)
        omega = self.register_module_input('omega', shape=(num_nodes, 1), units='rpm', computed_upstream=False)

        u = self.declare_variable(name='u',
                                  shape=(num_nodes, 1), units='rad', val=1)
        v = self.declare_variable(name='v',
                                  shape=(num_nodes, 1), units='rad', val=0)
        w = self.declare_variable(name='w',
                                  shape=(num_nodes, 1), units='rad', val=0)

        p = self.declare_variable(name='p',
                                  shape=(num_nodes, 1), units='rad', val=0)
        q = self.declare_variable(name='q',
                                  shape=(num_nodes, 1), units='rad', val=0)
        r = self.declare_variable(name='r',
                                  shape=(num_nodes, 1), units='rad', val=0)

        phi = self.declare_variable(name='phi',
                                    shape=(num_nodes, 1), units='rad', val=0)
        theta = self.declare_variable(name='theta',
                                      shape=(num_nodes, 1), units='rad', val=0)
        psi = self.declare_variable(name='psi',
                                    shape=(num_nodes, 1), units='rad', val=0)

        gamma = self.declare_variable(name='gamma',
                                      shape=(num_nodes, 1), units='rad', val=0)

        x = self.declare_variable(name='x',
                                  shape=(num_nodes, 1), units='rad', val=0)
        y = self.declare_variable(name='y',
                                  shape=(num_nodes, 1), units='rad', val=0)
        z = self.declare_variable(name='z',
                                  shape=(num_nodes, 1), units='rad', val=0)

        rho = 1.225
        # endregion

        rad = csdl.expand(var=prop_radius, shape=(num_nodes, 1))

        omega_RAD = (omega * 2 * np.pi) / 60.0  # rad/s
        V = (u ** 2 + v ** 2 + w ** 2) ** 0.5
        J = (np.pi * V) / (omega_RAD * rad)  # non-dimensional
        self.register_module_output('advance_ratio', J)
        Ct_interp = -0.1692121 * J ** 2 + 0.03545196 * J + 0.10446359  # non-dimensional

        T = (2 / np.pi) ** 2 * rho * \
            (omega_RAD * rad) ** 2 * Ct_interp + \
            p * q * r * phi * theta * psi * gamma * x * y * z * 0  # N

        self.register_output(name='T', var=T)

        F = self.create_output(name='F', shape=(num_nodes, 3), val=0)
        for i in range(3):
            if thrust_vector[i] == 1 or thrust_vector[i] == -1:
                F[:, i] = T * thrust_vector[i]
            elif thrust_vector[i] == 0:
                F[:, i] = T * 0
            else:
                raise ValueError
        offset = ref_pt - thrust_origin
        M = self.create_output(name='M', shape=(num_nodes, 3))
        M[:, 0] = T * 0
        for ii in range(num_nodes):
            M[ii, 1] = F[ii, 0] * csdl.reshape(offset[2], (1, 1)) + F[ii, 2] * csdl.reshape(offset[0], (1, 1))
        M[:, 2] = T * 0
        return