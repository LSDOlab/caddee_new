##skip

from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import \
    MechanicsModel
import csdl
import numpy as np
import m3l
from dataclasses import dataclass

class StabilityAdapterModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('arguments', types=dict)
        self.parameters.declare('num_nodes', types=int)
        self.parameters.declare('neglect', types=list, default=[])
    
    def define(self):
        args = self.parameters['arguments']
        num_nodes = self.parameters['num_nodes']
        ac_states = ['u', 'v', 'w', 'p', 'q', 'r', 'theta', 'phi', 'psi', 'x', 'y', 'z']
        special_cases = self.parameters['neglect']
        for key, value in args.items():
            if key in ac_states:
                csdl_var = self.declare_variable(key, shape=(num_nodes * 13, ))
                self.register_output(name=f'{key}_exp', var=csdl_var * 1)
            elif key in special_cases:
                csdl_var = self.declare_variable(key, shape=value.shape)
                self.register_output(name=f'{key}_exp', var=csdl_var * 1)
            else:
                csdl_var = self.declare_variable(key, shape=value.shape)
                if len(value.shape) == 1 and value.shape[0] == 1:
                    # print(key, value.shape)
                    csdl_var_exp = csdl.expand(csdl_var, shape=(num_nodes * 13, ))
                    self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) == 1 and value.shape[0] != 1:
                    # print(key, (13, ) + value.shape)
                    csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(13, ) + value.shape, indices='i->ji'), new_shape=(13, value.shape[0]))
                    self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) == 2:
                    if num_nodes == value.shape[0]:
                        csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(13, ) + value.shape, indices='ij->kij'), new_shape=(13*num_nodes, value.shape[1]))
                        self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                    elif num_nodes == value.shape[1]:
                        csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(13, ) + value.shape, indices='ij->kij'), new_shape=(13*num_nodes, value.shape[0]))
                        self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) > 2:
                    raise NotImplementedError
                
@dataclass
class C172PropRegressionOutputs:
    forces : m3l.Variable = None
    moments : m3l.Variable = None


class C172PropulsionModel(m3l.ExplicitOperation):

    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('num_nodes', types=int, default=1)
        self._stability_flag = False
        super().initialize(kwargs=kwargs)

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self) -> csdl.Model:
        num_nodes = self.parameters['num_nodes']
        
        if self._stability_flag:
            csdl_model = StabilityAdapterModelCSDL(
                arguments=self.arguments,
                num_nodes=num_nodes,
                neglect=['propeller_radius', 'ref_pt'],
            )

            solver_model = C172PropulsionModelCSDL(
                num_nodes=num_nodes * 13,
                stability_flag=self._stability_flag,
            )

            operation_name = self.parameters['name']

            csdl_model.add(solver_model, operation_name, promotes=[])
            for key, value in self.arguments.items():
                csdl_model.connect(f'{key}_exp',f'{operation_name}.{key}')

        else:
            csdl_model = C172PropulsionModelCSDL(
                num_nodes=num_nodes,
                stability_flag=self._stability_flag,
            )

        return csdl_model

    def evaluate(self, ac_states, rpm, prop_radius, thrust_origin, thrust_vector, ref_pt) -> C172PropRegressionOutputs:
        self.arguments = {}
        for key, value in ac_states.__dict__.items():
            if key in ['time', 'stability_flag']:
                pass
            else:
                self.arguments[key] = value

        self.arguments['omega'] = rpm
        self.arguments['propeller_radius'] = prop_radius
        self.arguments['ref_pt'] = ref_pt
        self.arguments['thrust_origin'] = thrust_origin
        self.arguments['thrust_vector'] = thrust_vector

        self._stability_flag = ac_states.stability_flag

        if self._stability_flag:
            num_nodes = self.parameters['num_nodes'] * 13
            forces = m3l.Variable(name=f'{self.name}.F', shape=(num_nodes, 3), operation=self)
            moments = m3l.Variable(name=f'{self.name}.M', shape=(num_nodes, 3), operation=self)
        else:
            num_nodes = self.parameters['num_nodes']
            forces = m3l.Variable(name='F', shape=(num_nodes, 3), operation=self)
            moments = m3l.Variable(name='M', shape=(num_nodes, 3), operation=self)

        c172_prop_reg_outputs = C172PropRegressionOutputs(
            forces=forces,
            moments=moments,
        )

        # if self._stability_flag:
        #     forces_perturbed = m3l.Variable(name='F_perturbed', shape=(8, 3), operation=self)
        #     moments_perturbed = m3l.Variable(name='M_perturbed', shape=(8, 3), operation=self)
        #     c172_prop_reg_outputs.forces_perturbed = forces_perturbed
        #     c172_prop_reg_outputs.moments_perturbed = moments_perturbed


        return c172_prop_reg_outputs# forces, moments


class C172PropulsionModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('stability_flag', types=bool, default=False)

    def define(self):
        num_nodes = self.parameters['num_nodes']
        stability_flag = self.parameters['stability_flag']

        # Inputs constant across conditions (segments)
        prop_radius = self.declare_variable(name='propeller_radius', shape=(1,), units='m')
        ref_pt = csdl.expand(self.declare_variable(name='ref_pt', shape=(3, ), units='m'), shape=(num_nodes, 3), indices='j->ij')
        # self.print_var(ref_pt)
        thrust_origin = self.declare_variable(name='thrust_origin', shape=(num_nodes, 3), units='m')
        thrust_vector = self.declare_variable(name='thrust_vector', shape=(num_nodes, 3), units='m')

        # Inputs changing across conditions (segments)
        omega = self.declare_variable('omega', shape=(num_nodes, 1), units='rpm')
        u = self.declare_variable(name='u',
                                  shape=(num_nodes, 1), units='rad', val=0)
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

        rad = csdl.expand(var=prop_radius, shape=(num_nodes, 1))

        omega_RAD = (omega * 2 * np.pi) / 60.0  # rad/s
        V = (u ** 2 + v ** 2 + w ** 2) ** 0.5
        J = (np.pi * V) / (omega_RAD * rad)  # non-dimensional
        self.register_output('advance_ratio', J)
        Ct_interp = -0.1692121 * J ** 2 + 0.03545196 * J + 0.10446359  # non-dimensional

        T = (2 / np.pi) ** 2 * rho * \
            (omega_RAD * rad) ** 2 * Ct_interp + \
            p * q * r * phi * theta * psi * gamma * x * y * z * 0  # N

        self.register_output(name='T', var=T)
        print(T.shape)

        F = self.create_output(name='F_compute', shape=(num_nodes, 3), val=0)
        for i in range(num_nodes):
            F[i, 0] = T[i, 0] * thrust_vector[i, 0] 
            F[i, 1] = T[i, 0] * thrust_vector[i, 1]
            F[i, 2] = T[i, 0] * thrust_vector[i, 2]
        # for i in range(3):
        #     if thrust_vector[i] == 1 or thrust_vector[i] == -1:
        #         F[:, i] = T * thrust_vector[i]
        #     elif thrust_vector[i] == 0:
        #         F[:, i] = T * 0
        #     else:
        #         raise ValueError
        offset = ref_pt - thrust_origin
        M = self.create_output(name='M_compute', shape=(num_nodes, 3))
        M[:, 0] = T * 0
        for ii in range(num_nodes):
            M[ii, 1] = F[ii, 0] * offset[ii, 2] + F[ii, 2] * offset[ii, 0]
        M[:, 2] = T * 0

        self.register_output(name='F', var=F*1)
        self.register_output(name='M', var=M*1)

