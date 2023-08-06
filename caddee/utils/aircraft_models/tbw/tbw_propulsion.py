
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import \
    MechanicsModel
import csdl
import numpy as np
import m3l


class tbwPropulsionModel(m3l.ExplicitOperation):

    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)
        self.num_nodes = 1

    def compute(self) -> csdl.Model:
        return tbwPropulsionModelCSDL(
            module=self,
            thrust_vector=np.array([1, 0, 0]),  # Cruise and Climb
            #thrust_vector=np.array([0, 0, -1]),  # Hover
        )

    def evaluate(self, ac_states):
        self.name = "tbw_prop_model"
        self.arguments = ac_states

        #tbw_prop_operation = m3l.CSDLOperation(name='tbw_prop_model', arguments=arguments, operation_csdl=operation_csdl)
        forces = m3l.Variable(name='F', shape=(self.num_nodes, 3), operation=self)
        moments = m3l.Variable(name='M', shape=(self.num_nodes, 3), operation=self)
        return forces, moments


class tbwPropulsionModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare(name='name', default='propulsion')
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('thrust_vector', default=np.array([1, 0, 0]), types=np.ndarray)
        return

    def define(self):
        name = self.parameters['name']
        num_nodes = self.parameters['num_nodes']
        thrust_vector = self.parameters['thrust_vector']

        ref_pt = self.register_module_input(name='ref_pt', shape=(3,), units='m', computed_upstream=False)
        thrust_origin = self.register_module_input(name='thrust_origin', shape=(3,), units='m', computed_upstream=False)
        throttle = self.register_module_input('throttle', 
                                              shape=(num_nodes, 1), 
                                              computed_upstream=False)

        # from scipy.interpolate import interp1d

        # throttle_data = [0, 1]
        # thrust_data = [0, 19467]  # N
        # interp_func = interp1d(throttle_data, thrust_data, kind='linear')
        # print(interp_func)

        # throttle = self.register_module_input('throttle', 
        #                                       shape=(num_nodes, 1), 
        #                                       computed_upstream=False)
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

        T = 2 * 19467 * throttle + x*y*z*gamma*psi*theta*u*v*w*p*q*r*phi*0
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


# if __name__ == '__main__':
#     prop_model = tbwPropulsionModelCSDL()
#     import python_csdl_backend
#     sim = python_csdl_backend.Simulator(prop_model)
#     sim['thrust_origin'] = np.array([61.009, 42.646, 6.256])
#     sim['ref_pt'] = np.array([0., 0., -2.8])
#     sim['throttle'] = 1.
#     sim.run()
#     print('Thrust: ', sim['T'])
#     print('Force: ', sim['F'])
#     print('Moment:', sim['M'])
