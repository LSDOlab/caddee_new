##skip

from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
from caddee.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import MechanicsModel
import csdl
import numpy

class C172PropulsionModel(MechanicsModel):

    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)

    def _assemble_csdl(self):

        csdl_model = C172PropulsionModelCSDL(num_nodes=1)

        return csdl_model


class C172PropulsionModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare(name='name', default='propulsion')
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('thrust_vector', types=np.ndarray)
        return

    def define(self):
        name = self.parameters['name']
        num_nodes = self.parameters['num_nodes']
        thrust_vector = self.parameters['thrust_vector']

        # Inputs constant across conditions (segments)
        prop_radius = self.declare_variable(name='propeller_radius', shape=(1,), units='m')
        ref_pt = self.declare_variable(name='ref_pt', shape=(3,), units='m')
        thrust_origin = self.declare_variable(name='thrust_origin', shape=(3,), units='m')

        # Inputs changing across conditions (segments)
        omega = self.declare_variable('omega', shape=(num_nodes, 1), units='rpm')

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

        Phi = self.declare_variable(name='Phi',
                                    shape=(num_nodes, 1), units='rad', val=0)
        Theta = self.declare_variable(name='Theta',
                                      shape=(num_nodes, 1), units='rad', val=0)
        Psi = self.declare_variable(name='Psi',
                                    shape=(num_nodes, 1), units='rad', val=0)

        x = self.declare_variable(name='x',
                                  shape=(num_nodes, 1), units='rad', val=0)
        y = self.declare_variable(name='y',
                                  shape=(num_nodes, 1), units='rad', val=0)
        z = self.declare_variable(name='z',
                                  shape=(num_nodes, 1), units='rad', val=0)

        # region Atmosisa
        # self.register_output(name='h_for_atmosisa', var=z + 0.)
        # atmosisa = AtmosphereModel(
        #     num_nodes=num_nodes
        # )
        #
        # self.add(submodel=atmosisa,
        #          name=CaddeeModelNames.AtmosisaModel.value,
        #          promotes=[])
        #
        # self.connect('h_for_atmosisa', f'{CaddeeModelNames.AtmosisaModel.value}.altitude')
        #
        # rho = self.declare_variable(name='rho', shape=(num_nodes, 1))
        # self.connect(f'{CaddeeModelNames.AtmosisaModel.value}.density', 'rho')

        # self.print_var(var=rho)
        rho = 1.225
        # endregion

        rad = csdl.expand(var=prop_radius, shape=(num_nodes, 1))

        omega_RAD = (omega * 2 * np.pi) / 60.0  # rad/s
        V = (u ** 2 + v ** 2 + w ** 2) ** 0.5
        J = (np.pi * V) / (omega_RAD * rad)  # non-dimensional

        Ct_interp = -0.1692121 * J ** 2 + 0.03545196 * J + 0.10446359  # non-dimensional

        T = (2 / np.pi) ** 2 * rho * \
            (omega_RAD * rad) ** 2 * Ct_interp  # N
        self.register_output(name='T', var=T)

        F = self.create_output(name='F', shape=(num_nodes, 3))
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