from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import csdl
import numpy as np
import m3l



class TbwViscousDragModel(m3l.ExplicitOperation):

    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)
        self.parameters.declare('geometry_units', default='m')
        self.num_nodes = 1

    def compute(self) -> csdl.Model:
        return TbwViscousDragModelCSDL(
            module=self,
            geometry_units=self.parameters['geometry_units']
        )

    def evaluate(self, ac_states):
        self.name = "tbw_viscous_drag_model"
        self.arguments = ac_states

        forces = m3l.Variable(name='F', shape=(self.num_nodes, 3), operation=self)
        moments = m3l.Variable(name='M', shape=(self.num_nodes, 3), operation=self)
        return forces, moments


class TbwViscousDragModelCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare(name='name', default='viscous_drag')
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('geometry_units', default='m')
        self.parameters.declare(name='reference_area', default=137.3107)  # 1478 ft^2 = 137.3107 m^2
        self.parameters.declare(name='wing_viscous_cf', default=0.00185)
        # self.parameters.declare(name='wing_viscous_cf', default=0.05)
        # self.parameters.declare(name='wing_viscous_cf', default=0.0075)
        return

    def define(self):
        name = self.parameters['name']
        num_nodes = self.parameters['num_nodes']
        geometry_units = self.parameters['geometry_units']
        reference_area_m2 = self.parameters['reference_area']

        rho = 1.225  # kg/m^3
        kinematic_viscosity = 1.4207E-5  # m^2/s

        ft2_2_m2 = 0.092903
        ft2m = 0.3048

        area = self.register_module_input('area',
                                          shape=(num_nodes, 1),
                                          computed_upstream=True)
        chord = self.register_module_input('chord',
                                          shape=(num_nodes, 1),
                                          computed_upstream=False)

        if geometry_units == 'ft':
            area_m2 = area * ft2_2_m2
            chord_m = chord * ft2m
        elif geometry_units == 'm':
            area_m2 = area * 1.
            chord_m = chord * 1.
        else:
            raise IOError

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

        VTAS = (u**2 + v**2 + w**2)**0.5 + (p+q+r+phi+theta+psi+gamma+x+y+z) * 0
        qBar = 0.5 * rho * VTAS**2

        Re = VTAS*chord_m/kinematic_viscosity

        area_fraction = area_m2 / reference_area_m2
        self.register_output('area_fraction', var=area_fraction)
        self.print_var(var=area_fraction)

        Cf = ( 0.664 / (Re)**0.5 ) * 0 + self.parameters['wing_viscous_cf']*area_fraction
        self.register_output(name='Cf', var=Cf)

        D = qBar * Cf * area_m2
        self.register_output(name='D', var=D)

        F = self.create_output(name='F', shape=(num_nodes, 3), val=0)
        F[:, 0] = D * -1.

        M = self.create_output(name='M', shape=(num_nodes, 3), val=0)
        M[:, 0] = D * 0
        return
