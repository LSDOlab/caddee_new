import csdl
import numpy as np
from caddee.utils.base_model_csdl import BaseModelCSDL


class InertialLoadsModel(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('num_nodes', default=1)
        return

    def define(self):
        num_nodes = self.parameters['num_nodes']

        # Inputs constant across conditions (segments)
        cgx = self.register_module_input('cgx_total', shape=(1, ), units='m')
        cgy = self.register_module_input('cgy_total', shape=(1, ), units='m')
        cgz = self.register_module_input('cgz_total', shape=(1, ), units='m')
        m = self.register_module_input('m_total', shape=(1, ), units='kg')
        ref_pt = self.register_module_input(name='ref_pt', shape=(3, ), val=np.array([0, 0, 0]), units='m')

        mass = csdl.expand(var=m, shape=(num_nodes, 1))

        # Inputs changing across conditions (segments)
        th = self.register_module_input('theta', shape=(num_nodes, 1), units='rad')
        ph = self.register_module_input('phi', shape=(num_nodes, 1), units='rad')
        # self.print_var(th)
        # self.print_var(ph)
       
        cg = self.register_module_output(name='cg', shape=(3, ))
        cg[0] = cgx
        cg[1] = cgy * 0 # NOTE: cgy should be exactly zero (even small deviations, e.g. 1e-4 will cause non-zero moments)
        cg[2] = cgz

        g = 9.81 # Earth acceleration hard-coded here

        F = self.register_module_output(name='F_inertial', shape=(num_nodes, 3))

        F[:, 0] = -mass * g * csdl.sin(th) 
        F[:, 1] = mass * g * csdl.cos(th) * csdl.sin(ph) 
        F[:, 2] = mass * g * csdl.cos(th) * csdl.cos(ph) 

        r_vec = cg - ref_pt
        r_vec = csdl.reshape(r_vec, (1, 3))
        M = self.register_module_output(name='M_inertial', shape=(num_nodes, 3))
        for n in range(num_nodes):
            M[n, :] = csdl.cross(r_vec, F[n, :], axis=1)
        # self.print_var(F)
        # self.print_var(r_vec)
        # self.print_var(cg)
        # self.print_var(M)

        return


