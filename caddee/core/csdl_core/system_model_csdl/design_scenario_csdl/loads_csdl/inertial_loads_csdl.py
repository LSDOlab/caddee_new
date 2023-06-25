import csdl
import numpy as np
from caddee.utils.base_model_csdl import BaseModelCSDL
import m3l


class InertialLoadsM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.num_nodes = 1
        pass

    def compute(self):
        return InertialLoadsModel(num_nodes=self.num_nodes)
    
    def evaluate(self, total_cg_vector, totoal_mass, ac_states):
        operation_csdl = self.compute()
        arguments = {
            'total_cg_vector' : total_cg_vector,
            'total_mass' : totoal_mass
        }

        phi = ac_states['phi']
        theta = ac_states['theta']

        arguments['phi'] = phi
        arguments['theta'] = theta

        inertial_loads_operation = m3l.CSDLOperation(name='inertial_loads', arguments=arguments, operation_csdl=operation_csdl)
        F_inertial = m3l.Variable(name='F_inertial', shape=(self.num_nodes, 3), operation=inertial_loads_operation)
        M_inertial = m3l.Variable(name='M_inertial', shape=(self.num_nodes, 3), operation=inertial_loads_operation)

        return F_inertial, M_inertial

class InertialLoadsModel(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('num_nodes', default=1)
        return

    def define(self):
        num_nodes = self.parameters['num_nodes']

        # Inputs constant across conditions (segments)
        cg_vector = self.register_module_input('total_cg_vector', shape=(3, ))
        cgx = cg_vector[0]
        cgy = cg_vector[1]
        cgz = cg_vector[2]        
        m = self.register_module_input('total_mass', shape=(1, ), units='kg')
        mass = csdl.expand(var=m, shape=(num_nodes, 1))

        ref_pt = self.register_module_input(name='ref_pt', shape=(3, ), val=np.array([0, 0, 0]), units='m')
        
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
            M[n, :] = csdl.cross(r_vec, F[n, :], axis=1) * 0
        
        # self.print_var(F)
        # self.print_var(r_vec)
        # self.print_var(cg)
        # self.print_var(M)

        return


