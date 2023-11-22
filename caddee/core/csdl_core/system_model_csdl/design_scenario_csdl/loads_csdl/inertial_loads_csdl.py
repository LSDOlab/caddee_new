import csdl
import numpy as np
import m3l


class InertialLoads(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('num_nodes', types=int, default=1)
        self.parameters.declare('load_factor', default=1.)
        self.parameters.declare('name', types=str, default='inertial_loads_model')
        self._stability_flag = False

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self):
        load_factor = self.parameters['load_factor']
        if self._stability_flag:
            num_nodes = self.parameters['num_nodes'] * 13
        else:
            num_nodes = self.parameters['num_nodes']

        csdl_model =  InertialLoadsModel(
            num_nodes=num_nodes, 
            load_factor=load_factor,
            stability_flag=self._stability_flag,
        )
        return csdl_model
    
    def evaluate(self, total_cg_vector, totoal_mass, ac_states, ref_pt=None, stability=False):
        num_nodes = self.parameters['num_nodes']
        self.arguments = {
            'total_cg_vector' : total_cg_vector,
            'total_mass' : totoal_mass
        }
        if ref_pt:
            self.arguments['ref_pt'] = ref_pt

        if stability:
            self._stability_flag = True

        phi = ac_states.phi
        theta = ac_states.theta

        self.arguments['phi'] = phi
        self.arguments['theta'] = theta


        F_inertial = m3l.Variable(name='F_inertial', shape=(num_nodes, 3), operation=self)
        M_inertial = m3l.Variable(name='M_inertial', shape=(num_nodes, 3), operation=self)

        return F_inertial, M_inertial

class InertialLoadsModel(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('load_factor', default=1.)
        self.parameters.declare('stability_flag', types=bool, default=False)
        
        return

    def define(self):
        num_nodes = self.parameters['num_nodes']
        load_factor = self.parameters['load_factor']
        stability_flag = self.parameters['stability_flag']

        # Inputs constant across conditions (segments)
        cg_vector = self.declare_variable('total_cg_vector', shape=(3, ))
        cgx = cg_vector[0]
        cgy = cg_vector[1]
        cgz = cg_vector[2]        
        m = self.declare_variable('total_mass', shape=(1, ), units='kg')
        mass = csdl.expand(var=m, shape=(num_nodes, 1))

        ref_pt = csdl.expand(self.declare_variable(name='ref_pt', shape=(3, ), units='m'), shape=(num_nodes, 3), indices='j->ij')
        
        # Inputs changing across conditions (segments)
        th = self.declare_variable('theta', shape=(num_nodes, 1), units='rad')
        ph = self.declare_variable('phi', shape=(num_nodes, 1), units='rad')
        # self.print_var(th)
        # self.print_var(ph)
       
        cg = self.create_output(name='cg', shape=(3, ))
        cg[0] = cgx
        cg[1] = cgy * 0 # NOTE: cgy should be exactly zero (even small deviations, e.g. 1e-4 will cause non-zero moments)
        cg[2] = cgz

        cg_vector = csdl.expand(var=cg, shape=(num_nodes, 3), indices='j->ij')

        g = 9.81 * load_factor  # todo: compute as a function of altitude

        F = self.create_output(name='F_inertial_compute', shape=(num_nodes, 3))

        F[:, 0] = -mass * g * csdl.sin(th) 
        F[:, 1] = mass * g * csdl.cos(th) * csdl.sin(ph) 
        F[:, 2] = mass * g * csdl.cos(th) * csdl.cos(ph) 

        r_vec = cg_vector - ref_pt
        # r_vec = csdl.reshape(r_vec, (1, 3))
        
        M = self.create_output(name='M_inertial_compute', shape=(num_nodes, 3))
        for n in range(num_nodes):
            M[n, :] = csdl.cross(r_vec[n, :], F[n, :], axis=1) 
        
        self.register_output('F_inertial', F * 1)
        self.register_output('M_inertial', M * 1)


        # self.print_var(F)
        # self.print_var(r_vec)
        # self.print_var(cg)
        # self.print_var(M)

        return


