import m3l
from dataclasses import dataclass
from caddee.core.caddee_core.system_model.design_scenario.design_condition.design_condition import AcStates
import csdl
import numpy as np


@dataclass
class DampingRatios:
    damping_long_11 : m3l.Variable
    damping_long_12 : m3l.Variable
    damping_long_21 : m3l.Variable
    damping_long_22 : m3l.Variable

    damping_lat_11 : m3l.Variable
    damping_lat_12 : m3l.Variable
    damping_lat_21 : m3l.Variable
    damping_lat_22 : m3l.Variable

class LinearStabilityAnalysis(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('design_condition')
        self.design_condition_operations = None
        return super().initialize(kwargs)
    
    
    def evaluate(self, ac_states : AcStates, cg_vector : m3l.Variable, vehicle_mass : m3l.Variable) -> DampingRatios:
        
        self.arguments = {}
        self.arguments['u'] = ac_states.u
        self.arguments['v'] = ac_states.v
        self.arguments['w'] = ac_states.w
        self.arguments['p'] = ac_states.p
        self.arguments['q'] = ac_states.q
        self.arguments['r'] = ac_states.r
        self.arguments['phi'] = ac_states.phi
        self.arguments['theta'] = ac_states.theta
        
        u = m3l.Variable(name='u_plus', shape=(8, 1), operation=self)
        v = m3l.Variable(name='v_plus', shape=(8, 1), operation=self)
        w = m3l.Variable(name='w_plus', shape=(8, 1), operation=self)
        p = m3l.Variable(name='p_plus', shape=(8, 1), operation=self)
        q = m3l.Variable(name='q_plus', shape=(8, 1), operation=self)
        r = m3l.Variable(name='r_plus', shape=(8, 1), operation=self)
        phi = m3l.Variable(name='phi_plus', shape=(8, 1), operation=self)
        theta = m3l.Variable(name='theta_plus', shape=(8, 1), operation=self)

        damping_ratios = DampingRatios(
            damping_long_11=u,
            damping_long_12=v,
            damping_long_21=w,
            damping_long_22=p,
            damping_lat_11=q,
            damping_lat_12=r,
            damping_lat_21=phi,
            damping_lat_22=theta,
        )

        return damping_ratios
    
    def compute(self):
        csdl_model = LinearStabilityCSDL(linear_stability_analysis=self)

        return csdl_model
    

class LinearStabilityCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('linear_stability_analysis', types=LinearStabilityAnalysis)

    def define(self):
        stability_operation = self.parameters['linear_stability_analysis']
        print(stability_operation)

        u_oper = self.declare_variable('u', shape=(1, 1))
        v_oper = self.declare_variable('v', shape=(1, 1))
        w_oper = self.declare_variable('w', shape=(1, 1))
        p_oper = self.declare_variable('p', shape=(1, 1))
        q_oper = self.declare_variable('q', shape=(1, 1))
        r_oper = self.declare_variable('r', shape=(1, 1))
        phi_oper = self.declare_variable('phi', shape=(1, 1))
        theta_oper = self.declare_variable('theta', shape=(1, 1))

        u_plus = self.create_output('u_plus', shape=(8, 1), val=0) 
        u_plus[0, 0] = u_oper + 1
        
        v_plus = self.create_output('v_plus', shape=(8, 1), val=0)
        v_plus[1, 0] = v_oper + 1
        
        w_plus = self.create_output('v_plus', shape=(8, 1), val=0)
        w_plus[2, 0] = w_oper + 1
        
        p_plus = self.create_output('p_plus', shape=(8, 1), val=0)
        p_plus[3, 0] = p_oper + np.deg2rad(2)
        
        q_plus = self.create_output('q_plus', shape=(8, 1), val=0)
        q_plus[4, 0] = q_oper + np.deg2rad(2)

        r_plus = self.create_output('r_plus', shape=(8, 1), val=0)
        r_plus[5, 0] = r_oper + np.deg2rad(2)

        phi_plus = self.create_output('phi_plus', shape=(8, 1), val=0)
        phi_plus[6, 0] = phi_oper + np.deg2rad(2)
        
        theta_plus = self.create_output('theta_plus', shape=(8, 1), val=0)
        theta_plus[7, 0] = theta_oper + np.deg2rad(2)
        


        # u_minus = u_oper - 1
        # v_minus = v_oper - 1
        # w_minus = w_oper - 1
        # p_minus = p_oper - np.deg2rad(2)
        # q_minus = q_oper - np.deg2rad(2)
        # r_minus = r_oper - np.deg2rad(2)
        # phi_minus = phi_oper - np.deg2rad(2)
        # theta_minus = theta_oper - np.deg2rad(2)
        
        # minus_pert = self.create_output('minus_perturbation', shape=(8, 8), val=0)
        # minus_pert[0, 0] = u_minus
        # minus_pert[1, 1] = v_minus
        # minus_pert[2, 2] = w_minus
        # minus_pert[3, 3] = p_minus
        # minus_pert[4, 4] = q_minus
        # minus_pert[5, 5] = r_minus
        # minus_pert[6, 6] = phi_minus
        # minus_pert[7, 7] = theta_minus


        
