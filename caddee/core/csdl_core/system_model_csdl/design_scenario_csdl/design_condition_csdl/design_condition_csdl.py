import csdl
from caddee.utils.base_model_csdl import BaseModelCSDL

from caddee.core.caddee_core.system_model.design_scenario.design_condition.design_condition import SteadyDesignCondition, CruiseCondition, HoverCondition, ClimbCondition
import numpy as np


class SteadyDesignConditionCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('steady_condition', types=SteadyDesignCondition)
        
    def define(self):
        design_condition = self.parameters['steady_condition']
        pass
    

class CruiseConditionCSDL(SteadyDesignConditionCSDL):
    def initialize(self): 
        self.parameters.declare('cruise_condition', types=CruiseCondition)

    def define(self):
        cruise_condition = self.parameters['cruise_condition']
        cruise_condition_name = cruise_condition.parameters['name']
        num_nodes = cruise_condition.num_nodes

        theta = self.declare_variable('pitch_angle', shape=(num_nodes))
        h = self.declare_variable('altitude', shape=(num_nodes))

        atmosphere_model = cruise_condition.atmosphere_model(name=f'atmosphere_model')
        atmosphere_model.num_nodes = num_nodes
        atmosphere_model_csdl = atmosphere_model.compute()
        self.add(atmosphere_model_csdl, 'atmosphere_model')

        speed_of_sound = self.declare_variable('speed_of_sound', shape=(num_nodes, ))

        print(cruise_condition.arguments)
        mach_m3l = cruise_condition.arguments['mach_number']
        speed_m3l = cruise_condition.arguments['cruise_speed']
        range_m3l = cruise_condition.arguments['cruise_range']
        time_m3l = cruise_condition.arguments['cruise_time']

        if all([range_m3l, time_m3l]):
            cruise_range = self.declare_variable('cruise_range', shape=(num_nodes, ))
            cruise_time = self.declare_variable('cruise_time', shape=(num_nodes, ))
            
            cruise_speed = cruise_range/cruise_time
            
            self.register_output(f'cruise_speed', cruise_speed)
        
        elif all([mach_m3l, range_m3l]):
            mach_number = self.declare_variable('mach_number', shape=(num_nodes, ))
            cruise_range = self.declare_variable('cruise_range', shape=(num_nodes, ))
            
            cruise_speed = speed_of_sound * mach_number
            cruise_time = cruise_range / cruise_speed 

            self.register_output('cruise_speed', cruise_speed)
            self.register_output('cruise_time', cruise_time)



        elif all([speed_m3l, time_m3l]):
            cruise_speed = self.declare_variable('cruise_speed', shape=(num_nodes, ))
            cruise_time = self.declare_variable('cruise_time', shape=(num_nodes, ))

            cruise_range = cruise_speed * cruise_time
            mach_number = cruise_speed / speed_of_sound

            self.register_output('cruise_range', cruise_range)
            self.register_output('mach_number', mach_number)

        elif all([speed_m3l, range_m3l]):
            cruise_speed = self.declare_variable('cruise_speed', shape=(num_nodes, ))
            cruise_range = self.declare_variable('cruise_range', shape=(num_nodes, ))

            mach_number = cruise_speed / speed_of_sound
            cruise_time = cruise_range / cruise_speed

            self.register_output('mach_number', mach_number)
            self.register_output('cruise_time', cruise_time)
        
        # Compute aircraft states
        phi = theta* 0
        gamma = theta * 0
        psi = theta * 0
        psi_w = theta * 0

        alfa = theta - gamma
        beta = psi + psi_w
        u = cruise_speed * csdl.cos(alfa) * csdl.cos(beta)
        v = cruise_speed * csdl.sin(beta)
        w = -cruise_speed * csdl.sin(alfa) * csdl.cos(beta)
        p = u * 0
        q = u * 0
        r = u * 0
        x = u * 0
        y = u * 0
        z = u * 0
        
        self.register_output('time', cruise_time * 1)

        # NOTE: below, we don't need to pre_pend the aircraft condition name any more since the vectorization will be handled by m3l
        if cruise_condition.parameters['stability_flag'] and (num_nodes>1):
            raise NotImplementedError("Stability analysis for vectorized design conditions not yet implemented. 'num_nodes' can at most be 1 if stability analysis is to be performed")
        
        if cruise_condition.parameters['stability_flag'] and (num_nodes==1):
        
            u_stab = self.create_output('u', shape=(num_nodes * 13, ), val=0)
            v_stab = self.create_output('v', shape=(num_nodes * 13, ), val=0)
            w_stab = self.create_output('w', shape=(num_nodes * 13, ), val=0)
            p_stab = self.create_output('p', shape=(num_nodes * 13, ), val=0)
            q_stab = self.create_output('q', shape=(num_nodes * 13, ), val=0)
            r_stab = self.create_output('r', shape=(num_nodes * 13, ), val=0)
            phi_stab = self.create_output('phi', shape=(num_nodes * 13, ), val=0)
            theta_stab = self.create_output('theta', shape=(num_nodes * 13, ), val=0)
            psi_stab = self.create_output('psi', shape=(num_nodes * 13, ), val=0)
            x_stab = self.create_output('x', shape=(num_nodes * 13, ), val=0)
            y_stab = self.create_output('y', shape=(num_nodes * 13, ), val=0)
            z_stab = self.create_output('z', shape=(num_nodes * 13, ), val=0)
            
            u_stab[0] = u
            u_stab[1] = u + 0.25
            u_stab[2:] = csdl.expand(u, shape=(11, ))

            v_stab[0:2] = csdl.expand(v, shape=(2, ))
            v_stab[2] = v + 0.25
            v_stab[3:] = csdl.expand(v, shape=(10, ))

            w_stab[0:3] = csdl.expand(var=w, shape=(3, ))
            w_stab[3] = w + 0.25
            w_stab[4:] = csdl.expand(var=w, shape=(9, ))

            p_stab[0:4] = csdl.expand(var=p, shape=(4, ))
            p_stab[4] = p + np.deg2rad(0.5)
            p_stab[5:] = csdl.expand(var=p, shape=(8, ))

            q_stab[0:5] = csdl.expand(var=q, shape=(5, ))
            q_stab[5] = q + np.deg2rad(0.5)
            q_stab[6:] = csdl.expand(var=q, shape=(7, ))

            r_stab[0:6] = csdl.expand(var=r, shape=(6, ))
            r_stab[6] = r + np.deg2rad(0.5)
            r_stab[7:] = csdl.expand(var=r, shape=(6, ))

            phi_stab[0:7] = csdl.expand(var=phi, shape=(7, ))
            phi_stab[7] =  phi + np.deg2rad(0.5)
            phi_stab[8:] = csdl.expand(var=phi, shape=(5, ))

            theta_stab[0:8] = csdl.expand(var=theta, shape=(8, ))
            theta_stab[8] =  theta + np.deg2rad(0.5)
            theta_stab[9:] = csdl.expand(var=theta, shape=(4, ))

            psi_stab[0:9] = csdl.expand(var=psi, shape=(9, ))
            psi_stab[9] =  psi + np.deg2rad(0.5)
            psi_stab[10:] = csdl.expand(var=psi, shape=(3, ))

            x_stab[0:10] = csdl.expand(var=x, shape=(10, ))
            x_stab[10] = x + 0.25
            x_stab[11:] = csdl.expand(var=x, shape=(2, ))

            y_stab[0:11] = csdl.expand(var=y, shape=(11, ))
            y_stab[11] = y + 0.25
            y_stab[12] = y

            z_stab[0:12] = csdl.expand(var=z, shape=(12, ))
            z_stab[12] = z + 0.25


            self.register_output('gamma', gamma * 1)

        else: 
            self.register_output('u', u)
            self.register_output('v', v)
            self.register_output('w', w)

            self.register_output('p', p)
            self.register_output('q', q)
            self.register_output('r', r)

            self.register_output('phi', phi * 1)
            self.register_output('gamma', gamma * 1)
            self.register_output('psi', psi * 1)
            self.register_output('theta', theta * 1)

            self.register_output('x', x * 1)
            self.register_output('y', y * 1)
            self.register_output('z', z * 1)


        # stability_um_nodes = num_nodes + num_nodes * 8

        return


class HoverConditionCSDL(SteadyDesignConditionCSDL):
    def initialize(self):
        self.parameters.declare('hover_condition', types=HoverCondition)

    def define(self):
        hover_condition = self.parameters['hover_condition']
        num_nodes = hover_condition.num_nodes

        h = self.declare_variable('altitude', shape=(num_nodes))
        t = self.declare_variable(f'hover_time', shape=(num_nodes, ))

        atmosphere_model = hover_condition.atmosphere_model(name=f'atmosphere_model')
        atmosphere_model.num_nodes = num_nodes
        atmosphere_model_csdl = atmosphere_model.compute()
        self.add(atmosphere_model_csdl, 'atmosphere_model')


        x = h * 0
        y = h * 0
        z = h * 0

        # NOTE: still need to register the 12 aircraft states but all except location should be zero
        self.register_output('u', x * 0)
        self.register_output('v', x * 0)
        self.register_output('w', x * 0)

        self.register_output('p', x * 0)
        self.register_output('q', x * 0)
        self.register_output('r', x * 0)

        self.register_output('phi', x * 0)
        self.register_output('gamma', x * 0)
        self.register_output('psi',  x * 0)
        self.register_output('theta', x * 0)

        self.register_output('x', x * 1)
        self.register_output('y', y * 1)
        self.register_output('z', z * 1)

        self.register_output('time', t * 1.)
        return


class ClimbConditionCSDL(SteadyDesignConditionCSDL):
    def initialize(self):
        self.parameters.declare('climb_condition', types=ClimbCondition)

    def define(self):
        climb_condition = self.parameters['climb_condition']
        num_nodes = climb_condition.num_nodes
        arguments = climb_condition.arguments

        mach_number_m3l = climb_condition.arguments['mach_number']
        flight_path_angle_m3l = climb_condition.arguments['flight_path_angle']
        climb_gradient_m3l =  climb_condition.arguments['climb_gradient']
        climb_speed_m3l =  climb_condition.arguments['climb_speed']
        climb_time_m3l = climb_condition.arguments['climb_time']

        ih = self.declare_variable('initial_altitude', shape=(num_nodes,))
        fh = self.declare_variable('final_altitude', shape=(num_nodes,))
        theta = self.declare_variable('pitch_angle', shape=(num_nodes,))

        self.register_output('altitude', (ih + fh) / 2)

        atmosphere_model = climb_condition.atmosphere_model(name=f'atmosphere_model')
        atmosphere_model.num_nodes = num_nodes
        atmosphere_model_csdl = atmosphere_model.compute()
        self.add(atmosphere_model_csdl, 'atmosphere_model')
        
        if all([climb_gradient_m3l, flight_path_angle_m3l]):
            gamma = self.declare_variable('flight_path_angle', shape=(num_nodes, ))
            cg = self.declare_variable('climb_gradient', shape=(num_nodes, ))
            a = self.declare_variable('speed_of_sound', shape=(num_nodes,))

            V = cg / csdl.sin(gamma)
            M = V / a
            self.register_output('climb_speed', V)
            self.register_output('mach_number', M)
        
        elif all([mach_number_m3l, climb_time_m3l]):
            a = self.declare_variable('speed_of_sound', shape=(num_nodes,))
            M = self.declare_variable('mach_number', shape=(num_nodes,))
            t = self.declare_variable('climb_time', shape=(num_nodes,))

            V = a * M
            total_distance_traveled = V * t
            vertical_distance_gained = fh - ih
            cg = vertical_distance_gained / t
            gamma = csdl.arcsin(vertical_distance_gained / total_distance_traveled)
            self.register_output('climb_speed', V)
            self.register_output('flight_path_angle', gamma)
            self.register_output('climb_gradient', cg)
        
        elif all([climb_speed_m3l, climb_time_m3l]):
            a = self.declare_variable('speed_of_sound', shape=(num_nodes,))
            V = self.declare_variable('climb_speed', shape=(num_nodes, ))
            t = self.declare_variable('climb_time', shape=(num_nodes,))


            M = V / a
            total_distance_traveled = V * t
            vertical_distance_gained = fh - ih
            cg = vertical_distance_gained / t
            gamma = csdl.arcsin(vertical_distance_gained / total_distance_traveled)
            self.register_output('flight_path_angle', gamma)
            self.register_output('climb_gradient', cg)
            self.register_output('mach_number', M)

        elif all([mach_number_m3l, flight_path_angle_m3l]):
            a = self.declare_variable('speed_of_sound', shape=(num_nodes,))
            M = self.declare_variable('mach_number', shape=(num_nodes,))
            gamma = self.declare_variable('flight_path_angle', shape=(num_nodes, ))

            V = a * M
            cg = V*csdl.sin(gamma) + 1e-4
            t = ((fh - ih) / cg)
            self.register_output('climb_speed', V)
            self.register_output('climb_time', t)

        elif all([climb_speed_m3l, flight_path_angle_m3l]):
            V = self.declare_variable('climb_speed', shape=(num_nodes, ))
            a = self.declare_variable('speed_of_sound', shape=(num_nodes,))
            gamma = self.declare_variable('flight_path_angle', shape=(num_nodes, ))

            M = V / a
            cg = V*csdl.sin(gamma) + 1e-4
            t = ((fh - ih) / cg)
            self.register_output('climb_speed', V)
            self.register_output('climb_time', t)
            self.register_output('mach_number', M)
        else:
            raise NotImplementedError

        # h = self.declare_variable(f'{climb_name}_altitude', shape=(1, ))

        # Compute aircraft states
        phi = theta * 0
        psi = theta * 0
        psi_w = theta * 0

        alfa = theta - gamma
        beta = psi + psi_w
        u = V * csdl.cos(alfa) * csdl.cos(beta)
        v = V * csdl.sin(beta)
        w = -V * csdl.sin(alfa) * csdl.cos(beta)
        p = u * 0
        q = u * 0
        r = u * 0
        x = u * 0
        y = u * 0
        z = u * 0

        # NOTE: below, we don't need to pre_pend the aircraft condition name any more since the vectorization will be handled by m3l
        self.register_output('u', u)
        self.register_output('v', v)
        self.register_output('w', w)

        self.register_output('p', p)
        self.register_output('q', q)
        self.register_output('r', r)

        self.register_output('phi', phi * 1)
        self.register_output('gamma', gamma * 1)
        self.register_output('psi', psi * 1)
        self.register_output('theta', theta * 1)

        self.register_output('x', x * 1)
        self.register_output('y', y * 1)
        self.register_output('z', z * 1)

        self.register_output('time', t * 1.)
        return


