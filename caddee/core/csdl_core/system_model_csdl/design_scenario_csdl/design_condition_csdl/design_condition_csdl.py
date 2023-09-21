import csdl
from caddee.utils.base_model_csdl import BaseModelCSDL
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
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
        observer_location = self.declare_variable('observer_location', shape=(3, num_nodes))

        atmosphere_model = cruise_condition.atmosphere_model(name=f'atmosphere_model')
        atmosphere_model.num_nodes = num_nodes
        atmosphere_model_csdl = atmosphere_model.compute()
        self.add(atmosphere_model_csdl, 'atmosphere_model')

        speed_of_sound = self.declare_variable('speed_of_sound', shape=(num_nodes, ))

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
        w = cruise_speed * csdl.sin(alfa) * csdl.cos(beta)
        p = u * 0
        q = u * 0
        r = u * 0
        x = observer_location[0, :]
        y = observer_location[1, :]
        z = observer_location[2, :]

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

        self.register_output('time', cruise_time * 1)
        return


class HoverConditionCSDL(SteadyDesignConditionCSDL):
    def initialize(self):
        self.parameters.declare('hover_condition', types=HoverCondition)

    def define(self):
        hover_condition = self.parameters['hover_condition']
        num_nodes = hover_condition.num_nodes

        h = self.declare_variable('altitude', shape=(num_nodes))
        observer_location = self.declare_variable('observer_location', shape=(3, num_nodes))
        t = self.declare_variable(f'hover_time', shape=(num_nodes, ))

        atmosphere_model = hover_condition.atmosphere_model(name=f'atmosphere_model')
        atmosphere_model.num_nodes = num_nodes
        atmosphere_model_csdl = atmosphere_model.compute()
        self.add(atmosphere_model_csdl, 'atmosphere_model')


        x = observer_location[0, :]
        y = observer_location[1, :]
        z = observer_location[2, :]

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

        mach_number_m3l = self.arguments['mach_number']
        flight_path_anlge_m3l = arguments['flight_path_anlge']
        climb_gradient_m3l =  arguments['climb_gradient']
        climb_speed_m3l =  arguments['climb_speed']
        climb_time_m3l = arguments['climb_time']

        ih = self.declare_variable('initial_altitude', shape=(num_nodes,))
        fh = self.declare_variable('final_altitude', shape=(num_nodes,))
        theta = self.declare_variable('pitch_angle', shape=(num_nodes,))

        self.register_output('altitude', (ih + fh) / 2)

        atmosphere_model = climb_condition.atmosphere_model(name=f'atmosphere_model')
        atmosphere_model.num_nodes = num_nodes
        atmosphere_model_csdl = atmosphere_model.compute()
        self.add(atmosphere_model_csdl, 'atmosphere_model')
        
        if all([climb_gradient_m3l, flight_path_anlge_m3l]):
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

        elif all([mach_number_m3l, flight_path_anlge_m3l]):
            a = self.declare_variable('speed_of_sound', shape=(num_nodes,))
            M = self.declare_variable('mach_number', shape=(num_nodes,))
            gamma = self.declare_variable('flight_path_angle', shape=(num_nodes, ))

            V = a * M
            cg = V*csdl.sin(gamma) + 1e-4
            t = ((fh - ih) / cg)
            self.register_output('climb_speed', V)
            self.register_output('climb_time', t)

        elif all([climb_speed_m3l, flight_path_anlge_m3l]):
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

        observer_location = self.declare_variable('observer_location', shape=(3, num_nodes))
        # h = self.register_module_input(f'{climb_name}_altitude', shape=(1, ))

        # Compute aircraft states
        phi = theta * 0
        psi = theta * 0
        psi_w = theta * 0

        alfa = theta - gamma
        beta = psi + psi_w
        u = V * csdl.cos(alfa) * csdl.cos(beta)
        v = V * csdl.sin(beta)
        w = V * csdl.sin(alfa) * csdl.cos(beta)
        p = u * 0
        q = u * 0
        r = u * 0
        x = observer_location[0, :]
        y = observer_location[1, :]
        z = observer_location[2, :]

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

        
        
        


if __name__ == '__main__':
    from lsdo_modules.module.module import Module
    from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
    from csdl import GraphRepresentation
    from python_csdl_backend import Simulator
    from caddee.utils.dummy_solvers.dummy_bem import DummyBEMCSDL, BEMDummyMesh

    class DummyBEMCSDL(ModuleCSDL):
        def initialize(self):
            self.parameters.declare('num_nodes')
            self.parameters.declare('num_blades')

        def define(self):
            num_nodes = self.parameters['num_nodes']
            num_blades = self.parameters['num_blades']

            # Aircraft states
            u = self.register_module_input('u', shape=(num_nodes, ))
            v = self.register_module_input('v', shape=(num_nodes, ))
            w = self.register_module_input('w', shape=(num_nodes, ))
            p = self.register_module_input('p', shape=(num_nodes, ))
            self.register_output('p_test', p * 1)
            q = self.register_module_input('q', shape=(num_nodes, ))
            self.register_output('q_test', q * 1)
            r = self.register_module_input('r', shape=(num_nodes, ))
            self.register_output('r_test', r * 1)
            phi = self.register_module_input('phi', shape=(num_nodes, ))
            self.register_output('phi_test', phi * 1)
            theta = self.register_module_input('theta', shape=(num_nodes, ))
            self.register_output('theta_test', theta * 1)
            psi = self.register_module_input('psi', shape=(num_nodes, ))
            self.register_output('psi_test', psi * 1)
            x = self.register_module_input('x', shape=(num_nodes, ))
            self.register_output('x_test', x * 1)
            y = self.register_module_input('y', shape=(num_nodes, ))
            self.register_output('y_test', y * 1)
            z = self.register_module_input('z', shape=(num_nodes, ))
            self.register_output('z_test', z * 1)

            # BEM-specific variables
            rpm = self.register_module_input('rpm', shape=(num_nodes, ), computed_upstream=False)
            self.print_var(rpm)
            self.print_var(u)
            R = csdl.expand(self.register_module_input('radius', shape=(1, )), (num_nodes, ))
            self.print_var(R)
            # NOTE: prefix only for mesh-like variables

            # Some dummy computations for thrust and torque
            angular_speed = (rpm / 60) * 2 * np.pi
            V_tip = R * angular_speed
            u_theta = 0.5 * V_tip
            V_x = (u**2 + v**2 + w**2)**0.5
            u_x = 1.3 * V_x

            dT = T = 4 * np.pi * R * u_x * (u_x - V_x) * num_blades
            dQ = Q = 2 * np.pi * R * u_x * u_theta * num_blades

            self.register_module_output('dT', dT*1)
            self.register_module_output('dQ', dQ*1)

            # T = csdl.sum(dT, axes = (1,)) / shape[2]
            # Q = csdl.sum(dQ, axes = (1,)) / shape[2]

            self.register_module_output('T', T)
            self.register_module_output('Q', Q)

            self.register_module_output('F', csdl.expand(T*2, (num_nodes, 3), 'i->ij'))
            self.register_module_output('M', csdl.expand(Q*2, (num_nodes, 3), 'i->ij'))

    class DummyBEM(Module):
        def _assemble_csdl(self): 
            csdl_model = DummyBEMCSDL(
                module=self,
                num_nodes=1, 
                num_blades=3,
                name='BEM'
            )
            # csdl_model.define()
            GraphRepresentation(csdl_model)
            return csdl_model


    class ACModule(Module):
        def __init__(self) -> None:
            self.models = []
            super().__init__()
        
        def add_model(self, model):
            # self.models.append(model._assemble_csdl())
            self.models.append(model)

        def _assemble_csdl(self):
            csdl_model = AircraftConditionCSDL(
                module=self,
                name='test_ac_condition',
                prefix='cruise',
            )
            GraphRepresentation(csdl_model)
            return csdl_model

    aircraft_condition = ACModule()

    aircraft_condition.set_module_input('range', 60000)
    aircraft_condition.set_module_input('time', 1800)
    aircraft_condition.set_module_input('roll_angle', 0)
    aircraft_condition.set_module_input('pitch_angle', 0.1)
    aircraft_condition.set_module_input('yaw_angle', 0.1)
    aircraft_condition.set_module_input('flight_path_angle', 0.1)
    aircraft_condition.set_module_input('wind_angle', 0.1)
    aircraft_condition.set_module_input('observer_location', np.array([0, 0, 0]))
    
    dummy_bem = DummyBEM()
    dummy_bem.set_module_input('rpm', 1200)

    aircraft_condition.add_model(dummy_bem)

    csdl_model = aircraft_condition._assemble_csdl()
    # print(csdl_model.module_inputs)
    # print(csdl_model.module_outputs)
    
    sim = Simulator(csdl_model)
    sim.run()


