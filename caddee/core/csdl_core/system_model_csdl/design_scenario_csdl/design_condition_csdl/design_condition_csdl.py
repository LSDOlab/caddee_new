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
        # ac_module = self.module # self.parameters['aircraft_condition_module']
        # cruise_name = self.prepend
        print('\n')
        print('inputs', cruise_condition.arguments)
        print('num_nodes', num_nodes)

        # Required variables (user needs to provide these)
        # theta = self.register_module_input(f'{cruise_name}_pitch_angle', shape=(1, ), computed_upstream=False)
        # h = self.register_module_input(f'{cruise_name}_altitude', shape=(1, ), computed_upstream=False)
        # observer_location = self.register_module_input(f'{cruise_name}_observer_location', shape=(3, ), computed_upstream=False)

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
            
            self.register_module_output(f'cruise_speed', cruise_speed)
        
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

        # if set(['range', 'time']).issubset(ac_module.inputs):
        #     range = self.register_module_input(f'{cruise_name}_range', shape=(1, ), computed_upstream=False)
        #     time = self.register_module_input(f'{cruise_name}_time', shape=(1, ), computed_upstream=False)
        #     speed = range/time
        #     self.register_module_output(f'{cruise_name}_speed', speed)
        #     raise Exception("This part of if-else has not been tested")
        # elif set(['mach_number', 'time']).issubset(ac_module.inputs):
        #     a = self.register_module_input(f'{cruise_name}_speed_of_sound', shape=(1, ))
        #     time = self.register_module_input(f'{cruise_name}_time', shape=(1, ), computed_upstream=False)
        #     M = self.register_module_input(f'{cruise_name}_mach_number', shape=(1, ), computed_upstream=False)
        #     speed = a * M
        #     range = speed * time
        #     self.register_module_output(f'{cruise_name}_range', range)
        #     self.register_module_output(f'{cruise_name}_speed', speed)
        #     raise Exception("This part of if-else has not been tested")
        # elif set(['mach_number', 'range']).issubset(ac_module.inputs):
        #     a = self.register_module_input(f'{cruise_name}_speed_of_sound', shape=(1, ))
        #     range = self.register_module_input(f'{cruise_name}_range', shape=(1, ), computed_upstream=False)
        #     M = self.register_module_input(f'{cruise_name}_mach_number', shape=(1, ), computed_upstream=False)
        #     speed = a * M
        #     time = range / speed
        #     self.register_module_output(f'{cruise_name}_time', time)
        #     self.register_module_output(f'{cruise_name}_speed', speed)
        # elif set(['speed', 'time']).issubset(ac_module.inputs):
        #     speed = self.register_module_input(f'{cruise_name}_speed', shape=(1, ), computed_upstream=False)
        #     time = self.register_module_input(f'{cruise_name}_time', shape=(1, ), computed_upstream=False)
        #     range = speed * time
        #     self.register_module_output(f'{cruise_name}_range', range)
        #     raise Exception("This part of if-else has not been tested")
        # elif set(['speed', 'range']).issubset(ac_module.inputs):
        #     speed = self.register_module_input(f'{cruise_name}_speed', shape=(1, ), computed_upstream=False)
        #     range = self.register_module_input(f'{cruise_name}_range', shape=(1, ), computed_upstream=False)
        #     time = range / speed 
        #     self.register_module_output(f'{cruise_name}_time', time)
        #     raise Exception("This part of if-else has not been tested")
        # else:
        #     raise Exception(f"Not enough information to determine 'speed', 'range', and 'time' for design condition '{cruise_name}'. Please specify either ('speed', 'range'), ('speed', 'time'), ('mach_number', 'range'), ('mach_number', 'time'), or ('range', 'time').")
        
        
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
        hover_module = self.module 
        hover_name = self.prepend

        if not set(hover_module.inputs).issubset(set(['altitude', 'hover_time', 'observer_location'])):
            raise Exception("The only allowed inputs are altitude, hover_time, and observer_location")

        h = self.register_module_input(f'{hover_name}_altitude', shape=(1, ), computed_upstream=False)
        t = self.register_module_input(f'{hover_name}_hover_time', shape=(1, ), computed_upstream=False)
        obs_loc = self.register_module_input(f'{hover_name}_observer_location', shape=(3, ), computed_upstream=False)

        self.register_module_output(f'{hover_name}_time', t * 1.)

        if hover_condition.atmosphere_model:
            self.add_module(hover_condition.atmosphere_model._assemble_csdl(hover_name), 'atmosphere_model')

        x = obs_loc[0]
        y = obs_loc[1]
        z = obs_loc[2]

        # NOTE: still need to register the 12 aircraft states but all except location should be zero
        self.register_module_output('u', x * 0)
        self.register_module_output('v', x * 0)
        self.register_module_output('w', x * 0)

        self.register_module_output('p', x * 0)
        self.register_module_output('q', x * 0)
        self.register_module_output('r', x * 0)

        self.register_module_output('phi', x * 0)
        self.register_module_output('gamma', x * 0)
        self.register_module_output('psi',  x * 0)
        self.register_module_output('theta', x * 0)

        self.register_module_output('x', x * 1)
        self.register_module_output('y', y * 1)
        self.register_module_output('z', z * 1)

        self.register_module_output('time', t * 1.)
        return


class ClimbConditionCSDL(SteadyDesignConditionCSDL):
    def initialize(self):
        self.parameters.declare('climb_condition', types=ClimbCondition)

    def define(self):
        climb_condition = self.parameters['climb_condition']
        climb_module = self.module 
        climb_name = self.prepend

        if not set(climb_module.inputs).issubset(set(['initial_altitude', 'final_altitude',
                                                      'altitude', 'mach_number',
                                                      'speed', 'time', 'climb_gradient',
                                                      'pitch_angle', 'flight_path_angle',
                                                      'observer_location'])):
            raise Exception("Provided an input that's not acceptable")

        if set(['initial_altitude', 'final_altitude']).issubset(climb_module.inputs):
            ih = self.register_module_input(f'{climb_name}_initial_altitude', shape=(1,), computed_upstream=False)
            fh = self.register_module_input(f'{climb_name}_final_altitude', shape=(1,), computed_upstream=False)
            self.register_module_output(f'{climb_name}_altitude', (ih + fh) / 2)

        if climb_condition.atmosphere_model:
            self.add_module(climb_condition.atmosphere_model._assemble_csdl(climb_name), 'atmosphere_model')
        
        if set(['climb_gradient', 'gamma', 'pitch_angle']).issubset(climb_module.inputs):
            gamma = self.register_module_input(f'{climb_name}_flight_path_angle', shape=(1, ), computed_upstream=False)
            cg = self.register_module_input(f'{climb_name}_climb_gradient', shape=(1, ), computed_upstream=False)
            theta = self.register_module_input(f'{climb_name}_pitch_angle', shape=(1,), computed_upstream=False)

            V = cg / csdl.sin(gamma)
            self.register_module_output(f'{climb_name}_speed', V)
            raise Exception("This part of if-else has not been tested")
        elif set(['mach_number', 'pitch_angle', 'initial_altitude', 'final_altitude', 'time']).issubset(climb_module.inputs):
            a = self.register_module_input(f'{climb_name}_speed_of_sound', shape=(1,))
            M = self.register_module_input(f'{climb_name}_mach_number', shape=(1,), computed_upstream=False)
            theta = self.register_module_input(f'{climb_name}_pitch_angle', shape=(1,), computed_upstream=False)
            t = self.register_module_input(f'{climb_name}_time', shape=(1,), computed_upstream=False)

            V = a * M
            total_distance_traveled = V * t
            vertical_distance_gained = fh - ih
            gamma = csdl.arcsin(vertical_distance_gained / total_distance_traveled)
            self.register_module_output(f'{climb_name}_speed', V)
            self.register_module_output(f'{climb_name}_flight_path_angle', gamma)
            raise Exception("This part of if-else has not been tested")
        elif set(['mach_number', 'pitch_angle', 'initial_altitude', 'final_altitude', 'flight_path_angle']).issubset(climb_module.inputs):
            a = self.register_module_input(f'{climb_name}_speed_of_sound', shape=(1,))

            M = self.register_module_input(f'{climb_name}_mach_number', shape=(1,), computed_upstream=False)
            theta = self.register_module_input(f'{climb_name}_pitch_angle', shape=(1,), computed_upstream=False)
            gamma = self.register_module_input(f'{climb_name}_flight_path_angle', shape=(1,), computed_upstream=False)

            V = a * M
            cg = V*csdl.sin(gamma) + 1e-4
            t = ((fh - ih) / cg)
            self.register_module_output(f'{climb_name}_speed', V)
            self.register_module_output(f'{climb_name}_time', t)
        else:
            raise NotImplementedError

        observer_location = self.register_module_input(f'{climb_name}_observer_location', shape=(3, ), computed_upstream=False)
        # h = self.register_module_input(f'{climb_name}_altitude', shape=(1, ), computed_upstream=False)

        # Compute aircraft states
        phi = observer_location[2] * 0
        psi = observer_location[2] * 0
        psi_w = observer_location[2] * 0

        alfa = theta - gamma
        beta = psi + psi_w
        u = V * csdl.cos(alfa) * csdl.cos(beta)
        v = V * csdl.sin(beta)
        w = V * csdl.sin(alfa) * csdl.cos(beta)
        p = u * 0
        q = u * 0
        r = u * 0
        x = observer_location[0]
        y = observer_location[1]
        z = observer_location[2]

        # NOTE: below, we don't need to pre_pend the aircraft condition name any more since the vectorization will be handled by m3l
        self.register_module_output('u', u)
        self.register_module_output('v', v)
        self.register_module_output('w', w)

        self.register_module_output('p', p)
        self.register_module_output('q', q)
        self.register_module_output('r', r)

        self.register_module_output('phi', phi * 1)
        self.register_module_output('gamma', gamma * 1)
        self.register_module_output('psi', psi * 1)
        self.register_module_output('theta', theta * 1)

        self.register_module_output('x', x * 1)
        self.register_module_output('y', y * 1)
        self.register_module_output('z', z * 1)

        self.register_module_output('time', t * 1.)
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


