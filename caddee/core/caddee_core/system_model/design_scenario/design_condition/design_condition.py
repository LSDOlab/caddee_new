from caddee.utils.caddee_base import CADDEEBase
import numpy as np
from csdl import GraphRepresentation, Model
import m3l
from dataclasses import dataclass
from typing import Union, Tuple
from caddee.core.caddee_core.system_model.design_scenario.design_condition.atmosphere.atmosphere import Atmosphere


@dataclass
class AcStates:
    """
    Container data class for aircraft states and time (time for steady cases only)
    """
    u : m3l.Variable = None
    v : m3l.Variable = None
    w : m3l.Variable = None
    p : m3l.Variable = None
    q : m3l.Variable = None
    r : m3l.Variable = None
    theta : m3l.Variable = None
    phi : m3l.Variable = None
    gamma : m3l.Variable = None
    psi : m3l.Variable = None
    x : m3l.Variable = None
    y : m3l.Variable = None
    z : m3l.Variable = None
    time : m3l.Variable = None

@dataclass
class AtmosphericProperties:
    """
    Container data class for atmospheric variables 
    """
    density : m3l.Variable = None
    temperature : m3l.Variable = None
    pressure : m3l.Variable = None
    dynamic_viscosity : m3l.Variable = None
    speed_of_sound : m3l.Variable = None


class SteadyDesignCondition(m3l.ExplicitOperation):
    """
    Class for steady-state analyses (e.g., steady cruise segment).

    state vector x = [x_d, x_k]
    x_d = [u, v, w, p, q, r]
    x_k = [x, y, z, φ, θ, ψ]
    with
        - (u, v, w) and (p, q, r) the body-axis frame components of aircraft center
        of mass velocity and of the aircraft angular velocity respectively
        - (x, y, z) the evolving coordinates of the center of mass into the chosen
        inertial reference frame. The standard NED (North-East-Down).
        - (φ, θ, ψ) the standard aircraft Euler angles representing the evolving body
        attitude with respect to the inertial reference frame

    Steady-state flight is defined as a condition in which all of the aircraft motion
     variables are constant or zero. That is, the linear and angular velocities are
     constant or zero and all the acceleration components are zero.
    accelerations ⇒ ˙u, ˙v, ˙w (or ˙V , ˙α, ˙β ) ≡ 0 , ˙p, ˙q, ˙r ≡ 0
    linear velocities ⇒ u, v, w ( or V, α, β) = prescribed constant values
    angular velocities ⇒ p, q, r = prescribed constant values
    Parameters:
    ---
        stability_flag : perform static stability analysis if True
    """

    def initialize(self, kwargs):
        self.atmosphere_model = Atmosphere
        
        # self.num_nodes = 1

        # Parameters
        self.parameters.declare(name='stability_flag', default=False, types=bool)
        self.parameters.declare(name='num_nodes', types=int, default=1)
        super().initialize(kwargs=kwargs)

    def assign_attributes(self):
        self.num_nodes = self.parameters['num_nodes']
        self.name = self.parameters['name']

    
    def _assemble_csdl(self):
        if len(self.m3l_models) > 1:
            raise Exception(f"More than one m3l model added to design condition {self.parameters['name']}")
        else:
            for m3l_model_name, m3l_model in self.m3l_models.items():
                csdl_model = m3l_model.assemble_csdl()

        return csdl_model
    



class CruiseCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended to define cruise mission segments of air vehicles.
    
    CADDEE inputs (set by set_module_input()):
    ---
        - range : range of a cruise condition
        - time : time of a cruise condition
        - altitude : altitude at cruise
        - mach_number : aircraft free-stream Mach number  (can't be specified if cruise_speed)
        - cruise_speed : aircraft cruise speed (can't be specified if mach_number)
        - theta : aircraft pitch angle
        - observer_location : x, y, z location of aircraft; z can be different from altitude
    """
    # def initialize(self, kwargs): pass
    #     # return super().initialize(kwargs)

    def compute(self) -> Model:
        """
        Returns a csdl model
        """ 
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import CruiseConditionCSDL
        csdl_model = CruiseConditionCSDL(
            cruise_condition=self,
        )
        return csdl_model
    

    def evaluate(self, mach_number : Union[m3l.Variable, None], pitch_angle : m3l.Variable, altitude : m3l.Variable, cruise_range : Union[m3l.Variable, None], 
                 observer_location : m3l.Variable, time : Union[m3l.Variable, None]=None, cruise_speed=None) -> tuple[AcStates, AtmosphericProperties]:
        """
        Returns a data class with aircraft states and atmospheric properties 
        
        
        Parameters
        ----------
        mach_number : m3l variable, None
            The intended mach number of the cruise condition. Can be zero if other inputs are provided from which mach number can be computed
        
        pitch_angle : m3l variable 
            The aircraft pitch angle (theta)
        
        altitude : m3l variable
            The altitude of the aircraft 
        
        cruise_range : m3l variable, None
            The range of the cruise condition. Can be None if other inputs are provided from which range can be computed
        
        observer_location : m3l Variable
            The observer location from the aircraft (in the Cartesian reference frame). Note that altitude can the z \
                variable (last entry) of the observer location can be different
        
        time : None, m3l Variable - optional (default: None)
            The time of the cruise condition (can't be specified if 'mach_number' and 'cruise_range' are already set)
        
        cruise_speed  : None, m3l Variable - optional (default: None)
            The cruise speed can be specified in place of mach_number 
        
            
        """
        dc_name = self.parameters['name']
        
        # Chck if user inputs are valid
        if all([mach_number, cruise_speed]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'mach_number' and 'cruise_speed' at the same time")
        elif all([mach_number, cruise_range, time]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'mach_number' and 'cruise_range', and 'time' at the same time")
        elif all([cruise_range, time, cruise_speed]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'mach_number' and 'time', and 'cruise_speed' at the same time")
        
        if all([mach_number, cruise_range]):
            pass
        elif all([cruise_range, time]):
            pass
        elif all([cruise_range, cruise_speed]):
            pass
        elif all([mach_number, time]):
            pass
        elif all([cruise_speed, time]):
            pass
        else:
            raise ValueError(f"Design condition '{dc_name}': Not enough information to determine 'speed', 'range', and 'time' for design condition '{dc_name}'. Please specify either ('speed', 'range'), ('speed', 'time'), ('mach_number', 'range'), ('mach_number', 'time'), or ('range', 'time').")


        self.arguments = {}
        # self.name = f"{self.parameters['name']}_ac_states_operation"

        self.arguments['mach_number'] = mach_number
        self.arguments['pitch_angle'] = pitch_angle
        self.arguments['altitude'] = altitude
        self.arguments['cruise_range'] = cruise_range
        self.arguments['observer_location'] = observer_location
        self.arguments['cruise_time'] = time
        self.arguments['cruise_speed'] = cruise_speed


        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=self)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=self)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=self)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=self)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=self)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=self)

        phi = m3l.Variable(name='phi', shape=(self.num_nodes, ), operation=self)
        gamma = m3l.Variable(name='gamma', shape=(self.num_nodes, ), operation=self)
        psi = m3l.Variable(name='psi', shape=(self.num_nodes, ), operation=self)
        theta = m3l.Variable(name='theta', shape=(self.num_nodes, ), operation=self)

        x = m3l.Variable(name='x', shape=(self.num_nodes, ), operation=self)
        y = m3l.Variable(name='y', shape=(self.num_nodes, ), operation=self)
        z = m3l.Variable(name='z', shape=(self.num_nodes, ), operation=self)

        t = m3l.Variable(name='time', shape=(self.num_nodes, ), operation=self)


        ac_states = AcStates(
            u=u,
            v=v,
            w=w,
            phi=phi,
            gamma=gamma,
            psi=psi,
            theta=theta,
            p=p,
            q=q,
            r=r,
            x=x,
            y=y,
            z=z,
            time=t,
        )


        rho = m3l.Variable(name='density', shape=(self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(self.num_nodes, ), operation=self)

        atmosphere = AtmosphericProperties(
            density=rho,
            dynamic_viscosity=mu,
            pressure=pressure,
            speed_of_sound=a,
            temperature=temp,
        )

        
        return ac_states, atmosphere
    

class HoverCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended for hover segments.

    Acceptable inputs
    ---
        - altitude : altitude at hover
        - hover_time : duration of hover
        - observer_location : x, y, z location of aircraft; z can be different from altitude

    """
    def evaluate(self, altitude : m3l.Variable, hover_time : m3l.Variable, 
                 observer_location : m3l.Variable) -> tuple[AcStates, AtmosphericProperties]:
        """
        Returns the aircraft states and atmospheric properties for the hover condition
        in the form of two data classes.

        Parameters
        ----------
        altitude : m3l.Variable
            The altitude at which the vehicle is hovering 

        hover_time : m3l.Variable
            The time spent in hover

        observer_location : m3l.Varible
            A vector of size 3 given the x, y, z coordinate in the inertial reference frame. 
            Note that z and altitude don't have to have the same value

        """
        
        self.arguments = {}

        self.arguments['altitude'] = altitude
        self.arguments['hover_time'] = hover_time
        self.arguments['observer_location'] = observer_location

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=self)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=self)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=self)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=self)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=self)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=self)

        phi = m3l.Variable(name='phi', shape=(self.num_nodes, ), operation=self)
        gamma = m3l.Variable(name='gamma', shape=(self.num_nodes, ), operation=self)
        psi = m3l.Variable(name='psi', shape=(self.num_nodes, ), operation=self)
        theta = m3l.Variable(name='theta', shape=(self.num_nodes, ), operation=self)

        x = m3l.Variable(name='x', shape=(self.num_nodes, ), operation=self)
        y = m3l.Variable(name='y', shape=(self.num_nodes, ), operation=self)
        z = m3l.Variable(name='z', shape=(self.num_nodes, ), operation=self)

        t = m3l.Variable(name='time', shape=(self.num_nodes, ), operation=self)


        ac_states = AcStates(
            u=u,
            v=v,
            w=w,
            phi=phi,
            gamma=gamma,
            psi=psi,
            theta=theta,
            p=p,
            q=q,
            r=r,
            x=x,
            y=y,
            z=z,
            time=t,
        )


        rho = m3l.Variable(name='density', shape=(self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(self.num_nodes, ), operation=self)

        atmosphere = AtmosphericProperties(
            density=rho,
            dynamic_viscosity=mu,
            pressure=pressure,
            speed_of_sound=a,
            temperature=temp,
        )

        
        return ac_states, atmosphere
        

    def compute(self): 
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import HoverConditionCSDL
        csdl_model = HoverConditionCSDL(
            hover_condition=self,
        ) 

        return csdl_model

    
class ClimbCondition(SteadyDesignCondition):
    """
    Sublcass of SteadyDesignCondition intended for climb or descent segments.

    Acceptable inputs
    ---
        - initial_altitude : initial altitude of a climb condition
        - final_altitude : final altitude of a climb condition
        - altitude : a necessary input for now to get correct atmosisa
        - mach_number : aircraft free-stream Mach number  (can't be specified if speed is specified)
        - speed : aircraft speed during climb (can't be specified if mach_number is specified)
        - time : duration of the climb
        - climb_gradient : vertical distance aircraft covers in m/s
        - pitch_angle: theta; one of the aircraft states
        - flight_path_angle: gamma;
        - observer_location : x, y, z location of aircraft; z can be different from altitude

    """

    def evaluate(self, initial_altitude : m3l.Variable, final_altitude : m3l.Variable, pitch_angle : m3l.Variable,
                 flight_path_anlge : Union[m3l.Variable , None], mach_number : Union[m3l.Variable, None], 
                 observer_location : m3l.Variable, climb_gradient : Union[m3l.Variable, None]=None, 
                 climb_speed : Union[m3l.Variable, None]=None, 
                 climb_time : Union[m3l.Variable, None]=None) -> tuple[AcStates, AtmosphericProperties]:
        """
        Returns the aircraft states and atmospheric properties for the climb condition
        in the form of two data classes. 

        Parameters
        ----------
        initial_altitude : m3l.Variable
            The initial altitude of the climb segement

        final_altitude : m3l.Variable
            The final  altitude of the climb segment. Note that if the final_altitude is lower than the initial altitude, we are defining a descent segment
        
        pitch_angle : m3l.Variable
            The vehicle pitch angle 

        flight_path_angle : m3l.Variable, None
            The flight path angle of the vehicle. Note that it can be specified as 'None' if it can be computed from other inputs. 

        mach_number : m3l.Variable, None
            The mach number for the climb condition. Note that it can be 'None' if it can be computed from other inputs. 
        """
        dc_name = self.parameters['name']

        # Chck if user inputs are valid
        if all([mach_number, climb_speed]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'mach_number' and 'climb_speed' at the same time")
        
        elif all([initial_altitude, final_altitude, climb_time, climb_gradient]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'initial_altitude', 'final_altitude', 'climb_time', and 'climb_gradient' at the same time")

        elif all([flight_path_anlge, climb_speed, climb_gradient]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'flight_path_anlge', 'climb_speed', and 'climb_gradient' at the same time")

        elif all([flight_path_anlge, mach_number, climb_gradient]):
            raise ValueError(f"Design condition {dc_name}: Cannot specify 'flight_path_anlge', 'mach_number', and 'climb_gradient' at the same time")


        self.arguments = {}

        self.arguments['initial_altitude'] = initial_altitude
        self.arguments['final_altitude'] = final_altitude
        self.arguments['pitch_angle'] = pitch_angle
        self.arguments['mach_number'] = mach_number
        self.arguments['flight_path_anlge'] = flight_path_anlge
        self.arguments['observer_location'] = observer_location
        self.arguments['climb_gradient'] = climb_gradient
        self.arguments['climb_speed'] = climb_speed
        self.arguments['climb_time'] = climb_time

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=self)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=self)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=self)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=self)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=self)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=self)

        phi = m3l.Variable(name='phi', shape=(self.num_nodes, ), operation=self)
        gamma = m3l.Variable(name='gamma', shape=(self.num_nodes, ), operation=self)
        psi = m3l.Variable(name='psi', shape=(self.num_nodes, ), operation=self)
        theta = m3l.Variable(name='theta', shape=(self.num_nodes, ), operation=self)

        x = m3l.Variable(name='x', shape=(self.num_nodes, ), operation=self)
        y = m3l.Variable(name='y', shape=(self.num_nodes, ), operation=self)
        z = m3l.Variable(name='z', shape=(self.num_nodes, ), operation=self)

        t = m3l.Variable(name='time', shape=(self.num_nodes, ), operation=self)


        ac_states = AcStates(
            u=u,
            v=v,
            w=w,
            phi=phi,
            gamma=gamma,
            psi=psi,
            theta=theta,
            p=p,
            q=q,
            r=r,
            x=x,
            y=y,
            z=z,
            time=t,
        )


        rho = m3l.Variable(name='density', shape=(self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(self.num_nodes, ), operation=self)

        atmosphere = AtmosphericProperties(
            density=rho,
            dynamic_viscosity=mu,
            pressure=pressure,
            speed_of_sound=a,
            temperature=temp,
        )

        
        return ac_states, atmosphere

    def compute(self):
        """
        Returns the csdl model for the ClimbCondition
        """
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import ClimbConditionCSDL
        csdl_model = ClimbConditionCSDL(
            climb_condition=self,
        )
        return csdl_model
    
class VectorizedDesignCondition(SteadyDesignCondition):
    """
    Sublcass of SteadyDesignCondition intended for vectorizing mission segments
    that contain exactly the same m3l model group 
    """
    def initialize(self, kwargs):
        self.parameters.declare('num_nodes', types=int)
        return super().initialize(kwargs)
    
    
    
    
    
    def add_subcondition(self, subcondition):
        """
        Method for adding steady sub design conditions to a vectorized design conditions.
        The models used must be exactly the same
        """
        name = subcondition.parameters['name']
        self.sub_conditions[name] = subcondition

    def evaluate_ac_states(self):
        self.num_nodes = len(self.sub_conditions)
