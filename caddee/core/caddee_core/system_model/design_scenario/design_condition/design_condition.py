from caddee.utils.caddee_base import CADDEEBase
import numpy as np
from csdl import GraphRepresentation
import m3l
from dataclasses import dataclass
from typing import Union


@dataclass
class AcStates:
    """
    Container data class for aircraft states and time (for steady cases only)
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
        # Each design condition needs a name 
        # self.parameters.declare(name='name', default='', types=str)
        super().initialize(kwargs=kwargs)

        self.atmosphere_model = None
        self.sub_conditions = dict()
        self.m3l_models = dict()

        self.num_nodes = 1

        # Parameters
        self.parameters.declare(name='stability_flag', default=False, types=bool)
        self.parameters.declare(name='dynamic_flag', default=False, types=bool)


    def add_m3l_model(self, name, model):
        from m3l import Model
        if not isinstance(model, Model):
            raise TypeError("model_group must be of type 'm3l.Model' ")
        else:
            self.m3l_models[name] = model
        return

    def evaluate(self, mach_number, pitch_angle, altitude, range, observer_location) -> Union[AcStates, AtmosphericProperties]:
        """
        Returns a data class
        """

        self.arguments = {}
        # self.name = f"{self.parameters['name']}_ac_states_operation"

        self.arguments['mach_number'] = mach_number
        self.arguments['pitch_angle'] = pitch_angle
        self.arguments['altitude'] = altitude
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

        # ac_states = {
        #     'u' : u,
        #     'v' : v,
        #     'w' : w,
        #     'p' : p,
        #     'q' : q,
        #     'r' : r,
        #     'phi' : phi,
        #     'gamma' : gamma,
        #     'psi' : psi,
        #     'theta' : theta,
        #     'x' : x,
        #     'y' : y,
        #     'z' : z,
        #     'time' : t,
        # }

        ac_states = AcStates()
        ac_states.u = u
        ac_states.v = v
        ac_states.w = w
        
        ac_states.phi = phi
        ac_states.gamma = gamma
        ac_states.psi = psi
        ac_states.theta = theta
        
        ac_states.p = p
        ac_states.q = q
        ac_states.r = r

        ac_states.x = x
        ac_states.y = y
        ac_states.z = z
        
        ac_states.time = t


        rho = m3l.Variable(name='density', shape=(self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(self.num_nodes, ), operation=self)

        atmosphere = AtmosphericProperties()
        atmosphere.density = rho
        atmosphere.dynamic_viscosity = mu
        atmosphere.pressure = pressure
        atmosphere.speed_of_sound = a 
        atmosphere.temperature = temp
        
        return ac_states, atmosphere
    
    def _assemble_csdl(self):
        if len(self.m3l_models) > 1:
            raise Exception(f"More than one m3l model added to design condition {self.parameters['name']}")
        else:
            for m3l_model_name, m3l_model in self.m3l_models.items():
                csdl_model = m3l_model.assemble_csdl()

        return csdl_model
    



class CruiseCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended to define cruise mission segmenst of air vehicles.
    
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
    def initialize(self, kwargs):
        return super().initialize(kwargs)

    def compute(self): 
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import CruiseConditionCSDL
        csdl_model = CruiseConditionCSDL(
            module=self,
            prepend=self.parameters['name'],
            cruise_condition=self,
        )
        return csdl_model
    

class HoverCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended for hover segments.

    Acceptable inputs
    ---
        - altitude : altitude at hover
        - hover_time : duration of hover
        - observer_location : x, y, z location of aircraft; z can be different from altitude

    """
    def initialize(self, kwargs):
        return super().initialize(kwargs)    

    def compute(self): 
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import HoverConditionCSDL
        csdl_model = HoverConditionCSDL(
            module=self,
            prepend=self.parameters['name'],
            hover_condition=self,
        ) 

        return csdl_model

    
class ClimbCondition(SteadyDesignCondition):
    """
    Sublcass of SteadyDesignCondition intended for climb segments.

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
    def initialize(self, kwargs):
        return super().initialize(kwargs)

    def compute(self):
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import ClimbConditionCSDL
        csdl_model = ClimbConditionCSDL(
            module=self,
            prepend=self.parameters['name'],
            climb_condition=self,
        )
        return csdl_model
    
class VectorizedDesignCondition(SteadyDesignCondition):
    """
    Sublcass of SteadyDesignCondition intended for vectorizing mission segments
    that contain exactly the same m3l model group 
    """
    def initialize(self, kwargs):
        self.num_nodes = 1
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
