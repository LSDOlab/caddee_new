from csdl import Model
import m3l
from dataclasses import dataclass, is_dataclass
from typing import Union, Tuple
from caddee.core.caddee_core.system_model.design_scenario.design_condition.atmosphere.atmosphere import Atmosphere
from caddee.core.caddee_core.system_model.design_scenario.design_condition.linear_stability_analysis import (
    LinearStabilityAnalysis,
    LongitudinalAircraftStability,
    LongitudinalStability,
    LateralAircraftStability,
    LateralStability,
)
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.inertial_loads_csdl import InertialLoads
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.total_forces_moments_csdl import TotalForcesMoments
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.equations_of_motion_csdl.eom_6dof_module import EoMEuler6DOF
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.constant_mass_properties_csdl import TotalMassPropertiesM3L
from caddee.utils.helper_functions.caddee_helper_functions import flatten_list
import warnings


@dataclass
class AcStates:
    """
    Container data class for aircraft states and time (time for steady cases only)
    """
    u: m3l.Variable = None
    v: m3l.Variable = None
    w: m3l.Variable = None
    p: m3l.Variable = None
    q: m3l.Variable = None
    r: m3l.Variable = None
    theta: m3l.Variable = None
    phi: m3l.Variable = None
    gamma: m3l.Variable = None
    psi: m3l.Variable = None
    x: m3l.Variable = None
    y: m3l.Variable = None
    z: m3l.Variable = None
    time: m3l.Variable = None
    stability_flag: bool = False


@dataclass
class AtmosphericProperties:
    """
    Container data class for atmospheric variables 
    """
    density: m3l.Variable = None
    temperature: m3l.Variable = None
    pressure: m3l.Variable = None
    dynamic_viscosity: m3l.Variable = None
    speed_of_sound: m3l.Variable = None


@dataclass
class TrimVariables:
    """
    Container class for trim variables
    """
    accelerations: m3l.Variable = None
    total_forces: m3l.Variable = None
    total_moments: m3l.Variable = None
    inertial_forces: m3l.Variable = None
    inertial_moments: m3l.Variable = None
    long_residual:  m3l.Variable = None
    lat_residual:  m3l.Variable = None
    longitudinal_stability: LongitudinalStability = None
    lateral_stability: LateralStability = None


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
    ----------
    stability_flag : bool
        perform static stability analysis if True

    num_nodes : int
        number of vectorized evaluations of models
    """

    def initialize(self, kwargs):
        self.atmosphere_model = Atmosphere
        # Parameters
        self.parameters.declare(name='stability_flag',
                                default=False, types=bool)
        self.parameters.declare(name='num_nodes', types=int, default=1)
        super().initialize(kwargs=kwargs)

    def assign_attributes(self):
        self.num_nodes = self.parameters['num_nodes']
        self.name = self.parameters['name']

    def assemble_trim_residual(self, mass_properties : list, aero_propulsive_outputs: list,
                               ac_states, ref_pt=None) -> TrimVariables:
        """
        Method that assembles the following m3l models for each design condition
        in the following order:

        1) InertialLoads :  
            this computes the inertial loads about a reference point given the aircraft states, mass properties 

        2) TotalForcesMoments :
            this computes the total loads from all aero-propulsive models and the inertial loads

        3) EoMEuler6DOF :
            this computes the accelerations of the aircraft states based on 6-dof equations of motion model 

        4) LinearStabilityAnalysis :
            this computes the longitudinal and lateral stability derivatives and returns handling qualities

        Parameters
        ----------
        mass_properties : dataclass
            dataclass containing mass properties 

        aero_propulsive_outputs : list
            list of data classes containing the outputs from aeropropulsive solvers

        ac_states : dataclass
            dataclass containing the aircraft states 

        Returns
        -------
        An instance of the TrimVariables dataclass


        """
        stability_flag = self.parameters['stability_flag']
        name = self.parameters['name']

        total_mass_props_model = TotalMassPropertiesM3L(
            name=f"{name}_total_mass_properties_model"
        )
        # print(len(mass_properties))
        total_mass_props = total_mass_props_model.evaluate(component_mass_properties=flatten_list(mass_properties))
        # exit()

        inertial_loads = InertialLoads(
            name=f"{name}_inertial_loads_model",
            num_nodes=self.num_nodes,
        )
        inertial_forces, inertial_moments = inertial_loads.evaluate(
            total_cg_vector=total_mass_props.cg_vector,
            totoal_mass=total_mass_props.mass,
            ac_states=ac_states,
            ref_pt=ref_pt,
            stability=stability_flag,

        )

        # Aero propulsive outputs 
        if not isinstance(aero_propulsive_outputs, list):
            raise TypeError(f"function argument 'aero_propulsive_outputs' must be a list - received {type(model_outputs)}")
        
        aero_prop_outputs_m3l = []
        required_aero_outputs = ['forces', 'moments']
        for model_outputs in aero_propulsive_outputs:
            aero_outputs_keys =  model_outputs.__annotations__.keys()
            available_required_outputs = [item for item in aero_outputs_keys if item in required_aero_outputs]
            if not is_dataclass(model_outputs):
                raise TypeError(f"function argument 'aero_propulsive_outputs' requires a list of data classes containing the outputs of a specific solver. Received entry of type {type(model_outputs)}")
            elif required_aero_outputs != available_required_outputs:
                raise ValueError(f"every aero propulsive solver must output a dataclass that contains at least the attributes 'forces' and 'moments', which are m3l Variables")

            if (model_outputs.forces is None) and (model_outputs.moments is None):
                raise ValueError("'forces' and 'moments' are both 'None'. Only 'Moments' can be None at this moment")
            
            elif not isinstance(model_outputs.forces, m3l.Variable):
                raise ValueError(f"'forces' must be an m3l Variable. Received {type(model_outputs.forces)}")

            if model_outputs.moments is None:
                aero_prop_outputs_m3l.append(model_outputs.forces)
                warnings.warn(f"Operation {model_outputs.forces.operation.name} does not compute moments")
            else:
                aero_prop_outputs_m3l.append(model_outputs.forces)
                aero_prop_outputs_m3l.append(model_outputs.moments)

        all_forces = aero_prop_outputs_m3l + \
            [inertial_forces, inertial_moments]

        # Total forces and moments
        total_forces_moments = TotalForcesMoments(
            name=f"{name}_total_forces_moments_model",
            num_nodes=self.num_nodes,
        )
        total_forces, total_moments = total_forces_moments.evaluate(
            *all_forces,
            stability=stability_flag,
        )

        # Equations of motion 
        eom_model = EoMEuler6DOF(
            name=f"{name}_eom_model",
            num_nodes=self.num_nodes,
        )
        accelerations, lhs_long, long_stab_state_vec, A_long, lhs_lat, lat_stab_state_vec, A_lat = eom_model.evaluate(
            total_mass=total_mass_props.mass,
            total_cg_vector=total_mass_props.cg_vector,
            total_inertia_tensor=total_mass_props.inertia_tensor,
            total_forces=total_forces,
            total_moments=total_moments,
            ac_states=ac_states,
            ref_pt=ref_pt,
            stability=stability_flag,
        )

        trim_variables = TrimVariables(
            accelerations=accelerations,
            total_forces=total_forces,
            total_moments=total_moments,
            inertial_forces=inertial_forces,
            inertial_moments=inertial_moments,
        )

        if stability_flag:
            stability_model = LinearStabilityAnalysis(
                name=f"{name}_linear_stability_model",
            )

            long_residual, lat_residual = stability_model.evaluate(
                lhs_long=lhs_long,
                long_stab_state_vec=long_stab_state_vec,
                A_long=A_long,
                lhs_lat=lhs_lat,
                lat_stab_state_vec=lat_stab_state_vec,
                A_lat=A_lat,
            )

            trim_variables.long_residual = long_residual
            trim_variables.lat_residual = lat_residual

            long_stab_model = LongitudinalAircraftStability(
                design_condition_name=self.parameters['name'],
            )
            long_stab_metrics = long_stab_model.evaluate()

            lat_stab_model = LateralAircraftStability(
                design_condition_name=self.parameters['name'],
            )
            lat_stab_metric = lat_stab_model.evaluate()

            trim_variables.longitudinal_stability = long_stab_metrics
            trim_variables.lateral_stability = lat_stab_metric

        return trim_variables


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

    def evaluate(self, mach_number: Union[m3l.Variable, None], pitch_angle: m3l.Variable, altitude: m3l.Variable, cruise_range: Union[m3l.Variable, None],
                 time: Union[m3l.Variable, None] = None, cruise_speed=None) -> tuple[AcStates, AtmosphericProperties]:
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

        time : None, m3l Variable - optional (default: None)
            The time of the cruise condition (can't be specified if 'mach_number' and 'cruise_range' are already set)

        cruise_speed  : None, m3l Variable - optional (default: None)
            The cruise speed can be specified in place of mach_number 


        Return
        ------
        Instance of the AcStates and AtmosphericProperties dataclasses
        """
        dc_name = self.parameters['name']
        stability_flag = self.parameters['stability_flag']
        # Chck if user inputs are valid
        if all([mach_number, cruise_speed]):
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'mach_number' and 'cruise_speed' at the same time")
        elif all([mach_number, cruise_range, time]):
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'mach_number' and 'cruise_range', and 'time' at the same time")
        elif all([cruise_range, time, cruise_speed]):
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'mach_number' and 'time', and 'cruise_speed' at the same time")

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
            raise ValueError(
                f"Design condition '{dc_name}': Not enough information to determine 'speed', 'range', and 'time' for design condition '{dc_name}'. Please specify either ('speed', 'range'), ('speed', 'time'), ('mach_number', 'range'), ('mach_number', 'time'), or ('range', 'time').")

        self.arguments = {}
        # self.name = f"{self.parameters['name']}_ac_states_operation"

        self.arguments['mach_number'] = mach_number
        self.arguments['pitch_angle'] = pitch_angle
        self.arguments['altitude'] = altitude
        self.arguments['cruise_range'] = cruise_range
        self.arguments['cruise_time'] = time
        self.arguments['cruise_speed'] = cruise_speed

        for var_name, var in self.arguments.items():
            if not var:
                pass
            elif var.shape != (self.num_nodes, ):
                raise ValueError(
                    f"Shape mismatch: variable '{var_name}' of condition '{self.name}' has shape {var.shape} but 'num_nodes' is {self.num_nodes}. Please set 'num_nodes' in the constructor accordingly or make sure that the shape of '{var_name}' matches 'num_nodes'.")
            else:
                pass

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=self)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=self)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=self)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=self)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=self)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=self)

        phi = m3l.Variable(name='phi', shape=(
            self.num_nodes, ), operation=self)
        gamma = m3l.Variable(name='gamma', shape=(
            self.num_nodes, ), operation=self)
        psi = m3l.Variable(name='psi', shape=(
            self.num_nodes, ), operation=self)
        theta = m3l.Variable(name='theta', shape=(
            self.num_nodes, ), operation=self)

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
        stability_flag = self.parameters['stability_flag']
        if stability_flag:
            ac_states.stability_flag = True

        rho = m3l.Variable(name='density', shape=(
            self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(
            self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(
            self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(
            self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(
            self.num_nodes, ), operation=self)

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

    """

    def evaluate(self, altitude: m3l.Variable, hover_time: m3l.Variable) -> tuple[AcStates, AtmosphericProperties]:
        """
        Returns the aircraft states and atmospheric properties for the hover condition
        in the form of two data classes.

        Parameters
        ----------
        altitude : m3l.Variable
            The altitude at which the vehicle is hovering 

        hover_time : m3l.Variable
            The time spent in hover
        """

        self.arguments = {}

        self.arguments['altitude'] = altitude
        self.arguments['hover_time'] = hover_time

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=self)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=self)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=self)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=self)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=self)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=self)

        phi = m3l.Variable(name='phi', shape=(
            self.num_nodes, ), operation=self)
        gamma = m3l.Variable(name='gamma', shape=(
            self.num_nodes, ), operation=self)
        psi = m3l.Variable(name='psi', shape=(
            self.num_nodes, ), operation=self)
        theta = m3l.Variable(name='theta', shape=(
            self.num_nodes, ), operation=self)

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

        rho = m3l.Variable(name='density', shape=(
            self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(
            self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(
            self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(
            self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(
            self.num_nodes, ), operation=self)

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

    """

    def evaluate(self, initial_altitude: m3l.Variable, final_altitude: m3l.Variable, pitch_angle: m3l.Variable,
                 flight_path_anlge: Union[m3l.Variable, None], mach_number: Union[m3l.Variable, None],
                 climb_gradient: Union[m3l.Variable, None] = None,
                 climb_speed: Union[m3l.Variable, None] = None,
                 climb_time: Union[m3l.Variable, None] = None) -> tuple[AcStates, AtmosphericProperties]:
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
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'mach_number' and 'climb_speed' at the same time")

        elif all([initial_altitude, final_altitude, climb_time, climb_gradient]):
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'initial_altitude', 'final_altitude', 'climb_time', and 'climb_gradient' at the same time")

        elif all([flight_path_anlge, climb_speed, climb_gradient]):
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'flight_path_anlge', 'climb_speed', and 'climb_gradient' at the same time")

        elif all([flight_path_anlge, mach_number, climb_gradient]):
            raise ValueError(
                f"Design condition {dc_name}: Cannot specify 'flight_path_anlge', 'mach_number', and 'climb_gradient' at the same time")

        self.arguments = {}

        self.arguments['initial_altitude'] = initial_altitude
        self.arguments['final_altitude'] = final_altitude
        self.arguments['pitch_angle'] = pitch_angle
        self.arguments['mach_number'] = mach_number
        self.arguments['flight_path_anlge'] = flight_path_anlge
        self.arguments['climb_gradient'] = climb_gradient
        self.arguments['climb_speed'] = climb_speed
        self.arguments['climb_time'] = climb_time

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=self)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=self)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=self)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=self)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=self)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=self)

        phi = m3l.Variable(name='phi', shape=(
            self.num_nodes, ), operation=self)
        gamma = m3l.Variable(name='gamma', shape=(
            self.num_nodes, ), operation=self)
        psi = m3l.Variable(name='psi', shape=(
            self.num_nodes, ), operation=self)
        theta = m3l.Variable(name='theta', shape=(
            self.num_nodes, ), operation=self)

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

        rho = m3l.Variable(name='density', shape=(
            self.num_nodes, ), operation=self)
        mu = m3l.Variable(name='dynamic_viscosity', shape=(
            self.num_nodes, ), operation=self)
        pressure = m3l.Variable(name='pressure', shape=(
            self.num_nodes, ), operation=self)

        a = m3l.Variable(name='speed_of_sound', shape=(
            self.num_nodes, ), operation=self)
        temp = m3l.Variable(name='temperature', shape=(
            self.num_nodes, ), operation=self)

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


# @dataclass
# class DampingRatios:
#     damping_long_11 : m3l.Variable
#     damping_long_12 : m3l.Variable
#     damping_long_21 : m3l.Variable
#     damping_long_22 : m3l.Variable

#     damping_lat_11 : m3l.Variable
#     damping_lat_12 : m3l.Variable
#     damping_lat_21 : m3l.Variable
#     damping_lat_22 : m3l.Variable


# class LinearStabilityAnalysis(m3l.ExplicitOperation):
#     def initialize(self, kwargs):
#         self.parameters.declare('design_condition', types=SteadyDesignCondition)
#         self.parameters.declare('name', types=str, default='stability_analysis')
#         self.parameters.declare('nested_connection_name', types=bool, default=False)
#         self.parameters.declare('not_nested_vars', types=list, default=['mass'])


#     # def evaluate(self, ac_states : AcStates, cg_vector : m3l.Variable, vehicle_mass : m3l.Variable, forces:list, moments:list, solvers : list=[]) -> DampingRatios:
#     def evaluate(self, vehicle_mass : m3l.Variable, aerodynamic_outputs : list) -> DampingRatios:
#         # NOTE: only providing forces should be enough but may providing both is needed for making connections
#         self.parameters['nested_connection_name'] = True
#         self.arguments = {}

#         self.arguments['vehicle_mass'] = vehicle_mass

#         for aero_output in aerodynamic_outputs:
#             F = aero_output.forces
#             M = aero_output.moments
#             F_perturbed = aero_output.forces_perturbed
#             M_perturbed = aero_output.moments_perturbed

#             self.arguments[f"{F.operation.name}_{F.name}"] = F
#             self.arguments[f"{M.operation.name}_{M.name}"] = M
#             self.arguments[f"{F_perturbed.operation.name}.{F_perturbed.operation.name}.{F_perturbed.name}"] = F_perturbed
#             self.arguments[f"{M_perturbed.operation.name}.{M_perturbed.operation.name}.{M_perturbed.name}"] = M_perturbed


#         self._aero_outputs = aerodynamic_outputs

#         A_long = m3l.Variable(name='A_long', shape=(4, 4), operation=self)
#         A_lat = m3l.Variable(name='A_lat', shape=(4, 4), operation=self)

#         # damping_ratios = DampingRatios(
#         #     damping_long_11=u,
#         #     damping_long_12=v,
#         #     damping_long_21=w,
#         #     damping_long_22=p,
#         #     damping_lat_11=q,
#         #     damping_lat_12=r,
#         #     damping_lat_21=phi,
#         #     damping_lat_22=theta,
#         # )

#         return A_long , A_lat #damping_ratios

#     def compute(self):
#         design_condition = self.parameters['design_condition']
#         csdl_model = LinearStabilityCSDL(
#             design_condition=design_condition,
#             aerodynamic_outputs=self._aero_outputs,
#         )

#         return csdl_model


# class LinearStabilityCSDL(Model):
#     def initialize(self):
#         self.parameters.declare('design_condition', types=SteadyDesignCondition)
#         self.parameters.declare('aerodynamic_outputs')

#     def define(self):
#         design_condition = self.parameters['design_condition']
#         aero_outputs = self.parameters['aerodynamic_outputs']

#         mass = csdl.expand(self.declare_variable('vehicle_mass', shape=(1, )), shape=(4, 4))
#         total_forces = self.create_input('total_unperturbed_forces_stability_analysis', shape=(3, 1), val=0)
#         total_forces_perturbed = self.create_input('total_perturbed_forces_stability_analysis', shape=(8, 3), val=0)
#         total_moments = self.create_input('total_unperturbed_moments_stability_analysis', shape=(3, 1), val=0)
#         total_moments_perturbed = self.create_input('total_perturbed_moments_stability_analysis', shape=(8, 3), val=0)
#         for aero_output in aero_outputs:
#             F = aero_output.forces
#             M = aero_output.moments
#             F_perturbed = aero_output.forces_perturbed
#             M_perturbed = aero_output.moments_perturbed

#             solver_force = self.declare_variable(f"{F.operation.name}_{F.name}", shape=(3, 1))
#             solver_moment = self.declare_variable(f"{M.operation.name}_{M.name}", shape=(3, 1))

#             total_forces = total_forces + solver_force
#             total_moments = total_moments + solver_moment

#             solver_force_perturbed = self.declare_variable(f"{F_perturbed.operation.name}.{F_perturbed.operation.name}.{F_perturbed.name}", shape=(8, 3))
#             solver_moment_perturbed = self.declare_variable(f"{M_perturbed.operation.name}.{M_perturbed.operation.name}.{M_perturbed.name}", shape=(8, 3))

#             total_forces_perturbed = total_forces_perturbed + solver_force_perturbed
#             total_moments_perturbed = total_moments_perturbed + solver_moment_perturbed


#         Fx = total_forces[0, 0]
#         Fy = total_forces[1, 0]
#         Fz = total_forces[2, 0]

#         Fx_perturbed_u = total_forces_perturbed[0, 0]
#         Fx_perturbed_v = total_forces_perturbed[1, 0]
#         Fx_perturbed_w = total_forces_perturbed[2, 0]
#         Fx_perturbed_p = total_forces_perturbed[3, 0]
#         Fx_perturbed_q = total_forces_perturbed[4, 0]
#         Fx_perturbed_r = total_forces_perturbed[5, 0]
#         Fx_perturbed_theta = total_forces_perturbed[6, 0]
#         Fx_perturbed_phi = total_forces_perturbed[7, 0]

#         Fy_perturbed_u = total_forces_perturbed[0, 1]
#         Fy_perturbed_v = total_forces_perturbed[1, 1]
#         Fy_perturbed_w = total_forces_perturbed[2, 1]
#         Fy_perturbed_p = total_forces_perturbed[3, 1]
#         Fy_perturbed_q = total_forces_perturbed[4, 1]
#         Fy_perturbed_r = total_forces_perturbed[5, 1]
#         Fy_perturbed_theta = total_forces_perturbed[6, 1]
#         Fy_perturbed_phi = total_forces_perturbed[7, 1]

#         Fz_perturbed_u = total_forces_perturbed[0, 2]
#         Fz_perturbed_v = total_forces_perturbed[1, 2]
#         Fz_perturbed_w = total_forces_perturbed[2, 2]
#         Fz_perturbed_p = total_forces_perturbed[3, 2]
#         Fz_perturbed_q = total_forces_perturbed[4, 2]
#         Fz_perturbed_r = total_forces_perturbed[5, 2]
#         Fz_perturbed_theta = total_forces_perturbed[6, 2]
#         Fz_perturbed_phi = total_forces_perturbed[7, 2]


#         # Fy_perturbed = total_forces_perturbed[1]

#         Mx = total_moments[0, 0]
#         My = total_moments[1, 0]
#         Mz = total_moments[2, 0]

#         # Mx_perturbed = total_moments_perturbed[0]

#         Mx_perturbed_u = total_moments_perturbed[0, 0]
#         Mx_perturbed_v = total_moments_perturbed[1, 0]
#         Mx_perturbed_w = total_moments_perturbed[2, 0]
#         Mx_perturbed_p = total_moments_perturbed[3, 0]
#         Mx_perturbed_q = total_moments_perturbed[4, 0]
#         Mx_perturbed_r = total_moments_perturbed[5, 0]
#         Mx_perturbed_theta = total_moments_perturbed[6, 0]
#         Mx_perturbed_phi = total_moments_perturbed[7, 0]

#         My_perturbed_u = total_moments_perturbed[0, 1]
#         My_perturbed_v = total_moments_perturbed[1, 1]
#         My_perturbed_w = total_moments_perturbed[2, 1]
#         My_perturbed_p = total_moments_perturbed[3, 1]
#         My_perturbed_q = total_moments_perturbed[4, 1]
#         My_perturbed_r = total_moments_perturbed[5, 1]
#         My_perturbed_theta = total_moments_perturbed[6, 1]
#         My_perturbed_phi = total_moments_perturbed[7, 1]

#         Mz_perturbed_u = total_moments_perturbed[0, 2]
#         Mz_perturbed_v = total_moments_perturbed[1, 2]
#         Mz_perturbed_w = total_moments_perturbed[2, 2]
#         Mz_perturbed_p = total_moments_perturbed[3, 2]
#         Mz_perturbed_q = total_moments_perturbed[4, 2]
#         Mz_perturbed_r = total_moments_perturbed[5, 2]
#         Mz_perturbed_theta = total_moments_perturbed[6, 2]
#         Mz_perturbed_phi = total_moments_perturbed[7, 2]

#         # Mz_perturbed = total_moments_perturbed[2]

#         A_long = self.create_output('A_long_compute', shape=(4, 4), val=0)

#         # A_long =
#         # [[dFx/du, dFx/dw, dFx/dq, dFx/dtheta]
#         # [ dFz/du, dFz/dw, dFz/dq, dFz/dtheta]
#         # [ dMy/du, dMy/dw, dMy/dq, dMy/dtheta]]
#         # [   0,      0,      1,        0     ]]


#         self.print_var(total_forces_perturbed)

#         A_long[0, 0] = (Fx_perturbed_u - Fx) / 1
#         A_long[0, 1] = (Fx_perturbed_w - Fx) / 1
#         A_long[0, 2] = (Fx_perturbed_q - Fx) / np.deg2rad(3)
#         A_long[0, 3] = (Fx_perturbed_theta - Fx) / np.deg2rad(3)

#         A_long[1, 0] = (Fz_perturbed_u - Fz) / 1
#         A_long[1, 1] = (Fz_perturbed_w - Fz) / 1
#         A_long[1, 2] = (Fz_perturbed_q - Fz) / np.deg2rad(3)
#         A_long[1, 3] = (Fz_perturbed_theta - Fz) / np.deg2rad(3)

#         A_long[2, 0] = (My_perturbed_u - My) / 1
#         A_long[2, 1] = (My_perturbed_w - My) / 1
#         A_long[2, 2] = (My_perturbed_q - My) / np.deg2rad(3)
#         A_long[2, 3] = (My_perturbed_theta - My) / np.deg2rad(3)

#         A_long[3, 2] = self.create_input('A_long_one_value', val=1, shape=(1, 1)) #(My_perturbed_u - My) / (My_perturbed_u - My)

#         self.register_output(name='A_long', var=A_long / mass)

#         A_lat = self.create_output('A_lat_compute', shape=(4, 4), val=0)

#         # A_lat =
#         # [[dFy/dv, dFy/dp, dFy/dr, dFy/dphi]
#         #  [dMx/dv, dMx/dp, dMx/dr, dMx/dphi]
#         #  [dMz/dv, dMz/dp, dMz/dr, dMz/dphi]
#         #  [   0       1       0       0    ]]

#         A_lat[0, 0] = (Fy_perturbed_v - Fy) / 1
#         A_lat[0, 1] = (Fy_perturbed_p - Fy) / np.deg2rad(3)
#         A_lat[0, 2] = (Fy_perturbed_r - Fy) / np.deg2rad(3)
#         A_lat[0, 3] = (Fy_perturbed_phi - Fy) / np.deg2rad(3)

#         A_lat[1, 0] = (Mx_perturbed_v - Mx) / 1
#         A_lat[1, 1] = (Mx_perturbed_p - Mx) / np.deg2rad(3)
#         A_lat[1, 2] = (Mx_perturbed_r - Mx) / np.deg2rad(3)
#         A_lat[1, 3] = (Mx_perturbed_phi - Mx) / np.deg2rad(3)

#         A_lat[2, 0] = (Mz_perturbed_v - Mz) / 1
#         A_lat[2, 1] = (Mz_perturbed_p - Mz) / np.deg2rad(3)
#         A_lat[2, 2] = (Mz_perturbed_r - Mz) / np.deg2rad(3)
#         A_lat[2, 3] = (Mz_perturbed_phi - Mz) / np.deg2rad(3)

#         A_lat[3, 1] = self.create_input('A_lat_one_value', val=1, shape=(1, 1))

#         self.register_output(name='A_lat', var=A_lat/mass)
