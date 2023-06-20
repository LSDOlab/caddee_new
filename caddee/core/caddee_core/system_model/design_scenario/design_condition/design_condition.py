from caddee.utils.caddee_base import CADDEEBase
import numpy as np


class SteadyDesignCondition(CADDEEBase):
    """
    Class for steady-state analyses (e.g., steady cruise segment).

    Parameters:
    ---
        stability_flag : perform static stability analysis if True
    """

    def initialize(self, kwargs):
        # Each design condition needs a name 
        self.parameters.declare(name='name', default='', types=str)

        self.mechanics_group = None
        self.nonmechanics_group = None
        self.power_group = None
        self.equations_of_motion_csdl= None 
        # Note: because EOM is a CADDEE submodule (implemented in CSDL)
        # we create a csdl model directly in the run script while all 
        # other class attributes are pure python objects
        self.atmosphere_model = None
        self.sub_conditions = dict()
        self.model_group = None

        # Parameters
        self.parameters.declare(name='stability_flag', default=False, types=bool)
        self.parameters.declare(name='dynamic_flag', default=False, types=bool)


    def add_model_group(self, model_group):
        from m3l import ModelGroup
        if not isinstance(model_group, ModelGroup):
            raise TypeError("model_group must be of type 'm3l.ModelGroup' ")


    def _assemble_csdl(self):
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import DesignConditionCSDL
        csdl_model = DesignConditionCSDL(
            design_condition=self,
        )

        return csdl_model
    

class CruiseCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended to define cruise mission segmenst of air vehicles.
    
    CADDEE inputs (set by set_module_input()):
    ---
        - range : range of a cruise condition
        - time : time of a hover condition
        - mach_number : aircraft free-stream Mach number  (can't be specified if cruise_speed)
        - cruise_speed : aircraft cruise speed (can't be specified if mach_number)
        - altitude : aircraft altitude 
        - theta : aircraft pitch angle
        - gamma : aircraft flight path angle 
        - psi : yaw angle 
        - psi_w : wind angle 
        - observer_location : x, y, z location of aircraft (if specified, z must be equal to altitude)
    """
    def initialize(self, kwargs):
        return super().initialize(kwargs)
    
    # NOTE: p, q, r will be part of the 12 aircraft states but don't affect the steady analysis 
    # as they are pitch, roll, yaw rates 

    def _assemble_csdl(self):
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import CruiseConditionCSDL
        csdl_model = CruiseConditionCSDL(
            cruise_condition=self,
        )

        return csdl_model

class HoverCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended for hover segments.

    Acceptable inputs
    ---
        - altitude
        - time
        - observer locations 

    """
    def initialize(self, kwargs):
        return super().initialize(kwargs)
    
class ClimbCondition(SteadyDesignCondition):
    """
    Sublcass of SteadyDesignCondition intended for climb segments.

    Acceptable inputs
    ---
        - initial_altitude : initial altitude of a climb condition
        - final_altitude : final altitude of a climb condition
        - speed
        - mach number 
        - time
        - flight path angle 
        - climb gradient 
        - pitch angle
        - yaw angle 
    """
    def initialize(self, kwargs):
        return super().initialize(kwargs)
    
class VectorizedDesignCondition(SteadyDesignCondition):
    """
    Sublcass of SteadyDesignCondition intended for vectorizing mission segments
    that contain exactly the same m3l model group 
    """
    def initialize(self, kwargs):
        return super().initialize(kwargs)
    
    def add_subcondition(self, subcondition):
        """
        Method for adding steady sub design conditions to a vectorized design conditions.
        The models used must be exactly the same
        """
        name = subcondition.parameters['name']
        self.sub_conditions[name] = subcondition
