from caddee.utils.caddee_base import CADDEEBase
import numpy as np
from csdl import GraphRepresentation
import m3l


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


    # def _assemble_csdl(self):
    #     from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import DesignConditionCSDL
    #     csdl_model = DesignConditionCSDL(
    #         module=self,
    #         cruise_condition=self,
    #     )

    #     return csdl_model
    

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

    def compute(self): 
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import CruiseConditionCSDL
        csdl_model = CruiseConditionCSDL(
            module=self,
            prepend=self.parameters['name'],
            cruise_condition=self,
        ) 

        return csdl_model

    def evaluate_ac_states(self): 
        operation_csdl = self.compute()
        arguments = {} 

        ac_state_operation = m3l.CSDLOperation(
            name=f"{self.parameters['name']}_ac_states_operation",
            arguments=arguments,
            operation_csdl=operation_csdl,
        )

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=ac_state_operation)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=ac_state_operation)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=ac_state_operation)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=ac_state_operation)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=ac_state_operation)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=ac_state_operation)

        phi = m3l.Variable(name='phi', shape=(self.num_nodes, ), operation=ac_state_operation)
        gamma = m3l.Variable(name='gamma', shape=(self.num_nodes, ), operation=ac_state_operation)
        psi = m3l.Variable(name='psi', shape=(self.num_nodes, ), operation=ac_state_operation)
        theta = m3l.Variable(name='theta', shape=(self.num_nodes, ), operation=ac_state_operation)

        x = m3l.Variable(name='x', shape=(self.num_nodes, ), operation=ac_state_operation)
        y = m3l.Variable(name='y', shape=(self.num_nodes, ), operation=ac_state_operation)
        z = m3l.Variable(name='z', shape=(self.num_nodes, ), operation=ac_state_operation)

        ac_states = {
            'u' : u, 
            'v' : v, 
            'w' : w, 
            'p' : p, 
            'q' : q, 
            'r' : r, 
            'phi' : phi, 
            'gamma' : gamma, 
            'psi' : psi, 
            'theta' : theta, 
            'x' : x, 
            'y' : y, 
            'z' : z
        }

        return ac_states




    def _assemble_csdl(self):
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import CruiseConditionCSDL
        # csdl_model = CruiseConditionCSDL(
        #     module=self,
        #     prepend=self.parameters['name'],
        #     cruise_condition=self,
        # )
        # GraphRepresentation(csdl_model)
        # print(self.inputs)
        # print('#################3', self.m3l_models)
        if len(self.m3l_models) > 1:
            raise Exception(f"More than one m3l model added to design condition {self.parameters['name']}")
        else:
            for m3l_model_name, m3l_model in self.m3l_models.items():
                csdl_model = m3l_model._assemble_csdl()

        return csdl_model

class HoverCondition(SteadyDesignCondition):
    """
    Subclass of SteadyDesignCondition intended for hover segments.

    Acceptable inputs
    ---
        - altitude
        - time
        - observer location 

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

    def evaluate_ac_states(self): 
        operation_csdl = self.compute()
        arguments = {} 

        ac_state_operation = m3l.CSDLOperation(
            name=f"{self.parameters['name']}_ac_states_operation",
            arguments=arguments,
            operation_csdl=operation_csdl,
        )

        u = m3l.Variable(name='u', shape=(self.num_nodes, ), operation=ac_state_operation)
        v = m3l.Variable(name='v', shape=(self.num_nodes, ), operation=ac_state_operation)
        w = m3l.Variable(name='w', shape=(self.num_nodes, ), operation=ac_state_operation)

        p = m3l.Variable(name='p', shape=(self.num_nodes, ), operation=ac_state_operation)
        q = m3l.Variable(name='q', shape=(self.num_nodes, ), operation=ac_state_operation)
        r = m3l.Variable(name='r', shape=(self.num_nodes, ), operation=ac_state_operation)

        phi = m3l.Variable(name='phi', shape=(self.num_nodes, ), operation=ac_state_operation)
        gamma = m3l.Variable(name='gamma', shape=(self.num_nodes, ), operation=ac_state_operation)
        psi = m3l.Variable(name='psi', shape=(self.num_nodes, ), operation=ac_state_operation)
        theta = m3l.Variable(name='theta', shape=(self.num_nodes, ), operation=ac_state_operation)

        x = m3l.Variable(name='x', shape=(self.num_nodes, ), operation=ac_state_operation)
        y = m3l.Variable(name='y', shape=(self.num_nodes, ), operation=ac_state_operation)
        z = m3l.Variable(name='z', shape=(self.num_nodes, ), operation=ac_state_operation)

        ac_states = {
            'u' : u, 
            'v' : v, 
            'w' : w, 
            'p' : p, 
            'q' : q, 
            'r' : r, 
            'phi' : phi, 
            'gamma' : gamma, 
            'psi' : psi, 
            'theta' : theta, 
            'x' : x, 
            'y' : y, 
            'z' : z
        }

        return ac_states
    
    def _assemble_csdl(self):
        if len(self.m3l_models) > 1:
            raise Exception(f"More than one m3l model added to design condition {self.parameters['name']}")
        else:
            for m3l_model_name, m3l_model in self.m3l_models.items():
                hover_model = m3l_model._assemble_csdl()

        return hover_model
    
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
