from caddee.utils.caddee_base import CADDEEBase
import numpy as np


class DesignCondition(CADDEEBase):
    """
    Class that is equivalent to mission segment in aircraft design context.

    Parameters:
    ---
        stability_flag : perform static stability analysis if True
        dynamic_flag : specifies dynamic design condition if True
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

        # Parameters
        self.parameters.declare(name='stability_flag', default=False, types=bool)
        self.parameters.declare(name='dynamic_flag', default=False, types=bool)


    def connect(self): return

    def _assemble_csdl(self):
        from caddee.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import DesignConditionCSDL
        csdl_model = DesignConditionCSDL(
            design_condition=self,
        )

        return csdl_model
    
    def _assemble_csdl_modules(self):
        from caddee.csdl_core_modules.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import DesignConditionCSDL, AircraftConditionCSDL
        if isinstance(self, AircraftCondition):
            csdl_model = AircraftConditionCSDL(
                module=self,
                prepend=self.parameters['name'],
                atmosphere_model=self.atmosphere_model,
            )
        else:
            raise Exception(f"Design condition of typ {type(self)} not implemented")
        

        return csdl_model

    def _declare_component_inputs(self, group):
        """
        This methods updates the variables metadata of a design condition 
        with inputs that are specific to a component but specified for a 
        certain design condition with the key 'design_condition='. 
        
        Ex: rpm of rotor: rotor.set_module_input('rpm', design_condition=)
        
        To facilitate vectorization, this method makes sure that rpm gets
        created as a csdl input at the condition level.
        """
        models_dict = group.models_dictionary
        for model_name, model in models_dict.items():
            component = model.parameters['component']
            comp_name = component.parameters['name']
            promoted_vars = model.promoted_variables
            for var_name, var_dict in component.variables_metadata.__dict__['_dict'].items():
                if var_name in self.variables_metadata:
                    # print('PASS----------------')
                    pass
                elif var_dict['design_condition'] is None and var_name not in promoted_vars:
                    # pass
                    raise Exception(
                        f"Attempting to add model {model_name}, which performs analysis on component {comp_name} \
                        and uses variable {var_name}. However, no condition has been specified for  \
                        variable {var_name}. Please specify the condition using the 'condition=' keyword \
                        in the 'set_module_input()' method."
                    ) 
                # elif var_dict['design_condition'] is None:
                    # print('NO DESIGN CONDITION!!')
                    # print(var_name)
                    # print(promoted_vars)
                    # print('------------')
                else:
                    name = '{}_{}'.format(comp_name, var_name)
                    # if var_dict['design_condition']:
                    #     print('MODEL NAME-----------', model_name)
                    #     print('NAME-----------------', var_name)
                    #     print('DESIGN CONDITION-----', var_dict['design_condition'].parameters['name'])
                    # if 'rpm' in name:
                    #     print('name: {}-----val: {}-----design condition: {}'.format(
                    #         name, var_dict['value'], var_dict['design_condition'].parameters['name']))
                    val = var_dict['value']
                    self.variables_metadata.declare(name=name, default=val)


class AircraftCondition(DesignCondition):
    """
    Subclass of DesignCondition intended to define mission 
    segments of air vehicles.
    
    CADDEE inputs (set by set_module_input()):
    ---
        - range : range of a cruise condition
        - time : time of a hover condition
        - initial_altitude : initial altitude of a climb condition
        - final_altitude : final altitude of a climb condition
        -mach_number : aircraft free-stream Mach number  (can't be specified if cruise_speed)
        - cruise_speed : aircraft cruise speed (can't be specified if mach_number)
        - altitude : aircraft altitude 
        - theta : aircraft pitch angle
        - gamma : aircraft flight path angle 
        - psi : yaw angle 
        - psi_w : wind angle 
        - observer_location : x, y, z location of aircraft (if specified, z must be equal to altitude)
    """
    def initialize(self, kwargs):
        # Thoughts
        #   1) p, q, r will require additional inputs from which they can be computed
        #   2) For most conditions (climb, cruise, hover) p, q, r would be zero
        #   3) Observer location (x, y, z)
        #      Could be a vector of size three (z would altitude)
        #      Would be an input to AircraftCondition
        
        # Condition-specific variables 
        super().initialize(kwargs)
        # self.variables_metadata.declare(name='range', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='time', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='initial_altitude', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='final_altitude', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='climb_gradient', default=None, types=(int, float), allow_none=True)
        
        # # Condition-and-system-specific variables (we compute 12 aircraft states from these)
        # self.variables_metadata.declare(name='mach_number', default=0, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='speed', default=0, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='altitude', default=0, types=(int, float))
        # self.variables_metadata.declare(name='pitch_angle', default=0, types=(int, float)) # theta 
        # self.variables_metadata.declare(name='flight_path_angle', default=0, types=(int, float)) # gamma
        # self.variables_metadata.declare(name='roll_angle', default=0, types=(int, float)) # phi
        # self.variables_metadata.declare(name='yaw_angle', default=0, types=(int, float)) # psi
        # self.variables_metadata.declare(name='wind_angle', default=0, types=(int, float)) # psi_w
        # self.variables_metadata.declare(name='observer_location', default=np.array([0, 0, 0]), types=np.ndarray, allow_none=True) # x, y, z

        # # 12 aircraft states directly
        # self.variables_metadata.declare(name='u', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='v', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='w', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='p', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='q', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='r', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='phi', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='theta', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='psi', default=None, types=np.ndarray, allow_none=True)
        # self.variables_metadata.declare(name='x', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='y', default=None, types=(int, float), allow_none=True)
        # self.variables_metadata.declare(name='z', default=None, types=(int, float), allow_none=True)

        # NOTE: consider removing/not create CruiseCondition, ClimbCondition etc, classes and 
        # user instantiates AircraftCondition directly 

