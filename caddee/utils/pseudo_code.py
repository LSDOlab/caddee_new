"""
strategy for unpacking information:
Break down the process into levels and have helper functions on each 
level that take in the pure python side and output the corresponding 
csdl side. 

Rule: if there are multiple csdl models of the same kind (e.g., spatial_rep-
anics models at the lowest levels or design scenarios at the highest
level), we have dictionaries where the keys are the name of each 
csdl model and the value is the csdl model 

The idea is that the actual csdl models are fairly clean and simple.
Dictionaries like design_condition_csdl_dictionary would be attributes
and in define(self) we would simply loop over the dictionaries and add them
"""

# Helper function 1: design_scenario --> design_scenario_csdl (will need to call next helper function )
def assemble_design_scenarios_csdl_dictionary(design_scenario_dictionary):
    return design_scenarios_csdl_dictionary 

# Helper function 2: design_condition --> design_condition_csdl (will need to call next helper function)
def assemble_design_conditions_csdl_dictionary(design_condition_dictionary):
    design_conditions_csdl_dictionary = {}
    
    for des_cond_name, des_cond in design_condition_dictionary.items():
        mechanics_group = des_cond.mechanics_group
        mechanics_group_csdl = assemble_mechanics_group_csdl(mechanics_group)
        design_condition_inputs_dictionary = des_cond.design_condition_variables_dictionary
        design_condition_csdl = DesignConditionCSDL(design_condition_inputs_dictionary)
        design_condition_csdl.mechanics_group_csdl = mechanics_group_csdl
    
        design_conditions_csdl_dictionary[des_cond_name] = design_condition_csdl

    return design_conditions_csdl_dictionary

# Helper function 3a: mechanics_group --> mechanics_group_csdl 
# (imagine analogous functions for nonmechanics (3b), power (3c); may be overkill to have individual functions for 3b/c)
# Would need to call next helper function
def assemble_mechanics_group_csdl(mechanics_group):
    mechanics_models_dictionary = mechanics_group.models_dictionary
    mechanics_models_csdl_dictionary = assemble_mechanics_model_csdl_dictionary(mechanics_models_dictionary)
    
    mechanics_group_csdl = MechanicsGroupCSDL(mechanics_models_csdl_dictionary)
    
    return mechanics_group_csdl

# Helper function 4a: mechanics_model --> csdl_model (potentially analogous functions for nonmechanics and power)
def assemble_mechanics_model_csdl_dictionary(mechanics_models_dictionary):
    mechanics_models_csdl_dictionary = {}
    for mech_model_name, mech_model in mechanics_models_dictionary.items():
        mechanics_models_csdl_dictionary[mech_model_name] = {
            'promoted_vars' : mech_model.promoted_vars,
            'csdl_model' : mech_model.return_csdl_model(),
        }
    
    return mechanics_models_csdl_dictionary



"""
What types of variables do we have (that could be potential csdl and design variables)?
    1) condition-specific system states (u, v, w, ..., x, y, z)
    2) component-specific, geometry-like variables (radius, wing area, ...)
    3) component and condition-specific states (rpm, elevator deflection, ...)
    4) condition-specific variables (range, hover time, initial altitude, ...)

    Methods to specify each of these variables are written in pseudo code below.
    In general, all these methods would call the __setitem__ method of an 
    OptionsDictionary - like class, right now it is called VariablesMetadataDictionary.
    This seems appropriate because for any of the these variables the user may want to
    specify some metadata.
"""


"""
Thoughts:
    - divide variables/states into:
        1) condition-specific
        2) component-specific
        3) condition-and-component-specific 
        - Conclusion for now: not do this division of variables because it is 
          a big gamble (e.g., material properties, power nodes, geometry)
    - "add" vs "set" vs "declare" 
        - "add" is not always the most accurate 
        - "set" would always work, given that every variable that 
          a user may set would be pre-declared 
    - "variable" vs "state"
        - The classificaiton of a paritular variable as a "state", "parameter", or 
        "design variable" is subjective
        - Conclusion: "variables" can be states, parameters, or design variables
          
    - Overall conclusion: introduce set_variable()
        - Replaces add_variable() on the base CADDEE class
        - 'condition=' is overwritten when the method is called on the condition
"""


def set_system_state(name, val, csdl_var=True, computed_upstream=False, dv_flg=False, upper=None, lower=None, scaler=None):
    """
    Method for variable type 1) 

    Note that "set" is more accurate than "add" since these variables 
    are already "added" or declared, and here, the user would simply be
    setting them.

    This method would live in the ModelGroup class. It would be inherited by 
        - MechanicsGroup
        - NonmechanicsGroup
        - PowerGroup 
    
    E.g., mech_group = MechanicsGroup()
    mech_group.add_system_state('u', 50.)

    This method would set values in an instance of VariableMetadataDictionary
    that could correspond to any of the three classes mentioned above. 
    """
    
    return

def set_geometric_variable(name, val, csdl_var=True, dv_flg=False, upper=None, lower=None, scaler=None):
    """
    Method for variables type 2): WILL BE DEPRECIATED 

    Again, "set" is more accurate than "add" for the same reason as before.

    For this method, the keyword "computed_upstream" is not needed because 
    these kind of variables wouldn't be specified any further upstream

    This method would live in the Component class. In the short term, we 
    may have variables like 'radius', 'wing_area' that we may want to set manually.
    Note that variables like 'thrust_vector', 'thrust_origin', or similar variables
    that are actually part of the model mesh wouldn't be added here. This is because
    they are not part of the component and also would not not be design variables. 
    """
    return

def add_component_state(name, val, design_condition, csdl_var=True, computed_upstream=False, dv_flg=False, upper=None, lower=None, scaler=None):
    """
    Method for variable type 3) 

    For this method we again have the keyword "computed_upstream". In addition, 
    we have a required "design_condition" argument because CADDEE needs to know
    to which condition the component state belongs. 

    This method also would live in the Component class. It would "add" the 
    states of component-specific variables like 'rpm', 'elevator_deflection'.
    This method would declare a new input to component's variables_metadata
    dictionary 

    The reason we're using add here is that it is more extensible. It will be 
    difficult to predict what kind of variables components will have. 
    """
    
    return

def set_condition_variable(name, val, csdl_var=True, computed_upstream=False, dv_flg=False, upper=None, lower=None, scaler=None):
    """
    Method for variable type 4) 

    This method is most similar 'set_system_state' except that the
    variables that are set here are specific to a condition only, such as 
    'range', 'hover_time', etc. While these variables could also be design
    variables, they are distinctly different from 'state' variables, which
    are associated with how the system is behaving (e.g., aircraft states).
    Condition variables describe properties of the condition (e.g., 
    condition length).

    This method would naturally live in the DesignCondition class. 
    """

    return