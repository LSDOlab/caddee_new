import pandas as pd
import numpy as np
from tabulate import tabulate
from caddee.system_design.geometry.geocore.component import Rotor
from caddee.system_analysis.scenario.condition.static_condition.static_conditions import StaticAircraftCondition
from caddee.utils.operation_states import AircraftStates

def assemble_user_input(system_design, system_analysis):
    """
    This is a helper function that organizes the user input defined in the run script
    Key tasks of this helper function are
        1) Determine the type of each (analysis) condition and how many of each type there are
        2) Determine the correct shape of solver input variables 
        3) Determine which solvers are used in which conditions 
        4) Determine component-specific variables (such as rpm or rotor radius) that are needed as solver inputs 
    """
    design = system_design
    analysis = system_analysis
    # Preprocessing steps
    # 1) Count number of static conditions
    num_static_cond = 0
    # 2) Count number of conditions with stability flag 
    num_stab_flags = 0 
    # 3) collect all geometric variables that will be csdl variables
    geometric_variables = {}
    # 4) collect all inputs
    static_aircraft_states = {}
    static_aircraft_states_w_stability = {}
    condition_specific_parameters = {}
    static_stability_aircraft_states = {}
    component_specific_parameters = {}
    static_component_states = {}
    # 5) collect all solvers 
    static_solvers_info = {}
    static_solvers = {}
    
    # Create a dataframe; an example is given below.
    ##################### Conditions #####################
    ###############| Hover | OEI | Transition | Climb | 
    # T # Static   |   0   |  0  |      0     |   1   |
    # y # Stability|   1   |  0  |      0     |   0   |
    # p # Dynamic  |   0   |  0  |      1     |   0   |
    # e # Coupled  |   0   |  1  |      0     |   0   |
    ###############|-------------------------------------- 
    # S # lift_BEM |   1   |  1  |      1     |   0   |   
    # o # push_BEM |   0   |  0  |      1     |   1   |
    # l # VLM      |   0   |  0  |      0     |   1   |
    # v # UVLM     |   0   |  0  |      1     |   0   |
    # e # IMGA     |   0   |  1  |      0     |   0   |
    # r # noise    |   1   |  0  |      1     |   0   |
    # s # lift_mot |   1   |  1  |      1     |   0   |

    # Solvers will be vectorized over the number of conditions of the same type
    # Example:
    #   Above, lift_BEM is used for Hover, OEI, and transition. However, we 
    #   will instantiate three separate solvers since all three conditions 
    #   are of different types. This is necessary because for the dynamic
    #   transition condition, the BEM solver will be placed inside Ozone, 
    #   while the Hover condition is static (and we also specify a stability
    #   flag) so we don't need Ozone.

    # Condition frame: here we consider the type of the condition
    condition_frame = pd.DataFrame({
    },index=[
        'Static condition', 
        'Static condition with'+'\n'+ 'stability analysis',
        'Dynamic condition',
        'Condition with' + '\n' + 'coupled Analysis']
    )

    # Solver frame: here we consider the solvers in each condition
    solver_frame = pd.DataFrame({})


    # Loop over design dictionary and add any geometric variables
    for component_name, component in design.component_dict.items():
        if isinstance(component,Rotor):
            geometric_variables[component_name + '_radius'] = component.rotor_radius
            # TODO: this will need to be more general, 
            # will change when getting the radius from geometry 
                    
    
    # Loop over analysis dictionary (typically only one analysis dict)
    for analysis_key, analysis_val in analysis.analysis_dict.items():
        # Loop over scenario dictionary (which contains the conditions)
        #   - Based on what kind of condition we're dealing with, 
        #     we add the states to a corresponding dictionary
        
        for condition_key, condition_val in analysis_val.scenario_dict.items():
            
            # Check if the condition is static with no stability_flag 
            if condition_val['static_condition'] == True and condition_val['stability_flag'] == False:
                # aircraft states
                static_aircraft_states[condition_key] = condition_val['static_aircraft_states']
                
                # condition specific parameters like cruise range or hover time 
                condition_specific_parameters.update(condition_val['condition_specific_parameter'])
                
                # TODO: set up solvers for actual analysis (not just dummy solvers)
                static_solvers_info[condition_key] = condition_val['static_solvers']
                static_solvers.update(condition_val['static_solvers'])
                
                # Update the condition data frame - this data 
                # frame will be used for vectorization 
                condition_frame[condition_key] = [1,0,0,0]
                
                # Update the solver data frame -
                # also used for vectorization 
                solver_frame[condition_key] = 0
                for solver_name, solver in static_solvers_info[condition_key].items():
                    solver_frame.loc[solver_name,condition_key] = 1
            
            elif condition_val['static_condition'] == True and condition_val['stability_flag'] == True:
                # aircraft states
                static_aircraft_states_w_stability[condition_key] = condition_val['static_aircraft_states']
                
                # condition specific parameters like cruise range or hover time 
                condition_specific_parameters.update(condition_val['condition_specific_parameter'])
                
                # TODO: set up solvers for actual analysis (not just dummy solvers)
                static_solvers_info[condition_key] = condition_val['static_solvers']
                static_solvers.update(condition_val['static_solvers'])
                
                # Update the condition data frame - this data 
                # frame will be used for vectorization 
                condition_frame[condition_key] = [0,1,0,0]
                
                
                # Update the solver data frame -
                # also used for vectorization 
                solver_frame[condition_key] = 0
                for solver_name, solver in static_solvers_info[condition_key].items():
                    solver_frame.loc[solver_name, condition_key] = 1
        
        for comp_oper_key, comp_oper_val in analysis_val.component_operation_dict.items():
            if comp_oper_val['static_condition'] == True and comp_oper_val['stability_flag'] == True:
                static_component_states.update(comp_oper_val['static_component_states'])
            elif condition_val['static_condition'] == True and comp_oper_val['stability_flag'] == False:
                static_component_states.update(comp_oper_val['static_component_states'])
            
    
    # Fill in nan in solver dataframe with zeros 
    solver_frame = solver_frame.fillna(0)

    # create numpy selection array from condition frame
    selection_mat = condition_frame.to_numpy()

    static_ac_states_solver_inputs_dict = {}
    
    # Loop over solvers inside solver frame- 
    # Here, the goal is to select the conditions across which we are vectorizing 
    for solver, conditions in solver_frame.to_dict(orient='index').items():
        shape = np.matmul(selection_mat,solver_frame.loc[solver].to_numpy())
        static_shape = int(shape[0])
        stability_shape = shape[1]
        dynamic_shape = shape[2]
        coupled_shape = shape[3]

        # In which conditions are we calling the solver
        A = (np.where(solver_frame.loc[solver] == 1)[0])

        # Where are the static conditions 
        B = (np.where(condition_frame.loc['Static condition'] == 1)[0])
        
        # Create a selection list where does A and B intersect
        selection_list = list(set(A) & set(B))# [i for i, j in zip(A, B) if i == j]

        # Get the right conditions based selection list 
        solver_condition_keys = condition_frame.columns[selection_list].tolist()

        # Depreciated: 
        #   - needed if all variables are already in the right namespace
        #   - currently, we are creating the correct namespace upon declaring csdl variables 
        # for i in range(len(solver_condition_keys)):
        #     solver_inputs.update(dict(filter(lambda item: solver_condition_keys[i] in item[0], static_aircraft_states.items())))
            # solver_inputs_2.update(dict(filter(lambda item: solver_condition_keys[i] in item[0], static_component_states.items())))

        # TODO: 
        #   - distinguish between solver inputs based on solver type
        #   - Right now, all inputs (ac states plus rotor parameters)
        #     are for APPM solvers, but this is not true in general 
        # storing all the information regarding static solver states 
        # this will be passed into the csdl models 
        static_ac_states_solver_inputs_dict[solver] = {
            'shape' : static_shape,
            'conditions' : solver_condition_keys,
            'solver' : static_solvers[solver],

        }        
        
    # Printing dataframes as a summary for the user 
    print(tabulate(condition_frame, headers='keys', tablefmt='fancy_grid'))
    print(tabulate(solver_frame, headers='keys', tablefmt='fancy_grid'))



    return static_aircraft_states, condition_specific_parameters, static_ac_states_solver_inputs_dict, geometric_variables, static_component_states







# def assemble_user_input(system_design, system_analysis):
#     design = system_design
#     analysis = system_analysis
#     # Preprocessing steps
#     # 1) Count number of static conditions
#     num_static_cond = 0
#     # 2) Count number of conditions with stability flag 
#     num_stab_flags = 0 
#     # 3) collect all geometric variables that will be csdl variables
#     geometric_variables = {}
#     # 4) collect all inputs
#     static_aircraft_states = {}
#     condition_specific_parameters = {}
#     static_stability_aircraft_states = {}
#     component_specific_parameters = {}
#     static_component_states = {}
#     # 5) collect all solvers 
#     static_solvers = {}
    
#     # Create a dataframe; an example is given below.
#     ##################### Conditions #####################
#     ###############| Hover | OEI | Transition | Climb | 
#     # T # Static   |   0   |  0  |      0     |   1   |
#     # y # Stability|   1   |  0  |      0     |   0   |
#     # p # Dynamic  |   0   |  0  |      1     |   0   |
#     # e # Coupled  |   0   |  1  |      0     |   0   |
#     ###############|-------------------------------------- 
#     # S # lift_BEM |   1   |  1  |      1     |   0   |   
#     # o # push_BEM |   0   |  0  |      1     |   1   |
#     # l # VLM      |   0   |  0  |      0     |   1   |
#     # v # UVLM     |   0   |  0  |      1     |   0   |
#     # e # IMGA     |   0   |  1  |      0     |   0   |
#     # r # noise    |   1   |  0  |      1     |   0   |
#     # s # lift_mot |   1   |  1  |      1     |   0   |

#     # Solvers will be vectorized over the number of conditions of the same type
#     # Example:
#     #   Above, lift_BEM is used for Hover, OEI, and transition. However, we 
#     #   will instantiate three separate solvers since all three conditions 
#     #   are of different types. This is necessary because for the dynamic
#     #   transition condition, the BEM solver will be placed inside Ozone, 
#     #   while the Hover condition is static (and we also specify a stability
#     #   flag) so we don't need Ozone.

#     condition_frame = pd.DataFrame({
#     },index=[
#         'Static condition', 
#         'Static condition with'+'\n'+ 'stability analysis',
#         'Dynamic condition',
#         'Condition with' + '\n' + 'coupled Analysis']
#     )

#     solver_frame = pd.DataFrame({})


#     # print('COMPONENTS-------------',design.component_dict.keys())
#     # Loop over design dictionary and add any operational parameters (e.g., RPM )
#     for component_name, component in design.component_dict.items():
#         if isinstance(component,Rotor):
#             geometric_variables[component_name + '_radius'] = component.rotor_radius
#             # TODO: this will need to be more general, 
#             # will change when getting the radius from geometry 
#             # if component.add_rpm_has_been_called is False:
#             #     raise Exception('Please add rpm')

#         # TODO: 
#         # throw error if user adds rpm to a rotor for a particular condition
#         # but no corresponding solver is defined 
#         # component_states_dict = component._finalize_component_parameters()
#         # for var_name, var_val in component_states_dict.items():
#         #     # # print(var_name)
#         #     # if isinstance(var_val['condition'],StaticAircraftCondition) and var_val['condition'].stability_flag == False:
#         #         component_specific_parameters[var_name] = var_val

#         # print(component_name)
#         # print(component_states_dict.keys())
#         # print('----------------')
        
#         # for var_name, var_dict in component_states_dict.items():
#         #     if var_dict['condition'] is not None:
#         #         condition = var_dict['condition']
#         #         if isinstance(condition,StaticAircraftCondition):
#         #             condition_name = condition.name
#         #             print(condition_name)
                    
    
#     # Loop over analysis dictionary 
#     for analysis_key, analysis_val in analysis.analysis_dict.items():
#         # Loop over scenario dictionary (which contains the conditions)
#         for condition_key, condition_val in analysis_val.scenario_dict.items():
#             print('condition_key------------------', condition_key)
#             print('static_aircraft_states---------', condition_val['static_aircraft_states'].keys())
#             # Check if the condition is static 
#             if condition_val['static_condition'] == True and condition_val['stability_flag'] == True:
#                 # TODO: separate dictionary for stability ac states
#                 static_aircraft_states.update(condition_val['static_aircraft_states'])
#                 condition_specific_parameters.update(condition_val['condition_specific_parameter'])
#                 static_solvers_info[condition_key] = condition_val['static_solvers']
#                 condition_frame[condition_key] = [0,1,0,0]
#                 solver_frame[condition_key] = 0
#                 for solver_name, solver in static_solvers_info[condition_key].items():
#                     solver_frame.loc[solver_name,condition_key] = 1
#             elif condition_val['static_condition'] == True and condition_val['stability_flag'] == False:
#                 static_aircraft_states.update(condition_val['static_aircraft_states'])
#                 condition_specific_parameters.update(condition_val['condition_specific_parameter'])
#                 static_solvers_info[condition_key] = condition_val['static_solvers']
#                 condition_frame[condition_key] = [1,0,0,0]
#                 solver_frame[condition_key] = 0
#                 for solver_name, solver in static_solvers_info[condition_key].items():
#                     solver_frame.loc[solver_name, condition_key] = 1
        
#         for comp_oper_key, comp_oper_val in analysis_val.component_operation_dict.items():
#             print(comp_oper_key)
#             print(comp_oper_val)
#             if comp_oper_val['static_condition'] == True and comp_oper_val['stability_flag'] == True:
#                 static_component_states.update(comp_oper_val['static_component_states'])
#             elif condition_val['static_condition'] == True and comp_oper_val['stability_flag'] == False:
#                 static_component_states.update(comp_oper_val['static_component_states'])
            


#     seen = set()
#     seen_add = seen.add
#     L = [i.split('_')[-1] for i in list(static_aircraft_states.keys())]
#     L2 = [i.split('_')[-1] for i in list(static_component_states.keys())]
#     unique_solver_inputs = [x for x in L if not (x in seen or seen_add(x))]
#     unique_solver_inputs_2 = [x for x in L2 if not (x in seen or seen_add(x))]
    
#     solver_frame = solver_frame.fillna(0)
#     selection_mat = condition_frame.to_numpy()

    
#     solver_inputs_dict = {}
#     # print('static_component_states',static_component_states.keys())
#     for solver, conditions in solver_frame.to_dict(orient='index').items():
#         # print('conditions',conditions)
#         print('static_component_states',static_component_states.keys())
#         # print('solver',solver)
#         # print('static_solvers',static_solvers)
#         # shared_items = {k: conditions[k] for k in conditions if k in static_solvers and conditions[k] == static_solvers_info[k]}
        
#         # for key in conditions:
#         #     if key in static_solvers:
#         #         print(static_solvers_info[key])
#         # print('----------------')
#         # print((shared_items))
#         solver_inputs = {}
#         solver_inputs_2 = {}
#         shape = np.matmul(selection_mat,solver_frame.loc[solver].to_numpy())
#         static_shape = shape[0]
#         stability_shape = shape[1]
#         dynamic_shape = shape[2]
#         coupled_shape = shape[3]


#         A = (np.where(solver_frame.loc[solver] == 1)[0])
#         # print('A',A)
#         B = (np.where(condition_frame.loc['Static condition'] == 1)[0])
#         # print('B',B)
#         selection_list = list(set(A) & set(B))# [i for i, j in zip(A, B) if i == j]
#         # print('selection_list',list(set(A) & set(B)))
#         # print('selection_list',selection_list)
#         # print('----------------')
#         solver_condition_keys= condition_frame.columns[selection_list].tolist()
#         # print('solver_condition_keys',solver_condition_keys)
#         for i in range(len(solver_condition_keys)):
#             solver_inputs.update(dict(filter(lambda item: solver_condition_keys[i] in item[0], static_aircraft_states.items())))
#             solver_inputs_2.update(dict(filter(lambda item: solver_condition_keys[i] in item[0], static_component_states.items())))
            

#         solver_inputs_dict[solver] = {
#             'shape' : static_shape,
#             'conditions' : solver_condition_keys,
#             'solver_inputs' : [unique_solver_inputs,solver_inputs],
#             'solver_inputs_comp' : [unique_solver_inputs_2,solver_inputs_2],
#             'comp_names' : list(design.component_dict.keys()),
#         }        
        

#     print(tabulate(condition_frame, headers='keys', tablefmt='fancy_grid'))
#     print(tabulate(solver_frame, headers='keys', tablefmt='fancy_grid'))



#     return static_aircraft_states,condition_specific_parameters, solver_inputs_dict, geometric_variables, static_component_states