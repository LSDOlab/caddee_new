from caddee.utils.caddee_base import CADDEEBase
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_group import MechanicsGroup
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import MechanicsModel
from caddee.core.caddee_core.system_model.design_scenario.design_condition.nonmechanics_group.nonmechanics_group import NonmechanicsGroup
from caddee.core.caddee_core.system_model.design_scenario.design_condition.power_group.power_group import PowerGroup
from caddee.core.caddee_core.system_model.design_scenario.design_condition.power_group.power_model.power_model import PowerModel
import pandas as pd
import numpy as np


class DesignScenario(CADDEEBase):
    def initialize(self, kwargs):
        # Design scenario name (needs to be an attribute)
        self.parameters.declare(name='name', default='', types=str)
        self.design_condition_dictionary = {}

        self.equations_of_motion_csdl = None
        self._psa_connections_list = None
        self._system_model_connections_list = None

    def add_design_condition(self, design_condition):
        if design_condition.parameters['name'] == '':
            raise Exception("Please give the design condition a unique name")
        self.design_condition_dictionary[design_condition.parameters['name']] = design_condition
        # TODO: implement check if the a design condition with the same (default name)
        # is added. If that's the case, it will just override the previous design condition
        # and there needs to be an exception

    def _assemble_csdl(self):
        # TODO: every _assemble_csdl will have an import similar to below (to avoid circular imports)
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_scenario_csdl import DesignScenarioCSDL
        csdl_model = DesignScenarioCSDL(
            design_scenario=self,
        )
        return csdl_model
    
    def _assemble_csdl_modules(self, system_config=None, system_param=None, sizing_group=None):
        # TODO: every _assemble_csdl will have an import similar to below (to avoid circular imports)
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_scenario_csdl import DesignScenarioCSDL
        csdl_model = DesignScenarioCSDL(
            design_scenario=self,
            system_config=system_config,
            sizing_group=sizing_group,
            system_param=system_param
        )
        return csdl_model

    def _assemble_groups(self, design_condition_dictionary):
        """
        Method to assemble groups across all design conditions.
        This method enables vectorization by determining how many 
        design conditions there are and what solvers are added 
        to each condition.
        """
        # number of total conditions 
        num_conditions = len(design_condition_dictionary)       
        # Lists
        condition_types_list = ['steady_condition', 'stability_flag', 'dynamic_condition']
        num_condition_types = len(condition_types_list)
        conditions_list = []
        mechanics_model_names_list = []
        power_model_names_list = []
        mechanics_models_list = []
        power_models_list = []

        mech_struct_models = []
        mech_struct_model_names = []
        
        
        model_names_list = []
        models_list = []
        
        # Empty array for solvers and conditions 
        condition_solver_array = np.zeros((num_condition_types, num_conditions))
        # Counters for conditions and models/solvers
        condition_counter = 0
        model_counter = len(condition_types_list)
        
        # Loop over design condition dictionary 
        mech_group_connections = []
        for name, design_condition in design_condition_dictionary.items():
            if design_condition.parameters['dynamic_flag'] == True:
                condition_solver_array[2, condition_counter] = 1
            elif design_condition.parameters['stability_flag'] == True:
                # Indexing 2 rows because when stability_flag is True, 
                # that segment is also a static segment
                condition_solver_array[0:2, condition_counter] = 1
            else:
                condition_solver_array[0, condition_counter] = 1
            
            models_dict = {}
            mech_group_connections += design_condition.mechanics_group.connections_list
            models_dict.update(design_condition.mechanics_group.models_dictionary)
            if design_condition.nonmechanics_group:
                models_dict.update(design_condition.nonmechanics_group.models_dictionary)
            if design_condition.power_group:
                models_dict.update(design_condition.power_group.models_dictionary)

            # Loop over mechanics model dictionary 
            # NOTE: assume models/solvers are used for steady conditions for now
            for model_name, model in models_dict.items():
                # print('model_name', model_name)
                # print('model', model)
                if model_name in model_names_list:
                    model_index = model_names_list.index(model_name)
                    condition_solver_array[model_index + num_condition_types, condition_counter] = 1
                else:
                    condition_solver_array = np.append(condition_solver_array, np.zeros((1, num_conditions)), axis=0)
                    condition_solver_array[model_counter, condition_counter] = 1
                    model_names_list.append(model_name)
                    models_list.append(model)
                    if isinstance(model, MechanicsModel):
                        mechanics_models_list.append(model)
                        mechanics_model_names_list.append(model_name)

                        if model.parameters.__contains__('compute_mass_properties'):
                            if model.parameters['compute_mass_properties'] is True:
                                mech_struct_model_names.append(model_name)
                                mech_struct_models.append(model)
                    elif isinstance(model, PowerModel):
                        power_models_list.append(model)
                        power_model_names_list.append(model_name)
                    
                    model_counter += 1

            conditions_list.append(name)
            condition_counter += 1
        
        # Create pandas dataframe
        condition_solver_frame = pd.DataFrame(
            data=condition_solver_array, 
            index=condition_types_list + model_names_list,
            columns=conditions_list)
        
        # ----- Start TODO: check if this is needed ---- #
        data_shape = condition_solver_array.shape
        num_solvers = data_shape[0] - num_condition_types
        steady_solvers_num_nodes_list = []
        for i in range(num_solvers):
            index = i + num_condition_types
            steady_condition_counter = 0
            for j in range(num_conditions):
                if condition_solver_array[index, j] == 1 and condition_solver_array[0, j] == 1:
                    steady_condition_counter += 1
            if steady_condition_counter != 0:
                steady_solvers_num_nodes_list.append(steady_condition_counter)
        # ----- End TODO: check if this is needed ---- #
        num_static_conditions = len(design_condition_dictionary)

        print(condition_solver_frame)

        mechanics_group = MechanicsGroup()
        mechanics_group._all_models_list = mechanics_models_list
        mechanics_group._all_models_names_list = mechanics_model_names_list
        mechanics_group._struct_models = mech_struct_models
        mechanics_group._struct_model_names = mech_struct_model_names
        mechanics_group._num_nodes = num_static_conditions # steady_solvers_num_nodes_list
        mechanics_group._condition_solver_frame = condition_solver_frame
        mechanics_group.connections_list = mech_group_connections
        
        # print('mechanics_models_list', mechanics_models_list)
        # exit()

        nonmechanics_group = None
        
        if power_models_list:
            power_group = PowerGroup()
            power_group._all_models_list = power_models_list
            power_group._all_models_names_list = power_model_names_list
            power_group._num_nodes = num_static_conditions# steady_solvers_num_nodes_list
            power_group._condition_solver_frame = condition_solver_frame
        else: 
            power_group=None

        return mechanics_group, nonmechanics_group, power_group


# from caddee.utils.caddee_base import CADDEEBase


# class DesignScenario(CADDEEBase):
#     def initialize(self, kwargs):
#         # Design scenario name (needs to be an attribute)
#         self.parameters.declare(name='name',default='',types=str)
#         self.design_condition_dictionary = {}

#     def add_design_condition(self,design_condition):
#         if design_condition.parameters['name'] == '':
#             raise Exception("Please give the design condition a unique name")
#         self.design_condition_dictionary[design_condition.parameters['name']] = design_condition
#         # TODO: implement check if the a design condition with the same (default name)
#         # is added. If that's the case, it will just override the previous design condition
#         # and there needs to be an exception

#     def _assemble_groups(self, design_condition_dictionary):
        
#         mechanics_group = None
#         nonmechanics_group = None
#         power_group = None
#         return mechanics_group, nonmechanics_group, power_group