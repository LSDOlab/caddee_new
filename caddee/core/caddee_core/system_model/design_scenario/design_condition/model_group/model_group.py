from caddee.utils.caddee_base import CADDEEBase
from caddee.utils.helper_functions.camel_to_snake import camel_to_snake


class ModelGroup(CADDEEBase):
    def initialize(self, kwargs):    
        self._all_models_list = None
        self._all_models_names_list = None
        self._all_models_num_nodes_list = None
        self._all_model_inputs = None
        self.models_dictionary = {}
        self.connections_list = []

    # DEPRECIATE: use add_module
    def add_model(self, model):
        model_name = camel_to_snake(type(model).__name__)

        if model.parameters['component']:
            comp = model.parameters['component']
            comp_vars_dict = comp.variables_metadata.__dict__['_dict']
            comp_name = comp.parameters['name']
            name =  comp_name + '_' + model_name
            for comp_var_name, comp_var in comp_vars_dict.items():
                if comp_var['value'] is None:
                    error_message = "Attempting to add model of type {}, ".format(type(model)) + \
                        "which performs analysis on component of name {}. ".format(comp_name) + \
                        "This component has variable {}, which has not been set. ".format(comp_var_name) + \
                        "Set this variable with <comp>.set_module_input()."
                    raise Exception(error_message)
            
        else:
            name = model_name
        self.models_dictionary[name] = model


    def add_module(self, model):
        model_name = camel_to_snake(type(model).__name__)

        if 'component' in model.parameters:
            comp = model.parameters['component']
            comp_name = comp.parameters['name']
            name =  f'{comp_name}_{model_name}'
            
        else:
            name = model_name
        self.models_dictionary[name] = model

    def connect(self, upstream_comp, upstream_vars, downstream_comp, downstream_vars_dict):
        """
        Method to connect components (and models) to specify data transfer. 

        Arguments:
        -----
            upstream_comp : (Component, Model)
                Upstream component or Model
            upstream_vars : list of strings
                Upstream variable(s) contained in instance of VariableGroup
            downstream_comp : (Component, Model)
                Downstream component or Model
            downstream_vars : list of strings
                Downstream variable(s) contained in instance of VariableGroup

        """
        self.connections_list.append((upstream_comp, upstream_vars, downstream_comp, downstream_vars_dict))
        return
