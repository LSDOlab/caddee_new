from caddee.utils.caddee_base import CADDEEBase
from caddee.core.caddee_core.system_model.sizing_group.sizing_group import SizingGroup
import m3l

# class SystemModel(CADDEEBase):
class SystemModel(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare(name='name', default='system_model')
        self.sizing_group = None
        self.design_scenario_dictionary = {}
        self.connections_list = []
        self.m3l_models = dict()

    def assign_attributes(self):
        self.name = self.parameters['name']

    def add_design_scenario(self,design_scenario):
        if design_scenario.parameters['name'] == '':
            raise Exception("Design scenario name is empty ''. Please name the design scenario uniquely.")
        else:
            self.design_scenario_dictionary[design_scenario.parameters['name']] = design_scenario


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
    
    def add_m3l_model(self, name, model):
        from m3l import Model
        if not isinstance(model, Model):
            raise TypeError("model_group must be of type 'm3l.Model' ")
        else:
            self.m3l_models[name] = model