from caddee.utils.caddee_base import CADDEEBase
from caddee.core.caddee_core.system_model.sizing_group.sizing_group import SizingGroup

class SystemModel(CADDEEBase):
    def initialize(self, kwargs):
        self.sizing_group = None
        self.design_scenario_dictionary = {}
        self.connections_list = []

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