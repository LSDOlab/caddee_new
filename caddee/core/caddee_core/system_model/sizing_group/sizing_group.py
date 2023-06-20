from caddee.core.caddee_core.system_model.design_scenario.design_condition.model_group.model_group import ModelGroup


class SizingGroup(ModelGroup):
    def initialize(self, kwargs):
        # Need super here to get some attributes from ModelGroup 
        # (would be overwritten by this initialize otherwise)
        super().initialize(kwargs)
        self.models_dictionary = {}
        self.connections_list = []
    def connect(self, upstream_comp, upstream_vars, downstream_comp, downstream_vars_dict):
        """
        Method to connect components (and models) to specify data transfer. 

        Arguments:
        -----
            upstream_comp : (Component, Model)
                Upstream component or Model
            upstream_vars : str
                Upstream variable(s) contained in instance of VariableGroup
            downstream_comp : (Component, Model)
                Downstream component or Model
            downstream_vars : str
                Downstream variable(s) contained in instance of VariableGroup

        """
        self.connections_list.append((upstream_comp, upstream_vars, downstream_comp, downstream_vars_dict))
        return

    def _assemble_csdl(self):
        from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_group_csdl import SizingGroupCSDL
        csdl_model = SizingGroupCSDL(
            sizing_group=self,
        )
        
        return csdl_model
    
    # def add_sizing_model(self,model):
    #     self.sizing_models_dictionary[model.name] = model