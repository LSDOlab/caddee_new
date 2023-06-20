from caddee.core.caddee_core.system_model.design_scenario.design_condition.model_group.model_group import ModelGroup
import numpy as np


class MechanicsGroup(ModelGroup): 
    def initialize(self, kwargs):
        self._struct_models = []
        self._struct_model_names = []
        super().initialize(kwargs)

    def _assemble_csdl(self):
        from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.mechanics_group_csdl.mechanics_group_csdl import MechanicsGroupCSDL
        csdl_model = MechanicsGroupCSDL(
            mechanics_group=self,
        )

        return csdl_model