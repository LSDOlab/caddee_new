from caddee.caddee_core.system_model.design_scenario.design_condition.model_group.model_group import ModelGroup
import numpy as np


class PowerGroup(ModelGroup): 
    def initialize(self, kwargs):    
        super().initialize(kwargs)

    def _assemble_csdl(self):
        from caddee.csdl_core_modules.system_model_csdl.design_scenario_csdl.design_condition_csdl.power_group_csdl.power_group_csdl import PowerGroupCSDL
        csdl_model = PowerGroupCSDL(
            power_group=self
        )

        return csdl_model