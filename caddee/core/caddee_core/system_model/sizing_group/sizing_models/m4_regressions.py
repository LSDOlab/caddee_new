from caddee.core.caddee_core.system_model.sizing_group.sizing_models.sizing_model import SizingModel
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_csdl_full import M4RegressionsCSDL


class M4Regressions(SizingModel):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)
        
    def _assemble_csdl(self):
        csdl_model = M4RegressionsCSDL()

        return csdl_model

