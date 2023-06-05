from caddee.caddee_core.system_model.sizing_group.sizing_models.sizing_model import SizingModel
from caddee.csdl_core_modules.system_model_csdl.sizing_group_csdl.sizing_models_csdl.simple_battery_sizing_csdl import SimpleBatterySizingCSDL


class SimpleBatterySizing(SizingModel):
    def initialize(self, kwargs):
        # Parameters
        self.parameters.declare('component', default=None, types=None)
        self.parameters.declare('battery_packaging_fraction', default=0.1, types=float, lower=0, upper=1)
       
    def _assemble_csdl(self):
        battery_packaging_fraction = self.parameters['battery_packaging_fraction']
        csdl_model = SimpleBatterySizingCSDL(
            battery_packaging_fraction=battery_packaging_fraction,
        )
        return csdl_model
