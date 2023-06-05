import csdl 
from caddee.utils.caddee_base import CADDEEBase
from caddee.caddee_core.system_model.design_scenario.design_condition.power_group.power_model.power_model import PowerModel
from caddee.caddee_core.system_representation.component.component import Component
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL


class DummyBatteryCSDL(ModuleCSDL):
    def initialize(self):
        pass

    def define(self):
        power_profile = self.register_module_input('power_profile', shape=(1, ))
        total_energy = self.register_module_input('total_energy', shape=(1, ))
        total_time = self.register_module_input('total_time', shape=(1, ), promotes=True)

        final_soc = total_energy - power_profile * total_time

        self.register_module_output('final_soc', final_soc)

class DummyBatteryMesh(CADDEEBase):
    def initialize(self, kwargs):
        self.parameters.declare('component', types=Component)

class DummyBatteryModel(PowerModel):
    def initialize(self, kwargs):
        # self.parameters.declare('mesh', types=DummyBatteryMesh)
        self.parameters.declare('component', types=Component)
        self.num_nodes = 1

    def _assemble_csdl(self):
        csdl_model = DummyBatteryCSDL()

        return csdl_model
