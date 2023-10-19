import csdl 
from caddee.utils.caddee_base import CADDEEBase
from caddee.core.caddee_core.system_model.design_scenario.design_condition.power_group.power_model.power_model import PowerModel
from caddee.core.caddee_core.system_representation.component.component import Component

from csdl import GraphRepresentation


class DummyMotorCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes', default=None, types=int, allow_none=True)
        self.parameters.declare('prefix', default=None, types=str, allow_none=True)

    def define(self):
        num_nodes = self.parameters['num_nodes']
        shape = (num_nodes, )

        prefix = self.parameters['prefix']
        rpm = self.declare_variable('rpm', shape=shape, vectorized=True)
        load_torque = self.declare_variable('Q', shape=shape)

        power = csdl.sum(rpm * load_torque, axes = (0,))
        self.register_output('power', power)


class DummyMotorMesh(CADDEEBase):
    def initialize(self, kwargs):
        self.parameters.declare('component', types=Component)
        self.parameters.declare('pole_pairs', default=6, types=int)
        self.parameters.declare('phases', default=3, types=int)
        self.parameters.declare('num_slots', default=36, types=int)
        self.parameters.declare('123', default=6, types=(int, float))

class DummyMotorModel(PowerModel):
    def initialize(self, kwargs):
        self.num_nodes = None 
        # self.parameters.declare('mesh', types=DummyMotorMesh)
        self.parameters.declare('component', types=Component)
        

    def _assemble_csdl(self):
        prefix = self.parameters['component'].parameters['name']
        csdl_model = DummyMotorCSDL(
            module=self,
            num_nodes=self.num_nodes,
            prefix=prefix,
        )
        GraphRepresentation(csdl_model)
        return csdl_model


