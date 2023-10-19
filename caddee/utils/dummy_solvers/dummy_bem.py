from csdl import Model 
import csdl
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import MechanicsModel
from caddee.core.caddee_core.system_representation.component.component import Component
from caddee.utils.caddee_base import CADDEEBase
from caddee.utils.variable_group import VariableGroup
import numpy as np

from csdl import GraphRepresentation



class DummyBEMCSDLModules(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes')
        self.parameters.declare('num_blades')
        self.parameters.declare('prefix')


    def define(self):
        num_nodes = self.parameters['num_nodes']
        num_blades = self.parameters['num_blades']
        prefix = self.parameters['prefix']

        # Aircraft states
        u = self.declare_variable('u', shape=(num_nodes, ), vectorized=True)
        v = self.declare_variable('v', shape=(num_nodes, ), vectorized=True)
        w = self.declare_variable('w', shape=(num_nodes, ), vectorized=True)
        p = self.declare_variable('p', shape=(num_nodes, ), vectorized=True)
        self.register_output('p_test', p * 1)
        q = self.declare_variable('q', shape=(num_nodes, ), vectorized=True)
        self.register_output('q_test', q * 1)
        r = self.declare_variable('r', shape=(num_nodes, ), vectorized=True)
        self.register_output('r_test', r * 1)
        phi = self.declare_variable('phi', shape=(num_nodes, ), vectorized=True)
        self.register_output('phi_test', phi * 1)
        theta = self.declare_variable('theta', shape=(num_nodes, ), vectorized=True)
        self.register_output('theta_test', theta * 1)
        psi = self.declare_variable('psi', shape=(num_nodes, ), vectorized=True)
        self.register_output('psi_test', psi * 1)
        x = self.declare_variable('x', shape=(num_nodes, ), vectorized=True)
        self.register_output('x_test', x * 1)
        y = self.declare_variable('y', shape=(num_nodes, ), vectorized=True)
        self.register_output('y_test', y * 1)
        z = self.declare_variable('z', shape=(num_nodes, ), vectorized=True)
        self.register_output('z_test', z * 1)

        # BEM-specific variables
        rpm = self.declare_variable('rpm', shape=(num_nodes, ), vectorized=True, computed_upstream=False)
        self.print_var(rpm)
        self.print_var(u)
        r = self.declare_variable(f'{prefix}_radius', shape=(1, ), promotes=True)
        R = csdl.expand(r, (num_nodes, ))
        self.print_var(R)
        # NOTE: prefix only for mesh-like variables

        # Some dummy computations for thrust and torque
        angular_speed = (rpm / 60) * 2 * np.pi
        V_tip = R * angular_speed
        u_theta = 0.5 * V_tip
        V_x = (u**2 + v**2 + w**2)**0.5
        u_x = 1.3 * V_x

        dT = T = 4 * np.pi * R * u_x * (u_x - V_x) * num_blades
        dQ = Q = 2 * np.pi * R * u_x * u_theta * num_blades

        self.register_output('dT', dT*1)
        self.register_output('dQ', dQ*1)

        # T = csdl.sum(dT, axes = (1,)) / shape[2]
        # Q = csdl.sum(dQ, axes = (1,)) / shape[2]

        self.register_output('T', T)
        self.register_output('Q', Q)

        self.register_output('F', csdl.expand(T*2, (num_nodes, 3), 'i->ij'))
        self.register_output('M', csdl.expand(Q*2, (num_nodes, 3), 'i->ij'))

class DummyBEMModules(MechanicsModel):
    def initialize(self, kwargs): 
        self.parameters.declare('component', types=Component)
        self.num_nodes = 1
        
    def _assemble_csdl(self): 
        component = self.parameters['component']
        prefix = component.parameters['name']
        csdl_model = DummyBEMCSDLModules(
            module=self,
            num_nodes=self.num_nodes, 
            num_blades=3,
            name='BEM',
            prefix=prefix,
        )
        GraphRepresentation(csdl_model)
        return csdl_model


class DummyBEMCSDL(Model):
    def initialize(self):
        self.parameters.declare('num_nodes', default=None, types=int, allow_none=True)
        self.parameters.declare('mesh', types=BEMDummyMesh, allow_none=True)
        self.parameters.declare('num_radial', default=1, types=int)
        self.parameters.declare('num_tangential', default=1, types=int)
        self.parameters.declare('prefix')
        self.parameters.declare('airfoil', default='NACA_4412', types=str)
        self.parameters.declare('num_blades', default=3, types=int)

    def define(self):
        num_nodes = self.parameters['num_nodes']
        num_radial = self.parameters['num_radial']
        num_tangential = self.parameters['num_tangential']
        shape = (num_nodes, num_radial, num_tangential)
        prefix = self.parameters['prefix']
        airfoil = self.parameters['airfoil']
        num_blades = self.parameters['num_blades']

        # Aircraft states
        u = self.declare_variable('u', shape=(num_nodes, ))
        v = self.declare_variable('v', shape=(num_nodes, ))
        w = self.declare_variable('w', shape=(num_nodes, ))
        p = self.declare_variable('p', shape=(num_nodes, ))
        self.register_output('p_test', p * 1)
        q = self.declare_variable('q', shape=(num_nodes, ))
        self.register_output('q_test', q * 1)
        r = self.declare_variable('r', shape=(num_nodes, ))
        self.register_output('r_test', r * 1)
        phi = self.declare_variable('phi', shape=(num_nodes, ))
        self.register_output('phi_test', phi * 1)
        theta = self.declare_variable('theta', shape=(num_nodes, ))
        self.register_output('theta_test', theta * 1)
        psi = self.declare_variable('psi', shape=(num_nodes, ))
        self.register_output('psi_test', psi * 1)
        x = self.declare_variable('x', shape=(num_nodes, ))
        self.register_output('x_test', x * 1)
        y = self.declare_variable('y', shape=(num_nodes, ))
        self.register_output('y_test', y * 1)
        z = self.declare_variable('z', shape=(num_nodes, ))
        self.register_output('z_test', z * 1)

        # BEM-specific variables
        rpm = self.declare_variable('rpm', shape=(num_nodes, ))
        self.print_var(rpm)
        self.print_var(u)
        R = csdl.expand(self.declare_variable(prefix + '_radius', shape=(1, )), (num_nodes, ))
        self.print_var(R)
        # NOTE: prefix only for mesh-like variables

        # Some dummy computations for thrust and torque
        angular_speed = (rpm / 60) * 2 * np.pi
        V_tip = R * angular_speed
        u_theta = 0.5 * V_tip
        V_x = (u**2 + v**2 + w**2)**0.5
        u_x = 1.3 * V_x

        dT = T = 4 * np.pi * R * u_x * (u_x - V_x) * num_blades
        dQ = Q = 2 * np.pi * R * u_x * u_theta * num_blades

        self.register_output('dT', dT*1)
        self.register_output('dQ', dQ*1)

        # T = csdl.sum(dT, axes = (1,)) / shape[2]
        # Q = csdl.sum(dQ, axes = (1,)) / shape[2]

        self.register_output('T', T)
        self.register_output('Q', Q)

        self.register_output('F', csdl.expand(T*2, (num_nodes, 3), 'i->ij'))
        self.register_output('M', csdl.expand(Q*2, (num_nodes, 3), 'i->ij'))
        
class BEMDummyMesh(CADDEEBase): 
    #NOTE: shouldn't be inheriting from CADDEEBase because it shouldn't need set_module_input()
    def initialize(self, kwargs):
        self.parameters.declare('component', types=Component)
        self.parameters.declare('num_radial', default=1, types=int)
        self.parameters.declare('num_tangential', default=1, types=int)
        self.parameters.declare('airfoil', default='NACA_4412', types=str)
        self.parameters.declare('num_blades', default=3, types=int)

class BEMDummyModel(MechanicsModel):
    def initialize(self, kwargs):
        # Attributes (will be set under the hood when user input is processed)
        self.num_nodes = None
        
        # Component as a parameter
        self.parameters.declare('component', types=Component)
        self.parameters.declare('mesh', default=None, types=BEMDummyMesh, allow_none=True)
        
        # Inputs and Outputs 
        self.inputs = BEMInputs()
        self.outputs = BEMOutputs()
        

        # Depreciated
        #  self.variables_metadata.declare('rpm', default=0., types=(int,float)) 
        # NOTE: The line above is not what variables_metadata was intended. Here, the model 
        # developer simply wants to tell what model inputs and outputs are 
        # self.variables_metadata.declare('radius', default=0., types=(int,float), computed_upstream=True)
        
        # # Promoted variables (Depreciated)
        # self.promoted_variables = ['radius']

    def _assemble_csdl(self):
        # airfoil = self.parameters['mesh'].parameters['airfoil']
        # num_blades = self.parameters['mesh'].parameters['num_blades']
        # num_radial = self.parameters['mesh'].parameters['num_radial']
        # num_tangential = self.parameters['mesh'].parameters['num_tangential']
        prefix = self.parameters['component'].parameters['name']
        mesh = self.parameters['mesh']
        # self.promoted_variables = [prefix + '_' + i for i in self.promoted_variables]

        csdl_model = DummyBEMCSDL(
            num_nodes=self.num_nodes,
            mesh=mesh, 
            prefix=prefix,
            airfoil='NACA_4412',
            num_blades=3,
        )
        
        return csdl_model


class BEMInputs(VariableGroup):
    def initialize(self, kwargs): # Note: input_variable should be variable group 
        self.input_variables.add('rpm', caddee_input=True)
        self.input_variables.add('radius', promoted_variable=True)

class BEMOutputs(VariableGroup):
    def initialize(self, kwargs):
        self.output_variables.add('dT')
        self.output_variables.add('dQ')
        self.output_variables.add('T')
        self.output_variables.add('Q')
        self.output_variables.add('F')
        self.output_variables.add('M')
        self.output_variables.add('rotor_rpm')


# Thoughts on data xfer
#   - introduce StateGroup or VariableGroup class 
#       Ex: 
#           surface tractions (pressure and shear forces)
#           Boundary layer parameters (skin frication, mom./disp. thickness) (subclass or instance of StateGroup)

# cruise_condition.connect(
#     bem_model.get_output('distributed'), # will have to return model itself and output_line
#     motor_model.get_input('high_voltage_line'), # name of the variable group
#     left_motor_model.get_input('high_voltage_line'), 
# )