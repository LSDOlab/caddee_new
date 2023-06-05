from csdl import Model


# Might not need this class 
class SolverInputs:
    def __init__(self, name, var_type, var_shape) -> None:
        # shape is NOT the variable shape, 
        # Ex: VLMMesh:
        #   - shape = (nx,ny,3)
        # vlm_solver_u = 
        # VLM requires caddee to get the right shape
        self.name = name
        self.type = var_type
        self.shape = var_shape
        self.solver_inputs = {}
    
    def set_input(self):
        self.solver_inputs[self.name] = [self.type,self.shape]
        return self.solver_inputs

# This BEM model class is a dummy substitute for BEM model from the solver package
class BEMModel(Model):
    def initialize(self):
        self.parameters.declare('mesh')
        self.parameters.declare('shape')
        self.parameters.declare('prefix')

    def define(self):
        mesh = self.parameters['mesh']
        shape = self.parameters['shape']
        prefix = self.parameters['prefix']
        thrust_vector = mesh.thrust_vector
        # print('thrust_vector:----------------', thrust_vector)
        tv = self.declare_variable('thrust_vector',thrust_vector)
        rpm = self.declare_variable(prefix + '_rpm',shape=shape)

        dummy_output_1 = rpm * 1e-3
        dummy_output_2 = tv * 10.

        self.register_output(prefix + '_dummy_output_1',dummy_output_1)
        self.register_output(prefix + '_dummy_output_2',dummy_output_2)

# This is what would be imported: from lsdo_rotor import BEMModel
class BEMModel():
    def __init__(self, mesh, shape=None) -> None:
        self.mesh = mesh
        self.solver_inputs = { # might be a good idea to specify the TYPE of model input (general across all models)
            'u' : (1,),
            'v' : (1,),
            'w' : (1,),
            'p' : (1,),
            'q' : (1,),
            'r' : (1,),
            'phi' : (1,),
            'theta' : (1,),
            'psi' : (1,),
            'z' : (1,),
            'rpm' : (1,),
        }
        self.solver_outputs = {
            'F' : (3,),
            'M' : (3,),
            'thrust' : (1,),
            'torque' : (1,),
        }
        self.shape = shape # shape will be set when the solver is added to a csdl submodel (pointer)
        self.prefix = self.mesh.component.name + '_' + self.__class__.__name__
    
    # This is creates the top-level csdl class within BEM
    def create_csdl_model(self):
        bem_csdl = BEMCSDL(
            mesh=self.mesh,
            shape=self.shape,
            prefix=self.prefix,
        )
        return bem_csdl

# -------------------------------------------------------------# 
"""Somewhere inside the caddee directory """
class MechanicsModel(BaseModel): # pure python side 
    """Variables that are common to all (csdl) models that inherit from mechanics model"""
    def initialize(self, kwargs):
        self.variables_metadata.declare('u')
        self.variables_metadata.declare('v')
        self.variables_metadata.declare('w')
        # ...


""" in lsdo_rotor (top-level) """ 
from caddee.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import MechanicsModel
from lsdo_rotor.core.bem_csdl import BEMCSDL # import from a lower-level the main csdl bem model that does all the computations 

class BEMModel(MechanicsModel): # Pure python side
    def initialize(self, kwargs):
        self.variables_metadata.declare('rpm')
        # new variable class for mesh metadata
        # these will be promoted variables while the rest
        # will be unpromoted 
        self.variables_metadata.declare('radius')
        self.variables_metadata.declare('thrust_vector')
        self.variables_metadata.declare('thrust_origin')
        self.variables_metadata.declare('chord_distribution') 
        
        self.promoted_variables = ['radius', 'thrust_vector', 'thrust_origin', 'chord_distribution']

        self.shape = None # Will be set under the hood if used within CADDEE
        self.prefix = None  # ----"---- 
    
    def create_bem_csdl(self):
        bem_csdl = BEMCSDL(
            shape=self.shape, 
            prefix=self.prefix, 
        )
        return bem_csdl

# list of OptionsDictionary-like classes
# 1) parameters
# 2) variables 
# 3) operation_parameters 

# TODO: write out types of classes 
# Ideas
# base classes for each major type of class
#  condition 
#  component
#  model
#  geometry  

"""In caddee user run script"""
from lsdo_rotor import BEMModel
# other imports 
# ----------------------
# set up configuration and parameterization

# set up design scenario

# set up design condition
cruise = DesignCondition()

# mechanics group
mechanics_group = MechanicsGroup()
bem_model = BEMModel()
mechanics_group.add_model(bem_model)


"""Somewhere under the hood in caddee"""
bem_csdl = bem_model.create_bem_csdl()


