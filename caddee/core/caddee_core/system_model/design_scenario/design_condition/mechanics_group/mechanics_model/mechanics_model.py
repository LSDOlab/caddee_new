from caddee.utils.base_model import BaseModel
from caddee.utils.caddee_base import CADDEEBase
import numpy as np


class MechanicsModel(CADDEEBase): 
    def initialize(self, kwargs):
        self.parameters.declare('struct_solver', default=False, types=bool)
        self.parameters.declare('compute_mass_properties', default=False, types=bool)
        
        # Standard outputs of any mechanics model (forces F and moments M)
        # self.output_variables_metadata.declare(name='F', default=np.array([0, 0, 0]))
        # self.output_variables_metadata.declare(name='M', default=np.array([0, 0, 0]))