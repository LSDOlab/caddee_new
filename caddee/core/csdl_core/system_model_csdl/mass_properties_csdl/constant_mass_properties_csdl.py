from csdl import Model
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.core.caddee_core.system_model.sizing_group.sizing_group import SizingGroup
import numpy as np


class ConstantMassPropertiesCSDL(BaseModelCSDL):
    """
    Computes total 'constant' mass properties of sizing models 
    that don't change across mission segments. 
    Ex: M4 Regressions, motor, battery sizing
    """
    def initialize(self):
        self.parameters.declare('sizing_group', default=None, types=SizingGroup, allow_none=True)

    def define(self):
        sizing_group = self.parameters['sizing_group']
        
        ref_pt = self.register_module_input('ref_pt', shape=(3,), val=np.array([0, 0, 0]))
        
        # Initialize mass proporties as CSDL variables with zero value
        # Total mass
        m = self.create_input('m_compute', val=0)
        
        # CG in the global reference frame
        cgx = self.create_input('cgx_compute', val=0)
        cgy = self.create_input('cgy_compute', val=0)
        cgz = self.create_input('cgz_compute', val=0)
        
        # Elements of the inertia tensor in the global reference frame
        ixx = self.create_input('ixx_compute', val=0)
        iyy = self.create_input('iyy_compute', val=0)
        izz = self.create_input('izz_compute', val=0)
        ixz = self.create_input('ixz_compute', val=0)

        # Loop over all sizing models and compute total mass properties 
        # of the system (using parallel axis theorem)

        models_dict = sizing_group.models_dictionary
        for model_name, model in models_dict.items():
            # Declare individual mass properties from models 
            m_model = self.register_module_input(f"{model_name}.mass", shape=(1, ))
            cgx_model = self.register_module_input(f"{model_name}.cgx", shape=(1, ))
            cgy_model = self.register_module_input(f"{model_name}.cgy", shape=(1, ))
            cgz_model = self.register_module_input(f"{model_name}.cgz", shape=(1, ))
            ixx_model = self.register_module_input(f"{model_name}.ixx", shape=(1, ))
            iyy_model = self.register_module_input(f"{model_name}.iyy", shape=(1, ))
            izz_model = self.register_module_input(f"{model_name}.izz", shape=(1, ))
            ixz_model = self.register_module_input(f"{model_name}.ixz", shape=(1, ))

            # Compute total cg
            cgx = (m * cgx + m_model * cgx_model) / (m + m_model)
            cgy = (m * cgy + m_model * cgy_model) / (m + m_model)
            cgz = (m * cgz + m_model * cgz_model) / (m + m_model)

            # Position vector elements
            pos_x = cgx_model - ref_pt[0]
            pos_y = cgy_model - ref_pt[1]
            pos_z = cgz_model - ref_pt[2]

            # Compute total inertia tensor
            ixx = ixx + ixx_model + m_model * (pos_y**2 + pos_z**2)
            iyy = iyy + iyy_model + m_model * (pos_x**2 + pos_z**2)
            izz = izz + izz_model + m_model * (pos_x**2 + pos_y**2)
            ixz = ixz + ixz_model + m_model * (pos_x * pos_z)

            # Compute total mass
            m = m + m_model

        # Register total constant mass properties 
        self.register_module_output('m_total_constant', m)
        self.register_module_output('cgx_total_constant', cgx)
        self.register_module_output('cgy_total_constant', cgy)
        self.register_module_output('cgz_total_constant', cgz)
        self.register_module_output('ixx_total_constant', ixx)
        self.register_module_output('iyy_total_constant', iyy)
        self.register_module_output('izz_total_constant', izz)
        self.register_module_output('ixz_total_constant', ixz)
