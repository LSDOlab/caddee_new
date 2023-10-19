from csdl import Model
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_group import MechanicsGroup
import numpy as np


class VaryingMassPropertiesCSDL(BaseModelCSDL):
    """
    Computes total 'varying' mass properties of sizing models 
    that do change across mission segments. 
    Ex: 
        - Structural sizing model that computes structural sizing
        estimates for different mission segment, 
        - Fuel burning mission where the total mass and cg of the 
        aircraft changes 
    """
    def initialize(self):
        self.parameters.declare('mechanics_group', default=None, types=MechanicsGroup)

    def define(self):
        mech_group = self.parameters['mechanics_group']
        
        ref_pt = self.declare_variable('ref_pt', shape=(3,), val=np.array([0, 0, 0]))
        
        # Initialize mass proporties as CSDL variables with zero value
        # Total mass
        m = self.create_input('m_compute_varying', val=0)
        
        # CG in the global reference frame
        cgx = self.create_input('cgx_varying', val=0)
        cgy = self.create_input('cgy_varying', val=0)
        cgz = self.create_input('cgz_varying', val=0)
        
        # Elements of the inertia tensor in the global reference frame
        ixx = self.create_input('ixx_varying', val=0)
        iyy = self.create_input('iyy_varying', val=0)
        izz = self.create_input('izz_varying', val=0)
        ixz = self.create_input('ixz_varying', val=0)

        # Loop over all sizing models and compute total mass properties 
        # of the system (using parallel axis theorem)

        mech_models = mech_group._struct_models
        mech_model_names = mech_group._struct_model_names
        counter = 0
        for mech_model in mech_models:
            model_name = mech_model_names[counter]
            # Declare individual mass properties from models 
            m_model = self.declare_variable(f"{model_name}.mass", shape=(1, ))
            cgx_model = self.declare_variable(f"{model_name}.cgx", shape=(1, ))
            cgy_model = self.declare_variable(f"{model_name}.cgy", shape=(1, ))
            cgz_model = self.declare_variable(f"{model_name}.cgz", shape=(1, ))
            ixx_model = self.declare_variable(f"{model_name}.ixx", shape=(1, ))
            iyy_model = self.declare_variable(f"{model_name}.iyy", shape=(1, ))
            izz_model = self.declare_variable(f"{model_name}.izz", shape=(1, ))
            ixz_model = self.declare_variable(f"{model_name}.ixz", shape=(1, ))

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
            
            counter += 1

        # Register total constant mass properties 
        self.register_output('m_total_varying', m * 1)
        self.register_output('cgx_total_varying', cgx * 1)
        self.register_output('cgy_total_varying', cgy * 1)
        self.register_output('cgz_total_varying', cgz * 1)
        self.register_output('ixx_total_varying', ixx * 1)
        self.register_output('iyy_total_varying', iyy * 1)
        self.register_output('izz_total_varying', izz * 1)
        self.register_output('ixz_total_varying', ixz * 1)
