from csdl import Model
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.core.caddee_core.system_model.sizing_group.sizing_group import SizingGroup
import numpy as np
import m3l
import csdl


class TotalConstantMassPropertiesM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        pass

    def compute(self) -> csdl.Model:
        return ConstantMassPropertiesCSDL(
            mass_input_names=self.mass_input_names,
            cg_input_names=self.cg_input_names,
            inertia_input_names=self.inertia_input_names,
        )
    
    def evaluate(self, *args):
        self.mass_input_names = []
        self.cg_input_names = []
        self.inertia_input_names = []
        self.name = 'total_constant_mass_properties'
        
        self.arguments = dict()
        for arg in args:
            arg_name = arg.name
            arg_model_name = arg.operation.name
            if arg_name == "mass":
                self.mass_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "inertia_tensor": 
                self.inertia_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "cg_vector":
                self.cg_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            else:
                raise Exception(f"Inputs muss either by 'mass', 'inertia_tensor', or 'cg_vector'. Received {arg_name}")
        
        # operation_csdl = self.compute(mass_input_names=mass_input_names, cg_input_names=cg_input_names, inertia_input_names=inertia_input_names)

        # total_mass_operation = m3l.CSDLOperation(name='total_mass_properties', arguments=arguments, operation_csdl=operation_csdl)

        mass = m3l.Variable(name='total_constant_mass', shape=(1, ), operation=self)
        cg_vector = m3l.Variable(name='total_constant_cg_vector', shape=(3, ), operation=self)
        inertia_tensor = m3l.Variable(name='total_constant_inertia_tensor', shape=(3, 3), operation=self)
        
        return mass, cg_vector, inertia_tensor

        # print(arguments)
            
class TotalMassPropertiesM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        pass

    def compute(self) -> csdl.Model:
        return VaryingMassPropertiesCSDL(
            mass_input_names=self.mass_input_names,
            cg_input_names=self.cg_input_names,
            inertia_input_names=self.inertia_input_names,
        )
    
    def evaluate(self, *args):
        self.mass_input_names = []
        self.cg_input_names = []
        self.inertia_input_names = []
        self.name = 'total_constant_mass_properties'
        
        self.arguments = dict()
        for arg in args:
            arg_name = arg.name
            arg_model_name = arg.operation.name
            if arg_name == "mass":
                self.mass_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "inertia_tensor": 
                self.inertia_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "cg_vector":
                self.cg_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "total_constant_mass":
                self.cg_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "total_constant_cg_vector":
                self.cg_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            elif arg_name == "total_constant_inertia_tensor":
                self.cg_input_names.append(f"{arg_model_name}.{arg_name}")
                self.arguments[f"{arg_model_name}.{arg_name}"] = arg
            else:
                raise Exception(f"Inputs muss either by 'mass', 'inertia_tensor', 'cg_vector', 'total_constant_mass', 'total_constant_cg_vector', or 'total_constant_inertia_tensor'. Received {arg_name}")
        
        # operation_csdl = self.compute(mass_input_names=mass_input_names, cg_input_names=cg_input_names, inertia_input_names=inertia_input_names)

        # total_mass_operation = m3l.CSDLOperation(name='total_mass_properties', arguments=arguments, operation_csdl=operation_csdl)

        mass = m3l.Variable(name='total_mass', shape=(1, ), operation=self)
        cg_vector = m3l.Variable(name='total_constant_cg_vector', shape=(3, ), operation=self)
        inertia_tensor = m3l.Variable(name='total_constant_inertia_tensor', shape=(3, 3), operation=self)
        
        return mass, cg_vector, inertia_tensor     

class ConstantMassPropertiesCSDL(BaseModelCSDL):
    """
    Computes total 'constant' mass properties of sizing models 
    that don't change across mission segments. 
    Ex: M4 Regressions, motor, battery sizing
    """
    def initialize(self):
        # self.parameters.declare('sizing_group', default=None, types=SizingGroup, allow_none=True)
        self.parameters.declare('mass_input_names', types=list)
        self.parameters.declare('cg_input_names', types=list)
        self.parameters.declare('inertia_input_names', types=list)

    def define(self):
        # sizing_group = self.parameters['sizing_group']
        mass_input_names = self.parameters['mass_input_names']
        cg_input_names = self.parameters['cg_input_names']
        inertia_input_names = self.parameters['inertia_input_names']
        
        
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

        for i in range(len(mass_input_names)):
            mass_name = mass_input_names[i]
            cg_name = cg_input_names[i]
            inertia_name = inertia_input_names[i]
            
            m_model = self.register_module_input(mass_name, shape=(1, ))
            cg_model = self.register_module_input(cg_name, shape=(3, ))
            cgx_model = cg_model[0]
            cgy_model = cg_model[1]
            cgz_model = cg_model[2]

            inertia_model = self.register_module_input(inertia_name, shape=(3, 3))
            ixx_model = csdl.reshape(inertia_model[0, 0], (1, ))
            iyy_model = csdl.reshape(inertia_model[1, 1], (1, ))
            izz_model = csdl.reshape(inertia_model[2, 2], (1, ))
            ixz_model = csdl.reshape(inertia_model[0, 2], (1, ))

            # Compute total cg
            cgx = (m * cgx + m_model * cgx_model) / (m + m_model)
            cgy = (m * cgy + m_model * cgy_model) / (m + m_model)
            cgz = (m * cgz + m_model * cgz_model) / (m + m_model)

            # Position vector elements
            pos_x = cgx_model - ref_pt[0]
            pos_y = cgy_model - ref_pt[1]
            pos_z = cgz_model - ref_pt[2]

            # Compute total inertia tensor 
            # NOTE: inertias from inidividual models are already taken with respect to a common 
            # global reference point (typically the nose of the aircraft); hence we don't apply 
            # Parallel axis theorem

            ixx = ixx + ixx_model # + m_model * (pos_y**2 + pos_z**2)
            iyy = iyy + iyy_model # + m_model * (pos_x**2 + pos_z**2)
            izz = izz + izz_model # + m_model * (pos_x**2 + pos_y**2)
            ixz = ixz + ixz_model #+ m_model * (-pos_x * pos_z)

            m_fudge = self.declare_variable('m_fudge', shape=(1, ), val=0)

            # Compute total mass
            m = m + m_model + m_fudge



        # models_dict = sizing_group.models_dictionary
        # for model_name, model in models_dict.items():
        #     # Declare individual mass properties from models 
        #     m_model = self.register_module_input(f"{model_name}.mass", shape=(1, ))
        #     cgx_model = self.register_module_input(f"{model_name}.cgx", shape=(1, ))
        #     cgy_model = self.register_module_input(f"{model_name}.cgy", shape=(1, ))
        #     cgz_model = self.register_module_input(f"{model_name}.cgz", shape=(1, ))
        #     ixx_model = self.register_module_input(f"{model_name}.ixx", shape=(1, ))
        #     iyy_model = self.register_module_input(f"{model_name}.iyy", shape=(1, ))
        #     izz_model = self.register_module_input(f"{model_name}.izz", shape=(1, ))
        #     ixz_model = self.register_module_input(f"{model_name}.ixz", shape=(1, ))

        #     # Compute total cg
        #     cgx = (m * cgx + m_model * cgx_model) / (m + m_model)
        #     cgy = (m * cgy + m_model * cgy_model) / (m + m_model)
        #     cgz = (m * cgz + m_model * cgz_model) / (m + m_model)

        #     # Position vector elements
        #     pos_x = cgx_model - ref_pt[0]
        #     pos_y = cgy_model - ref_pt[1]
        #     pos_z = cgz_model - ref_pt[2]

        #     # Compute total inertia tensor
        #     ixx = ixx + ixx_model + m_model * (pos_y**2 + pos_z**2)
        #     iyy = iyy + iyy_model + m_model * (pos_x**2 + pos_z**2)
        #     izz = izz + izz_model + m_model * (pos_x**2 + pos_y**2)
        #     ixz = ixz + ixz_model + m_model * (pos_x * pos_z)

        #     # Compute total mass
        #     m = m + m_model

        inertia_tensor = self.register_module_output('total_constant_inertia_tensor', shape=(3, 3), val=0)
        inertia_tensor[0, 0] = csdl.reshape(ixx, (1, 1))
        inertia_tensor[0, 2] = csdl.reshape(ixz, (1, 1))
        inertia_tensor[2, 0] = csdl.reshape(ixz, (1, 1))
        inertia_tensor[1, 1] = csdl.reshape(iyy, (1, 1))
        inertia_tensor[2, 2] = csdl.reshape(izz, (1, 1))

        cg_vector = self.register_module_output('total_constant_cg_vector', shape=(3, ), val=0)
        cg_vector[0] = cgx
        cg_vector[1] = cgy
        cg_vector[2] = cgz


        # Register total constant mass properties 
        self.register_module_output('total_constant_mass', m)
        
        
        # self.register_module_output('cgx_total_constant', cgx)
        # self.register_module_output('cgy_total_constant', cgy)
        # self.register_module_output('cgz_total_constant', cgz)
        # self.register_module_output('ixx_total_constant', ixx)
        # self.register_module_output('iyy_total_constant', iyy)
        # self.register_module_output('izz_total_constant', izz)
        # self.register_module_output('ixz_total_constant', ixz)


class VaryingMassPropertiesCSDL(BaseModelCSDL):
    """
    Computes total 'constant' mass properties of sizing models 
    that don't change across mission segments. 
    Ex: M4 Regressions, motor, battery sizing
    """
    def initialize(self):
        # self.parameters.declare('sizing_group', default=None, types=SizingGroup, allow_none=True)
        self.parameters.declare('mass_input_names', types=list)
        self.parameters.declare('cg_input_names', types=list)
        self.parameters.declare('inertia_input_names', types=list)

    def define(self):
        # sizing_group = self.parameters['sizing_group']
        mass_input_names = self.parameters['mass_input_names']
        cg_input_names = self.parameters['cg_input_names']
        inertia_input_names = self.parameters['inertia_input_names']
        
        
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

        for i in range(len(mass_input_names)):
            mass_name = mass_input_names[i]
            cg_name = cg_input_names[i]
            inertia_name = inertia_input_names[i]
            
            m_model = self.register_module_input(mass_name, shape=(1, ))
            cg_model = self.register_module_input(cg_name, shape=(3, ))
            cgx_model = cg_model[0]
            cgy_model = cg_model[1]
            cgz_model = cg_model[2]

            inertia_model = self.register_module_input(inertia_name, shape=(3, 3))
            ixx_model = csdl.reshape(inertia_model[0, 0], (1, ))
            iyy_model = csdl.reshape(inertia_model[1, 1], (1, ))
            izz_model = csdl.reshape(inertia_model[2, 2], (1, ))
            ixz_model = csdl.reshape(inertia_model[0, 2], (1, ))

            # Compute total cg
            cgx = (m * cgx + m_model * cgx_model) / (m + m_model)
            cgy = (m * cgy + m_model * cgy_model) / (m + m_model)
            cgz = (m * cgz + m_model * cgz_model) / (m + m_model)

            # Position vector elements
            pos_x = cgx_model - ref_pt[0]
            pos_y = cgy_model - ref_pt[1]
            pos_z = cgz_model - ref_pt[2]

            # Compute total inertia tensor 
            # NOTE: inertias from inidividual models are already taken with respect to a common 
            # global reference point (typically the nose of the aircraft); hence we don't apply 
            # Parallel axis theorem

            ixx = ixx + ixx_model # + m_model * (pos_y**2 + pos_z**2)
            iyy = iyy + iyy_model # + m_model * (pos_x**2 + pos_z**2)
            izz = izz + izz_model # + m_model * (pos_x**2 + pos_y**2)
            ixz = ixz + ixz_model #+ m_model * (-pos_x * pos_z)

            m_fudge = self.declare_variable('m_fudge', shape=(1, ), val=0)

            # Compute total mass
            m = m + m_model + m_fudge


        inertia_tensor = self.register_module_output('total_inertia_tensor', shape=(3, 3), val=0)
        inertia_tensor[0, 0] = csdl.reshape(ixx, (1, 1))
        inertia_tensor[0, 2] = csdl.reshape(ixz, (1, 1))
        inertia_tensor[2, 0] = csdl.reshape(ixz, (1, 1))
        inertia_tensor[1, 1] = csdl.reshape(iyy, (1, 1))
        inertia_tensor[2, 2] = csdl.reshape(izz, (1, 1))

        cg_vector = self.register_module_output('total_cg_vector', shape=(3, ), val=0)
        cg_vector[0] = cgx
        cg_vector[1] = cgy
        cg_vector[2] = cgz


        # Register total constant mass properties 
        self.register_module_output('total_mass', m)
        
