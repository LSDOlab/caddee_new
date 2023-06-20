from csdl import Model
# from caddee.core.caddee_core.caddee import CADDEE
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
# from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
# from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization
from caddee.core.caddee_core.system_model.system_model import SystemModel
from caddee.core.csdl_core.system_model_csdl.system_model_csdl import SystemModelCSDL
from caddee.core.csdl_core.system_parameterization_csdl.system_parameterization_csdl import SystemParameterizationCSDL
from caddee.core.csdl_core.system_representation_csdl.system_representation_csdl import SystemRepresentationCSDL

class CADDEECSDL(ModuleCSDL):
    """
    Top-level caddee csdl class

    Parameters
    ----------
    caddee - python class that contains all the user information
    """
    
    def initialize(self):
        self.parameters.declare('caddee')
        # establish a pattern where the pure python instances corresponding to 
        # csdl object are declared as parameters (or their contained classes)

    
    def define(self):
        # caddee
        caddee = self.parameters['caddee']
        # system configuration & parameterization
        system_representation = caddee.system_representation
        if system_representation.power_systems_architecture:
            psa_connections = system_representation.power_systems_architecture.connections_list
        else:
            psa_connections = []
        system_parameterization = caddee.system_parameterization
        if system_parameterization is not None:
            system_parameterization_csdl = SystemParameterizationCSDL(
                # system_representation=system_representation,
                system_parameterization=system_parameterization,
            )
            self.add_module(system_parameterization_csdl, 'system_parameterization')

        if system_representation.outputs:
            system_representation_csdl = SystemRepresentationCSDL(
                system_representation=system_representation,
            )
            self.add_module(system_representation_csdl, 'system_representation')


        # system model
        system_model = caddee.system_model
        system_model_csdl = SystemModelCSDL(
            system_model=system_model,
            psa_connections=psa_connections,
            system_representation=system_representation,
            system_param=system_parameterization
        )
        self.add_module(system_model_csdl, 'system_model')
        
        
        # NOTE: previously we would suppress promotions but now, objects like meshes 
        # that live in system_representation_csdl need to be known downstream in 
        # system_model_csdl, so here, it is ok to promote
        
                
       


# from csdl import Model
# from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
# from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization
# from caddee.core.caddee_core.system_model.system_model import SystemModel
# from caddee.core.csdl_core.system_model_csdl.system_model_csdl import SystemModelCSDL
# from caddee.core.csdl_core.system_representation_csdl.system_representation_csdl import SystemRepresentationCSDL


# class CADDEECSDL(Model):
#     """
#     Top-level caddee csdl class

#     There are three parameters that contain the three 
#     python classes contained in the CADDEE class
#         1) SystemRepresentation
#         2) SystemParameterization
#         3) SystemModel
#     """
    
#     def initialize(self):
#         self.parameters.declare('caddee', types=CADDEE)
#         self.parameters.declare('system_representation', types=SystemRepresentation)
#         self.parameters.declare('system_parameterization')# , types=(SystemParameterization, None))
#         self.parameters.declare('system_model', types=SystemModel)
#         # establish a pattern where the pure python instances corresponding to 
#         # csdl object are declared as parameters (or their contained classes)

#         self.system_representation_csdl = None
#         self.system_model_csdl = None
    
#     def define(self):
#         # system configuration & parameterization
#         system_representation = self.parameters['system_representation']
#         system_parameterization = self.parameters['system_parameterization']
#         system_representation_csdl = SystemRepresentationCSDL(
#             system_representation=system_representation,
#             system_parameterization=system_parameterization,
#         )
#         self.add(system_representation_csdl, 'system_representation')
#         self.system_representation_csdl = system_representation_csdl

#         # system model
#         system_model = self.parameters['system_model']
#         system_model_csdl = SystemModelCSDL(system_model=system_model)
#         self.add(system_model_csdl, 'system_model')
#         self.system_model_csdl = system_model_csdl
        
        
#         # NOTE: previously we would suppress promotions but now, objects like meshes 
#         # that live in system_representation_csdl need to be known downstream in 
#         # system_model_csdl, so here, it is ok to promote
        

#         test_input = self.declare_variable('test_csdl_input', 0.)
#         self.register_output('caddee_csdl_test_output', test_input + 1)
                
       