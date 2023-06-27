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
        system_parameterization = caddee.system_parameterization
        if system_parameterization is not None:
            system_parameterization_csdl = SystemParameterizationCSDL(
                system_parameterization=system_parameterization,
            )
            self.add_module(system_parameterization_csdl, 'system_parameterization')

       
        if system_representation is not None:
            print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$', system_representation.spatial_representation.outputs)
            if system_representation.spatial_representation.outputs:
                
                system_representation_csdl = SystemRepresentationCSDL(
                    system_representation=system_representation,
                )
                self.add_module(system_representation_csdl, 'system_representation')


        # system model
        system_model = caddee.system_model
        system_model_csdl = SystemModelCSDL(
            system_model=system_model,
            system_representation=system_representation,
            system_param=system_parameterization
        )
        self.add_module(system_model_csdl, 'system_model')

        # system_model_csdl = ModuleCSDL()

        # system_model_m3l_models = system_model.m3l_models
        # for m3l_model_name, m3l_model in system_model_m3l_models.items():
        #     m3l_csdl_model = m3l_model.assemble_csdl()
        #     system_model_csdl.add_module(m3l_csdl_model, m3l_model_name)

        # design_scenarios = system_model.design_scenario_dictionary
        # for design_scenario_name, design_scenario in design_scenarios.items():
        #     design_conditions_dict = design_scenario.design_condition_dictionary
        #     for design_condition_name, design_condition in design_conditions_dict.items():
        #         design_condition_csdl_model = design_condition._assemble_csdl()
        #         system_model_csdl.add_module(design_condition_csdl_model, design_condition_name)
                
        #         design_condition_m3l_models = design_condition.m3l_models
        #         for design_condition_m3l_model_name, design_condition_m3l_model in design_condition_m3l_models.items():
        #             design_condition_m3l_model_csdl_model = design_condition_m3l_model.assemble_csdl()
        #             design_condition_csdl_model.add_module(design_condition_m3l_model_csdl_model, design_condition_m3l_model_name)


        
        
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
                
       