from csdl import Model
from caddee.utils.caddee_base import CADDEEBase
# from caddee.core.csdl_core.caddee_csdl import CADDEECSDL



class CADDEE(CADDEEBase):
    def initialize(self, kwargs):
        self.system_representation = None 
        self.system_parameterization = None
        self.system_model = None

    # def assemble_csdl(self):
    #     """
    #     High-level method that assembles the caddee csdl model
        
    #     The class attributes of this class are passed in as csdl 
    #     model parameters into CADDEECSDL
        
    #     This method also checks that at least system_representation and
    #     system_model are not None. A user should not be using CADDEE 
    #     if they only want to run a single or a collection of csdl models, 
    #     and so the minimum requirement is to a have system_representation 
    #     and system_model.
        
    #     It is allowed if the user does not instantiate a SystemParameterization
    #     object since ffd, actuation, and material properties are not always
    #     needed in the analysis.
    #     """
        
    #     # if self.system_representation is None:
    #     #     pass
    #         # raise Exception('No instance of class SystemRepresentation added.')
    #     if self.system_model is None:
    #         raise Exception('No instance of class SystemModel added.')
    #     else:
    #         caddee_csdl = CADDEECSDL(
    #             caddee=self,
    #         )

    #     return caddee_csdl
    
    # def assemble_csdl_modules(self):
    #     from caddee.core.csdl_core.caddee_csdl import CADDEECSDL
    #     """
    #     High-level method that assembles the caddee csdl model
        
    #     The class attributes of this class are passed in as csdl 
    #     model parameters into CADDEECSDL
        
    #     This method also checks that at least system_representation and
    #     system_model are not None. A user should not be using CADDEE 
    #     if they only want to run a single or a collection of csdl models, 
    #     and so the minimum requirement is to a have system_representation 
    #     and system_model.
        
    #     It is allowed if the user does not instantiate a SystemParameterization
    #     object since ffd, actuation, and material properties are not always
    #     needed in the analysis.
    #     """
        
    #     if self.system_representation is None:
    #         raise Exception('SystemRepresentation attribute not set.')
        
    #     if self.system_model is None:
    #         raise Exception('SystemModel attribute not set.')
        
    #     else:
    #         caddee_csdl = CADDEECSDL(
    #             caddee=self,
    #         )
        
    #     # Some random csdl computation for testing 
    #     # test_input = caddee_csdl.create_input('caddee_test_input', val=10)
    #     # caddee_csdl.register_output('caddee_test_output', test_input*10)
        
    #     # caddee_csdl.connect('u', 
    #     #                     'bem_dummy_model.u')

    #     return caddee_csdl