from csdl import Model


class NonmechanicsGroupCSDL(Model):
    def initialize(self):
        self.parameters.declare('nonmechanics_models_dictionary')
    
    def define(self):
        nonmech_models_dict = self.parameters['nonmechanics_models_dictionary']
        
        # Ideas is the that the object created in line 9 is a dictionary that 
        # contains all nonmechanics models that user has added. These models would
        # get added here to NonmechanicsGroupCSDL model. 
        for nonmech_model_name, nonmech_model_dict in nonmech_models_dict.items():
            nonmech_csdl_model = nonmech_model_dict['csdl_model']
            promoted_vars = nonmech_model_dict['promoted_vars']
            self.add(nonmech_model_name, nonmech_csdl_model, promotes=promoted_vars)
        
        # dummy csdl operations for testing
        test_var = self.declare_variable('nonmechanics_group_csdl_test', 10.)
        self.register_output('nonmechanics_group_csdl_test_out', test_var * 10)