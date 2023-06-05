from caddee.utils.options_dictionary import OptionsDictionary
from caddee.utils.variables_metadata_dictionary import VariablesMetadataDictionary
from abc import ABC, abstractmethod
from lsdo_modules.module.module import Module



class CADDEEBase(Module): #pass
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.assign_attributes()  # Added this to make code more developer friendly (more familiar looking)
        
    def assign_attributes(self):
        pass

# class CADDEEBase(ABC):
#     def __init__(self, **kwargs):
#         self.parameters = OptionsDictionary()
#         self.variables_metadata = VariablesMetadataDictionary()
#         self.initialize(kwargs)
#         self.parameters.update(kwargs)
#         self.connections_info_dictionary = {}
        
#     @abstractmethod
#     def initialize(self, kwargs):
#         raise NotImplementedError

#     def set_caddee_input(self, name, val, csdl_var=True, units='', 
#                     computed_upstream=False, dv_flag=False, lower=None, upper=None, 
#                     scaler=None):
#         """
#         General method to set a pre-declared variable. 
#         Classes (and variables) for which this method is meant:
#             - Component (e.g., rotor -->> rpm)
#             - DesignCondition (e.g., range, hover_time)
#             - MechanicsGroup (e.g., u, v, w, ..., x, y, z)
#             - NonmechanicsGroup
#             - PowerGroup
#         """
#         design_condition = None
#         if name not in self.variables_metadata:
#             raise ValueError("Unknown variable '{}'. "
#                              "Acceptable variables are {}.".format(name,list(self.variables_metadata.__dict__['_dict'].keys())))       
#         else:
#             # self.variables_metadata[name] = [val, csdl_var, design_condition, computed_upstream, dv_flag, lower, upper, scaler]
#             self.variables_metadata.__setitem__(name, val, csdl_var, design_condition, units, computed_upstream,
#                                                 dv_flag, lower, upper, scaler)

#     def connect_models(self, upstream, downstream):
#         """
#         General method to connect components (and models implicitly). 
#         """
#         from caddee.utils.helper_functions.camel_to_snake import camel_to_snake
#         connection_name = '{}_to_{}'.format(
#             camel_to_snake(type(upstream)), 
#             camel_to_snake(type(downstream))
#         ) 
#         self.connections_info_dictionary[connection_name] = (upstream, downstream)