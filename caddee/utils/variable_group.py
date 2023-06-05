from abc import ABC, abstractmethod


class VariableGroup(ABC):
    def __init__(self, **kwargs):
        self.connection_strings = ConnectionsDictionary()
        self.input_variables = InputsDictionary()
        self.output_variables = OutputsDictionary()
        self.initialize(kwargs)
        self.connection_strings.update(kwargs)

    @abstractmethod
    def initialize(self, kwargs):
        raise NotImplementedError


class InputsDictionary(dict):
    def __init__(self): pass

    def add(self, name : str, caddee_input : bool=False, computed_upstream : bool=False, used_upstream : bool=False, promoted_variable : bool=False):
        """
        Add a variable to the inputs dictionary

        Parameters
        ----------
        name : str
            The string name of the variable 
        computed_upstream : bool
            Whether the variables is computed upstream; 
            if True, this is a promoted variable
        caddee_input : bool 
            Whether the variable is a set as a CADDEE input by the user; 
            if True, caddee will connect this variable after vectorizing it. 
        
        """
        if caddee_input is True and computed_upstream is True:
            raise Exception(f"Attempting to specify that model input {name} is a 'caddee_input' while setting 'computed_upstream' to 'True'. "
            + "This is not possible since a 'caddee_input' is set at the top-level (user-defined script) and will therefore not be computed. "
            )
        self[name] = {
            'computed_upstream': computed_upstream,
            'used_upstream': used_upstream, # For variables like RPM that may be used across multiple models (may be confusing to solver developer)
            'caddee_input': caddee_input,
            'promoted_variable': promoted_variable, 
            # NOTE: user may not be familiar with promotions
        }
        # TODO: Error messages
        #   1) If caddee_input == True; 
        #       - computed_upstream can't be True
        #       - used_upstream can be True
        #       - promoted_variable shouldn't be True
        #   --> potential rule: only caddee_inputs will be vectorized 

    def update(self, in_dict):
        """
        Update the internal dictionary with the given one.

        Parameters
        ----------
        in_dict : dict
            The incoming dictionary to add to the internal one.
        """
        for name in in_dict:
            self[name] = in_dict[name]

class OutputsDictionary(dict):
    def __init__(self): pass

    def add(self, name : str, promote : bool=False):
        """
        Add a variable to the outputs dictionary

        Parameters
        ----------
        name : str
            The string name of the output variable 
        promote : bool
            Whether to "promote" this variable; 
            If True, CADDEE promotes this variable such that other models 
            also have access to this variable. 
        """
        self[name] = {
            'promote' : promote,
        }

    def update(self, in_dict):
        """
        Update the internal dictionary with the given one.

        Parameters
        ----------
        in_dict : dict
            The incoming dictionary to add to the internal one.
        """
        for name in in_dict:
            self[name] = in_dict[name]

class ConnectionsDictionary(dict): 
    def __init__(self): pass
        # self._dict = {}

    def add(self, name : str, promoted_variable=False): 
        # Note: added a keyword to justify the use of a nested dictionary.
        # Otherwise, we can just store the connections string in a list 
        # promoted variable seems reasonable as it will tell caddee when 
        # skip an explicit connection if a variable is promoted
        self[name] = {
            'promoted_variable' : promoted_variable,
        }

    def update(self, in_dict):
        """
        Update the internal dictionary with the given one.

        Parameters
        ----------
        in_dict : dict
            The incoming dictionary to add to the internal one.
        """
        for name in in_dict:
            self[name] = in_dict[name]


class BEMOutputs(VariableGroup):
    def initialize(self, kwargs):
        self.connection_strings.add('dT', promoted_variable=True)
        self.connection_strings.add('dQ')

class AcousticsInputs(VariableGroup):
    def initialize(self, kwargs):
        self.connection_strings.add('dT')
        self.connection_strings.add('dQ')

# bem_outputs = BEMOutputs()
# print(bem_outputs.connection_strings)

