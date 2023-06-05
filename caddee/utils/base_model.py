from caddee.utils.variables_metadata_dictionary import VariablesMetadataDictionary
from caddee.utils.output_variables_dictionary import OutputVariablesDictionary
from abc import ABC, abstractmethod


class BaseModel(ABC):
    def __init__(self, **kwargs):
        self.variables_metadata = VariablesMetadataDictionary()
        self.output_variables = OutputVariablesDictionary()
        self.initialize(kwargs)
        self.variables_metadata.update(kwargs)
        self.output_variables.update(kwargs)

    @abstractmethod
    def initialize(self, kwargs):
        raise NotImplementedError



