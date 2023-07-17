# from caddee.utils.caddee_base import CADDEEBase

import array_mapper as am
import numpy as np

from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation

# class SystemParameterization(CADDEEBase):
class SystemParameterization():
    # def initialize(self, kwargs):
    #     pass

    def __init__(self, system_representation:SystemRepresentation, geometry_parameterizations:dict={}, material_parameterizations:dict={}):
        '''
        Creates a SystemParameterization object.
        '''
        self.system_representation = system_representation
        self.geometry_parameterizations = geometry_parameterizations.copy()
        self.material_parameterizations = material_parameterizations.copy()
        self.inputs = {}

    def add_geometry_parameterization(self, geometry_parameterization):
        '''
        Adds a geometry parameterization to the system parameterization.

        Parameters
        ------------
        geometry_parameterization: {FFDSet}
            The parameterization object
        '''
        self.geometry_parameterizations[geometry_parameterization.name] = geometry_parameterization


    def add_input(self, name:str, quantity:am.NonlinearMappedArray, value:np.ndarray=None):
        '''
        Adds an input to the geometry parameterization solver.

        Parameters
        ----------
        name : str
            The name of the variable. This name will be used for connection purposes.
        input : am.NonlinearMappedarray
            The quantity to be driven to the input value.
        value : np.ndarray
            The default value for input.
        '''
        parameterization_input = ParameterizationInput(name=name, quantity=quantity, shape=quantity.shape, value=value)
        self.inputs[name] = parameterization_input


    def setup(self):
        '''
        Performs setup 
        '''
        for geometry_parameterization in list(self.geometry_parameterizations.values()):
            geometry_parameterization.setup()
        for material_parameterization in list(self.material_parameterizations.values()):
            material_parameterization.setup()
    

    def evaluate(self, inputs):
        '''
        Evaluates the outputs given the inputs.
        '''
        raise Exception("Sorry, not implemented yet.")
    

    def assemble_csdl(self):
        '''
        Constructs and returns the CADDEE model.
        '''
        from caddee.core.csdl_core.system_parameterization_csdl.system_parameterization_csdl import SystemParameterizationCSDL
        return SystemParameterizationCSDL(system_parameterization=self)


from dataclasses import dataclass

@dataclass
class ParameterizationInput:
    name : str
    quantity : am.NonlinearMappedArray
    shape : tuple
    value : np.ndarray = None

    def __post_init__(self):
        if self.value is None:
            self.value = self.quantity.value