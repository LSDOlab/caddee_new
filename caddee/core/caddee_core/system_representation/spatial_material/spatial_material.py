import numpy as np

from utils.caddee_base import CADDEEBase
from caddee.caddee_core.system_representation.system_representation import SystemRepresentation
from caddee.caddee_core.system_representation.component.component import Component
from caddee.caddee_core.system_representation.utils.material import Material
from caddee.primitives.primitive import Primitive
from caddee.primitives.bsplines import bspline_functions as bsf
from caddee.csdl_core.system_configuration_csdl.heaviside import PiecewiseHeavisideModel
import scipy.sparse as sps
import vedo
import array_mapper as am


class SpatialMaterial:
    '''
    Definition of a single material over some parametric space
    
    Parameters
    ----------

    
    '''
    def __init__(self, name:str, index:int, level_set_primitive:Primitive=None, heaviside_h:float=1e-4, properties:dict={}):
        self.name = name
        self.ls_primitive = level_set_primitive
        self.h = heaviside_h
        self.properties=properties
        properties['name'] = name
        properties['index'] = index



    def am_piecewise_heaviside(self, input, eps:float=1e-4):
        return am.custom_nonlinear_operation(input, PiecewiseHeavisideModel(shape=input.shape, eps=eps), None)

    def import_material(self, material:Material):
        pass

    def fit_levelset(self, fitting_points:np.ndarray, parametric_coords=None,
                     num_control_points:tuple=(10,), order:tuple=(4,)):
        self.ls_primitive = bsf.fit_bspline(fitting_points, paramatric_coordinates=parametric_coords,
                                            num_control_points=num_control_points, order=order)

    # def _heaviside(self, x):
    #     if x < -self.h:
    #         return 0
    #     elif x > self.h:
    #         return 1
    #     else: 
    #         return -1/4*(x/self.h)**3+3/4*(x/self.h)+1/2
    
    # def _heaviside_derivative(self, x):
    #     if x<-self.h or x > self.h:
    #         return 0
    #     return -3/4*(x/self.h)**2/self.h+3/4/self.h

    def fit_property(self, name:str, function_value):
        '''
        Fits new primitive to input data
        '''
        pass

    def evaluate_points(self, u_vec, v_vec):
        '''
        Evaluates the level set function at the parametric coordinates.
        '''
        points = self.ls_primitive.evaluate_points(u_vec, v_vec)
        points = self.am_piecewise_heaviside(points)
        return points

    def evaluate_derivative(self, u_vec, v_vec):
        '''
        Evaluates the derivative of the level set function at the parametric coordinates.
        '''
        # num_control_points = self.shape[0] * self.shape[1]
        
        # basis1 = self.compute_derivative_evaluation_map(u_vec, v_vec)
        # derivs1 = basis1.dot(self.control_points.reshape((num_control_points, 3)))

        # return derivs1 
        pass

    def evaluate_second_derivative(self, u_vec, v_vec):
        '''
        Evaluates the second derivative of the level set function at the parametric coordinates.
        '''
        # num_control_points = self.shape[0] * self.shape[1]
        
        # basis2 = self.compute_second_derivative_evaluation_map(u_vec, v_vec)
        # derivs2 = basis2.dot(self.control_points.reshape((num_control_points, 3)))

        # return derivs2
        pass