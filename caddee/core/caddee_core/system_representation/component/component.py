import numpy as np
import m3l
from caddee.utils.caddee_base import CADDEEBase
from caddee.core.caddee_core.system_representation.spatial_representation import SpatialRepresentation

class Component(m3l.ExplicitOperation):
    '''
    Groups a component of the system.

    Parameters
    -----------
    name : str
        The name of the component

    spatial_representation : SpatialRepresentation, optional
        The mechanical structure that this component is grouping

    primitive_names : list of strings
        The names of the mechanical structure primitives to be included in the component
    '''
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str)
        self.parameters.declare('spatial_representation', default=None, types=SpatialRepresentation, allow_none=True)
        self.parameters.declare('primitive_names', default=[], allow_none=True, types=list)

    def assign_attributes(self):
        self.name = self.parameters['name']
        self.spatial_representation = self.parameters['spatial_representation']
        self.primitive_names = self.parameters['primitive_names']
        # self.primitives = self.spatial_representation.get_primitives(search_names=self.primitive_names)

    def get_primitives(self):
        return self.spatial_representation.get_primitives(search_names=self.primitive_names)

    def get_geometry_primitives(self):
        return self.spatial_representation.get_geometry_primitives(search_names=self.primitive_names)
    
    def project(self, points:np.ndarray, properties:list=['geometry'], direction:np.ndarray=None, grid_search_n:int=25,
                max_iterations=100, offset:np.ndarray=None, plot:bool=False, force_reprojection:bool=False):
        '''
        Projects points onto the system.

        Parameters
        -----------
        points : {np.ndarray, am.MappedArray}
            The points to be projected onto the system.
        targets : list, optional
            The list of primitives to project onto.
        direction : {np.ndarray, am.MappedArray}, optional
            An axis for perfoming projection along an axis. The projection will return the closest point to the axis.
        grid_search_n : int, optional
            The resolution of the grid search prior to the Newton iteration for solving the optimization problem.
        max_iterations : int, optional
            The maximum number of iterations for the Newton iteration.
        properties : list
            The list of properties to be returned (in order) {geometry, parametric_coordinates, (material_name, array_of_properties),...}
        offset : np.ndarray
            An offset to apply after the parametric evaluation of the projection. TODO Fix offset!!
        plot : bool
            A boolean on whether or not to plot the projection result.
        '''
        
        comp_name = self.parameters['name']
        projected_points = self.spatial_representation.project(points=points, targets=self.primitive_names.copy(), properties=properties, 
                direction=direction, grid_search_n=grid_search_n, max_iterations=max_iterations, offset=offset, plot=plot, comp_name=comp_name, force_reprojection=force_reprojection)

        return projected_points

    def plot(self):
        self.spatial_representation.plot(primitives=self.primitive_names)

        
































    # def define_custom_geometry(self, name, val, csdl_var=True, computed_upstream=True, dv_flag=False, lower=None, upper=None, scaler=None):
    #     """
    #     Method to define custom geometry of a component.
    #     Examples: rotor radius, wing corner points, etc.

    #     This method declares a new variable in the variable metadata_dictionary
    #     of the component.
        
    #     Will be depreciated in the future!
    #     """
        
    #     return
    
    # def add_component_state(name, val, design_condition, csdl_var=True, computed_upstream=False, dv_flg=False, upper=None, lower=None, scaler=None):
    #     """
    #     Method for setting a component state.

    #     For this method we again have the keyword "computed_upstream". In addition, 
    #     we have a required "design_condition" argument because CADDEE needs to know
    #     to which condition the component state belongs. 

    #     This method also would live in the Component class. It would "add" the 
    #     states of component-specific variables like 'rpm' or 'elevator_deflection'.
    #     This method would declare a new input to component's variables_metadata
    #     dictionary 

    #     The reason we're using add here is that it is more extensible. It will be 
    #     difficult to predict what kind of variables components will have. 
        
    #     Caddee will later on check whether this variable also exists as an entry 
    #     in the variable_metadata dictionary of the model/solver associated with 
    #     the component and throw an error if it doesn't exist in there. 
    #     """


    #     return