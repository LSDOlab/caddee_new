import numpy as np

from caddee.utils.caddee_base import CADDEEBase

from caddee.core.caddee_core.system_representation.spatial_representation import SpatialRepresentation
from caddee.core.caddee_core.system_representation.component.component import Component, LiftingSurface, Rotor

# Type checking imports
import array_mapper as am


class SystemRepresentation(CADDEEBase):
    '''
    A SystemRepresentation object is the description of the phyiscal system.
    This description includes all components required to perform the desired analysis.

    Parameters
    -----------
    spatial_representation: SpatialRepresentation = None
        The spatial representation of the physical system.
    power_systems_architecture: list = None
        The power_systems_architecture reprsentation of the physical system.
    components: list = None
        The list of Component objects. Components are user-defined groupings of the system.
    '''

    def initialize(self, kwargs):
        # self.parameters.declare(name='file', allow_none=True)
        self.parameters.declare(name='spatial_representation', default=None, allow_none=True, types=SpatialRepresentation)
        # self.parameters.declare(name='power_systems_architecture', default=None, allow_none=True, types=power_systems_architectureRepresentation)
        self.parameters.declare(name='power_systems_architecture', default=None, allow_none=True, types=list)  # temporarily leaving this here so no error is thrown.
        self.parameters.declare(name='components', default=None, allow_none=True, types=list)
        self.power_systems_architecture = None
        self.components_dict = {}
        self.configurations = {}
        self.components : Components = None

    def assign_attributes(self):
        self.spatial_representation = self.parameters['spatial_representation']
        self.components_dict = self.parameters['components']
        if self.components_dict is None:
            self.components_dict = {}
        self.power_systems_architecture = self.parameters['power_systems_architecture']
        if self.power_systems_architecture is None:
            self.power_systems_architecture = {}

        if self.spatial_representation is None:
            self.spatial_representation = SpatialRepresentation()

        # file = self.parameters['file']

        # if file:
        #     self.spatial_representation.import_file(file_name=file)
        #     self.spatial_representation.refit_geometry(file_name=file)

    def set_spatial_representation(self, spatial_representation:SpatialRepresentation):
        self.spatial_representation = spatial_representation

    # def add_component(self, component):
    #     self.components_dict[component.name] = component
    def add_component(self, component):
        # print(component.parameters.__dict__['_dict'])
        component_name = component.parameters['name']
        if component_name in self.components_dict:
            raise Exception(f"Component with name '{component_name}' already exists.")
        else:
            if component_name == 'motor_comp':
                print(component_name)
                exit()
            self.components_dict[component_name] = component



    def assemble_components(self, **kwargs): pass

    '''
    Defines a connection between two components at a location or region on the respective components.
    '''
    def connect(self, component1:Component, component2:Component, 
                region_on_component1:am.MappedArray=None, region_on_component2:am.MappedArray=None,
                type='mechanical'):
        # NOTE: The regions can also be level set functions isntead of Mapped Arrays.
        pass
        

    def import_geometry(self, file_name : str):
        '''
        Imports geometry primitives from a file.

        Parameters
        ----------
        file_name : str
            The name of the file (with path) that containts the geometric information.
        '''
        self.spatial_representation.import_file(file_name=file_name)
        return self.spatial_representation

    
    def project(self, points:np.ndarray, targets:tuple=None, direction:np.ndarray=None,
                grid_search_n:int=25, max_iterations=100, offset:np.ndarray=None, plot:bool=False):
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
        return self.spatial_representation.project(points=points, targets=targets, direction=direction,
            grid_search_n=grid_search_n, max_iterations=max_iterations, offset=offset, plot=plot)

    def add_input(self, function, connection_name=None, val=None):
        pass
    
    def add_output(self, name, quantity):
        '''
        Adds an output to the system configuration.
        '''
        self.spatial_representation.add_output(name=name, quantity=quantity)


    def declare_configurations(self, names:list):
        '''
        Create new configurations based on the design configuration.
        '''
        # NOTE: This should return pointers to some sort of dummy objects that can store their additional information.
        #   -- These dummy objects must have methods for taking in the new information like transform or whatever its long term name is.
        for name in names:
            configuration = SystemConfiguration(system_representation=self, name=name)
            self.configurations[name] = configuration    # TODO replace name with dummy return object!!
        return self.configurations
    

    def make_components(self, **kwargs):
        """
        This method sets the 'Components' attribute of the SystemRepresentation class
        (i.e., it creates an instance of the 'Components' container class)

        Parameters:
        ------------
            kwargs: specify the desired name of your component as well as the corresponding name of the component within the OpenVSP geometry

        Example:
        -----------
            ```py
            assemble_components(
                fuselage='Fuselage_***.main',
                wing='Wing',
                h_tail='Tail_1',
                v_tail='Tail_2',
                rotor_blade_1='Rotor_1_blades, 0',
            )
            ```
            
            Here, the keys (i.e., fuselage, wing, h_tail, v_tail, rotor_blade_1) are user specified and the values
            (i.e., 'Fuselage_***.main', 'Wing', 'Tail_1', 'Tail_2', 'Rotor_1_blades, 0') are examples of how components are 
            named inside an OpenVSP geometry.
            
            After calling the method, a user can then access components to perform projections 
            or FFD by calling: 'comp = system_representation.components.comp'
        """

        comps = Components(
            spatial_representation=self.spatial_representation, **kwargs,
        )

        self.components = comps
        

    def assemble_csdl(self):
        '''
        Constructs and returns the CADDEE model.
        '''
        from caddee.core.csdl_core.system_representation_csdl.system_representation_csdl import SystemRepresentationCSDL
        return SystemRepresentationCSDL(system_representation = self)


class Components:
    """
    Container class for all instantiated components. 
    This class will be instantiated upon calling the 'assemble_components' method
    of the SystemRepresentation class. 
    """
    def __init__(self, spatial_representation : SpatialRepresentation, **kwargs) -> None:
        for comp_name, search_name in kwargs.items():
            prim_names = list(spatial_representation.get_geometry_primitives(search_names=[search_name]).keys())
            component = Component(
                name=comp_name, 
                spatial_representation=spatial_representation, 
                primitive_names=prim_names,
            )

            setattr(self, comp_name, component)

from caddee.core.caddee_core.system_representation.prescribed_actuations import PrescribedActuation

class SystemConfiguration(CADDEEBase):
    '''
    A SystemRepresentation object is the description of the phyiscal system.
    This description includes all description required to perform the desired analysis.

    Parameters
    -----------
    system_representation: SystemRepresentation
        The system representation that this is a configuration of.
    '''

    def initialize(self, kwargs):
        self.parameters.declare(name='system_representation', allow_none=False, types=SystemRepresentation)
        self.parameters.declare(name='name', allow_none=False, types=str)

        self.transformations = {}

    def assign_attributes(self):
        self.system_representation = self.parameters['system_representation']
        self.name = self.parameters['name']
        self.num_nodes = 1

    def set_num_nodes(self, num_nodes:int):
        self.num_nodes = num_nodes

    def actuate(self, transformation:PrescribedActuation):
        if self.num_nodes != 1:
            if np.isscalar(transformation.value) or len(transformation.value) != self.num_nodes:
                raise Exception(f"For {self.name} transformation, please input an actuation profile with the same length as num nodes.")
        else:
            if not np.isscalar(transformation.value):
                raise Exception(f"For {self.name} transformation, please input a scalar rotation value since it is not a transient configuration.")
        self.transformations[transformation.name] = transformation

    def add_output(self, name, output):
        self.outputs[name] = output


if __name__ == '__main__':
    import caddee.api as cd
    from caddee import IMPORTS_FILES_FOLDER

    lpc_rep = SystemRepresentation()
    lpc_param = cd. SystemParameterization(system_representation=lpc_rep)

    file_name = IMPORTS_FILES_FOLDER / 'LPC_final_custom_blades.stp'
    spatial_rep = lpc_rep.spatial_representation
    spatial_rep.import_file(file_name=file_name)
    spatial_rep.refit_geometry(file_name=file_name)

    lpc_rep.make_components(
        fuselage='Fuselage_***.main',
        wing='Wing',
        h_tail='Tail_1',
    )

    wing_geom_primitives = lpc_rep.components.wing


    wing_ffd_block = cd.make_ffd_block(
        coordinates='cartesian', # cartesian by default
        type='SRBG', # SRBG by default
        components=[lpc_rep.components.wing],
        num_condtrol_points=(11, 2, 2),
        order=(4, 2, 2),
        xyz_to_uvw=(1, 0, 2), 
    )

    wing_ffd_block.add_scale_v(order=2, num_dof=3) # name is optional (if None, name is component_name_scale_v)