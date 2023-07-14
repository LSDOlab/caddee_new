import csdl
from csdl_om import Simulator
import numpy as np
import scipy.sparse as sps
import array_mapper as am

'''
NOTE: As of now, this only works for one actuation per configuration (and doesn't expand across time!)
'''

class TransientConfigurationCSDL(csdl.Model):
    '''
    Expands the design geometry to be copied across the number of time steps.
    '''

    def initialize(self):
      self.parameters.declare('configuration')

    def define(self):
        # Input parameters
        configuration = self.parameters['configuration']
        num_nodes = configuration.num_nodes
        initial_geometry_control_points = configuration.system_representation.spatial_representation.control_points['geometry']

        # Input this configuration's copy of geometry
        design_geometry = self.declare_variable('system_representation_geometry', val=initial_geometry_control_points)
        design_configuration_geometry = csdl.expand(design_geometry, shape=(num_nodes,) + design_geometry.shape, indices='ij->kij')
        self.register_output(configuration.name + '_geometry', design_configuration_geometry)
        

if __name__ == "__main__":
    # TODO
    pass