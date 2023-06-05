from caddee.utils.caddee_base import CADDEEBase
import numpy as np
import array_mapper as am


class VLMMesh(CADDEEBase):
    def initialize(self, kwargs):
        self.parameters.declare('component')
        self.parameters.declare('num_spanwise_vlm')
        self.parameters.declare('num_chordwise_vlm')
    
    def _assemble_mesh(self):
        wing = self.parameters['component']
        num_spanwise = self.parameters['num_spanwise_vlm']
        num_chordwise = self.parameters['num_chordwise_vlm']
        leading_edge = wing.project(np.linspace(np.array([0., -10., 0.]), np.array([0., 10., 0.]), num_spanwise))  # returns MappedArray
        trailing_edge = wing.project(np.linspace(np.array([10., -10., 0.]), np.array([10., 10., 0.]), num_spanwise))   # returns MappedArray
        chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise)
        wing_upper_surface_wireframe = wing.project(chord_surface + np.array([0., 0., 10.]), direction=np.array([0., 0., -1.]))
        wing_lower_surface_wireframe = wing.project(chord_surface - np.array([0., 0., 10.]), direction=np.array([0., 0., 1.]))
        wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1
    
        return wing_camber_surface
