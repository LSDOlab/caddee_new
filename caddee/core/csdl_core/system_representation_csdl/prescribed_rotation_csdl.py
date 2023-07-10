import csdl
from csdl_om import Simulator
import numpy as np
import scipy.sparse as sps
import array_mapper as am

'''
NOTE: As of now, this only works for one actuation per configuration (and doesn't expand across time!)
'''

class PrescribedRotationCSDL(csdl.Model):
    '''
    Evaluates the system configuration outputs including meshes and user-specified quantities.
    '''

    def initialize(self):
      self.parameters.declare('configuration')
      self.parameters.declare('prescribed_rotation')

    def define(self):
        # Input parameters
        configuration = self.parameters['configuration']
        initial_geometry_control_points = configuration.system_representation.spatial_representation.control_points['geometry']
        prescribed_rotation = self.parameters['prescribed_rotation']

        rotation_name = prescribed_rotation.name
        component = prescribed_rotation.component
        axis_origin = prescribed_rotation.axis_origin
        axis_vector = prescribed_rotation.axis_vector
        default_value = prescribed_rotation.value
        units = prescribed_rotation.units

        # Input this configuration's copy of geometry
        initial_geometry_control_points_csdl = self.declare_variable('system_representation_geometry', val=initial_geometry_control_points)
        num_control_points = initial_geometry_control_points.shape[0]   # geometry already has to be in shape (ncp,3)

        # Evaluate actuation origin
        axis_origin_linear_map = axis_origin.linear_map
        axis_origin_offset_map = axis_origin.offset_map
        updated_axis_origin_csdl_without_offset = csdl.sparsematmat(initial_geometry_control_points_csdl, sparse_mat=axis_origin_linear_map)

        if axis_origin_offset_map is not None:
            axis_offset_map_csdl = self.create_output(f'{rotation_name}_axis_origin_offset_map', axis_origin_offset_map)
            axis_origin_csdl = updated_axis_origin_csdl_without_offset + axis_offset_map_csdl
        else:
            axis_origin_csdl = updated_axis_origin_csdl_without_offset
        axis_origin_csdl = csdl.reshape(axis_origin_csdl, axis_origin.shape)

        # Evaluate actuation axis
        axis_vector_linear_map = axis_vector.linear_map
        axis_vector_offset_map = axis_vector.offset_map
        updated_axis_vector_csdl_without_offset = csdl.sparsematmat(initial_geometry_control_points_csdl, sparse_mat=axis_vector_linear_map)

        if axis_vector_offset_map is not None:
            axis_offset_map_csdl = self.create_output(f'{rotation_name}_axis_vector_offset_map', axis_vector_offset_map)
            axis_vector_csdl = updated_axis_vector_csdl_without_offset + axis_offset_map_csdl
        else:
            axis_vector_csdl = updated_axis_vector_csdl_without_offset
        axis_vector_csdl = csdl.reshape(axis_vector_csdl, axis_vector.shape)
        normalized_axis = axis_vector_csdl / csdl.expand(csdl.pnorm(axis_vector_csdl), axis_vector_csdl.shape, 'i->ij')
        normalized_axis = csdl.reshape(normalized_axis, new_shape=(normalized_axis.shape[-1],))

        # Get the control points that are going to be affected by the actuation
        primitive_names = component.primitive_names
        spatial_representation = configuration.system_representation.spatial_representation
        rotating_control_points_indices = []
        for primitive_name in primitive_names:
            primitive_indices = list(spatial_representation.primitive_indices[primitive_name]['geometry'])
            rotating_control_points_indices = rotating_control_points_indices + primitive_indices
        num_rotating_points = len(rotating_control_points_indices)
        data = np.ones((num_rotating_points))
        indexing_map = sps.coo_matrix((data, (np.arange(num_rotating_points), np.array(rotating_control_points_indices))),
                                        shape=(num_rotating_points, num_control_points))
        indexing_map = indexing_map.tocsc()
        rotating_control_points = csdl.sparsematmat(initial_geometry_control_points_csdl, sparse_mat=indexing_map)

        # Translate control points into actuation origin frame
        axis_origin_csdl_expanded = csdl.expand(csdl.reshape(axis_origin_csdl, new_shape=(axis_origin_csdl.shape[-1],)),
                                                 shape=(num_rotating_points,axis_origin_csdl.shape[-1]), indices='i->ji')
        control_points_origin_frame = rotating_control_points - axis_origin_csdl_expanded

        # Construct quaternion from rotation value
        rotation_value = self.declare_variable(name=rotation_name, val=default_value)
        if units == 'deg':
            rotation_value = rotation_value * 2*np.pi/180

        quaternion = self.create_output(f'quat_{rotation_name}', shape=(num_rotating_points,) + (4,))
        quaternion[:, 0] = csdl.expand(csdl.cos(rotation_value / 2), (num_rotating_points,) + (1,), 'i->ij')
        quaternion[:, 1] = csdl.expand(csdl.sin(rotation_value / 2) * normalized_axis[0], (num_rotating_points,) + (1,), 'i->ij')
        quaternion[:, 2] = csdl.expand(csdl.sin(rotation_value / 2) * normalized_axis[1], (num_rotating_points,) + (1,), 'i->ij')
        quaternion[:, 3] = csdl.expand(csdl.sin(rotation_value / 2) * normalized_axis[2], (num_rotating_points,) + (1,), 'i->ij')

        # Apply rotation
        rotated_control_points_origin_frame = csdl.quatrotvec(quaternion, control_points_origin_frame)

        # Translate rotated control points back into original coordinate frame
        rotated_control_points = rotated_control_points_origin_frame + axis_origin_csdl_expanded

        # Assemble full geometry
        # -- Expand actuated points to size of full geometry (zeros for non-actuated points)
        data = np.ones((num_rotating_points))
        indexing_map = sps.coo_matrix((data, (np.array(rotating_control_points_indices), np.arange(num_rotating_points))),
                                        shape=(num_control_points, num_rotating_points))
        indexing_map = indexing_map.tocsc()
        if num_rotating_points != 0:
            updated_geometry_component = csdl.sparsematmat(rotated_control_points, sparse_mat=indexing_map)

        num_unchanged_points = num_control_points - num_rotating_points
        if num_unchanged_points == 0:
            configuration_geometry = updated_geometry_component
        elif num_rotating_points == 0:
            configuration_geometry = initial_geometry_control_points_csdl*1
        else:
            data = np.ones((num_unchanged_points,))
            unchanged_indices = np.delete(np.arange(num_control_points), rotating_control_points_indices)
            indexing_map = sps.coo_matrix((data, (unchanged_indices, unchanged_indices)),
                                        shape=(num_control_points, num_control_points))
            unchanged_indexing_map = indexing_map.tocsc()
            initial_geometry_component = csdl.sparsematmat(initial_geometry_control_points_csdl, sparse_mat=unchanged_indexing_map)
            configuration_geometry = updated_geometry_component + initial_geometry_component
                
        self.register_output(f'{configuration.name}_geometry', configuration_geometry)

        


if __name__ == "__main__":
    import csdl
    from python_csdl_backend import Simulator
    import numpy as np
    import vedo
    import array_mapper as am

    from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
    system_representation = SystemRepresentation()
    spatial_rep = system_representation.spatial_representation
    from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization
    system_parameterization = SystemParameterization(system_representation=system_representation)

    '''
    Single wing
    '''
    file_path = 'models/stp/'
    spatial_rep.import_file(file_name=file_path+'rect_wing.stp')

    # Create Components
    from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
    wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
    wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)  # TODO add material arguments
    system_representation.add_component(wing)

    # Mesh definition
    num_spanwise_vlm = 21
    num_chordwise_vlm = 5
    leading_edge = wing.project(np.linspace(np.array([0., -9., 0.]), np.array([0., 9., 0.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))  # returns MappedArray
    trailing_edge = wing.project(np.linspace(np.array([4., -9., 0.]), np.array([4., 9., 0.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))   # returns MappedArray
    chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
    wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25)
    wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1

    system_representation.add_output('wing_camber_surface', wing_camber_surface)

    starboard_tip = wing.project(np.array([2., 9., 7.5]), direction=np.array([0., 0., -1.]))
    port_tip = wing.project(np.array([2., -9., 7.5]), direction=np.array([0., 0., -1.]))
    wingspan_vector = starboard_tip - port_tip
    wingspan = am.norm(wingspan_vector)     # NOTE: Nonlinear operations don't return MappedArrays. They return NonlinearMappedarrays
    spatial_rep.add_output(name='wingspan', quantity=wingspan)
    root_leading = wing.project(np.array([0., 0., 7.5]), direction=np.array([0., 0., -1.]))
    root_trailing = wing.project(np.array([4., 0., 7.5]), direction=np.array([0., 0., -1.]))
    root_chord_vector = root_leading - root_trailing
    root_chord = am.norm(root_chord_vector)     # NOTE: Nonlinear operations don't return MappedArrays. They return NonlinearMappedarrays
    # spatial_rep.add_output(name='wing_root_chord', quantity=root_chord)

    # # Parameterization
    from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
    from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

    wing_geometry_primitives = wing.get_geometry_primitives()
    wing_ffd_bspline_volume = create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
    wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)

    wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10, value=-1/2*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))
    wing_ffd_block.add_rotation_v(name='wingtip_twist', order=4, num_dof=10, value=-np.array([np.pi/2, 0., 0., 0., 0., 0., 0., 0., 0., -np.pi/2]))
    wing_ffd_block.add_translation_w(name='wingtip_translation', order=4, num_dof=10, value=np.array([2., 0., 0., 0., 0., 0., 0., 0., 0., 2.]))

    from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
    ffd_set = SRBGFFDSet(name='ffd_set', ffd_blocks={wing_ffd_block.name : wing_ffd_block})

    ffd_set.setup(project_embedded_entities=True)
    affine_section_properties = ffd_set.evaluate_affine_section_properties()
    affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations()
    rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
    rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations()
    ffd_control_points = ffd_set.evaluate_control_points()
    embedded_entities = ffd_set.evaluate_embedded_entities()
    spatial_rep.update(embedded_entities)

    updated_wing_camber_surface = wing_camber_surface.evaluate()

    sim = Simulator(SystemRepresentationOutputsCSDL(system_representation=system_representation))
    sim.run()

    print('wingspan', sim['wingspan'])
    print("Python and CSDL difference: wingspan", np.linalg.norm(wingspan.value - sim['wingspan']))
    # print('wing root chord', sim['wing_root_chord'])
    # print("Python and CSDL difference", np.linalg.norm(root_chord.value - sim['wing_root_chord']))

    wing_camber_surface_csdl = sim['wing_camber_surface'].reshape(wing_camber_surface.shape)
    print("Python and CSDL difference: wing camber surface", np.linalg.norm(wing_camber_surface_csdl - updated_wing_camber_surface))

    spatial_rep.plot_meshes([wing_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)

    '''
    Multiple FFD blocks
    '''
    system_representation = SystemRepresentation()
    spatial_rep = system_representation.spatial_representation
    file_path = 'models/stp/'
    spatial_rep.import_file(file_name=file_path+'lift_plus_cruise_final_3.stp')

    # Create Components
    from caddee.core.caddee_core.system_representation.component.component import LiftingSurface
    wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
    wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)  # TODO add material arguments
    tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
    horizontal_stabilizer = LiftingSurface(name='tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)  # TODO add material arguments
    system_representation.add_component(wing)
    system_representation.add_component(horizontal_stabilizer)

    # Meshes definitions
    num_spanwise_vlm = 21
    num_chordwise_vlm = 5
    leading_edge = wing.project(np.linspace(np.array([8., -26., 7.5]), np.array([8., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))  # returns MappedArray
    trailing_edge = wing.project(np.linspace(np.array([15., -26., 7.5]), np.array([15., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))   # returns MappedArray
    chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15)
    wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15)
    wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1

    num_spanwise_vlm = 11
    num_chordwise_vlm = 3
    leading_edge = horizontal_stabilizer.project(np.linspace(np.array([27., -6.5, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=15)  # returns MappedArray
    trailing_edge = horizontal_stabilizer.project(np.linspace(np.array([31.5, -6.5, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=15)   # returns MappedArray
    chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    horizontal_stabilizer_upper_surface_wireframe = horizontal_stabilizer.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15)
    horizontal_stabilizer_lower_surface_wireframe = horizontal_stabilizer.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15)
    horizontal_stabilizer_camber_surface = am.linspace(horizontal_stabilizer_upper_surface_wireframe, horizontal_stabilizer_lower_surface_wireframe, 1) # this linspace will return average when n=1

    system_representation.add_output('wing_camber_surface', wing_camber_surface)
    system_representation.add_output('horizontal_stabilizer_camber_surface', horizontal_stabilizer_camber_surface)
    starboard_trailing_tip = wing.project(np.array([15., 26., 7.5]), direction=np.array([0., 0., -1.]))
    port_trailing_tip = wing.project(np.array([15., -26., 7.5]), direction=np.array([0., 0., -1.]))
    wingspan_vector = starboard_trailing_tip - port_trailing_tip
    wingspan = am.norm(wingspan_vector)     # NOTE: Nonlinear operations don't return MappedArrays. They return NonlinearMappedarrays
    spatial_rep.add_output(name='wingspan', quantity=wingspan)
    root_leading = wing.project(np.array([9., 0., 7.5]), direction=np.array([0., 0., -1.]))
    root_trailing = wing.project(np.array([15., 0., 7.5]), direction=np.array([0., 0., -1.]))
    root_chord_vector = root_leading - root_trailing
    root_chord = am.norm(root_chord_vector)     # NOTE: Nonlinear operations don't return MappedArrays. They return NonlinearMappedarrays
    spatial_rep.add_output(name='wing_root_chord', quantity=root_chord)

    # # Parameterization
    from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
    from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

    wing_geometry_primitives = wing.get_geometry_primitives()
    wing_ffd_bspline_volume = create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
    wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
    wing_ffd_block.add_scale_v(name='linear_taper', order=2, num_dof=3, value=np.array([0., 1., 0.]), cost_factor=1.)
    wing_ffd_block.add_scale_w(name='constant_thickness_scaling', order=1, num_dof=1, value=np.array([0.5]))
    wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10, value=-1/4*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))

    horizontal_stabilizer_geometry_primitives = horizontal_stabilizer.get_geometry_primitives()
    horizontal_stabilizer_ffd_bspline_volume = create_cartesian_enclosure_volume(horizontal_stabilizer_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
    horizontal_stabilizer_ffd_block = SRBGFFDBlock(name='horizontal_stabilizer_ffd_block', primitive=horizontal_stabilizer_ffd_bspline_volume, embedded_entities=horizontal_stabilizer_geometry_primitives)
    horizontal_stabilizer_ffd_block.add_scale_v(name='horizontal_stabilizer_linear_taper', order=2, num_dof=3, value=np.array([0.5, 0.5, 0.5]), cost_factor=1.)
    horizontal_stabilizer_ffd_block.add_rotation_u(name='horizontal_stabilizer_twist_distribution', order=1, num_dof=1, value=np.array([np.pi/10]))

    from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
    ffd_set = SRBGFFDSet(name='ffd_set', ffd_blocks={wing_ffd_block.name : wing_ffd_block, horizontal_stabilizer_ffd_block.name : horizontal_stabilizer_ffd_block})

    ffd_set.setup(project_embedded_entities=True)
    affine_section_properties = ffd_set.evaluate_affine_section_properties()
    rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
    affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations()
    rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations()
    ffd_control_points = ffd_set.evaluate_control_points()
    ffd_embedded_entities = ffd_set.evaluate_embedded_entities()

    updated_primitives_names = wing.primitive_names.copy()
    updated_primitives_names.extend(horizontal_stabilizer.primitive_names.copy())
    spatial_rep.update(ffd_embedded_entities, updated_primitives_names)

    wing_camber_surface.evaluate(spatial_rep.control_points)
    horizontal_stabilizer_camber_surface.evaluate(spatial_rep.control_points)

    sim = Simulator(SystemRepresentationOutputsCSDL(system_representation=system_representation))
    sim.run()

    print('wingspan', sim['wingspan'])
    print("Python and CSDL difference: wingspan", np.linalg.norm(wingspan.value - sim['wingspan']))
    print('wing root chord', sim['wing_root_chord'])
    print("Python and CSDL difference: wing root chord", np.linalg.norm(root_chord.value - sim['wing_root_chord']))

    wing_camber_surface_csdl = sim['wing_camber_surface'].reshape(wing_camber_surface.shape)
    horizontal_stabilizer_camber_surface_csdl = sim['horizontal_stabilizer_camber_surface'].reshape(horizontal_stabilizer_camber_surface.shape)
    print("Python and CSDL difference: wing camber surface", np.linalg.norm(wing_camber_surface_csdl - wing_camber_surface.value))
    print("Python and CSDL difference: horizontal stabilizer camber surface", np.linalg.norm(horizontal_stabilizer_camber_surface_csdl - horizontal_stabilizer_camber_surface.value))

    spatial_rep.plot_meshes([wing_camber_surface_csdl, horizontal_stabilizer_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)