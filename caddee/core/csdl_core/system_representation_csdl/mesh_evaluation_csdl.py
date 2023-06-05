import csdl
from csdl_om import Simulator
import numpy as np
import scipy.sparse as sps
import array_mapper as am

class MeshEvaluationCSDL(csdl.Model):
    '''
    Evaluates the mesh from the geometry control points.
    '''

    def initialize(self):
        self.parameters.declare('system_representation')
        self.parameters.declare('groups')

    def define(self):
        system_representation = self.parameters['system_representation']
        spatial_rep = system_representation.spatial_representation
        groups = self.parameters['groups']
        mech_group, non_mech_group, pwr_group = groups
        

        models_list = []
        if mech_group:
            models_list += mech_group._all_models_list 
        if non_mech_group:
            models_list += non_mech_group._all_models_list 
        if pwr_group:
            models_list += pwr_group._all_models_list
        
        
        for model in models_list:
            if model.parameters.__contains__('mesh'):
                if hasattr(model, 'model_selection'):
                    model_selection = model.model_selection
                else:
                    raise Exception(f"Model of type '{type(model)}' requires attribute 'model_selection' in order to expand its mesh to the right shape.")
                
                # Count number of nonlinear outputs
                num_nonlinear_outputs = 0
                for output in model.parameters['mesh'].parameters['meshes']:
                    if type(output) is am.NonlinearMappedArray:
                        num_nonlinear_outputs += 1

                num_active_nodes = int(sum(model_selection))

                # Declare input
                #system_representation_geometry = self.declare_variable('system_representation_geometry', val=spatial_rep.control_points)
                system_representation_geometry = self.declare_variable('system_representation_geometry', val=spatial_rep.control_points['geometry'])
                mesh = model.parameters['mesh']
                                                
                if mesh:
                    meshes = mesh.parameters['meshes']
    
                    nonlinear_output_counter = 0
                    for mesh_name in list(meshes):
                        mesh = meshes[mesh_name]
                        if type(mesh) is am.MappedArray:
                            # updated_mesh_csdl = mesh.evaluate(system_representation_geometry)  # Option 1 where I implement this in array_mapper

                            # Option 2
                            mesh_linear_map = mesh.linear_map
                            mesh_offset_map = mesh.offset_map
                            updated_mesh_csdl_without_offset = csdl.sparsematmat(system_representation_geometry, sparse_mat=mesh_linear_map)

                            if mesh_offset_map is not None:
                                mesh_offset_map_csdl = self.create_output(f'{mesh_name}_offset_map', mesh_offset_map)
                                updated_mesh_csdl = updated_mesh_csdl_without_offset + mesh_offset_map_csdl
                            else:
                                updated_mesh_csdl = updated_mesh_csdl_without_offset

                            if mesh_name in ['left_wing_linear_beam','right_wing_linear_beam','center_wing_linear_beam']:
                            #if mesh_name == 'wing' or mesh_name == 'tail':
                                output = csdl.expand(csdl.reshape(updated_mesh_csdl, new_shape=mesh.shape[1:]), (num_active_nodes, ) + mesh.shape[1:],  'jkl->ijkl')
                                self.register_output(mesh_name, output)
                            else:
                                self.register_output(mesh_name, csdl.reshape(updated_mesh_csdl, new_shape=mesh.shape))
                        elif type(mesh) is am.NonlinearMappedArray:

                            nonlinear_outputs = csdl.custom(system_representation_geometry, op=NonlinearOutputsOperation(system_representation=system_representation, mesh_name=mesh_name, mesh=mesh))

                            self.register_output(mesh_name, nonlinear_outputs)

                            nonlinear_output_counter += 1


class NonlinearOutputsOperation(csdl.CustomExplicitOperation):
    def initialize(self):
        self.parameters.declare('system_representation')
        self.parameters.declare('mesh_name')
        self.parameters.declare('mesh')


    def define(self):
        system_representation = self.parameters['system_representation']
        mesh_name = self.parameters['mesh_name']
        mesh = self.parameters['mesh']
        spatial_rep = system_representation.spatial_representation
        print('spatial_rep', spatial_rep.control_points)
        self.add_input('system_representation_geometry', val=spatial_rep.control_points['geometry'].copy())

        self.add_output(name=mesh_name, val=mesh.value)

        self.declare_derivatives('*','*')


    def compute(self, inputs, outputs):
        system_representation = self.parameters['system_representation']
        mesh_name = self.parameters['mesh_name']
        mesh = self.parameters['mesh']
        spatial_rep = system_representation.spatial_representation

        input = inputs['system_representation_geometry']

        outputs[mesh_name] = mesh.evaluate(input)
    


    def compute_derivatives(self, inputs, derivatives):
        system_representation = self.parameters['system_representation']
        spatial_rep = system_representation.spatial_representation
        mesh_name = self.parameters['mesh_name']
        mesh = self.parameters['mesh']

        input = inputs['system_representation_geometry']
        
        derivatives[mesh_name, 'system_representation_geometry'] = mesh.evaluate_derivative(input)



if __name__ == "__main__":
    import csdl
    # from csdl_om import Simulator
    from python_csdl_backend import Simulator
    import numpy as np
    from vedo import Points, Plotter
    import array_mapper as am

    from caddee.caddee_core.system_representation.system_representation import SystemRepresentation
    system_representation = SystemRepresentation()
    spatial_rep = system_representation.spatial_representation
    from caddee.caddee_core.system_parameterization.system_parameterization import SystemParameterization
    system_parameterization = SystemParameterization(system_representation=system_representation)

    '''
    Single FFD Block
    '''
    file_path = 'models/stp/'
    spatial_rep.import_file(file_name=file_path+'rect_wing.stp')

    # Create Components
    from caddee.caddee_core.system_representation.component.component import LiftingSurface, Component
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

    meshes = {'wing_camber_surface': wing_camber_surface}

    # # Parameterization
    from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
    from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

    wing_geometry_primitives = wing.get_geometry_primitives()
    wing_ffd_bspline_volume = create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
    wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)

    wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10, value=-1/2*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))
    wing_ffd_block.add_rotation_v(name='wingtip_twist', order=4, num_dof=10, value=-np.array([np.pi/2, 0., 0., 0., 0., 0., 0., 0., 0., -np.pi/2]))
    wing_ffd_block.add_translation_w(name='wingtip_translation', order=4, num_dof=10, value=np.array([2., 0., 0., 0., 0., 0., 0., 0., 0., 2.]))

    from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
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

    sim = Simulator(MeshEvaluationCSDL(system_representation=system_representation, meshes=meshes))
    sim.run()
    # sim.visualize_implementation()        # Only csdl_om can do this

    wing_camber_surface_csdl = sim['wing_camber_surface'].reshape(wing_camber_surface.shape)
    print("Python and CSDL difference", np.linalg.norm(wing_camber_surface_csdl - updated_wing_camber_surface))

    spatial_rep.plot_meshes([wing_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)

    '''
    Multiple FFD blocks
    '''
    system_representation = SystemRepresentation()
    spatial_rep = system_representation.spatial_representation
    file_path = 'models/stp/'
    spatial_rep.import_file(file_name=file_path+'lift_plus_cruise_final_3.stp')

    # Create Components
    from caddee.caddee_core.system_representation.component.component import LiftingSurface
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

    meshes = {'wing_camber_surface': wing_camber_surface, 'horizontal_stabilizer_camber_surface' : horizontal_stabilizer_camber_surface}

    # # Parameterization
    from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
    from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

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

    from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
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
    # spatial_rep.plot_meshes([wing_camber_surface, horizontal_stabilizer_camber_surface], mesh_plot_types=['wireframe'], mesh_opacity=1.)

    sim = Simulator(MeshEvaluationCSDL(system_representation=system_representation, meshes=meshes))
    sim.run()

    wing_camber_surface_csdl = sim['wing_camber_surface'].reshape(wing_camber_surface.shape)
    horizontal_stabilizer_camber_surface_csdl = sim['horizontal_stabilizer_camber_surface'].reshape(horizontal_stabilizer_camber_surface.shape)
    print("Python and CSDL difference: wing camber surface", np.linalg.norm(wing_camber_surface_csdl - wing_camber_surface.value))
    print("Python and CSDL difference: horizontal stabilizer camber surface", np.linalg.norm(horizontal_stabilizer_camber_surface_csdl - horizontal_stabilizer_camber_surface.value))

    spatial_rep.plot_meshes([wing_camber_surface_csdl, horizontal_stabilizer_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)

# import csdl
# from csdl_om import Simulator
# import numpy as np
# import scipy.sparse as sps

# class MeshEvaluationCSDL(csdl.Model):
#     '''
#     Evaluates the mesh from the geometry control points.
#     '''

#     def initialize(self):
#         self.parameters.declare('system_representation')
#         self.parameters.declare('groups')

#     def define(self):
#         system_representation = self.parameters['system_representation']
#         spatial_rep = system_representation.spatial_representation
#         groups = self.parameters['groups']
#         mech_group, non_mech_group, pwr_group = groups
        
#         models_list = []
#         if mech_group:
#             models_list += mech_group._all_models_list 
#         if non_mech_group:
#             models_list += non_mech_group._all_models_list 
#         if pwr_group:
#             models_list += pwr_group._all_models_list
        
        
#         for model in models_list:
#             if model.parameters.__contains__('mesh'):
#                 if hasattr(model, 'model_selection'):
#                     model_selection = model.model_selection
#                 else:
#                     raise Exception(f"Model of type '{type(model)}' requires attribute 'model_selection' in order to expand its mesh to the right shape.")

#                 num_active_nodes = int(sum(model_selection))

#                 # Declare input
#                 system_representation_geometry = self.declare_variable('system_representation_geometry', val=spatial_rep.control_points)
#                 mesh = model.parameters['mesh']
#                 if mesh:
#                     meshes = mesh.parameters['meshes']
    
#                     for mesh_name in list(meshes):
#                         mesh = meshes[mesh_name]
#                         # updated_mesh_csdl = mesh.evaluate(system_representation_geometry)  # Option 1 where I implement this in array_mapper

#                         # Option 2
#                         mesh_linear_map = mesh.linear_map
#                         mesh_offset_map = mesh.offset_map
#                         updated_mesh_csdl_without_offset = csdl.sparsematmat(system_representation_geometry, sparse_mat=mesh_linear_map)

#                         if mesh_offset_map is not None:
#                             mesh_offset_map_csdl = self.create_output(f'{mesh_name}_offset_map', mesh_offset_map)
#                             updated_mesh_csdl = updated_mesh_csdl_without_offset + mesh_offset_map_csdl
#                         else:
#                             updated_mesh_csdl = updated_mesh_csdl_without_offset

#                         if mesh_name == 'wing' or mesh_name == 'tail':
#                             output = csdl.expand(csdl.reshape(updated_mesh_csdl, new_shape=mesh.shape[1:]), (num_active_nodes, ) + mesh.shape[1:],  'jkl->ijkl')
#                             self.register_output(mesh_name, output)
#                         else:
#                             self.register_output(mesh_name, csdl.reshape(updated_mesh_csdl, new_shape=mesh.shape))


# if __name__ == "__main__":
#     import csdl
#     # from csdl_om import Simulator
#     from python_csdl_backend import Simulator
#     import numpy as np
#     from vedo import Points, Plotter
#     import array_mapper as am

#     from caddee.caddee_core.system_representation.system_representation import SystemRepresentation
#     system_representation = SystemRepresentation()
#     spatial_rep = system_representation.spatial_representation
#     from caddee.caddee_core.system_parameterization.system_parameterization import SystemParameterization
#     system_parameterization = SystemParameterization(system_representation=system_representation)

#     '''
#     Single FFD Block
#     '''
#     file_path = 'models/stp/'
#     spatial_rep.import_file(file_name=file_path+'rect_wing.stp')

#     # Create Components
#     from caddee.caddee_core.system_representation.component.component import LiftingSurface, Component
#     wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
#     wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)  # TODO add material arguments
#     system_representation.add_component(wing)

#     # Mesh definition
#     num_spanwise_vlm = 21
#     num_chordwise_vlm = 5
#     leading_edge = wing.project(np.linspace(np.array([0., -9., 0.]), np.array([0., 9., 0.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))  # returns MappedArray
#     trailing_edge = wing.project(np.linspace(np.array([4., -9., 0.]), np.array([4., 9., 0.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))   # returns MappedArray
#     chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
#     wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
#     wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25)
#     wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1

#     meshes = {'wing_camber_surface': wing_camber_surface}

#     # # Parameterization
#     from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
#     from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

#     wing_geometry_primitives = wing.get_geometry_primitives()
#     wing_ffd_bspline_volume = create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
#     wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)

#     wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10, value=-1/2*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))
#     wing_ffd_block.add_rotation_v(name='wingtip_twist', order=4, num_dof=10, value=-np.array([np.pi/2, 0., 0., 0., 0., 0., 0., 0., 0., -np.pi/2]))
#     wing_ffd_block.add_translation_w(name='wingtip_translation', order=4, num_dof=10, value=np.array([2., 0., 0., 0., 0., 0., 0., 0., 0., 2.]))

#     from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
#     ffd_set = SRBGFFDSet(name='ffd_set', ffd_blocks={wing_ffd_block.name : wing_ffd_block})

#     ffd_set.setup(project_embedded_entities=True)
#     affine_section_properties = ffd_set.evaluate_affine_section_properties()
#     affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations()
#     rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
#     rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations()
#     ffd_control_points = ffd_set.evaluate_control_points()
#     embedded_entities = ffd_set.evaluate_embedded_entities()
#     spatial_rep.update(embedded_entities)

#     updated_wing_camber_surface = wing_camber_surface.evaluate()

#     sim = Simulator(MeshEvaluationCSDL(system_representation=system_representation, meshes=meshes))
#     sim.run()
#     # sim.visualize_implementation()        # Only csdl_om can do this

#     wing_camber_surface_csdl = sim['wing_camber_surface'].reshape(wing_camber_surface.shape)
#     print("Python and CSDL difference", np.linalg.norm(wing_camber_surface_csdl - updated_wing_camber_surface))

#     spatial_rep.plot_meshes([wing_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)

#     '''
#     Multiple FFD blocks
#     '''
#     system_representation = SystemRepresentation()
#     spatial_rep = system_representation.spatial_representation
#     file_path = 'models/stp/'
#     spatial_rep.import_file(file_name=file_path+'lift_plus_cruise_final_3.stp')

#     # Create Components
#     from caddee.caddee_core.system_representation.component.component import LiftingSurface
#     wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
#     wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)  # TODO add material arguments
#     tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
#     horizontal_stabilizer = LiftingSurface(name='tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)  # TODO add material arguments
#     system_representation.add_component(wing)
#     system_representation.add_component(horizontal_stabilizer)

#     # Meshes definitions
#     num_spanwise_vlm = 21
#     num_chordwise_vlm = 5
#     leading_edge = wing.project(np.linspace(np.array([8., -26., 7.5]), np.array([8., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))  # returns MappedArray
#     trailing_edge = wing.project(np.linspace(np.array([15., -26., 7.5]), np.array([15., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]))   # returns MappedArray
#     chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
#     wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15)
#     wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15)
#     wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1

#     num_spanwise_vlm = 11
#     num_chordwise_vlm = 3
#     leading_edge = horizontal_stabilizer.project(np.linspace(np.array([27., -6.5, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=15)  # returns MappedArray
#     trailing_edge = horizontal_stabilizer.project(np.linspace(np.array([31.5, -6.5, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=15)   # returns MappedArray
#     chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
#     horizontal_stabilizer_upper_surface_wireframe = horizontal_stabilizer.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15)
#     horizontal_stabilizer_lower_surface_wireframe = horizontal_stabilizer.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15)
#     horizontal_stabilizer_camber_surface = am.linspace(horizontal_stabilizer_upper_surface_wireframe, horizontal_stabilizer_lower_surface_wireframe, 1) # this linspace will return average when n=1

#     meshes = {'wing_camber_surface': wing_camber_surface, 'horizontal_stabilizer_camber_surface' : horizontal_stabilizer_camber_surface}

#     # # Parameterization
#     from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
#     from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

#     wing_geometry_primitives = wing.get_geometry_primitives()
#     wing_ffd_bspline_volume = create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
#     wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
#     wing_ffd_block.add_scale_v(name='linear_taper', order=2, num_dof=3, value=np.array([0., 1., 0.]), cost_factor=1.)
#     wing_ffd_block.add_scale_w(name='constant_thickness_scaling', order=1, num_dof=1, value=np.array([0.5]))
#     wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10, value=-1/4*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))

#     horizontal_stabilizer_geometry_primitives = horizontal_stabilizer.get_geometry_primitives()
#     horizontal_stabilizer_ffd_bspline_volume = create_cartesian_enclosure_volume(horizontal_stabilizer_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
#     horizontal_stabilizer_ffd_block = SRBGFFDBlock(name='horizontal_stabilizer_ffd_block', primitive=horizontal_stabilizer_ffd_bspline_volume, embedded_entities=horizontal_stabilizer_geometry_primitives)
#     horizontal_stabilizer_ffd_block.add_scale_v(name='horizontal_stabilizer_linear_taper', order=2, num_dof=3, value=np.array([0.5, 0.5, 0.5]), cost_factor=1.)
#     horizontal_stabilizer_ffd_block.add_rotation_u(name='horizontal_stabilizer_twist_distribution', order=1, num_dof=1, value=np.array([np.pi/10]))

#     from caddee.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
#     ffd_set = SRBGFFDSet(name='ffd_set', ffd_blocks={wing_ffd_block.name : wing_ffd_block, horizontal_stabilizer_ffd_block.name : horizontal_stabilizer_ffd_block})

#     ffd_set.setup(project_embedded_entities=True)
#     affine_section_properties = ffd_set.evaluate_affine_section_properties()
#     rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
#     affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations()
#     rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations()
#     ffd_control_points = ffd_set.evaluate_control_points()
#     ffd_embedded_entities = ffd_set.evaluate_embedded_entities()

#     updated_primitives_names = wing.primitive_names.copy()
#     updated_primitives_names.extend(horizontal_stabilizer.primitive_names.copy())
#     spatial_rep.update(ffd_embedded_entities, updated_primitives_names)

#     wing_camber_surface.evaluate(spatial_rep.control_points)
#     horizontal_stabilizer_camber_surface.evaluate(spatial_rep.control_points)
#     # spatial_rep.plot_meshes([wing_camber_surface, horizontal_stabilizer_camber_surface], mesh_plot_types=['wireframe'], mesh_opacity=1.)

#     sim = Simulator(MeshEvaluationCSDL(system_representation=system_representation, meshes=meshes))
#     sim.run()

#     wing_camber_surface_csdl = sim['wing_camber_surface'].reshape(wing_camber_surface.shape)
#     horizontal_stabilizer_camber_surface_csdl = sim['horizontal_stabilizer_camber_surface'].reshape(horizontal_stabilizer_camber_surface.shape)
#     print("Python and CSDL difference: wing camber surface", np.linalg.norm(wing_camber_surface_csdl - wing_camber_surface.value))
#     print("Python and CSDL difference: horizontal stabilizer camber surface", np.linalg.norm(horizontal_stabilizer_camber_surface_csdl - horizontal_stabilizer_camber_surface.value))

#     spatial_rep.plot_meshes([wing_camber_surface_csdl, horizontal_stabilizer_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)