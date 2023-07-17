import csdl
import numpy as np
import scipy.sparse as sps

import time

class GeometryParameterizationSolverCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('system_parameterization')

    def define(self):
        system_parameterization = self.parameters['system_parameterization']

        inputs = system_parameterization.inputs.copy()

        input_vector_length = 0
        for input_name, input in inputs.items():
            input_vector_length += np.prod(input.shape)

        inputs_csdl = self.create_output('parameterization_inputs', shape=(input_vector_length,))

        starting_index = 0
        for input in inputs:
            input_csdl = self.declare_variable(input.name, val=input.value)

            num_flattened_inputs = np.prod(input.shape)
            flattened_input_csdl = csdl.reshape(input_csdl, new_shape=(num_flattened_inputs,))

            inputs_csdl[starting_index:starting_index+num_flattened_inputs] = flattened_input_csdl
            starting_index += num_flattened_inputs

        ffd_free_dof, parameterization_lagrange_multipliers = csdl.custom(inputs_csdl,
                                op=GeometryParameterizationSolverOperation(system_parameterization=system_parameterization))

        self.register_output('ffd_free_dof', ffd_free_dof)
        self.register_output('parameterization_lagrange_multipliers', parameterization_lagrange_multipliers)


class GeometryParameterizationSolverOperation(csdl.CustomImplicitOperation):
    """
    """
    def initialize(self):
        self.parameters.declare('system_parameterization')


    def define(self):
        system_parameterization = self.parameters['system_parameterization']

        geometry_parameterizations = system_parameterization.geometry_parameterizations

        # Collect the parameterizations which have free dof (one parameterization is like one FFDSet)
        free_geometry_parameterizations = {}
        for geometry_parameterization_name, geometry_parameterization in geometry_parameterizations.items():
            if geometry_parameterization.free_affine_dof != 0:
                free_geometry_parameterizations[geometry_parameterization_name] = geometry_parameterization
                # NOTE! Hardcoding for one FFDSet as the only geometry parameterization
                ffd_set = geometry_parameterization

        num_affine_free_dof = ffd_set.num_affine_free_dof

        input_vector_length = 0
        for input_name, input in system_parameterization.inputs.items():
            input_vector_length += np.prod(input.shape)


        self.add_input('parameterization_inputs', shape=(input_vector_length,))
        self.add_output('ffd_free_dof', val=np.zeros((num_affine_free_dof,)))
        self.add_output('parameterization_lagrange_multipliers', val=np.zeros((input_vector_length,)))
        # self.declare_derivatives('ffd_free_dof', 'ffd_free_dof')
        # self.declare_derivatives('ffd_free_dof', 'lagrange_multipliers')
        # self.declare_derivatives('lagrange_multipliers', 'ffd_free_dof')
        # self.declare_derivatives('lagrange_multipliers', 'lagrange_multipliers')
        # self.declare_derivatives('ffd_free_dof', 'inputs_and_constraints')
        # self.declare_derivatives('lagrange_multipliers', 'inputs_and_constraints')

        self.declare_derivatives('*','*')

        self.linear_solver = csdl.ScipyKrylov()
        self.nonlinear_solver = csdl.NewtonSolver(solve_subsystems=True, maxiter=10)


        # ffd_free_dof_to_affine_ffd_control_points_map = ffd_set.ffd_free_dof_to_affine_ffd_control_points_map
        # affine_geometry_control_points_map = ffd_set.affine_geometry_control_points_map
        # pointset_map = geometry.eval_map

        # # Preallocate memory
        # self.ffd_section_properties = np.zeros((ffd_set.num_affine_section_properties,)) # Section properties model output!

        # # Precompute constant maps
        # ffd_control_points_to_pointsets_linear_map = pointset_map.dot(affine_geometry_control_points_map)
        # ffd_control_points_to_pointsets_linear_map_tensor = np.zeros(tuple((pointset_map.shape[0],3)) + (ffd_set.num_affine_free_ffd_control_points,3))
        # for i in range(3):
        #     ffd_control_points_to_pointsets_linear_map_tensor[:,i,:,i] = ffd_control_points_to_pointsets_linear_map.toarray()
        # self.ffd_free_dof_to_pointsets_linear_map = np.tensordot(ffd_control_points_to_pointsets_linear_map_tensor, ffd_free_dof_to_affine_ffd_control_points_map)


    def evaluate_residuals(self, inputs, outputs, residuals):
        # inputs
        system_parameterization = self.parameters['system_parameterization']
        system_representation = system_parameterization.system_representation

        geometry_parameterizations = system_parameterization.geometry_parameterizations

        # Collect the parameterizations which have free dof (one parameterization is like one FFDSet)
        free_geometry_parameterizations = {}
        for geometry_parameterization_name, geometry_parameterization in geometry_parameterizations.items():
            if geometry_parameterization.free_affine_dof != 0:
                free_geometry_parameterizations[geometry_parameterization_name] = geometry_parameterization
                # NOTE! Hardcoding for one FFDSet as the only geometry parameterization
                ffd_set = geometry_parameterization
        ffd_blocks = ffd_set.ffd_blocks

        parameterization_inputs = inputs['parameterization_inputs']
        num_inputs = len(parameterization_inputs)
        ffd_free_dof = outputs['ffd_free_dof']
        lagrange_multipliers = outputs['parameterization_lagrange_multipliers']

        NUM_PARAMETRIC_DIMENSIONS = 3       # This type of FFD block has 3 parametric dimensions by definition.
        # Evaluate affine section properties from FFD dof
        # affine_section_properties = ffd_set.evaluate_affine_section_properties(free_affine_dof=ffd_free_dof)
        if ffd_set.num_affine_dof == 0:
            translations = np.zeros((ffd_set.num_sections, NUM_PARAMETRIC_DIMENSIONS))
            return

        # affine_section_properties_free_component = ffd_set.free_affine_section_properties_map.dot(ffd_free_dof)
        # affine_section_properties_prescribed_component = ffd_set.prescribed_affine_section_properties_map.dot(prescribed_affine_dof)
        # affine_section_properties = affine_section_properties_free_component + affine_section_properties_prescribed_component
        affine_section_properties = ffd_set.free_affine_section_properties_map.dot(ffd_free_dof)

        affine_section_properties = np.zeros((ffd_set.num_affine_section_properties,))
        translations = None

        ffd_block_section_properties_starting_index = 0
        for ffd_block in list(ffd_set.active_ffd_blocks.values()):
            ffd_block_scaling_properties_starting_index = ffd_block_section_properties_starting_index + ffd_block.num_sections*(ffd_block.num_affine_properties-ffd_block.num_scaling_properties)
            ffd_block_section_properties_ending_index = ffd_block_section_properties_starting_index + ffd_block.num_affine_section_properties

            # Add 1 to scaling parameters to make initial scaling=1.
            affine_section_properties[ffd_block_scaling_properties_starting_index:ffd_block_section_properties_ending_index] += 1

            if translations is None:
                translations = affine_section_properties[ffd_block_section_properties_starting_index:ffd_block_scaling_properties_starting_index]
            else:
                translations = np.append(translations,
                                    affine_section_properties[ffd_block_section_properties_starting_index:ffd_block_scaling_properties_starting_index])

            ffd_block_section_properties_starting_index = ffd_block_section_properties_ending_index

        # Evaluate rotational section properties from ffd rotational dof # TODO: Come back to this
        # rotational_section_properties = ffd_set.evaluate_rotational_section_properties()    # If want, these could be passed in as csdl var

        # Evaluate FFD block affine deformations by applying affine section properties
        # affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations(affine_section_properties=affine_section_properties)
        affine_deformed_control_points_flattened = ffd_set.affine_block_deformations_map.dot(affine_section_properties)
        affine_deformed_control_points = affine_deformed_control_points_flattened.reshape((ffd_set.num_control_points, NUM_PARAMETRIC_DIMENSIONS))

        # Evaluate FFD block rotational deformations by applying rotational sectino properties #TODO Come back to this
        # rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations()

        # Evaluate FFD control points by rotating back into global frame
        # ffd_control_points = ffd_set.evaluate_control_points()
        ffd_control_points = np.zeros((ffd_set.num_control_points,NUM_PARAMETRIC_DIMENSIONS))
        starting_index = 0
        for ffd_block in list(ffd_set.active_ffd_blocks.values()):
            ending_index = starting_index + ffd_block.num_control_points

            ffd_block_control_points_local_frame = affine_deformed_control_points[starting_index:ending_index]

            ffd_block_control_points_rotated_back_wrong_axis = np.tensordot(ffd_block.local_to_global_rotation,
                                                                            ffd_block_control_points_local_frame, axes=([-1],[-1]))
            ffd_block_control_points_rotated_back = np.moveaxis(ffd_block_control_points_rotated_back_wrong_axis, 0, 1)
            ffd_block_control_points_rotated_back_reshaped = ffd_block_control_points_rotated_back.reshape(ffd_block.primitive.shape)

            ffd_block_control_points_reshaped = ffd_block_control_points_rotated_back_reshaped + ffd_block.local_to_global_translations
            ffd_block_control_points = ffd_block_control_points_reshaped.reshape((ffd_block.num_control_points, NUM_PARAMETRIC_DIMENSIONS))

            ffd_control_points[starting_index:ending_index] = ffd_block_control_points
            starting_index = ending_index

        # Evaluate geometry control points
        # ffd_embedded_entities = ffd_set.evaluate_embedded_entities()
        ffd_embedded_entities = ffd_set.embedded_entities_map.dot(ffd_control_points)

        # Assemble geometry from FFD outputs
        initial_system_representation_geometry = system_representation.spatial_representation.control_points['geometry'].copy()
        system_representation_geometry = initial_system_representation_geometry.copy()
        parameterization_indices = []
        for ffd_block in list(geometry_parameterization.active_ffd_blocks.values()):
            ffd_block_embedded_primitive_names = list(ffd_block.embedded_entities.keys())
            ffd_block_embedded_primitive_indices = []
            for primitive_name in ffd_block_embedded_primitive_names:
                ffd_block_embedded_primitive_indices.extend(list(
                    system_representation.spatial_representation.primitive_indices[primitive_name]['geometry']))
            parameterization_indices.extend(ffd_block_embedded_primitive_indices)

        num_points_system_representation = initial_system_representation_geometry.shape[0]
        data = np.ones((len(parameterization_indices)))
        indexing_map = sps.coo_matrix((data, (np.array(parameterization_indices), np.arange(len(parameterization_indices)))),
                                        shape=(num_points_system_representation, len(parameterization_indices)))
        indexing_map = indexing_map.tocsc()
        updated_geometry_component = csdl.sparsematmat(ffd_embedded_entities, sparse_mat=indexing_map)

        system_representation_geometry[parameterization_indices,:] = updated_geometry_component

        ''' Start implementing from this point on! '''

        for input_name, parameterization_input in system_parameterization.inputs.items():

            # Evaluate linear mapped array

            # Evaluate nonlinear mapped array



        # geometric_calculations evaluation
        c = np.zeros((num_inputs,))   # for displacement, may need to insert more rows since that's 3 constraints.
        dc_dx = np.zeros(((num_inputs,) + ffd_free_dof.shape))

        constraint_counter = 0
        for input_name, parameterization_input in parameterization_inputs.items():
            quantity = parameterization_input.quantity
            quantity_num_constraints = np.prod(quantity.shape)
            c[constraint_counter:constraint_counter+quantity_num_constraints] = quantity.evaluate(ffd_embedded_entities)

            if type(geometric_calculation) is DisplacementCalculation:
                pointset = geometric_calculation.pointset
                starting_index = pointset.output_starting_ind
                pointset_length = np.cumprod(pointset.shape)[-2]
                ending_index = starting_index + pointset_length
                indices = np.arange(starting_index, ending_index)
                pointset_subset_map = np.zeros((pointset_length, pointset_points.shape[0]))
                for i, index in enumerate(indices):
                    pointset_subset_map[i, index] = 1.

                total_linear_map = np.tensordot(pointset_subset_map, self.ffd_free_dof_to_pointsets_linear_map, axes=([-1],[0]))

                displacement = pointset_subset_map.dot(pointset_points)
                c[constraint_counter:constraint_counter+3] = (displacement.reshape(-1,)).T - inputs_and_constraints[constraint_counter:constraint_counter+3].T
                # NOTE: The axis of length one must be removed for proper casting
                dc_dx[constraint_counter:constraint_counter+3,:] = (total_linear_map).reshape((3,) + tuple(ffd_free_dof.shape)) # first 2 axes should be transposed
                # NOTE: The axis of length one must be removed for proper casting
                constraint_counter += 3

            elif type(geometric_calculation) is MagnitudeCalculation:
                pointset = geometric_calculation.pointset
                starting_index = pointset.output_starting_ind
                pointset_length = np.cumprod(pointset.shape)[-2]
                ending_index = starting_index + pointset_length
                indices = np.arange(starting_index, ending_index)
                pointset_subset_map = np.zeros((pointset_length, pointset_points.shape[0]))
                for i, index in enumerate(indices):
                    pointset_subset_map[i, index] = 1.

                total_linear_map = np.tensordot(pointset_subset_map, self.ffd_free_dof_to_pointsets_linear_map, axes=([-1],[0]))

                displacement = pointset_subset_map.dot(pointset_points)
                magnitude = np.linalg.norm(displacement)
                # c[constraint_counter] = magnitude - chord_dv
                c[constraint_counter] = magnitude - inputs_and_constraints[constraint_counter]

                # print('displacement', displacement)
                # print('magnitude', magnitude)
                # print('c', c)

                displacement_tensor = displacement.reshape((1,1) + tuple(pointset.shape))  # geometric output has shape (1,)  (it should technically be (1,1), but I think (1,) will work same in practice)
                nonlinear_map = displacement_tensor/magnitude   # dgeometricoutputs_dgeometricoutputpointset

                dc_dx[constraint_counter,:] = np.tensordot(nonlinear_map, total_linear_map)

                constraint_counter += 1

            elif type(geometric_calculation) is AreaCalculation:
                pointsets = geometric_calculation.pointsets
                num_pointsets_points = 0
                indices = None
                for pointset in pointsets:
                    num_pointsets_points += np.cumprod(pointset.shape[-1])[-1]
                    starting_index = pointset.output_starting_ind
                    ending_index = starting_index + np.cumprod(pointset.shape[-1])[-1]
                    if indices is None:
                        indices = np.arange(starting_index, ending_index)
                    else:
                        indices = np.append(indices, np.arange(starting_index, ending_index))

                pointset_subset_map = np.zeros((num_pointsets_points, pointset_points.shape[0]))
                for pointset in pointsets:
                    for i, index in enumerate(indices):
                        pointset_subset_map[i, index] = 1.

                total_linear_map = np.tensordot(pointset_subset_map, self.ffd_free_dof_to_pointsets_linear_map, axes=([-1],[0]))

                basis_vectors = pointset_subset_map.dot(pointset_points)
                normal_vector = np.cross(basis_vectors[0,:], basis_vectors[1,:])
                area = np.linalg.norm(normal_vector)

                c[constraint_counter] = area - inputs_and_constraints[constraint_counter]

                # TODO finish implementation
                # NOTE: Area is more difficult because the cross product and norm are both nonlinear. Derivatives must be thought through thouroughly.

                # cross_product_map =

                displacement_tensor = displacement.reshape((1,1) + tuple(pointset.shape))  # geometric output has shape (1,)  (it should technically be (1,1), but I think (1,) will work same in practice)
                nonlinear_map = displacement_tensor/magnitude   # dgeometricoutputs_dgeometricoutputpointset

                dc_dx[constraint_counter,:] = np.tensordot(nonlinear_map, total_linear_map)

                constraint_counter += 1


        dObjective_dx = 2*cost_matrix.dot(ffd_free_dof)

        residuals['ffd_free_dof'] = dObjective_dx + np.tensordot(lagrange_multipliers, dc_dx, axes=([-1],[0]))
        residuals['parameterization_lagrange_multipliers'] = c.T


    def compute_derivatives(self, inputs, outputs, derivatives):
        # inputs
        geometry = self.parameters['geometry']
        ffd_set = geometry.ffd_set
        ffd_blocks = ffd_set.ffd_blocks

        cost_matrix = ffd_set.cost_matrix
        free_section_properties_map = ffd_set.free_section_properties_map
        ffd_control_points_map = ffd_set.ffd_control_points_map
        ffd_control_points_x_map = ffd_set.ffd_control_points_x_map
        ffd_control_points_y_map = ffd_set.ffd_control_points_y_map
        ffd_control_points_z_map = ffd_set.ffd_control_points_z_map
        affine_free_ffd_control_points_map = ffd_set.affine_free_ffd_control_points_map
        affine_free_ffd_control_points_x_map = ffd_set.affine_free_ffd_control_points_x_map
        affine_free_ffd_control_points_y_map = ffd_set.affine_free_ffd_control_points_y_map
        affine_free_ffd_control_points_z_map = ffd_set.affine_free_ffd_control_points_z_map
        local_to_global_ffd_control_points_map = ffd_set.local_to_global_ffd_control_points_map
        geometry_control_points_map = ffd_set.geometry_control_points_map
        unchanged_geometry_indexing_map = ffd_set.unchanged_geometry_indexing_map
        pointset_map = geometry.eval_map

        inputs_and_constraints = inputs['inputs_and_constraints']
        num_inputs_and_constraints = inputs_and_constraints.shape[0]
        ffd_free_dof = outputs['ffd_free_dof']
        lagrange_multipliers = outputs['parameterization_lagrange_multipliers']

        num_affine_free_dof = ffd_set.num_affine_free_dof
        num_affine_section_properties = ffd_set.num_affine_section_properties
        num_affine_ffd_control_points = ffd_set.num_affine_ffd_control_points
        num_affine_free_ffd_control_points = ffd_set.num_affine_free_ffd_control_points
        num_embedded_points = ffd_set.num_embedded_points
        num_geometry_control_points = ffd_set.num_geometry_control_points

        # ffd section properties evaluation
        ffd_free_section_properties_without_initial = free_section_properties_map.dot(ffd_free_dof)

        # An initial value of 1 must be added to the scaling section properties. This is not differentiated because it's a constant 1.
        self.ffd_section_properties = np.zeros((num_affine_section_properties,)) # Section properties model output!
        ffd_block_starting_index = 0
        for ffd_block in ffd_blocks:
            if ffd_block.num_affine_dof == 0:
                continue

            ffd_block_ending_index = ffd_block_starting_index + ffd_block.num_sections * ffd_block.num_affine_properties
            NUM_SCALING_PROPERTIES = 3
            ffd_block_scaling_properties_starting_index = ffd_block_starting_index + ffd_block.num_sections*(ffd_block.num_affine_properties-NUM_SCALING_PROPERTIES)    #The last 2 properties are scaling
            ffd_block_scaling_properties_ending_index = ffd_block_scaling_properties_starting_index + ffd_block.num_sections*(NUM_SCALING_PROPERTIES)

            # Use calculated values for non-scaling parameters
            self.ffd_section_properties[ffd_block_starting_index:ffd_block_scaling_properties_starting_index] = \
                ffd_free_section_properties_without_initial[ffd_block_starting_index:ffd_block_scaling_properties_starting_index] # -3 is because scale_y and z are the last 2 properties

            # Add 1 to scaling parameters to make initial scaling=1.
            self.ffd_section_properties[ffd_block_scaling_properties_starting_index:ffd_block_scaling_properties_ending_index] = \
                ffd_free_section_properties_without_initial[ffd_block_scaling_properties_starting_index:ffd_block_scaling_properties_ending_index] + 1.  # adding 1 which is initial scale value

            ffd_block_starting_index = ffd_block_ending_index


        # ffd control points evaluation (affine)
        ffd_control_points_x = affine_free_ffd_control_points_x_map.dot(self.ffd_section_properties)
        ffd_control_points_y = affine_free_ffd_control_points_y_map.dot(self.ffd_section_properties)
        ffd_control_points_z = affine_free_ffd_control_points_z_map.dot(self.ffd_section_properties)

        # Combine x,y,z components back to list of points
        affine_ffd_control_points = np.zeros((num_affine_free_ffd_control_points, 3))
        affine_ffd_control_points[:,0] = ffd_control_points_x
        affine_ffd_control_points[:,1] = ffd_control_points_y
        affine_ffd_control_points[:,2] = ffd_control_points_z

        # Construct ffd control points vector from ffd blocks with affine and/or rotational dof
        ffd_control_points_local_frame = affine_ffd_control_points

        # Transformation back into global frame evaluation
        ffd_control_points_without_origin = np.tensordot(local_to_global_ffd_control_points_map, ffd_control_points_local_frame)
        ffd_control_points = ffd_control_points_without_origin
        ffd_block_starting_index = 0
        for ffd_block in ffd_blocks:
            if ffd_block.num_affine_free_dof == 0:
                continue
            ffd_block_num_control_points = ffd_block.nxp * ffd_block.nyp * ffd_block.nzp
            ffd_block_ending_index = ffd_block_starting_index + ffd_block_num_control_points
            ffd_control_points[ffd_block_starting_index:ffd_block_ending_index] += np.repeat(ffd_block.section_origins, ffd_block.nyp*ffd_block.nzp, axis=0)
            ffd_block_starting_index = ffd_block_ending_index

        # vp_init = Plotter()
        # vps = []
        # vps1 = Points(ffd_control_points, r=8, c = 'blue')
        # vps2 = Points(ffd_control_points_without_origin, r=7, c='cyan')
        # vps3 = Points(ffd_control_points_local_frame, r=7, c='red')
        # vps4 = Points(affine_ffd_control_points, r=7, c='magenta')
        # vps.append(vps1)
        # vps.append(vps2)
        # vps.append(vps3)
        # vps.append(vps4)

        # vp_init.show(vps, 'FFD Changes', axes=1, viewup="z", interactive = True)

        # Geometry control points evaluation
        if num_embedded_points != num_geometry_control_points:  #
            # Get unchanged geometry control points (points not included in FFD)
            initial_geometry_control_points = geometry.total_cntrl_pts_vector
            unchanged_geometry_control_points = unchanged_geometry_indexing_map.dot(initial_geometry_control_points)

        # --Construct map (ffd control points (in ffd blocks with affine free dof) --> geometry control points)
        affine_geometry_control_points_map = None
        ffd_block_starting_index = 0
        for ffd_block in ffd_blocks:
            if ffd_block.num_affine_free_dof == 0:
                continue

            ffd_block_num_control_points = ffd_block.nxp * ffd_block.nyp * ffd_block.nzp
            ffd_block_ending_index = ffd_block_starting_index + ffd_block_num_control_points

            if affine_geometry_control_points_map is None:
                affine_geometry_control_points_map = geometry_control_points_map[:,ffd_block_starting_index:ffd_block_ending_index]
            else:
                affine_geometry_control_points_map = sps.hstack((affine_geometry_control_points_map, geometry_control_points_map[:,ffd_block_starting_index:ffd_block_ending_index]))

            ffd_block_starting_index = ffd_block_ending_index

        # --Evaluate updated geometry control points using map
        updated_geometry_control_points = affine_geometry_control_points_map.dot(ffd_control_points)

        # --Combine updated and unchanged portions of the geometry to complete the geometry
        if num_embedded_points != num_geometry_control_points:
            geometry_control_points = updated_geometry_control_points + unchanged_geometry_control_points
        else:
            geometry_control_points = updated_geometry_control_points
        # pointset evaluation
        pointset_points = pointset_map.dot(geometry_control_points)

        # geometric_calculations evaluation
        # c = np.zeros((num_geometric_outputs,))   # Don't need c for Hessian.
        dc_dx = np.zeros(((num_inputs_and_constraints,) + ffd_free_dof.shape))
        d2c_dx2 = np.zeros(((num_inputs_and_constraints,)  + ffd_free_dof.shape + ffd_free_dof.shape))

        inputs_and_constraints_python = geometry.inputs.copy() + geometry.constraints.copy()
        constraint_counter = 0
        for inner_optimization_constraint in inputs_and_constraints_python:
            geometric_calculation = inner_optimization_constraint.geometric_calculation

            if type(geometric_calculation) is DisplacementCalculation:
                pointset = geometric_calculation.pointset
                starting_index = pointset.output_starting_ind
                pointset_length = np.cumprod(pointset.shape)[-2]
                ending_index = starting_index + pointset_length
                indices = np.arange(starting_index, ending_index)
                pointset_subset_map = np.zeros((pointset_length, pointset_points.shape[0]))
                for i, index in enumerate(indices):
                    pointset_subset_map[i, index] = 1.

                total_linear_map = np.tensordot(pointset_subset_map, self.ffd_free_dof_to_pointsets_linear_map, axes=([-1],[0]))

                displacement = pointset_subset_map.dot(pointset_points)
                # NOTE: The axis of length one must be removed for proper casting
                dc_dx[constraint_counter:constraint_counter+3,:] = (total_linear_map).reshape((3,) + tuple(ffd_free_dof.shape)) # first 2 axes should be transposed
                # NOTE: The axis of length one must be removed for proper casting
                d2c_dx2[constraint_counter:constraint_counter+3,:,:] = np.zeros((3,) + tuple(ffd_free_dof.shape) + tuple(ffd_free_dof.shape))

                constraint_counter += 3
            elif type(geometric_calculation) is MagnitudeCalculation:
                pointset = geometric_calculation.pointset
                starting_index = pointset.output_starting_ind
                pointset_length = np.cumprod(pointset.shape)[-2]
                ending_index = starting_index + pointset_length
                indices = np.arange(starting_index, ending_index)
                pointset_subset_map = np.zeros((pointset_length, pointset_points.shape[0]))
                for i, index in enumerate(indices):
                    pointset_subset_map[i, index] = 1.

                total_linear_map = np.tensordot(pointset_subset_map, self.ffd_free_dof_to_pointsets_linear_map, axes=([-1],[0]))

                displacement = pointset_subset_map.dot(pointset_points)
                magnitude = np.linalg.norm(displacement)
                # c[constraint_counter] = magnitude - chord_dv

                displacement_tensor = displacement.reshape((1,1) + tuple(pointset.shape))  # geometric output has shape (1,)  (it should technically be (1,1), but I think (1,) will work same in practice)
                nonlinear_map = displacement_tensor/magnitude   # dgeometricoutputs_dgeometricoutputpointset

                dc_dx[constraint_counter,:] = np.tensordot(nonlinear_map, total_linear_map)

                ddisplacement_displacement = np.zeros(tuple(pointset.shape) + tuple(pointset.shape))
                ddisplacement_displacement[0,:,0,:] = np.eye(pointset.shape[1])     # should just be a (3,3)
                ddisplacement_displacement_T = np.zeros(tuple(pointset.shape[::-1]) + tuple(pointset.shape))
                ddisplacement_displacement_T[:,0,0,:] = np.eye(pointset.shape[1])     # should just be a (3,3)
                a_term_dot_db_term = ddisplacement_displacement.reshape((1,1) + tuple(ddisplacement_displacement.shape))/magnitude
                a_term_dot_db_term_T = np.swapaxes(a_term_dot_db_term, 0, 3)    # need to transpose for upcoming computation anyway
                da_term = -np.tensordot(displacement/magnitude**3, ddisplacement_displacement_T, axes=([-1],[0]))
                b_term_T_dot_da_term_T = np.tensordot(displacement_tensor.T, da_term, axes=([-1],[0]))
                # b_term_T_dot_da_term_T_all_T = np.swapaxes(b_term_T_dot_da_term_T, 0, 3)  # need to transpose back anywyay
                d2magnitude_dpointsetddisplacement_T = a_term_dot_db_term_T + b_term_T_dot_da_term_T
                d2magnitude_dffddv2_term1_T = np.tensordot(total_linear_map.T, d2magnitude_dpointsetddisplacement_T)
                # d2magnitude_dpointset2_term1 = np.transpose(d2magnitude_dpointset2_term1_T, axes=(3, 2, 1, 0, 4, 5))
                d2magnitude_dffddv2_term1 = np.transpose(d2magnitude_dffddv2_term1_T, axes=(2, 1, 0, 3, 4))
                d2magnitude_dffddv2 = np.tensordot(d2magnitude_dffddv2_term1, total_linear_map)

                d2c_dx2[constraint_counter,:,:] = d2magnitude_dffddv2

                constraint_counter += 1

        d2objective_dx2 = 2*cost_matrix

        hessian = np.zeros((num_affine_free_dof+num_inputs_and_constraints,num_affine_free_dof+num_inputs_and_constraints))
        hessian[:num_affine_free_dof, :num_affine_free_dof] = d2objective_dx2 + np.tensordot(lagrange_multipliers, d2c_dx2, axes=([-1],[0]))    # upper left blockk
        hessian[:num_affine_free_dof, num_affine_free_dof:] = dc_dx.T   # uper right block
        hessian[num_affine_free_dof:, :num_affine_free_dof] = dc_dx # lower left block
        # hessian[num_affine_free_dof:, num_affine_free_dof:] = 0.  lower right block preallocated to zeroes

        derivatives['ffd_free_dof', 'ffd_free_dof'] = hessian[:num_affine_free_dof, :num_affine_free_dof]
        derivatives['ffd_free_dof', 'lagrange_multipliers'] = hessian[:num_affine_free_dof, num_affine_free_dof:]
        derivatives['lagrange_multipliers', 'ffd_free_dof'] = hessian[num_affine_free_dof:, :num_affine_free_dof]
        derivatives['lagrange_multipliers', 'lagrange_multipliers'] = hessian[num_affine_free_dof:, num_affine_free_dof:]

        # df_dx = pf_px - pf_py.dot(pr_py**-1).dot(pr_px)
        pr_px = np.zeros((num_inputs_and_constraints+num_affine_free_dof, num_inputs_and_constraints))
        # pr_px[:num_affine_free_dof,:] = 0.
        pr_px[num_affine_free_dof:,:] = -np.eye(num_inputs_and_constraints)
        # pr_py = -hessian
        # dy_dx = np.linalg.solve(pr_py, pr_px)   # solve using direct method since num inputs < num outputs
        # df_dx = dy_dx   # df_dy = I since f=y, also_pf_py = 0, so df_dx = 0 - I.dot(dy_dx)
        # dffddof_dinputsandconstraints = df_dx[:num_affine_free_dof]
        # # dlagrangemultipliers_dinputsandconstraints = df_dx[num_affine_free_dof:]
        # # derivatives['ffd_free_dof', 'inputs_and_constraints'] = dffddof_dinputsandconstraints
        # # derivatives['lagrange_multipliers', 'inputs_and_constraints'] = dlagrangemultipliers_dinputsandconstraints    # don't care about variations in lagrange multipliers

        derivatives['ffd_free_dof', 'inputs_and_constraints'] = pr_px[:num_affine_free_dof,:]
        derivatives['lagrange_multipliers', 'inputs_and_constraints'] = pr_px[num_affine_free_dof:,:]

    '''
    Might be able to take advantage of convexity here.
    '''
    # def solve_residual_equations(self, inputs, outputs):
    #     pass


if __name__ == "__main__":
    import csdl
    from python_csdl_backend import Simulator
    # from csdl_om import Simulator
    import numpy as np
    from src.caddee.concept.geometry.geometry import Geometry
    from src.caddee.concept.geometry.geocore.component import Component
    from src.caddee.concept.geometry.geocore.geometric_calculations import GeometricOuputs, MagnitudeCalculation
    from src.caddee.concept.geometry.geocore.ffd import FFDParameter, FFDTranslationXParameter, FFDTranslationYParameter, FFDTranslationZParameter, FFDScaleYParameter, FFDScaleZParameter

    from src import STP_FILES_FOLDER
    from vedo import Points, Plotter

    # # Single FFD block
    # stp_path = STP_FILES_FOLDER / 'rect_wing.stp'
    # geo = Geometry()
    # geo.read_file(file_name=stp_path)
    # wing_comp = Component(stp_entity_names=['RectWing'], name='wing')       # Creating a wing component and naming it wing
    # geo.add_component(wing_comp)

    # top_wing_surface_names = [
    #     'RectWing, 0, 3',
    #     'RectWing, 1, 9',
    #     ]

    # bot_wing_surface_names = [
    #     'RectWing, 0, 2',
    #     'RectWing, 1, 8',
    #     ]

    # up_direction = np.array([0., 0., 1.])
    # down_direction = np.array([0., 0., -1.])

    # left_lead_point = np.array([0., -9000., 2000.])/1000
    # left_trail_point = np.array([4000.0, -9000.0, 2000.])/1000
    # right_lead_point = np.array([0.0, 9000.0, 2000.])/1000
    # right_trail_point = np.array([4000.0, 9000.0, 2000.])/1000

    # '''Project points'''
    # wing_lead_left, wing_lead_left_coord = geo.project_points(left_lead_point, projection_direction = down_direction, projection_targets_names=["wing"])
    # wing_trail_left, wing_trail_left_coord = geo.project_points(left_trail_point, projection_direction = down_direction, projection_targets_names=["wing"])
    # wing_lead_right, wing_lead_right_coord = geo.project_points(right_lead_point, projection_direction = down_direction, projection_targets_names=["wing"])
    # wing_trail_right, wing_trail_right_coord = geo.project_points(right_trail_point, projection_direction = down_direction, projection_targets_names=["wing"])
    # wing_lead_mid, _ = geo.project_points(np.array([0., 0., 2.]), projection_direction = down_direction, projection_targets_names=["wing"])
    # wing_trail_mid, _ = geo.project_points(np.array([4., 0., 2.]), projection_direction = down_direction, projection_targets_names=["wing"])

    # chord = geo.subtract_pointsets(wing_lead_mid, wing_trail_mid)
    # span = geo.subtract_pointsets(wing_lead_right, wing_lead_left)

    # # Adding dof to the wing FFD block
    # # wing_comp.add_ffd_parameter(parameter_type='scale_y', degree=1, num_dof=2, cost_factor=1.)  # without object
    # wing_comp.add_ffd_parameter(FFDScaleYParameter(degree=1, num_dof=3, cost_factor=1.))    # with object
    # wing_comp.add_ffd_parameter(FFDScaleYParameter(degree=2, num_dof=4, cost_factor=100.))
    # wing_comp.add_ffd_parameter(FFDTranslationXParameter(degree=1, num_dof=2, cost_factor=1.))
    # wing_comp.add_ffd_parameter(FFDRotationXParameter(degree=1, num_dof=3, value=np.array([0., -0.2, 0.])))
    # wing_comp.add_ffd_parameter(FFDScaleZParameter(degree=1, num_dof=3, value=np.array([0., 10., 0.])))
    # # wing_comp.add_ffd_parameter()
    # # from src.utils.constants import SCALE_Y
    # # wing_comp.add_ffd_parameter(parameter_type=nasa_uli_tc1.SCALE_Y, degree=1, num_dof=2, cost_factor=1.)

    # # Adding inputs to the geometry model
    # # geo.add_input(MagnitudeCalculation(pointset=chord))
    # # geo.add_input(MagnitudeCalculation(pointset=span), connection_name='span')
    # geo.add_input(MagnitudeCalculation(pointset=chord), connection_name='chord')
    # # geo.add_constraint(MagnitudeCalculation(pointset=chord), value=15.)
    # # geo.add_input(calculation_type=nasa_uli_tc1.MAGNITUDE, pointset=chord, connection_name='chord')

    # inner_opt_implicit_model = InnerOptimizationModel(geometry=geo)

    # test_model = csdl.Model()
    # test_model.create_input('chord', val=10.)
    # # test_model.create_input('span', val=30.)

    # test_model.add(submodel=inner_opt_implicit_model, name='InnerOptimizationModel', promotes=[])

    # for geometric_input in geo.inputs:
    #     test_model.connect(f'{geometric_input.connection_name}', f'InnerOptimizationModel.{geometric_input.connection_name}')

    # # sim = Simulator(inner_opt_implicit_model)
    # sim = Simulator(test_model)
    # sim.run()
    # # sim.visualize_implementation()
    # # sim.prob.check_totals(of='InnerOptimizationModel.ffd_free_dof', wrt='chord')
    # # print('ffd_free_dof', sim['ffd_free_dof'].reshape((NUM_PROPERTIES,NUM_DV_PER_PARAMETER)))


    # Multiple FFD blocks
    stp_path = STP_FILES_FOLDER / 'dw_with_nacelles.stp'
    geo = Geometry()
    geo.read_file(file_name=stp_path)

    wing_comp = Component(stp_entity_names=['Wing'], name='wing', nxp=2, nyp=3, nzp=2)  # Creating a wing component and naming it wing
    tail_comp = Component(stp_entity_names=['Tail'], name='tail', nxp=2, nyp=3, nzp=2)
    front_left_nacelle_comp = Component(stp_entity_names=['LiftNacelleFrontLeft'], name='lift_nacelle_front_left')
    front_right_nacelle_comp = Component(stp_entity_names=['LiftNacelleFrontRight'], name='lift_nacelle_front_right')
    geo.add_component(wing_comp)
    geo.add_component(tail_comp)
    geo.add_component(front_left_nacelle_comp)
    geo.add_component(front_right_nacelle_comp)

    wing_comp.add_ffd_parameter(FFDScaleYParameter(degree=1, num_dof=3, cost_factor=1.))
    wing_comp.add_ffd_parameter(FFDScaleZParameter(degree=1, num_dof=3, cost_factor=1.))
    wing_comp.add_ffd_parameter(FFDRotationXParameter(degree=1, num_dof=3, value=np.array([0., -1., 0.])))
    wing_comp.add_ffd_parameter(FFDTranslationXParameter(degree=1, num_dof=2, cost_factor=1.))
    tail_comp.add_ffd_parameter(FFDScaleZParameter(degree=1, num_dof=3, value=np.array([0., 1., 0.])))
    # tail_comp.add_ffd_parameter(FFDScaleYParameter(degree=1, num_dof=3))
    tail_comp.add_ffd_parameter(FFDTranslationXParameter(degree=0, num_dof=1))
    tail_comp.add_ffd_parameter(FFDTranslationYParameter(degree=0, num_dof=1))
    tail_comp.add_ffd_parameter(FFDTranslationZParameter(degree=0, num_dof=1))
    # front_left_nacelle_comp.add_ffd_parameter(FFDScaleYParameter(degree=2, num_dof=5, cost_factor=1.))
    # front_right_nacelle_comp.add_ffd_parameter(FFDTranslationXParameter(degree=1, num_dof=2, connection_name='hanging_input'))

    up_direction = np.array([0., 0., 1.])
    down_direction = np.array([0., 0., -1.])

    left_lead_point = np.array([0., -9000., 2000.])/1000
    left_trail_point = np.array([2000.0, -9000.0, 2000.])/1000
    right_lead_point = np.array([0.0, 9000.0, 2000.])/1000
    right_trail_point = np.array([2000.0, 9000.0, 2000.])/1000
    mid_lead_point = np.array([0., 0., 2.])
    mid_trail_point = np.array([2., 0., 2.])

    tail_lead_mid_point = np.array([2., 0., 2.])

    '''Project points'''
    wing_lead_left, wing_lead_left_coord = geo.project_points(left_lead_point, projection_direction = down_direction, projection_targets_names=["wing"])
    wing_trail_left, wing_trail_left_coord = geo.project_points(left_trail_point, projection_direction = down_direction, projection_targets_names=["wing"])
    wing_lead_right, wing_lead_right_coord = geo.project_points(right_lead_point, projection_direction = down_direction, projection_targets_names=["wing"])
    wing_trail_right, wing_trail_right_coord = geo.project_points(right_trail_point, projection_direction = down_direction, projection_targets_names=["wing"])
    wing_lead_mid, _ = geo.project_points(mid_lead_point, projection_direction = down_direction, projection_targets_names=["wing"])
    wing_trail_mid, _ = geo.project_points(mid_trail_point, projection_direction = down_direction, projection_targets_names=["wing"])

    tail_lead_mid, _ = geo.project_points(tail_lead_mid_point, projection_targets_names=['tail'])

    root_chord = geo.subtract_pointsets(wing_lead_mid, wing_trail_mid)
    left_tip_chord = geo.subtract_pointsets(wing_lead_left, wing_trail_left)
    right_tip_chord = geo.subtract_pointsets(wing_lead_right, wing_trail_right)
    span = geo.subtract_pointsets(wing_trail_right, wing_trail_left)
    wing_to_tail_displacement = geo.subtract_pointsets(tail_lead_mid, wing_trail_mid)

    # Adding inputs to the geometry model
    geo.add_input(MagnitudeCalculation(pointset=root_chord), connection_name='chord')
    # geo.add_constraint(MagnitudeCalculation(pointset=left_tip_chord))
    # geo.add_constraint(MagnitudeCalculation(pointset=right_tip_chord))
    geo.add_input(MagnitudeCalculation(pointset=span), connection_name='span')
    # geo.add_input(DisplacementCalculation(wing_to_tail_displacement), connection_name='wing_to_tail_displacement')
    geo.add_constraint(DisplacementCalculation(wing_to_tail_displacement))

    inner_opt_implicit_model = InnerOptimizationModel(geometry=geo)

    test_model = csdl.Model()
    test_model.create_input('chord', val=6.)
    test_model.create_input('span', val=20.)
    # test_model.create_input('wing_to_tail_displacement', val=np.array([10., 0., 0.]))

    test_model.add(submodel=inner_opt_implicit_model, name='InnerOptimizationModel', promotes=[])

    for geometric_input in geo.inputs:
        test_model.connect(f'{geometric_input.connection_name}', f'InnerOptimizationModel.{geometric_input.connection_name}')


    sim = Simulator(test_model)
    sim.run()
    sim.check_totals(of='InnerOptimizationModel.ffd_free_dof', wrt='chord')
    sim.check_totals(of='InnerOptimizationModel.ffd_free_dof', wrt='span')
    # sim.check_totals(of='InnerOptimizationModel.ffd_free_dof', wrt='wing_to_tail_displacement')



    geometry = geo
    ffd_set = geometry.ffd_set

    ffd_free_dof = sim['InnerOptimizationModel.ffd_free_dof']     # output
    lagrange_multipliers = sim['InnerOptimizationModel.lagrange_multipliers']   # output

    print('ffd_free_dof', ffd_free_dof)
    print('norm(ffd_free_dof)', np.linalg.norm(sim['InnerOptimizationModel.ffd_free_dof']))
    print('lagrange multipliers', lagrange_multipliers)

    ffd_set = geometry.ffd_set
    ffd_blocks = ffd_set.ffd_blocks

    cost_matrix = ffd_set.cost_matrix
    free_section_properties_map = ffd_set.free_section_properties_map
    prescribed_section_properties_map = ffd_set.prescribed_section_properties_map
    ffd_control_points_map = ffd_set.ffd_control_points_map
    ffd_control_points_x_map = ffd_set.ffd_control_points_x_map
    ffd_control_points_y_map = ffd_set.ffd_control_points_y_map
    ffd_control_points_z_map = ffd_set.ffd_control_points_z_map
    sectional_rotations_map = ffd_set.sectional_rotations_map
    # rotated_ffd_control_points_map = ffd_set.rotated_ffd_control_points_map   # not real map since nonlinear. Must be constructed from prescribed rotational dof
    geometry_control_points_map = ffd_set.geometry_control_points_map
    unchanged_geometry_indexing_map = ffd_set.unchanged_geometry_indexing_map
    pointset_map = geometry.eval_map

    num_affine_free_dof = ffd_set.num_affine_free_dof
    num_affine_section_properties = ffd_set.num_affine_section_properties
    num_affine_ffd_control_points = ffd_set.num_affine_ffd_control_points
    num_affine_free_ffd_control_points = ffd_set.num_affine_free_ffd_control_points
    num_embedded_points = ffd_set.num_embedded_points
    num_geometry_control_points = ffd_set.num_geometry_control_points

    # ffd section properties evaluation
    ffd_free_section_properties_without_initial = free_section_properties_map.dot(ffd_free_dof)

    # An initial value of 1 must be added to the scaling section properties. This is not differentiated because it's a constant 1.
    ffd_section_properties = np.zeros((num_affine_section_properties,)) # Section properties model output!
    ffd_block_starting_index = 0
    for ffd_block in ffd_blocks:
        if ffd_block.num_affine_dof == 0:
            continue

        ffd_block_ending_index = ffd_block_starting_index + ffd_block.num_sections * ffd_block.num_affine_properties
        NUM_SCALING_PROPERTIES = 3
        ffd_block_scaling_properties_starting_index = ffd_block_starting_index + ffd_block.num_sections*(ffd_block.num_affine_properties-NUM_SCALING_PROPERTIES)    #The last 2 properties are scaling
        ffd_block_scaling_properties_ending_index = ffd_block_scaling_properties_starting_index + ffd_block.num_sections*(NUM_SCALING_PROPERTIES)

        # Use calculated values for non-scaling parameters
        ffd_section_properties[ffd_block_starting_index:ffd_block_scaling_properties_starting_index] = \
            ffd_free_section_properties_without_initial[ffd_block_starting_index:ffd_block_scaling_properties_starting_index] # -3 is because scale_y and z are the last 2 properties

        # Add 1 to scaling parameters to make initial scaling=1.
        ffd_section_properties[ffd_block_scaling_properties_starting_index:ffd_block_scaling_properties_ending_index] = \
            ffd_free_section_properties_without_initial[ffd_block_scaling_properties_starting_index:ffd_block_scaling_properties_ending_index] + 1.  # adding 1 which is initial scale value

        ffd_block_starting_index = ffd_block_ending_index


    # ffd control points evaluation (affine)
    # NOTE: x, y, and z are split up to take advantage of scipy sparse. numpy tensor will likely be used for derivative for simplicity
    ffd_block_starting_index = 0
    affine_free_ffd_control_points_x_map = None
    for ffd_block in ffd_blocks:
        ffd_block_ending_index = ffd_block_starting_index + ffd_block.num_ffd_control_points

        if ffd_block.num_affine_free_dof != 0:
            if affine_free_ffd_control_points_x_map is None:
                affine_free_ffd_control_points_x_map = ffd_control_points_x_map[ffd_block_starting_index:ffd_block_ending_index, :]
                affine_free_ffd_control_points_y_map = ffd_control_points_y_map[ffd_block_starting_index:ffd_block_ending_index, :]
                affine_free_ffd_control_points_z_map = ffd_control_points_z_map[ffd_block_starting_index:ffd_block_ending_index, :]
            else:
                affine_free_ffd_control_points_x_map = sps.vstack((affine_free_ffd_control_points_x_map, ffd_control_points_x_map[ffd_block_starting_index:ffd_block_ending_index, :]))
                affine_free_ffd_control_points_y_map = sps.vstack((affine_free_ffd_control_points_y_map, ffd_control_points_y_map[ffd_block_starting_index:ffd_block_ending_index, :]))
                affine_free_ffd_control_points_z_map = sps.vstack((affine_free_ffd_control_points_z_map, ffd_control_points_z_map[ffd_block_starting_index:ffd_block_ending_index, :]))

        ffd_block_starting_index = ffd_block_ending_index

    ffd_control_points_x = affine_free_ffd_control_points_x_map.dot(ffd_section_properties)
    ffd_control_points_y = affine_free_ffd_control_points_y_map.dot(ffd_section_properties)
    ffd_control_points_z = affine_free_ffd_control_points_z_map.dot(ffd_section_properties)

    affine_free_ffd_control_points_map = np.zeros((num_affine_free_ffd_control_points, 3, num_affine_section_properties))
    affine_free_ffd_control_points_map[:,0,:] = affine_free_ffd_control_points_x_map.toarray()
    affine_free_ffd_control_points_map[:,1,:] = affine_free_ffd_control_points_y_map.toarray()
    affine_free_ffd_control_points_map[:,2,:] = affine_free_ffd_control_points_z_map.toarray()

    # Combine x,y,z components back to list of points
    affine_ffd_control_points = np.zeros((num_affine_free_ffd_control_points, 3))
    affine_ffd_control_points[:,0] = ffd_control_points_x
    affine_ffd_control_points[:,1] = ffd_control_points_y
    affine_ffd_control_points[:,2] = ffd_control_points_z


    # Construct ffd control points vector from ffd blocks with affine and/or rotational dof
    ffd_control_points_local_frame = np.zeros((num_affine_free_ffd_control_points,3))
    ffd_control_points_starting_index = 0
    affine_ffd_control_points_starting_index = 0
    for ffd_block in ffd_blocks:
        if ffd_block.num_affine_free_dof == 0:
            continue

        ffd_block_num_control_points = ffd_block.nxp * ffd_block.nyp * ffd_block.nzp
        ffd_control_points_ending_index = ffd_control_points_starting_index + ffd_block_num_control_points

        affine_ffd_control_points_ending_index = affine_ffd_control_points_starting_index + ffd_block_num_control_points
        ffd_control_points_local_frame[ffd_control_points_starting_index:ffd_control_points_ending_index] = affine_ffd_control_points[affine_ffd_control_points_starting_index:affine_ffd_control_points_ending_index]
        affine_ffd_control_points_starting_index = affine_ffd_control_points_ending_index

        ffd_control_points_starting_index = ffd_control_points_ending_index


    # transformation back into global frame construction
    global_frame_rotation_map = np.zeros((num_affine_free_ffd_control_points, 3, num_affine_free_ffd_control_points, 3))
    ffd_block_starting_index = 0
    for ffd_block in ffd_blocks:
        if ffd_block.num_affine_free_dof == 0:
            continue

        ffd_block_rotation_matrix_tensor = np.zeros((ffd_block.num_ffd_control_points, 3, ffd_block.num_ffd_control_points, 3))
        for i in range(ffd_block_rotation_matrix_tensor.shape[0]):
            ffd_block_rotation_matrix_tensor[i,:,i,:] = ffd_block.rotation_matrix

        ffd_block_ending_index = ffd_block_starting_index + ffd_block.num_ffd_control_points
        global_frame_rotation_map[ffd_block_starting_index:ffd_block_ending_index,:,ffd_block_starting_index:ffd_block_ending_index,:] = ffd_block_rotation_matrix_tensor
        ffd_block_starting_index = ffd_block_ending_index

    # Transformation back into global frame evaluation
    ffd_control_points_without_origin = np.tensordot(global_frame_rotation_map, ffd_control_points_local_frame)
    ffd_control_points = ffd_control_points_without_origin
    ffd_block_starting_index = 0
    for ffd_block in ffd_blocks:
        if ffd_block.num_affine_free_dof == 0:
            continue
        ffd_block_num_control_points = ffd_block.nxp * ffd_block.nyp * ffd_block.nzp
        ffd_block_ending_index = ffd_block_starting_index + ffd_block_num_control_points
        ffd_control_points[ffd_block_starting_index:ffd_block_ending_index] += np.repeat(ffd_block.section_origins, ffd_block.nyp*ffd_block.nzp, axis=0)
        ffd_block_starting_index = ffd_block_ending_index

    # vp_init = Plotter()
    # vps = []
    # vps1 = Points(ffd_control_points, r=8, c = 'blue')
    # vps2 = Points(ffd_control_points_without_origin, r=7, c='cyan')
    # vps3 = Points(ffd_control_points_local_frame, r=7, c='red')
    # vps4 = Points(affine_ffd_control_points, r=7, c='magenta')
    # vps.append(vps1)
    # vps.append(vps2)
    # vps.append(vps3)
    # vps.append(vps4)

    # vp_init.show(vps, 'FFD Changes', axes=1, viewup="z", interactive = True)

    # Geometry control points evaluation
    if num_embedded_points != num_geometry_control_points:  #
        # Get unchanged geometry control points (points not included in FFD)
        initial_geometry_control_points = geometry.total_cntrl_pts_vector
        unchanged_geometry_control_points = unchanged_geometry_indexing_map.dot(initial_geometry_control_points)

    # --Construct map (ffd control points (in ffd blocks with affine free dof) --> geometry control points)
    affine_geometry_control_points_map = None
    ffd_block_starting_index = 0
    for ffd_block in ffd_blocks:
        if ffd_block.num_affine_free_dof == 0:
            continue

        ffd_block_num_control_points = ffd_block.nxp * ffd_block.nyp * ffd_block.nzp
        ffd_block_ending_index = ffd_block_starting_index + ffd_block_num_control_points

        if affine_geometry_control_points_map is None:
            affine_geometry_control_points_map = geometry_control_points_map[:,ffd_block_starting_index:ffd_block_ending_index]
        else:
            affine_geometry_control_points_map = sps.hstack((affine_geometry_control_points_map, geometry_control_points_map[:,ffd_block_starting_index:ffd_block_ending_index]))

        ffd_block_starting_index = ffd_block_ending_index

    # --Evaluate updated geometry control points using map
    updated_geometry_control_points = affine_geometry_control_points_map.dot(ffd_control_points)

    # --Combine updated and unchanged portions of the geometry to complete the geometry
    if num_embedded_points != num_geometry_control_points:
        geometry_control_points = updated_geometry_control_points + unchanged_geometry_control_points
    else:
        geometry_control_points = updated_geometry_control_points

    ''' Plotting results of Application model '''
    pts_shape = wing_comp.ffd_control_points.shape
    nxp = pts_shape[0]
    nyp = pts_shape[1]
    nzp = pts_shape[2]

    ffd_pts_reshape_updated = np.reshape(wing_comp.ffd_control_points, (nxp * nyp * nzp, 3))
    wing_ffd = list(geo.components_ffd_dict.values())[0]
    cp = wing_ffd.evaluate(ffd_pts_reshape_updated)

    vp_init = Plotter()
    vps = []
    vps1 = Points(ffd_control_points, r=8, c = 'blue')
    # vps2 = Points(np.reshape(initial_ffd_block_control_points, (nxp * nyp * nzp, 3)), r=9, c = 'red')
    vps3 = Points(np.reshape(ffd_blocks[0].control_points, (nxp * nyp * nzp, 3)), r=9, c = 'red')
    vps4 = Points(geo.total_cntrl_pts_vector, r=6, c='black')
    vps5 = Points(geometry_control_points, r=7, c='cyan')
    vps.append(vps1)
    vps.append(vps3)
    vps.append(vps4)
    vps.append(vps5)

    vp_init.show(vps, 'FFD Changes', axes=1, viewup="z", interactive = True)
