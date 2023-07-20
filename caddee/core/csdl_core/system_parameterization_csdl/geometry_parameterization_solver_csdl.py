import csdl
import numpy as np
import array_mapper as am
import scipy.sparse as sps

import time

'''
IDEA: If solution fails (NaN or max iteration), the constraints are enforced with penalty method instead of lagrange multipliers
    which turns the problem into a regularized least squares problem (Note: There is a nonlinear operation, so can't use pseudo-inverse).

NOTE: Currently converges for simple case, but I'm almost sure that the d2c_dx2 is wrong. Also not sure that dc_dx is correct.
-- Also need to connect the output of this model to the FFD model.
'''

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
        for input_name, input in inputs.items():
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
        system_representation = system_parameterization.system_representation

        geometry_parameterizations = system_parameterization.geometry_parameterizations

        # Collect the parameterizations which have free dof (one parameterization is like one FFDSet)
        free_geometry_parameterizations = {}
        for geometry_parameterization_name, geometry_parameterization in geometry_parameterizations.items():
            if geometry_parameterization.num_affine_free_dof != 0:
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
        self.declare_derivatives('ffd_free_dof', 'ffd_free_dof')
        self.declare_derivatives('ffd_free_dof', 'parameterization_lagrange_multipliers')
        self.declare_derivatives('parameterization_lagrange_multipliers', 'ffd_free_dof')
        self.declare_derivatives('parameterization_lagrange_multipliers', 'parameterization_lagrange_multipliers')
        self.declare_derivatives('ffd_free_dof', 'parameterization_inputs')
        self.declare_derivatives('parameterization_lagrange_multipliers', 'parameterization_inputs')

        # self.declare_derivatives('*','*')

        self.linear_solver = csdl.ScipyKrylov()
        self.nonlinear_solver = csdl.NewtonSolver(solve_subsystems=True, maxiter=10)

        # Precompute maps
        ffd_free_dof_to_local_ffd_control_points = ffd_set.affine_block_deformations_map.dot(ffd_set.free_affine_section_properties_map)
        # Apply rotation to each map
        rotation_matrices = []
        for ffd_block in list(ffd_set.ffd_blocks.values()):
            if ffd_block.num_dof == 0:
                continue
            rotation_matrices.extend([ffd_block.local_to_global_rotation]*ffd_block.num_control_points)
        local_to_global = sps.block_diag(rotation_matrices)
        ffd_free_dof_to_ffd_control_points = local_to_global.dot(ffd_free_dof_to_local_ffd_control_points)
        # Map to embedded entities (geometry control points that are embedded) (need to expand/repeat map from num_ffd_cp to num_ffd_cp*3)
        NUM_PHYSICAL_DIMENSIONS = 3
        expanded_ffd_embedded_entity_map = sps.lil_matrix(
            (ffd_set.embedded_entities_map.shape[0]*NUM_PHYSICAL_DIMENSIONS,ffd_set.embedded_entities_map.shape[1]*NUM_PHYSICAL_DIMENSIONS))
        for i in range(NUM_PHYSICAL_DIMENSIONS):
            input_indices = np.arange(0,ffd_set.embedded_entities_map.shape[1])*NUM_PHYSICAL_DIMENSIONS + i
            output_indices = np.arange(0,ffd_set.embedded_entities_map.shape[0])*NUM_PHYSICAL_DIMENSIONS + i
            expanded_ffd_embedded_entity_map[np.ix_(output_indices,input_indices)] = ffd_set.embedded_entities_map
        expanded_ffd_embedded_entity_map = expanded_ffd_embedded_entity_map.tocsc()
        ffd_free_dof_to_embedded_entities = expanded_ffd_embedded_entity_map.dot(ffd_free_dof_to_ffd_control_points)
        # Map from embedded entities to all geometry control points
        initial_system_representation_geometry = system_representation.spatial_representation.control_points['geometry'].copy()
        indexing_indices = []
        for ffd_block in list(geometry_parameterization.active_ffd_blocks.values()):
            ffd_block_embedded_primitive_names = list(ffd_block.embedded_entities.keys())
            ffd_block_embedded_primitive_indices = []
            for primitive_name in ffd_block_embedded_primitive_names:
                ffd_block_embedded_primitive_indices.extend(list(
                    system_representation.spatial_representation.primitive_indices[primitive_name]['geometry']))
            indexing_indices.extend(ffd_block_embedded_primitive_indices)

        indexing_indices_array = np.array(indexing_indices)
        expanded_indexing_indices = indexing_indices_array*NUM_PHYSICAL_DIMENSIONS
        row_indices = np.concatenate((expanded_indexing_indices, expanded_indexing_indices + 1, expanded_indexing_indices + 2))
        num_points = len(indexing_indices)
        col_indices_array = np.arange(num_points)*NUM_PHYSICAL_DIMENSIONS
        col_indices = np.concatenate((col_indices_array, col_indices_array + 1, col_indices_array + 2))
        num_points_system_representation = initial_system_representation_geometry.shape[0]
        data = np.ones((num_points*3))
        geometry_assembly_map = sps.coo_matrix((data, (row_indices, col_indices)),
                        shape=(num_points_system_representation*NUM_PHYSICAL_DIMENSIONS, num_points*NUM_PHYSICAL_DIMENSIONS))
        geometry_assembly_map = geometry_assembly_map.tocsc()
        ffd_free_dof_to_geometry_control_points = geometry_assembly_map.dot(ffd_free_dof_to_embedded_entities)
        # Map from geometry control points to mapped arrays
        mapped_array_mappings = []
        # self.mapped_array_indices = {}
        mapped_array_indices_counter = 0
        for input_name, parameterization_input in system_parameterization.inputs.items():
            if type(parameterization_input.quantity) is am.MappedArray:
                mapped_array = parameterization_input.quantity
            elif type(parameterization_input.quantity) is am.NonlinearMappedArray:
                mapped_array = parameterization_input.quantity.input
            # num_mapped_array_outputs = mapped_array.linear_map.shape[0]
            # self.mapped_array_indices[input_name] = \
            #     np.arange(mapped_array_indices_counter, mapped_array_indices_counter + num_mapped_array_outputs)
            # mapped_array_indices_counter += num_mapped_array_outputs

            mapped_array_map = mapped_array.linear_map
            expanded_mapped_array_map = sps.lil_matrix(
                (mapped_array_map.shape[0]*NUM_PHYSICAL_DIMENSIONS,mapped_array_map.shape[1]*NUM_PHYSICAL_DIMENSIONS))
            for i in range(NUM_PHYSICAL_DIMENSIONS):
                input_indices = np.arange(0,mapped_array_map.shape[1])*NUM_PHYSICAL_DIMENSIONS + i
                output_indices = np.arange(0,mapped_array_map.shape[0])*NUM_PHYSICAL_DIMENSIONS + i
                expanded_mapped_array_map[np.ix_(output_indices,input_indices)] = mapped_array_map
            expanded_mapped_array_map = expanded_mapped_array_map.tocsc()

            # mapped_array_mapping = sps.block_diag([mapped_array.linear_map]*NUM_PHYSICAL_DIMENSIONS)
            mapped_array_mappings.append(expanded_mapped_array_map)

        mapped_arrays_map = sps.vstack(mapped_array_mappings).tocsc()

        self.ffd_free_dof_to_mapped_arrays = mapped_arrays_map.dot(ffd_free_dof_to_geometry_control_points)


    def evaluate_residuals(self, inputs, outputs, residuals):
        # inputs
        system_parameterization = self.parameters['system_parameterization']
        system_representation = system_parameterization.system_representation

        geometry_parameterizations = system_parameterization.geometry_parameterizations

        # Collect the parameterizations which have free dof (one parameterization is like one FFDSet)
        free_geometry_parameterizations = {}
        for geometry_parameterization_name, geometry_parameterization in geometry_parameterizations.items():
            if geometry_parameterization.num_affine_free_dof != 0:
                free_geometry_parameterizations[geometry_parameterization_name] = geometry_parameterization
                # NOTE! Hardcoding for one FFDSet as the only geometry parameterization
                ffd_set = geometry_parameterization

        parameterization_inputs = inputs['parameterization_inputs']
        num_inputs = len(parameterization_inputs)
        ffd_free_dof = outputs['ffd_free_dof']
        lagrange_multipliers = outputs['parameterization_lagrange_multipliers']

        NUM_PARAMETRIC_DIMENSIONS = 3       # This type of FFD block has 3 parametric dimensions by definition.
        affine_section_properties = ffd_set.evaluate_affine_section_properties(free_affine_dof=ffd_free_dof)
        rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
        affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations(
            affine_section_properties=affine_section_properties)
        rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations(
            affine_deformed_control_points=affine_deformed_ffd_control_points)
        ffd_control_points = ffd_set.evaluate_control_points(rotated_control_points_local_frame=rotated_ffd_control_points)
        ffd_embedded_entities = ffd_set.evaluate_embedded_entities(control_points=ffd_control_points, plot=True)

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

        system_representation_geometry[parameterization_indices,:] = ffd_embedded_entities

        # geometric_calculations evaluation
        c = np.zeros((num_inputs,))   # for displacement, may need to insert more rows since that's 3 constraints.
        dc_dx = np.zeros(((num_inputs,) + ffd_free_dof.shape))

        constraint_counter = 0
        for input_name, parameterization_input in system_parameterization.inputs.items():
            quantity = parameterization_input.quantity
            quantity_num_constraints = np.prod(quantity.shape)

            if type(quantity) is am.MappedArray: # displacement constraint
                c[constraint_counter:constraint_counter+quantity_num_constraints] = \
                    quantity.evaluate(system_representation_geometry) \
                    - parameterization_inputs[constraint_counter:constraint_counter+quantity_num_constraints]
            
                dc_dx[constraint_counter:constraint_counter+quantity_num_constraints,:] = \
                    self.ffd_free_dof_to_mapped_arrays.toarray()
            else:
                nonlinear_operation_input = quantity.input.evaluate(system_representation_geometry)
                c[constraint_counter:constraint_counter+quantity_num_constraints] = \
                    quantity.evaluate(system_representation_geometry, evaluate_input=True) \
                        - parameterization_inputs[constraint_counter:constraint_counter+quantity_num_constraints]
                
                nonlinear_derivative = quantity.evaluate_derivative(nonlinear_operation_input, evaluate_input=False)
                dc_dx[constraint_counter:constraint_counter+quantity_num_constraints,:] = \
                    nonlinear_derivative.dot(self.ffd_free_dof_to_mapped_arrays.toarray())

            constraint_counter += quantity_num_constraints

        dObjective_dx = 2*ffd_set.cost_matrix.dot(ffd_free_dof)

        num_affine_free_dof = geometry_parameterization.num_affine_free_dof
        self.residual_testing = np.zeros((num_affine_free_dof+num_inputs,))
        self.residual_testing[:num_affine_free_dof] = dObjective_dx + np.tensordot(lagrange_multipliers, dc_dx, axes=([-1],[0]))
        self.residual_testing[num_affine_free_dof:] = c.T

        residuals['ffd_free_dof'] = dObjective_dx + np.tensordot(lagrange_multipliers, dc_dx, axes=([-1],[0]))
        residuals['parameterization_lagrange_multipliers'] = c.T


    def compute_derivatives(self, inputs, outputs, derivatives):
        # inputs
        system_parameterization = self.parameters['system_parameterization']
        system_representation = system_parameterization.system_representation

        geometry_parameterizations = system_parameterization.geometry_parameterizations

        # Collect the parameterizations which have free dof (one parameterization is like one FFDSet)
        free_geometry_parameterizations = {}
        for geometry_parameterization_name, geometry_parameterization in geometry_parameterizations.items():
            if geometry_parameterization.num_affine_free_dof != 0:
                free_geometry_parameterizations[geometry_parameterization_name] = geometry_parameterization
                # NOTE! Hardcoding for one FFDSet as the only geometry parameterization
                ffd_set = geometry_parameterization

        parameterization_inputs = inputs['parameterization_inputs']
        num_inputs = len(parameterization_inputs)
        ffd_free_dof = outputs['ffd_free_dof']
        lagrange_multipliers = outputs['parameterization_lagrange_multipliers']

        NUM_PARAMETRIC_DIMENSIONS = 3       # This type of FFD block has 3 parametric dimensions by definition.
        affine_section_properties = ffd_set.evaluate_affine_section_properties(free_affine_dof=ffd_free_dof)
        rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
        affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations(
            affine_section_properties=affine_section_properties)
        rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations(
            affine_deformed_control_points=affine_deformed_ffd_control_points)
        ffd_control_points = ffd_set.evaluate_control_points(rotated_control_points_local_frame=rotated_ffd_control_points)
        ffd_embedded_entities = ffd_set.evaluate_embedded_entities(control_points=ffd_control_points)

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

        system_representation_geometry[parameterization_indices,:] = ffd_embedded_entities

        # quantity evaluation
        # c = np.zeros((num_geometric_outputs,))   # Don't need c for Hessian.
        dc_dx = np.zeros(((num_inputs,) + ffd_free_dof.shape))
        d2c_dx2 = np.zeros(((num_inputs,)  + ffd_free_dof.shape + ffd_free_dof.shape))

        constraint_counter = 0
        for input_name, parameterization_input in system_parameterization.inputs.items():
            quantity = parameterization_input.quantity
            quantity_num_constraints = np.prod(quantity.shape)

            if type(quantity) is am.MappedArray:
                dc_dx[constraint_counter:constraint_counter+quantity_num_constraints,:] = \
                    self.ffd_free_dof_to_mapped_arrays.toarray()

                d2c_dx2[constraint_counter:constraint_counter+quantity_num_constraints,:,:] = 0.
            else:
                nonlinear_operation_input = quantity.input.evaluate(system_representation_geometry)
                mapped_array_size = np.prod(nonlinear_operation_input.shape)

                nonlinear_derivative = quantity.evaluate_derivative(nonlinear_operation_input, evaluate_input=False)
                dc_dx[constraint_counter:constraint_counter+quantity_num_constraints,:] = \
                    nonlinear_derivative.dot(self.ffd_free_dof_to_mapped_arrays.toarray())
                
                nonlinear_second_derivative = quantity.evaluate_second_derivative(nonlinear_operation_input).reshape(
                    (quantity_num_constraints, mapped_array_size, mapped_array_size))

                first_term = np.tensordot(nonlinear_second_derivative, self.ffd_free_dof_to_mapped_arrays.toarray(), axes=([2, 0]))
                total_second_derivative = np.tensordot(first_term, self.ffd_free_dof_to_mapped_arrays.toarray(), axes=([1, 0]))
                d2c_dx2[constraint_counter:constraint_counter+quantity_num_constraints,:,:] = total_second_derivative

            constraint_counter += quantity_num_constraints

        d2objective_dx2 = 2*ffd_set.cost_matrix

        num_affine_free_dof = ffd_set.num_affine_free_dof
        hessian = np.zeros((num_affine_free_dof+num_inputs,num_affine_free_dof+num_inputs))
        hessian[:num_affine_free_dof, :num_affine_free_dof] = \
            d2objective_dx2 + np.tensordot(lagrange_multipliers, d2c_dx2, axes=([-1],[0]))    # upper left block
        hessian[:num_affine_free_dof, num_affine_free_dof:] = dc_dx.T   # uper right block
        hessian[num_affine_free_dof:, :num_affine_free_dof] = dc_dx # lower left block
        # hessian[num_affine_free_dof:, num_affine_free_dof:] = 0.  lower right block preallocated to zeroes

        derivatives['ffd_free_dof', 'ffd_free_dof'] = hessian[:num_affine_free_dof, :num_affine_free_dof]
        derivatives['ffd_free_dof', 'parameterization_lagrange_multipliers'] = hessian[:num_affine_free_dof, num_affine_free_dof:]
        derivatives['parameterization_lagrange_multipliers', 'ffd_free_dof'] = hessian[num_affine_free_dof:, :num_affine_free_dof]
        derivatives['parameterization_lagrange_multipliers', 'parameterization_lagrange_multipliers'] = \
            hessian[num_affine_free_dof:, num_affine_free_dof:]
        
        # df_dx = pf_px - pf_py.dot(pr_py**-1).dot(pr_px)
        pr_px = np.zeros((num_inputs+num_affine_free_dof, num_inputs))
        # pr_px[:num_affine_free_dof,:] = 0.
        pr_px[num_affine_free_dof:,:] = -np.eye(num_inputs)
        # pr_py = -hessian
        # dy_dx = np.linalg.solve(pr_py, pr_px)   # solve using direct method since num inputs < num outputs
        # df_dx = dy_dx   # df_dy = I since f=y, also_pf_py = 0, so df_dx = 0 - I.dot(dy_dx)
        # dffddof_dinputsandconstraints = df_dx[:num_affine_free_dof]
        # # dlagrangemultipliers_dinputsandconstraints = df_dx[num_affine_free_dof:]
        # # derivatives['ffd_free_dof', 'inputs_and_constraints'] = dffddof_dinputsandconstraints
        # # derivatives['parameterization_lagrange_multipliers', 'inputs_and_constraints'] = dlagrangemultipliers_dinputsandconstraints    # don't care about variations in lagrange multipliers

        derivatives['ffd_free_dof', 'parameterization_inputs'] = pr_px[:num_affine_free_dof,:]
        derivatives['parameterization_lagrange_multipliers', 'parameterization_inputs'] = pr_px[num_affine_free_dof:,:]


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
    parameterization_lagrange_multipliers = sim['InnerOptimizationModel.parameterization_lagrange_multipliers']   # output

    print('ffd_free_dof', ffd_free_dof)
    print('norm(ffd_free_dof)', np.linalg.norm(sim['InnerOptimizationModel.ffd_free_dof']))
    print('lagrange multipliers', parameterization_lagrange_multipliers)

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
