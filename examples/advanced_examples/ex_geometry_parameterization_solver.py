import csdl
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am

from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization
from caddee import GEOMETRY_FILES_FOLDER

system_representation = SystemRepresentation()
spatial_rep = system_representation.spatial_representation
file_name = 'lift_plus_cruise_final.stp'
# file_path = 'models/stp/'
spatial_rep.import_file(file_name=file_name)
spatial_rep.refit_geometry(num_control_points=15, fit_resolution=30, file_name=file_name)
# spatial_rep.plot(point_types=['evaluated_points'])
# spatial_rep.plot(point_types=['control_points'])
system_parameterization = SystemParameterization(system_representation=system_representation)

# Create Components
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
horizontal_stabilizer = LiftingSurface(name='tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
fuselage_primitive_names = list(spatial_rep.get_primitives(search_names=['Fuselage']))
fuselage = Component(name='fuselage', spatial_representation=spatial_rep, primitive_names=fuselage_primitive_names)

system_representation.add_component(wing)
system_representation.add_component(horizontal_stabilizer)
system_representation.add_component(fuselage)

# Meshes definitions
num_spanwise_vlm = 21
num_chordwise_vlm = 5
leading_edge = wing.project(am.linspace(am.array([8., -26., 7.6]), am.array([8., 26., 7.5]), num_spanwise_vlm),
                            direction=am.array([0., 0., -1.]))  # returns MappedArray
trailing_edge = wing.project(np.linspace(np.array([15., -26., 7.7]), np.array([15., 26., 7.5]), num_spanwise_vlm),
                             direction=np.array([0., 0., -1.]))   # returns MappedArray
chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15)
wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15)
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1
wing_camber_surface = wing_camber_surface.reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
system_representation.add_output(name='chord_distribution', quantity=am.norm(leading_edge-trailing_edge))


num_spanwise_vlm = 11
num_chordwise_vlm = 3
leading_edge = horizontal_stabilizer.project(np.linspace(np.array([27., -6.5, 6.1]), np.array([27., 6.75, 6.1]), num_spanwise_vlm),
                                             direction=np.array([0., 0., -1.]), grid_search_n=15)
trailing_edge = horizontal_stabilizer.project(np.linspace(np.array([31.5, -6.5, 6.1]), np.array([31.5, 6.75, 6.1]), num_spanwise_vlm),
                                              direction=np.array([0., 0., -1.]), grid_search_n=15)
chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
horizontal_stabilizer_upper_surface_wireframe = horizontal_stabilizer.project(chord_surface.value + np.array([0., 0., 1.]),
                                                                              direction=np.array([0., 0., -1.]), grid_search_n=15)
horizontal_stabilizer_lower_surface_wireframe = horizontal_stabilizer.project(chord_surface.value - np.array([0., 0., 1.]),
                                                                              direction=np.array([0., 0., 1.]), grid_search_n=15)
horizontal_stabilizer_camber_surface = am.linspace(horizontal_stabilizer_upper_surface_wireframe, horizontal_stabilizer_lower_surface_wireframe, 1)
horizontal_stabilizer_camber_surface = horizontal_stabilizer_camber_surface.reshape((num_chordwise_vlm, num_spanwise_vlm, 3))

# system_representation.add_output('horizontal_stabilizer_camber_surface', horizontal_stabilizer_camber_surface)
starboard_trailing_tip = wing.project(np.array([15., 26., 7.5]), direction=np.array([0., 0., -1.]))
port_trailing_tip = wing.project(np.array([15., -26., 7.5]), direction=np.array([0., 0., -1.]))
wingspan_vector = starboard_trailing_tip - port_trailing_tip
wingspan = am.norm(wingspan_vector)     # NOTE: Nonlinear operations don't return MappedArrays. They return NonlinearMappedarrays
system_representation.add_output(name='wingspan', quantity=wingspan)
root_leading = wing.project(np.array([9., 0., 7.5]), direction=np.array([0., 0., -1.]))
root_trailing = wing.project(np.array([15., 0., 7.5]), direction=np.array([0., 0., -1.]))
root_chord_vector = root_leading - root_trailing
root_chord = am.norm(root_chord_vector)     # NOTE: Nonlinear operations don't return MappedArrays. They return NonlinearMappedarrays
system_representation.add_output(name='wing_root_chord', quantity=root_chord)
system_representation.add_output(name='wing_root_chord_vector', quantity=root_chord_vector)

# # Parameterization
from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock

# points = np.array([
#     [
#         [
#             [10., -26., 5.],
#             [10., -26., 10.]
#         ],
#         [
#             [14., -26., 5.],
#             [14., -26., 10.]
#         ]
#     ],
#     [
#         [
#             [8., 0., 5.],
#             [8., 0., 10.]
#         ],
#         [
#             [15., 0., 5.],
#             [15., 0., 10.]
#         ]
#     ],
#     [
#         [
#             [10., 26., 5.],
#             [10., 26., 10.]
#         ],
#         [
#             [14., 26., 5.],
#             [14., 26., 10.]
#         ]
#     ]
# ])
# wing_geometry_primitives = wing.get_geometry_primitives()
# from caddee.core.primitives.bsplines.bspline_functions import create_bspline_from_corners
# wing_ffd_bspline_volume = create_bspline_from_corners(points, order=(3,2,2), num_control_points=(2,2,2))
# # wing_ffd_bspline_volume.plot()
# wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
# wing_ffd_block.plot(plot_embedded_entities=True)
# wing_ffd_block.plot_sections(opacity=1.)

wing_geometry_primitives = wing.get_geometry_primitives()
wing_ffd_bspline_volume = create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(3, 2, 2), order=(3,2,2),
                                                            xyz_to_uvw_indices=(1,0,2))
wing_ffd_block = SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
wing_ffd_block.add_scale_v(name='linear_taper', order=2, num_dof=3, cost_factor=1.)
wing_ffd_block.add_scale_w(name='constant_thickness_scaling', order=1, num_dof=1)
wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10,
                              value=1/4*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))
wing_ffd_block.set_section_origins(np.array([[0., 0.5],
                                            [1., 0.5],
                                            [0.5, 0.5]]))

horizontal_stabilizer_geometry_primitives = horizontal_stabilizer.get_geometry_primitives()
horizontal_stabilizer_ffd_bspline_volume = create_cartesian_enclosure_volume(horizontal_stabilizer_geometry_primitives,num_control_points=(4, 2, 2),
                                                                             order=(3,2,2), xyz_to_uvw_indices=(1,0,2))
horizontal_stabilizer_ffd_block = SRBGFFDBlock(name='horizontal_stabilizer_ffd_block', primitive=horizontal_stabilizer_ffd_bspline_volume,
                                               embedded_entities=horizontal_stabilizer_geometry_primitives)
horizontal_stabilizer_ffd_block.add_scale_v(name='horizontal_stabilizer_linear_taper', order=2, num_dof=3, value=np.array([0.5, 0.5, 0.5]),
                                            cost_factor=1.)
horizontal_stabilizer_ffd_block.add_rotation_u(name='horizontal_stabilizer_twist_distribution', order=1, num_dof=1, value=np.array([np.pi/10]))

point_on_wing = wing.project(np.array([10., 0., 10.]), direction=np.array([0., 0., -1.]))
point_on_tail = horizontal_stabilizer.project(np.array([28., 0., 10.]), direction=np.array([0., 0., -1.]))
wing_to_tail_distance = am.norm(point_on_wing-point_on_tail)

from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet
ffd_blocks = {wing_ffd_block.name : wing_ffd_block, horizontal_stabilizer_ffd_block.name : horizontal_stabilizer_ffd_block}
ffd_set = SRBGFFDSet(name='ffd_set', ffd_blocks=ffd_blocks)
system_parameterization.add_geometry_parameterization(ffd_set)

system_parameterization.add_input(name='root_chord', quantity=root_chord)

system_parameterization.setup()

# Configuration creations and prescribed actuations
configuration_names = ["hover_configuration", "cruise_configuration"]
system_configurations = system_representation.declare_configurations(names=configuration_names)
hover_configuration = system_configurations['hover_configuration']
cruise_configuration = system_configurations['cruise_configuration']

cruise_num_nodes = 5
cruise_configuration.set_num_nodes(num_nodes=cruise_num_nodes)
cruise_configuration.add_output('horizontal_stabilizer_camber_surface', horizontal_stabilizer_camber_surface)
cruise_configuration.add_output('wing_camber_surface', wing_camber_surface)

system_representation.add_output('wing_to_tail_distance', wing_to_tail_distance)
# cruise_configuration.add_output('wing_to_tail_distance', wing_to_tail_distance)

horizontal_stabilizer_quarter_chord_port = horizontal_stabilizer.project(np.array([28.5, -10., 8.]))
horizontal_stabilizer_quarter_chord_starboard = horizontal_stabilizer.project(np.array([28.5, 10., 8.]))
horizontal_stabilizer_actuation_axis = horizontal_stabilizer_quarter_chord_starboard - horizontal_stabilizer_quarter_chord_port
from caddee.core.caddee_core.system_representation.prescribed_actuations import PrescribedRotation
horizontal_stabilizer_actuator_solver = PrescribedRotation(component=horizontal_stabilizer, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuation_profile = np.linspace(0., -1., cruise_num_nodes)
horizontal_stabilizer_actuator_solver.set_rotation(name='cruise_tail_actuation', value=horizontal_stabilizer_actuation_profile, units='radians')
# horizontal_stabilizer_actuator_solver.set_rotation(name='cruise_tail_actuation', value=-0.5 , units='radians')
cruise_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_quarter_chord_port = wing.project(np.array([28.5, -10., 8.]))
wing_quarter_chord_starboard = wing.project(np.array([28.5, 10., 8.]))
wing_actuation_axis = wing_quarter_chord_starboard - wing_quarter_chord_port
wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuation_profile = np.linspace(0., 1., cruise_num_nodes)
wing_actuator_solver.set_rotation(name='cruise_wing_actuation', value=wing_actuation_profile, units='radians')
# wing_actuator_solver.set_rotation(name='cruise_wing_actuation', value=0.2 , units='radians')
cruise_configuration.actuate(transformation=wing_actuator_solver)

system_representation_model = system_representation.assemble_csdl()
system_parameterization_model = system_parameterization.assemble_csdl()

my_model = csdl.Model()
my_model.add(system_parameterization_model, 'system_parameterization')
my_model.add(system_representation_model, 'system_representation')
# my_model.add(solver_model, 'solver_model_name')

# initial_guess_linear_taper = np.array([0., 2., 0.])
# my_model.create_input('linear_taper', val=initial_guess_linear_taper)
# my_model.add_design_variable('linear_taper')
# my_model.add_objective('wing_camber_surface')
my_model.create_input('root_chord', 10.)
my_model.add_design_variable('root_chord')
my_model.add_objective('wing_to_tail_distance')

sim = Simulator(my_model, analytics=True, display_scripts=True)
sim.run()
# sim.compute_total_derivatives()
# sim.check_totals(of='wing_root_chord',wrt='linear_taper', step=1e-6)

# affine_section_properties = ffd_set.evaluate_affine_section_properties(prescribed_affine_dof=np.append(initial_guess_linear_taper, np.zeros(4,)))
affine_section_properties = ffd_set.evaluate_affine_section_properties()
rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
affine_deformed_ffd_control_points = ffd_set.evaluate_affine_block_deformations()
rotated_ffd_control_points = ffd_set.evaluate_rotational_block_deformations()
ffd_control_points = ffd_set.evaluate_control_points()
ffd_embedded_entities = ffd_set.evaluate_embedded_entities()

# updated_primitives_names = wing.primitive_names.copy()
# updated_primitives_names.extend(horizontal_stabilizer.primitive_names.copy())

print('wing to tail distance: ', sim['wing_to_tail_distance'])
# exit()
for t in range(cruise_num_nodes):
    updated_primitives_names = list(spatial_rep.primitives.keys()).copy()
    if cruise_num_nodes == 1:
        cruise_geometry = sim['cruise_configuration_geometry']
    else:
        cruise_geometry = sim['cruise_configuration_geometry'][t]
    # cruise_geometry = sim['design_geometry']
    spatial_rep.update(cruise_geometry, updated_primitives_names)

    wing_camber_surface.evaluate(spatial_rep.control_points['geometry'])
    horizontal_stabilizer_camber_surface.evaluate(spatial_rep.control_points['geometry'])

    # print('wingspan', sim['wingspan'])
    # print("Python and CSDL difference: wingspan", np.linalg.norm(wingspan.value - sim['wingspan']))
    # print('wing root chord', sim['wing_root_chord'])
    # print("Python and CSDL difference: wing root chord", np.linalg.norm(root_chord.value - sim['wing_root_chord']))

    if cruise_num_nodes == 1:
        wing_camber_surface_csdl = sim['wing_camber_surface']
        horizontal_stabilizer_camber_surface_csdl = sim['horizontal_stabilizer_camber_surface']
    else:
        wing_camber_surface_csdl = sim['wing_camber_surface'][t]
        horizontal_stabilizer_camber_surface_csdl = sim['horizontal_stabilizer_camber_surface'][t]
    # print("Python and CSDL difference: wing camber surface", np.linalg.norm(wing_camber_surface_csdl - wing_camber_surface.value))
    # print("Python and CSDL difference: horizontal stabilizer camber surface", 
    #       np.linalg.norm(horizontal_stabilizer_camber_surface_csdl - horizontal_stabilizer_camber_surface.value))

    # print(f'wing to tail distance at time {t}: ', sim['wing_to_tail_distance'][t])
    # spatial_rep.plot()
    spatial_rep.plot_meshes([wing_camber_surface_csdl, horizontal_stabilizer_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)
