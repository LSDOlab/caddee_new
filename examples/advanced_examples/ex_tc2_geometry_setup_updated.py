'''Example TC2 geometry setup: Description of example 2'''
import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from caddee import GEOMETRY_FILES_FOLDER
from caddee.utils.helper_functions.geometry_helpers import make_rotor_mesh, make_vlm_camber_mesh, make_1d_box_beam_mesh, compute_component_surface_area, BladeParameters
from caddee.utils.aircraft_models.drag_models.drag_build_up import DragComponent
from lsdo_geo.core.parameterization.volume_sectional_parameterization import VolumeSectionalParameterization
import lsdo_geo.splines.b_splines as bsp
import gc
gc.enable()
# Instantiate system model
system_model = m3l.Model()

# Importing and refitting the geometry
geometry = lg.import_geometry(GEOMETRY_FILES_FOLDER / 'LPC_final_custom_blades.stp', parallelize=True)
geometry.refit(parallelize=True, order=(4, 4))
geometry.plot()


# region Declaring all components
# Wing, tails, fuselage
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['Wing'])
# wing.plot()
h_tail = geometry.declare_component(component_name='h_tail', b_spline_search_names=['Tail_1'])
# h_tail.plot()
v_tail = geometry.declare_component(component_name='v_tail', b_spline_search_names=['Tail_2'])
# vtail.plot()
fuselage = geometry.declare_component(component_name='fuselage', b_spline_search_names=['Fuselage_***.main'])
# fuselage.plot()

# Nose hub
nose_hub = geometry.declare_component(component_name='weird_nose_hub', b_spline_search_names=['EngineGroup_10'])
# nose_hub.plot()


# Pusher prop
pp_disk = geometry.declare_component(component_name='pp_disk', b_spline_search_names=['Rotor-9-disk'])
# pp_disk.plot()
pp_blade_1 = geometry.declare_component(component_name='pp_blade_1', b_spline_search_names=['Rotor_9_blades, 0'])
# pp_blade_1.plot()
pp_blade_2 = geometry.declare_component(component_name='pp_blade_2', b_spline_search_names=['Rotor_9_blades, 1'])
# pp_blade_2.plot()
pp_blade_3 = geometry.declare_component(component_name='pp_blade_3', b_spline_search_names=['Rotor_9_blades, 2'])
# pp_blade_3.plot()
pp_blade_4 = geometry.declare_component(component_name='pp_blade_4', b_spline_search_names=['Rotor_9_blades, 3'])
# pp_blade_4.plot()
pp_hub = geometry.declare_component(component_name='pp_hub', b_spline_search_names=['Rotor_9_Hub'])
# pp_hub.plot()
pp_components = [pp_disk, pp_blade_1, pp_blade_2, pp_blade_3, pp_blade_4, pp_hub]

# Rotor: rear left outer
rlo_disk = geometry.declare_component(component_name='rlo_disk', b_spline_search_names=['Rotor_2_disk'])
# rlo_disk.plot()
rlo_blade_1 = geometry.declare_component(component_name='rlo_blade_1', b_spline_search_names=['Rotor_2_blades, 0'])
# rlo_blade_1.plot()
rlo_blade_2 = geometry.declare_component(component_name='rlo_blade_2', b_spline_search_names=['Rotor_2_blades, 1'])
# rlo_blade_2.plot()
rlo_hub = geometry.declare_component(component_name='rlo_hub', b_spline_search_names=['Rotor_2_Hub'])
# rlo_hub.plot()
rlo_boom = geometry.declare_component(component_name='rlo_boom', b_spline_search_names=['Rotor_2_Support'])
# rlo_boom.plot()
rlo_components = [rlo_disk, rlo_blade_1, rlo_blade_2, rlo_hub]

# Rotor: rear left inner
rli_disk = geometry.declare_component(component_name='rli_disk', b_spline_search_names=['Rotor_4_disk'])
# rli_disk.plot()
rli_blade_1 = geometry.declare_component(component_name='rli_blade_1', b_spline_search_names=['Rotor_4_blades, 0'])
# rli_blade_1.plot()
rli_blade_2 = geometry.declare_component(component_name='rli_blade_2', b_spline_search_names=['Rotor_4_blades, 1'])
# rli_blade_2.plot()
rli_hub = geometry.declare_component(component_name='rli_hub', b_spline_search_names=['Rotor_4_Hub'])
# rli_hub.plot()
rli_boom = geometry.declare_component(component_name='rli_boom', b_spline_search_names=['Rotor_4_Support'])
# rli_boom.plot()
rli_components = [rli_disk, rli_blade_1, rli_blade_2, rli_hub]

# Rotor: rear right inner
rri_disk = geometry.declare_component(component_name='rri_disk', b_spline_search_names=['Rotor_6_disk'])
# rri_disk.plot()
rri_blade_1 = geometry.declare_component(component_name='rri_blade_1', b_spline_search_names=['Rotor_6_blades, 0'])
# rri_blade_1.plot()
rri_blade_2 = geometry.declare_component(component_name='rri_blade_2', b_spline_search_names=['Rotor_6_blades, 1'])
# rri_blade_2.plot()
rri_hub = geometry.declare_component(component_name='rri_hub', b_spline_search_names=['Rotor_6_Hub'])
# rri_hub.plot()
rri_boom = geometry.declare_component(component_name='rri_boom', b_spline_search_names=['Rotor_6_Support'])
# rri_boom.plot()
rri_components = [rri_disk, rri_blade_1, rri_blade_2, rri_hub]

# Rotor: rear right outer
rro_disk = geometry.declare_component(component_name='rro_disk', b_spline_search_names=['Rotor_8_disk'])
# rro_disk.plot()
rro_blade_1 = geometry.declare_component(component_name='rro_blade_1', b_spline_search_names=['Rotor_8_blades, 0'])
# rro_blade_1.plot()
rro_blade_2 = geometry.declare_component(component_name='rro_blade_2', b_spline_search_names=['Rotor_8_blades, 1'])
# rro_blade_2.plot()
rro_hub = geometry.declare_component(component_name='rro_hub', b_spline_search_names=['Rotor_8_Hub'])
# rro_hub.plot()
rro_boom = geometry.declare_component(component_name='rro_boom', b_spline_search_names=['Rotor_8_Support'])
# rro_boom.plot()
rro_components = [rro_disk, rro_blade_1, rro_blade_2, rro_hub]

# Rotor: front left outer
flo_disk = geometry.declare_component(component_name='flo_disk', b_spline_search_names=['Rotor_1_disk'])
# flo_disk.plot()
flo_blade_1 = geometry.declare_component(component_name='flo_blade_1', b_spline_search_names=['Rotor_1_blades, 0'])
# flo_blade_1.plot()
flo_blade_2 = geometry.declare_component(component_name='flo_blade_2', b_spline_search_names=['Rotor_1_blades, 1'])
# flo_blade_2.plot()
flo_hub = geometry.declare_component(component_name='flo_hub', b_spline_search_names=['Rotor_1_Hub'])
# flo_hub.plot()
flo_boom = geometry.declare_component(component_name='flo_boom', b_spline_search_names=['Rotor_1_Support'])
# flo_boom.plot()
flo_components = [flo_disk, flo_blade_1, flo_blade_2, flo_hub]

# Rotor: front left inner
fli_disk = geometry.declare_component(component_name='fli_disk', b_spline_search_names=['Rotor_3_disk'])
# fli_disk.plot()
fli_blade_1 = geometry.declare_component(component_name='fli_blade_1', b_spline_search_names=['Rotor_3_blades, 0'])
# fli_blade_1.plot()
fli_blade_2 = geometry.declare_component(component_name='fli_blade_2', b_spline_search_names=['Rotor_3_blades, 1'])
# fli_blade_2.plot()
fli_hub = geometry.declare_component(component_name='fli_hub', b_spline_search_names=['Rotor_3_Hub'])
# fli_hub.plot()
fli_boom = geometry.declare_component(component_name='fli_boom', b_spline_search_names=['Rotor_3_Support'])
# fli_boom.plot()
fli_components = [fli_disk, fli_blade_1, fli_blade_2, fli_hub]

# Rotor: front right inner
fri_disk = geometry.declare_component(component_name='fri_disk', b_spline_search_names=['Rotor_5_disk'])
# fri_disk.plot()
fri_blade_1 = geometry.declare_component(component_name='fri_blade_1', b_spline_search_names=['Rotor_5_blades, 0'])
# fri_blade_1.plot()
fri_blade_2 = geometry.declare_component(component_name='fri_blade_2', b_spline_search_names=['Rotor_5_blades, 1'])
# fri_blade_2.plot()
fri_hub = geometry.declare_component(component_name='fri_hub', b_spline_search_names=['Rotor_5_Hub'])
# fri_hub.plot()
fri_boom = geometry.declare_component(component_name='fri_boom', b_spline_search_names=['Rotor_5_Support'])
# fri_boom.plot()
fri_components = [fri_disk, fri_blade_1, fri_blade_2, fri_hub]

# Rotor: front right outer
fro_disk = geometry.declare_component(component_name='fro_disk', b_spline_search_names=['Rotor_7_disk'])
# fro_disk.plot()
fro_blade_1 = geometry.declare_component(component_name='fro_blade_1', b_spline_search_names=['Rotor_7_blades, 0'])
# fro_blade_1.plot()
fro_blade_2 = geometry.declare_component(component_name='fro_blade_2', b_spline_search_names=['Rotor_7_blades, 1'])
# fro_blade_2.plot()
fro_hub = geometry.declare_component(component_name='fro_hub', b_spline_search_names=['Rotor_7_Hub'])
# fro_hub.plot()
fro_boom = geometry.declare_component(component_name='fro_boom', b_spline_search_names=['Rotor_7_Support'])
# fro_boom.plot()
fro_components = [fro_disk, fro_blade_1, fro_blade_2, fro_hub]
lift_rotor_related_components = [rlo_components, rli_components, rri_components, rro_components, 
                                 flo_components, fli_components, fri_components, fro_components]
# endregion

# region Defining key points
wing_te_right = wing.project(np.array([13.4, 25.250, 7.5]), plot=False)
wing_te_left = wing.project(np.array([13.4, -25.250, 7.5]), plot=False)
wing_te_center = wing.project(np.array([14.332, 0., 8.439]), plot=False)
wing_le_left = wing.project(np.array([12.356, -25.25, 7.618]), plot=False)
wing_le_right = wing.project(np.array([12.356, 25.25, 7.618]), plot=False)
wing_le_center = wing.project(np.array([8.892, 0., 8.633]), plot=False)
wing_qc = wing.project(np.array([10.25, 0., 8.5]), plot=False)

tail_te_right = h_tail.project(np.array([31.5, 6.75, 6.]), plot=False)
tail_te_left = h_tail.project(np.array([31.5, -6.75, 6.]), plot=False)
tail_le_right = h_tail.project(np.array([26.5, 6.75, 6.]), plot=False)
tail_le_left = h_tail.project(np.array([26.5, -6.75, 6.]), plot=False)
tail_te_center = h_tail.project(np.array([31.187, 0., 8.009]), plot=False)
tail_le_center = h_tail.project(np.array([27.428, 0., 8.009]), plot=False)
tail_qc = h_tail.project(np.array([24.15, 0., 8.]), plot=False)

fuselage_wing_qc = fuselage.project(np.array([10.25, 0., 8.5]), plot=False)
fuselage_wing_te_center = fuselage.project(np.array([14.332, 0., 8.439]), plot=False)
fuselage_tail_qc = fuselage.project(np.array([24.15, 0., 8.]), plot=False)
fuselage_tail_te_center = fuselage.project(np.array([31.187, 0., 8.009]), plot=False)

rlo_disk_pt = np.array([19.200, -18.750, 9.635])
rro_disk_pt = np.array([19.200, 18.750, 9.635])
rlo_boom_pt = np.array([12.000, -18.750, 7.613])
rro_boom_pt = np.array([12.000, 18.750, 7.613])

flo_disk_pt = np.array([5.070, -18.750, 7.355])
fro_disk_pt = np.array([5.070, 18.750, 7.355])
flo_boom_pt = np.array([12.200, -18.750, 7.615])
fro_boom_pt = np.array([12.200, 18.750, 7.615])

rli_disk_pt = np.array([18.760, -8.537, 9.919])
rri_disk_pt = np.array([18.760, 8.537, 9.919])
rli_boom_pt = np.array([11.500, -8.250, 7.898])
rri_boom_pt = np.array([11.500, 8.250, 7.898])

fli_disk_pt = np.array([4.630, -8.217, 7.659])
fri_disk_pt = np.array([4.630, 8.217, 7.659])
fli_boom_pt = np.array([11.741, -8.250, 7.900])
fri_boom_pt = np.array([11.741, 8.250, 7.900])

rlo_disk_center = rlo_disk.project(rlo_disk_pt)
rli_disk_center = rli_disk.project(rli_disk_pt)
rri_disk_center = rri_disk.project(rri_disk_pt)
rro_disk_center = rro_disk.project(rro_disk_pt)
flo_disk_center = flo_disk.project(flo_disk_pt)
fli_disk_center = fli_disk.project(fli_disk_pt)
fri_disk_center = fri_disk.project(fri_disk_pt)
fro_disk_center = fro_disk.project(fro_disk_pt)

rlo_disk_center_on_wing = wing.project(rlo_disk_pt)
rli_disk_center_on_wing = wing.project(rli_disk_pt)
rri_disk_center_on_wing = wing.project(rri_disk_pt)
rro_disk_center_on_wing = wing.project(rro_disk_pt)
flo_disk_center_on_wing = wing.project(flo_disk_pt)
fli_disk_center_on_wing = wing.project(fli_disk_pt)
fri_disk_center_on_wing = wing.project(fri_disk_pt)
fro_disk_center_on_wing = wing.project(fro_disk_pt)
# endregion

# region Projection for meshes
num_spanwise_vlm = 17
num_chordwise_vlm = 5
leading_edge_line_parametric = wing.project(np.linspace(np.array([8.356, -26., 7.618]), np.array([8.356, 26., 7.618]), num_spanwise_vlm), 
                                 direction=np.array([0., 0., -1.]))
trailing_edge_line_parametric = wing.project(np.linspace(np.array([15.4, -25.250, 7.5]), np.array([15.4, 25.250, 7.5]), num_spanwise_vlm), 
                                  direction=np.array([0., 0., -1.]))
leading_edge_line = geometry.evaluate(leading_edge_line_parametric)
trailing_edge_line = geometry.evaluate(trailing_edge_line_parametric)
chord_surface = m3l.linspace(leading_edge_line, trailing_edge_line, num_chordwise_vlm)
upper_surface_wireframe_parametric = wing.project(chord_surface.value.reshape((num_chordwise_vlm,num_spanwise_vlm,3))+np.array([0., 0., 1.]), 
                                       direction=np.array([0., 0., -1.]), plot=False)
lower_surface_wireframe_parametric = wing.project(chord_surface.value.reshape((num_chordwise_vlm,num_spanwise_vlm,3))+np.array([0., 0., -1.]), 
                                       direction=np.array([0., 0., 1.]), plot=False)
upper_surface_wireframe = geometry.evaluate(upper_surface_wireframe_parametric)
lower_surface_wireframe = geometry.evaluate(lower_surface_wireframe_parametric)
camber_surface = m3l.linspace(upper_surface_wireframe, lower_surface_wireframe, 1).reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
# geometry.plot_meshes([camber_surface])
# endregion

# region Parameterization
from lsdo_geo.core.parameterization.free_form_deformation_functions import construct_ffd_block_around_entities
from lsdo_geo.core.parameterization.volume_sectional_parameterization import VolumeSectionalParameterization
from lsdo_geo.core.parameterization.parameterization_solver import ParameterizationSolver
import lsdo_geo.splines.b_splines as bsp

constant_b_spline_curve_1_dof_space = bsp.BSplineSpace(name='constant_b_spline_curve_1_dof_space', order=1, parametric_coefficients_shape=(1,))
linear_b_spline_curve_2_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_2_dof_space', order=2, parametric_coefficients_shape=(2,))
linear_b_spline_curve_3_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_3_dof_space', order=3, parametric_coefficients_shape=(3,))
cubic_b_spline_curve_5_dof_space = bsp.BSplineSpace(name='cubic_b_spline_curve_5_dof_space', order=4, parametric_coefficients_shape=(5,))

# region Parameterization Setup
parameterization_solver = ParameterizationSolver()

# region Wing Parameterization setup
wing_ffd_block = construct_ffd_block_around_entities('wing_ffd_block', entities=wing, num_coefficients=(2,11,2), order=(2,4,2))
wing_ffd_block.coefficients.name = 'wing_ffd_block_coefficients'
wing_ffd_block_sectional_parameterization = VolumeSectionalParameterization(name='wing_sectional_parameterization',
                                                                            parameterized_points=wing_ffd_block.coefficients,
                                                                            parameterized_points_shape=wing_ffd_block.coefficients_shape,
                                                                            principal_parametric_dimension=1)
wing_ffd_block_sectional_parameterization.add_sectional_stretch(name='sectional_wing_chord_stretch', axis=0)
wing_ffd_block_sectional_parameterization.add_sectional_translation(name='sectional_wingspan_stretch', axis=1)
# wing_ffd_block_sectional_parameterization.add_sectional_rotation(name='sectional_wing_twist', axis=1)

wing_chord_stretch_coefficients = m3l.Variable(name='wing_chord_stretch_coefficients', shape=(3,), value=np.array([0., 0., 0.]))
wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space, 
                                          coefficients=wing_chord_stretch_coefficients, num_physical_dimensions=1)

wing_wingspan_stretch_coefficients = m3l.Variable(name='wing_wingspan_stretch_coefficients', shape=(2,), value=np.array([-0., 0.]))
wing_wingspan_stretch_b_spline = bsp.BSpline(name='wing_wingspan_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                          coefficients=wing_wingspan_stretch_coefficients, num_physical_dimensions=1)

# wing_twist_coefficients = m3l.Variable(name='wing_twist_coefficients', shape=(5,), value=np.array([0., 0., 0., 0., 0.]))
# wing_twist_b_spline = bsp.BSpline(name='wing_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
#                                           coefficients=wing_twist_coefficients, num_physical_dimensions=1)

parameterization_solver.declare_state(name='wing_chord_stretch_coefficients', state=wing_chord_stretch_coefficients)
parameterization_solver.declare_state(name='wing_wingspan_stretch_coefficients', state=wing_wingspan_stretch_coefficients, penalty_factor=1.e3)

# endregion

# region Horizontal Stabilizer setup
h_tail_ffd_block = construct_ffd_block_around_entities('h_tail_ffd_block', entities=h_tail, num_coefficients=(2,11,2), order=(2,4,2))
h_tail_ffd_block_sectional_parameterization = VolumeSectionalParameterization(name='h_tail_sectional_parameterization',
                                                                            parameterized_points=h_tail_ffd_block.coefficients,
                                                                            parameterized_points_shape=h_tail_ffd_block.coefficients_shape,
                                                                            principal_parametric_dimension=1)
h_tail_ffd_block_sectional_parameterization.add_sectional_stretch(name='sectional_h_tail_chord_stretch', axis=0)
h_tail_ffd_block_sectional_parameterization.add_sectional_translation(name='sectional_h_tail_span_stretch', axis=1)
# h_tail_ffd_block_sectional_parameterization.add_sectional_rotation(name='sectional_h_tail_twist', axis=1)
h_tail_ffd_block_sectional_parameterization.add_sectional_translation(name='sectional_h_tail_translation_x', axis=0)
# Don't need to add translation_y because the span stretch covers that
h_tail_ffd_block_sectional_parameterization.add_sectional_translation(name='sectional_h_tail_translation_z', axis=2)

h_tail_chord_stretch_coefficients = m3l.Variable(name='h_tail_chord_stretch_coefficients', shape=(3,), value=np.array([0., 0., 0.]))
h_tail_chord_stretch_b_spline = bsp.BSpline(name='h_tail_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space, 
                                          coefficients=h_tail_chord_stretch_coefficients, num_physical_dimensions=1)

h_tail_span_stretch_coefficients = m3l.Variable(name='h_tail_span_stretch_coefficients', shape=(2,), value=np.array([-0., 0.]))
h_tail_span_stretch_b_spline = bsp.BSpline(name='h_tail_span_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                          coefficients=h_tail_span_stretch_coefficients, num_physical_dimensions=1)

# h_tail_twist_coefficients = m3l.Variable(name='h_tail_twist_coefficients', shape=(5,), value=np.array([0., 0., 0., 0., 0.]))
# h_tail_twist_b_spline = bsp.BSpline(name='h_tail_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
#                                           coefficients=h_tail_twist_coefficients, num_physical_dimensions=1)

h_tail_translation_x_coefficients = m3l.Variable(name='h_tail_translation_x_coefficients', shape=(1,), value=np.array([0.]))
h_tail_translation_x_b_spline = bsp.BSpline(name='h_tail_translation_x_b_spline', space=constant_b_spline_curve_1_dof_space,
                                          coefficients=h_tail_translation_x_coefficients, num_physical_dimensions=1)
h_tail_translation_z_coefficients = m3l.Variable(name='h_tail_translation_z_coefficients', shape=(1,), value=np.array([0.]))
h_tail_translation_z_b_spline = bsp.BSpline(name='h_tail_translation_z_b_spline', space=constant_b_spline_curve_1_dof_space,
                                          coefficients=h_tail_translation_z_coefficients, num_physical_dimensions=1)

parameterization_solver.declare_state(name='h_tail_chord_stretch_coefficients', state=h_tail_chord_stretch_coefficients)
parameterization_solver.declare_state(name='h_tail_span_stretch_coefficients', state=h_tail_span_stretch_coefficients)
parameterization_solver.declare_state(name='h_tail_translation_x_coefficients', state=h_tail_translation_x_coefficients)
parameterization_solver.declare_state(name='h_tail_translation_z_coefficients', state=h_tail_translation_z_coefficients)
# endregion

# # # region Vertical Stabilizer setup
# # v_tail_ffd_block = construct_ffd_block_around_entities('v_tail_ffd_block', entities=v_tail, num_coefficients=(2,2,2), order=(2,2,2))
# # v_tail_ffd_block_sectional_parameterization = VolumeSectionalParameterization(name='v_tail_sectional_parameterization',
# #                                                                             parameterized_points=v_tail_ffd_block.coefficients,
# #                                                                             parameterized_points_shape=v_tail_ffd_block.coefficients_shape,
# #                                                                             principal_parametric_dimension=0)
# # v_tail_ffd_block_sectional_parameterization.add_sectional_translation(name='v_tail_stretch', axis=0)

# # v_tail_stretch_coefficients = m3l.Variable(name='v_tail_stretch_coefficients', shape=(2,), value=np.array([0., 0.]))
# # v_tail_stretch_b_spline = bsp.BSpline(name='v_tail_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
# #                                           coefficients=v_tail_stretch_coefficients, num_physical_dimensions=1)
# # # endregion

# region Fuselage setup
fuselage_ffd_block = construct_ffd_block_around_entities('fuselage_ffd_block', entities=[fuselage, nose_hub], num_coefficients=(2,2,2), order=(2,2,2))
fuselage_ffd_block.coefficients.name = 'fuselage_ffd_block_coefficients'
fuselage_ffd_block_sectional_parameterization = VolumeSectionalParameterization(name='fuselage_sectional_parameterization',
                                                                            parameterized_points=fuselage_ffd_block.coefficients,
                                                                            parameterized_points_shape=fuselage_ffd_block.coefficients_shape,
                                                                            principal_parametric_dimension=0)
fuselage_ffd_block_sectional_parameterization.add_sectional_translation(name='sectional_fuselage_stretch', axis=0)

fuselage_stretch_coefficients = m3l.Variable(name='fuselage_stretch_coefficients', shape=(2,), value=np.array([0., -0.]))
fuselage_stretch_b_spline = bsp.BSpline(name='fuselage_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                          coefficients=fuselage_stretch_coefficients, num_physical_dimensions=1)

parameterization_solver.declare_state(name='fuselage_stretch_coefficients', state=fuselage_stretch_coefficients)
# endregion

# region Lift Rotors setup
def add_rigid_body_translation(components_name, components):
    components_ffd_block = construct_ffd_block_around_entities(f'{components_name}_ffd_block', entities=components, num_coefficients=(2,2,2),
                                                               order=(2,2,2))
    components_ffd_block.coefficients.name = components_name + '_coefficients'
    components_ffd_block_sectional_parameterization = VolumeSectionalParameterization(name=f'{components_name}_sectional_parameterization',
                                                                                parameterized_points=components_ffd_block.coefficients,
                                                                                parameterized_points_shape=components_ffd_block.coefficients_shape,
                                                                                principal_parametric_dimension=0)
    components_ffd_block_sectional_parameterization.add_sectional_translation(name=f'{components_name}_translation_x', axis=0)
    components_ffd_block_sectional_parameterization.add_sectional_translation(name=f'{components_name}_translation_y', axis=1)
    components_ffd_block_sectional_parameterization.add_sectional_translation(name=f'{components_name}_translation_z', axis=2)

    components_translation_x_coefficients = m3l.Variable(name=f'{components_name}_translation_x_coefficients', shape=(1,), value=np.array([0.]))
    components_translation_x_b_spline = bsp.BSpline(name=f'{components_name}_translation_x_b_spline', space=constant_b_spline_curve_1_dof_space, 
                                            coefficients=components_translation_x_coefficients, num_physical_dimensions=1)
    components_translation_y_coefficients = m3l.Variable(name=f'{components_name}_translation_y_coefficients', shape=(1,), value=np.array([0.]))
    components_translation_y_b_spline = bsp.BSpline(name=f'{components_name}_translation_y_b_spline', space=constant_b_spline_curve_1_dof_space, 
                                            coefficients=components_translation_y_coefficients, num_physical_dimensions=1)
    components_translation_z_coefficients = m3l.Variable(name=f'{components_name}_translation_z_coefficients', shape=(1,), value=np.array([0.]))
    components_translation_z_b_spline = bsp.BSpline(name=f'{components_name}_translation_z_b_spline', space=constant_b_spline_curve_1_dof_space, 
                                            coefficients=components_translation_z_coefficients, num_physical_dimensions=1)

    return [components_translation_x_b_spline, components_translation_y_b_spline, components_translation_z_b_spline], \
        components_ffd_block_sectional_parameterization, components_ffd_block

lift_rotor_parameterization_objects = {}
for component_set in lift_rotor_related_components:
    components_name = component_set[0].name[:3] + '_lift_rotor_components'
    # for component in component_set:
    #     component_parameterization_b_splines, component_sectional_parameterization, component_ffd_block = add_rigid_body_translation(component)
    #     lift_rotor_parameterization_objects[f'{component.name}_parameterization_b_splines'] = component_parameterization_b_splines
    #     lift_rotor_parameterization_objects[f'{component.name}_sectional_parameterization'] = component_sectional_parameterization
    #     lift_rotor_parameterization_objects[f'{component.name}_ffd_block'] = component_ffd_block
    component_parameterization_b_splines, component_sectional_parameterization, component_ffd_block = add_rigid_body_translation(
                                                                                                                components_name, component_set)
    lift_rotor_parameterization_objects[f'{components_name}_parameterization_b_splines'] = component_parameterization_b_splines
    lift_rotor_parameterization_objects[f'{components_name}_sectional_parameterization'] = component_sectional_parameterization
    lift_rotor_parameterization_objects[f'{components_name}_ffd_block'] = component_ffd_block

    parameterization_solver.declare_state(name=f'{components_name}_translation_x_coefficients', state=component_parameterization_b_splines[0].coefficients)
    parameterization_solver.declare_state(name=f'{components_name}_translation_y_coefficients', state=component_parameterization_b_splines[1].coefficients)
    parameterization_solver.declare_state(name=f'{components_name}_translation_z_coefficients', state=component_parameterization_b_splines[2].coefficients)

# endregion

# endregion

# region Parameterization Solver Setup Evaluations

# region Wing Parameterization Evaluation for Parameterization Solver
section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_wing_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_wing_wingspan_stretch = wing_wingspan_stretch_b_spline.evaluate(section_parametric_coordinates)
# sectional_wing_twist = wing_twist_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = {
    'sectional_wing_chord_stretch':sectional_wing_chord_stretch,
    'sectional_wingspan_stretch':sectional_wing_wingspan_stretch,
    # 'sectional_wing_twist':sectional_wing_twist,
                        }

wing_ffd_block_coefficients = wing_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=False)
geometry.assign_coefficients(coefficients=wing_coefficients, b_spline_names=wing.b_spline_names)
# geometry.plot()
# endregion

# region Horizontal Stabilizer Parameterization Evaluation for Parameterization Solver
section_parametric_coordinates = np.linspace(0., 1., h_tail_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_h_tail_chord_stretch = h_tail_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_span_stretch = h_tail_span_stretch_b_spline.evaluate(section_parametric_coordinates)
# sectional_h_tail_twist = h_tail_twist_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_translation_x = h_tail_translation_x_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_translation_z = h_tail_translation_z_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = {
    'sectional_h_tail_chord_stretch':sectional_h_tail_chord_stretch,
    'sectional_h_tail_span_stretch':sectional_h_tail_span_stretch,
    # 'sectional_h_tail_twist':sectional_h_tail_twist,
    'sectional_h_tail_translation_x':sectional_h_tail_translation_x,
    'sectional_h_tail_translation_z':sectional_h_tail_translation_z
                        }

h_tail_ffd_block_coefficients = h_tail_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
h_tail_coefficients = h_tail_ffd_block.evaluate(h_tail_ffd_block_coefficients, plot=False)
geometry.assign_coefficients(coefficients=h_tail_coefficients, b_spline_names=h_tail.b_spline_names)
# geometry.plot()
# endregion

# region Fuselage Parameterization Evaluation for Parameterization Solver
section_parametric_coordinates = np.linspace(0., 1., fuselage_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_fuselage_stretch = fuselage_stretch_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = {'sectional_fuselage_stretch':sectional_fuselage_stretch}

fuselage_ffd_block_coefficients = fuselage_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
fuselage_and_nose_hub_coefficients = fuselage_ffd_block.evaluate(fuselage_ffd_block_coefficients, plot=False)
fuselage_coefficients = fuselage_and_nose_hub_coefficients['fuselage_coefficients']
nose_hub_coefficients = fuselage_and_nose_hub_coefficients['weird_nose_hub_coefficients']
geometry.assign_coefficients(coefficients=fuselage_coefficients, b_spline_names=fuselage.b_spline_names)
geometry.assign_coefficients(coefficients=nose_hub_coefficients, b_spline_names=nose_hub.b_spline_names)
# geometry.plot()
# endregion

# region Lift Rotor Evaluation for Parameterization Solver
for component_set in lift_rotor_related_components:
    components_name = component_set[0].name[:3] + '_lift_rotor_components'
    # for component in component_set:
        # component_parameterization_b_splines = lift_rotor_parameterization_objects[f'{component.name}_parameterization_b_splines']
        # component_sectional_parameterization = lift_rotor_parameterization_objects[f'{component.name}_sectional_parameterization']
        # component_ffd_block = lift_rotor_parameterization_objects[f'{component.name}_ffd_block']

        # section_parametric_coordinates = np.linspace(0., 1., component_sectional_parameterization.num_sections).reshape((-1,1))
        # sectional_translation_x = component_parameterization_b_splines[0].evaluate(section_parametric_coordinates)
        # sectional_translation_y = component_parameterization_b_splines[1].evaluate(section_parametric_coordinates)
        # sectional_translation_z = component_parameterization_b_splines[2].evaluate(section_parametric_coordinates)

        # sectional_parameters = {
        #     f'{component.name}_translation_x':sectional_translation_x,
        #     f'{component.name}_translation_y':sectional_translation_y,
        #     f'{component.name}_translation_z':sectional_translation_z,
        #                         }
        
        # component_ffd_block_coefficients = component_sectional_parameterization.evaluate(sectional_parameters, plot=True)
        # component_coefficients = component_ffd_block.evaluate(component_ffd_block_coefficients, plot=True)
        # geometry.assign_coefficients(coefficients=component_coefficients, b_spline_names=component.b_spline_names)
    component_parameterization_b_splines = lift_rotor_parameterization_objects[f'{components_name}_parameterization_b_splines']
    component_sectional_parameterization = lift_rotor_parameterization_objects[f'{components_name}_sectional_parameterization']
    component_ffd_block = lift_rotor_parameterization_objects[f'{components_name}_ffd_block']

    section_parametric_coordinates = np.linspace(0., 1., component_sectional_parameterization.num_sections).reshape((-1,1))
    sectional_translation_x = component_parameterization_b_splines[0].evaluate(section_parametric_coordinates)
    sectional_translation_y = component_parameterization_b_splines[1].evaluate(section_parametric_coordinates)
    sectional_translation_z = component_parameterization_b_splines[2].evaluate(section_parametric_coordinates)

    sectional_parameters = {
        f'{components_name}_translation_x':sectional_translation_x,
        f'{components_name}_translation_y':sectional_translation_y,
        f'{components_name}_translation_z':sectional_translation_z,
                            }
    
    component_ffd_block_coefficients = component_sectional_parameterization.evaluate(sectional_parameters, plot=False)
    component_coefficients = component_ffd_block.evaluate(component_ffd_block_coefficients, plot=False)
    disk = component_set[0]
    blade_1 = component_set[1]
    blade_2 = component_set[2]
    hub = component_set[3]
    # component_names = disk.b_spline_names + blade_1.b_spline_names + blade_2.b_spline_names + hub.b_spline_names
    # component_names = [disk.b_spline_names, blade_1.b_spline_names,  blade_2.b_spline_names,  hub.b_spline_names]
    # component_coefficients_list = list(component_coefficients.values())
    # component_coefficients = [component_coefficients_list[0], component_coefficients_list[1], component_coefficients_list[2], 
    #                           component_coefficients_list[3]]
    # component_coefficients_list = []
    # for i in range(len(component_coefficients)):
    #     component_coefficients_list.extend(list(component_coefficients.values())[i])    
    # geometry.assign_coefficients(coefficients=component_coefficients_list, b_spline_names=component_names)
    geometry.assign_coefficients(coefficients=component_coefficients[disk.name+'_coefficients'], b_spline_names=disk.b_spline_names)
    geometry.assign_coefficients(coefficients=component_coefficients[blade_1.name+'_coefficients'], b_spline_names=blade_1.b_spline_names)
    geometry.assign_coefficients(coefficients=component_coefficients[blade_2.name+'_coefficients'], b_spline_names=blade_2.b_spline_names)
    geometry.assign_coefficients(coefficients=component_coefficients[hub.name+'_coefficients'], b_spline_names=hub.b_spline_names)

# endregion

# endregion

# region Defining/Declaring Parameterization Solver Inputs
parameterization_inputs = {}

# region wing design parameterization inputs
wingspan = m3l.norm(geometry.evaluate(wing_le_right) - geometry.evaluate(wing_le_left))
root_chord = m3l.norm(geometry.evaluate(wing_te_center) - geometry.evaluate(wing_le_center))
tip_chord_left = m3l.norm(geometry.evaluate(wing_te_left) - geometry.evaluate(wing_le_left))
tip_chord_right = m3l.norm(geometry.evaluate(wing_te_right) - geometry.evaluate(wing_le_right))

parameterization_solver.declare_input(name='wingspan', input=wingspan)
parameterization_solver.declare_input(name='root_chord', input=root_chord)
parameterization_solver.declare_input(name='tip_chord_left', input=tip_chord_left)
parameterization_solver.declare_input(name='tip_chord_right', input=tip_chord_right)

parameterization_inputs['wingspan'] = m3l.Variable(name='wingspan', shape=(1,), value=np.array([60.]), dv_flag=True)
parameterization_inputs['root_chord'] = m3l.Variable(name='root_chord', shape=(1,), value=np.array([5.]), dv_flag=True)
parameterization_inputs['tip_chord_left'] = m3l.Variable(name='tip_chord_left', shape=(1,), value=np.array([1.]))
parameterization_inputs['tip_chord_right'] = m3l.Variable(name='tip_chord_right', shape=(1,), value=np.array([1.]))
# endregion

# region h_tail design parameterization inputs
h_tail_span = m3l.norm(geometry.evaluate(tail_le_right) - geometry.evaluate(tail_le_left))
h_tail_root_chord = m3l.norm(geometry.evaluate(tail_te_center) - geometry.evaluate(tail_le_center))
h_tail_tip_chord_left = m3l.norm(geometry.evaluate(tail_te_left) - geometry.evaluate(tail_le_left))
h_tail_tip_chord_right = m3l.norm(geometry.evaluate(tail_te_right) - geometry.evaluate(tail_le_right))

parameterization_solver.declare_input(name='h_tail_span', input=h_tail_span)
parameterization_solver.declare_input(name='h_tail_root_chord', input=h_tail_root_chord)
parameterization_solver.declare_input(name='h_tail_tip_chord_left', input=h_tail_tip_chord_left)
parameterization_solver.declare_input(name='h_tail_tip_chord_right', input=h_tail_tip_chord_right)

parameterization_inputs['h_tail_span'] = m3l.Variable(name='h_tail_span', shape=(1,), value=np.array([10.]), dv_flag=True)
parameterization_inputs['h_tail_root_chord'] = m3l.Variable(name='h_tail_root_chord', shape=(1,), value=np.array([5.]), dv_flag=True)
parameterization_inputs['h_tail_tip_chord_left'] = m3l.Variable(name='h_tail_tip_chord_left', shape=(1,), value=np.array([2.]))
parameterization_inputs['h_tail_tip_chord_right'] = m3l.Variable(name='h_tail_tip_chord_right', shape=(1,), value=np.array([2.]))
# endregion

# region tail moment arm inputs
tail_moment_arm = m3l.norm(geometry.evaluate(tail_qc) - geometry.evaluate(wing_qc))
# tail_moment_arm = m3l.norm(geometry.evaluate(fuselage_tail_te_center) - geometry.evaluate(fuselage_wing_te_center))

wing_fuselage_connection = geometry.evaluate(wing_te_center) - geometry.evaluate(fuselage_wing_te_center)
h_tail_fuselage_connection = geometry.evaluate(tail_te_center) - geometry.evaluate(fuselage_tail_te_center)

parameterization_solver.declare_input(name='tail_moment_arm', input=tail_moment_arm)
# parameterization_solver.declare_input(name='wing_to_fuselage_connection', input=wing_fuselage_connection)
parameterization_solver.declare_input(name='h_tail_to_fuselage_connection', input=h_tail_fuselage_connection)

parameterization_inputs['tail_moment_arm'] = m3l.Variable(name='tail_moment_arm', shape=(1,), value=np.array([20.]), dv_flag=True)
# parameterization_inputs['wing_to_fuselage_connection'] = m3l.Variable(name='wing_to_fuselage_connection', shape=(3,), value=wing_fuselage_connection.value)
parameterization_inputs['h_tail_to_fuselage_connection'] = m3l.Variable(name='h_tail_to_fuselage_connection', shape=(3,), value=h_tail_fuselage_connection.value)
# endregion

# region lift rotors inputs
# for component_set in lift_rotor_related_components:
#     connection = geometry.evaluate() - geometry.evaluate(wing_le_right)

#     parameterization_solver.declare_input(name='wingspan', input=wingspan)
#     parameterization_solver.declare_input(name='root_chord', input=root_chord)
#     parameterization_solver.declare_input(name='tip_chord_left', input=tip_chord_left)
#     parameterization_solver.declare_input(name='tip_chord_right', input=tip_chord_right)

#     parameterization_inputs['wingspan'] = m3l.Variable(name='wingspan', shape=(1,), value=np.array([100.]))
#     parameterization_inputs['root_chord'] = m3l.Variable(name='root_chord', shape=(1,), value=np.array([20.]))
#     parameterization_inputs['tip_chord_left'] = m3l.Variable(name='tip_chord_left', shape=(1,), value=np.array([5.]))
#     parameterization_inputs['tip_chord_right'] = m3l.Variable(name='tip_chord_right', shape=(1,), value=np.array([5.]))  

rlo_connection = geometry.evaluate(rlo_disk_center) - geometry.evaluate(rlo_disk_center_on_wing)
rli_connection = geometry.evaluate(rli_disk_center) - geometry.evaluate(rli_disk_center_on_wing)
rri_connection = geometry.evaluate(rri_disk_center) - geometry.evaluate(rri_disk_center_on_wing)
rro_connection = geometry.evaluate(rro_disk_center) - geometry.evaluate(rro_disk_center_on_wing)
flo_connection = geometry.evaluate(flo_disk_center) - geometry.evaluate(flo_disk_center_on_wing)
fli_connection = geometry.evaluate(fli_disk_center) - geometry.evaluate(fli_disk_center_on_wing)
fri_connection = geometry.evaluate(fri_disk_center) - geometry.evaluate(fri_disk_center_on_wing)
fro_connection = geometry.evaluate(fro_disk_center) - geometry.evaluate(fro_disk_center_on_wing)

parameterization_solver.declare_input(name='rlo_lift_rotor_connection', input=rlo_connection)
parameterization_solver.declare_input(name='rli_lift_rotor_connection', input=rli_connection)
parameterization_solver.declare_input(name='rri_lift_rotor_connection', input=rri_connection)
parameterization_solver.declare_input(name='rro_lift_rotor_connection', input=rro_connection)
parameterization_solver.declare_input(name='flo_lift_rotor_connection', input=flo_connection)
parameterization_solver.declare_input(name='fli_lift_rotor_connection', input=fli_connection)
parameterization_solver.declare_input(name='fri_lift_rotor_connection', input=fri_connection)
parameterization_solver.declare_input(name='fro_lift_rotor_connection', input=fro_connection)

parameterization_inputs['rlo_lift_rotor_connection'] = m3l.Variable(name='rlo_lift_rotor_connection', shape=(3,), value=rlo_connection.value)
parameterization_inputs['rli_lift_rotor_connection'] = m3l.Variable(name='rli_lift_rotor_connection', shape=(3,), value=rli_connection.value)
parameterization_inputs['rri_lift_rotor_connection'] = m3l.Variable(name='rri_lift_rotor_connection', shape=(3,), value=rri_connection.value)
parameterization_inputs['rro_lift_rotor_connection'] = m3l.Variable(name='rro_lift_rotor_connection', shape=(3,), value=rro_connection.value)
parameterization_inputs['flo_lift_rotor_connection'] = m3l.Variable(name='flo_lift_rotor_connection', shape=(3,), value=flo_connection.value)
parameterization_inputs['fli_lift_rotor_connection'] = m3l.Variable(name='fli_lift_rotor_connection', shape=(3,), value=fli_connection.value)
parameterization_inputs['fri_lift_rotor_connection'] = m3l.Variable(name='fri_lift_rotor_connection', shape=(3,), value=fri_connection.value)
parameterization_inputs['fro_lift_rotor_connection'] = m3l.Variable(name='fro_lift_rotor_connection', shape=(3,), value=fro_connection.value)
# endregion


# endregion

# region Parameterization Evaluation

parameterization_solver_states = parameterization_solver.evaluate(parameterization_inputs)

# region Wing Parameterization Evaluation for Parameterization Solver
wing_chord_stretch_coefficients = parameterization_solver_states['wing_chord_stretch_coefficients']
wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space, 
                                          coefficients=wing_chord_stretch_coefficients, num_physical_dimensions=1)

wing_wingspan_stretch_coefficients = parameterization_solver_states['wing_wingspan_stretch_coefficients']
wing_wingspan_stretch_b_spline = bsp.BSpline(name='wing_wingspan_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                          coefficients=wing_wingspan_stretch_coefficients, num_physical_dimensions=1)

wing_twist_coefficients = m3l.Variable(name='wing_twist_coefficients', shape=(5,), value=np.array([0., 0., 0., 0., 0.]))
wing_twist_b_spline = bsp.BSpline(name='wing_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
                                          coefficients=wing_twist_coefficients, num_physical_dimensions=1)

section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_wing_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_wing_wingspan_stretch = wing_wingspan_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_wing_twist = wing_twist_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = {
    'sectional_wing_chord_stretch':sectional_wing_chord_stretch,
    'sectional_wingspan_stretch':sectional_wing_wingspan_stretch,
    'sectional_wing_twist':sectional_wing_twist,
                        }

wing_ffd_block_coefficients = wing_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=False)
geometry.assign_coefficients(coefficients=wing_coefficients, b_spline_names=wing.b_spline_names)
# geometry.plot()
# endregion

# region Horizontal Stabilizer Parameterization Evaluation for Parameterization Solver
h_tail_chord_stretch_b_spline.coefficients = parameterization_solver_states['h_tail_chord_stretch_coefficients']
h_tail_span_stretch_b_spline.coefficients = parameterization_solver_states['h_tail_span_stretch_coefficients']
h_tail_translation_x_b_spline.coefficients = parameterization_solver_states['h_tail_translation_x_coefficients']
h_tail_translation_z_b_spline.coefficients = parameterization_solver_states['h_tail_translation_z_coefficients']

section_parametric_coordinates = np.linspace(0., 1., h_tail_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_h_tail_chord_stretch = h_tail_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_span_stretch = h_tail_span_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_translation_x = h_tail_translation_x_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_translation_z = h_tail_translation_z_b_spline.evaluate(section_parametric_coordinates)

h_tail_twist_coefficients = m3l.Variable(name='h_tail_twist_coefficients', shape=(5,), value=np.array([0., 0., 0., 0., 0.]))
h_tail_twist_b_spline = bsp.BSpline(name='h_tail_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
                                          coefficients=h_tail_twist_coefficients, num_physical_dimensions=1)
sectional_h_tail_twist = h_tail_span_stretch_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = {
    'sectional_h_tail_chord_stretch':sectional_h_tail_chord_stretch,
    'sectional_h_tail_span_stretch':sectional_h_tail_span_stretch,
    'sectional_h_tail_twist':sectional_h_tail_twist,
    'sectional_h_tail_translation_x':sectional_h_tail_translation_x,
    'sectional_h_tail_translation_z':sectional_h_tail_translation_z
                        }

h_tail_ffd_block_coefficients = h_tail_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
h_tail_coefficients = h_tail_ffd_block.evaluate(h_tail_ffd_block_coefficients, plot=False)
geometry.assign_coefficients(coefficients=h_tail_coefficients, b_spline_names=h_tail.b_spline_names)
# geometry.plot()
# endregion

# region Fuselage Parameterization Evaluation for Parameterization Solver
section_parametric_coordinates = np.linspace(0., 1., fuselage_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
fuselage_stretch_b_spline.coefficients = parameterization_solver_states['fuselage_stretch_coefficients']
sectional_fuselage_stretch = fuselage_stretch_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = {'sectional_fuselage_stretch':sectional_fuselage_stretch}

fuselage_ffd_block_coefficients = fuselage_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
fuselage_and_nose_hub_coefficients = fuselage_ffd_block.evaluate(fuselage_ffd_block_coefficients, plot=False)
fuselage_coefficients = fuselage_and_nose_hub_coefficients['fuselage_coefficients']
nose_hub_coefficients = fuselage_and_nose_hub_coefficients['weird_nose_hub_coefficients']
geometry.assign_coefficients(coefficients=fuselage_coefficients, b_spline_names=fuselage.b_spline_names)
geometry.assign_coefficients(coefficients=nose_hub_coefficients, b_spline_names=nose_hub.b_spline_names)
# geometry.plot()
# endregion

# region Lift Rotor Evaluation for Parameterization Solver
for component_set in lift_rotor_related_components:
    components_name = component_set[0].name[:3] + '_lift_rotor_components'

    component_parameterization_b_splines = lift_rotor_parameterization_objects[f'{components_name}_parameterization_b_splines']
    component_parameterization_b_splines[0].coefficients = parameterization_solver_states[f'{components_name}_translation_x_coefficients']
    component_parameterization_b_splines[1].coefficients = parameterization_solver_states[f'{components_name}_translation_y_coefficients']
    component_parameterization_b_splines[2].coefficients = parameterization_solver_states[f'{components_name}_translation_z_coefficients']
    component_sectional_parameterization = lift_rotor_parameterization_objects[f'{components_name}_sectional_parameterization']
    component_ffd_block = lift_rotor_parameterization_objects[f'{components_name}_ffd_block']

    section_parametric_coordinates = np.linspace(0., 1., component_sectional_parameterization.num_sections).reshape((-1,1))
    sectional_translation_x = component_parameterization_b_splines[0].evaluate(section_parametric_coordinates)
    sectional_translation_y = component_parameterization_b_splines[1].evaluate(section_parametric_coordinates)
    sectional_translation_z = component_parameterization_b_splines[2].evaluate(section_parametric_coordinates)

    sectional_parameters = {
        f'{components_name}_translation_x':sectional_translation_x,
        f'{components_name}_translation_y':sectional_translation_y,
        f'{components_name}_translation_z':sectional_translation_z,
                            }
    
    component_ffd_block_coefficients = component_sectional_parameterization.evaluate(sectional_parameters, plot=False)
    component_coefficients = component_ffd_block.evaluate(component_ffd_block_coefficients, plot=False)
    disk = component_set[0]
    blade_1 = component_set[1]
    blade_2 = component_set[2]
    hub = component_set[3]
    # component_names = disk.b_spline_names + blade_1.b_spline_names + blade_2.b_spline_names + hub.b_spline_names
    # component_names = [disk.b_spline_names, blade_1.b_spline_names,  blade_2.b_spline_names,  hub.b_spline_names]
    # component_coefficients_list = list(component_coefficients.values())
    # component_coefficients = [component_coefficients_list[0], component_coefficients_list[1], component_coefficients_list[2], 
    #                           component_coefficients_list[3]]
    # component_coefficients_list = []
    # for i in range(len(component_coefficients)):
    #     component_coefficients_list.extend(list(component_coefficients.values())[i])    
    # geometry.assign_coefficients(coefficients=component_coefficients_list, b_spline_names=component_names)
    geometry.assign_coefficients(coefficients=component_coefficients[disk.name+'_coefficients'], b_spline_names=disk.b_spline_names)
    geometry.assign_coefficients(coefficients=component_coefficients[blade_1.name+'_coefficients'], b_spline_names=blade_1.b_spline_names)
    geometry.assign_coefficients(coefficients=component_coefficients[blade_2.name+'_coefficients'], b_spline_names=blade_2.b_spline_names)
    geometry.assign_coefficients(coefficients=component_coefficients[hub.name+'_coefficients'], b_spline_names=hub.b_spline_names)

geometry.plot()
# endregion

# endregion
# geometry.plot()

# region Mesh Evaluation
upper_surface_wireframe = geometry.evaluate(upper_surface_wireframe_parametric)
lower_surface_wireframe = geometry.evaluate(lower_surface_wireframe_parametric)
vlm_mesh = m3l.linspace(upper_surface_wireframe, lower_surface_wireframe, 1).reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
dummy_objective = m3l.norm(vlm_mesh.reshape((-1,)))
# endregion

# endregion
# m3l_model.register_output(geometry.coefficients)
m3l_model.register_output(vlm_mesh)
m3l_model.register_output(dummy_objective)
# m3l_model.add_objective(dummy_objective)

csdl_model = m3l_model.assemble()
sim = Simulator(csdl_model)
sim.run()
print(geometry.coefficients.operation.name)
print(geometry.coefficients.name)

print(vlm_mesh.operation.name)
print(vlm_mesh.name)
geometry.coefficients = sim['10548_plus_10550_operation.10551']
# camber_surface = sim['10562_reshape_operation_Hryi2.10563']
# geometry.plot_meshes([camber_surface])
# geometry.plot()
sim.check_totals(of=[vlm_mesh.operation.name + '.' + vlm_mesh.name],
                 wrt=['root_chord', 'wingspan', 'tip_chord_left', 'tip_chord_right'])



















exit()
plot_meshes = False
geometry_dv = False
FFD = False

# region Declaring all components
# Wing, tails, fuselage
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['Wing'])
h_tail = geometry.declare_component(component_name='h_tail', b_spline_search_names=['Tail_1'])
vtail = geometry.declare_component(component_name='vtail', b_spline_search_names=['Tail_2'])
fuselage = geometry.declare_component(component_name='fuselage', b_spline_search_names=['Fuselage_***.main'])

# Nose hub
nose_hub = geometry.declare_component(component_name='weird_nose_hub', b_spline_search_names=['EngineGroup_10'])

# Pusher prop
pp_disk = geometry.declare_component(component_name='pp_disk', b_spline_search_names=['Rotor-9-disk'])
pp_blade_1 = geometry.declare_component(component_name='pp_blade_1', b_spline_search_names=['Rotor_9_blades, 0'])
pp_blade_2 = geometry.declare_component(component_name='pp_blade_2', b_spline_search_names=['Rotor_9_blades, 1'])
pp_blade_3 = geometry.declare_component(component_name='pp_blade_3', b_spline_search_names=['Rotor_9_blades, 2'])
pp_blade_4 = geometry.declare_component(component_name='pp_blade_4', b_spline_search_names=['Rotor_9_blades, 3'])
pp_hub = geometry.declare_component(component_name='pp_hub', b_spline_search_names=['Rotor_9_Hub'])

# Rotor: rear left outer
rlo_disk = geometry.declare_component(component_name='rlo_disk', b_spline_search_names=['Rotor_2_disk'])
rlo_blade_1 = geometry.declare_component(component_name='rlo_blade_1', b_spline_search_names=['Rotor_2_blades, 0'])
rlo_blade_2 = geometry.declare_component(component_name='rlo_blade_2', b_spline_search_names=['Rotor_2_blades, 1'])
rlo_hub = geometry.declare_component(component_name='rlo_hub', b_spline_search_names=['Rotor_2_Hub'])
rlo_boom = geometry.declare_component(component_name='rlo_boom', b_spline_search_names=['Rotor_2_Support'])

# Rotor: rear left inner
rli_disk = geometry.declare_component(component_name='rli_disk', b_spline_search_names=['Rotor_4_disk'])
rli_blade_1 = geometry.declare_component(component_name='rli_blade_1', b_spline_search_names=['Rotor_4_blades, 0'])
rli_blade_2 = geometry.declare_component(component_name='rli_blade_2', b_spline_search_names=['Rotor_4_blades, 1'])
rli_hub = geometry.declare_component(component_name='rli_hub', b_spline_search_names=['Rotor_4_Hub'])
rli_boom = geometry.declare_component(component_name='rli_boom', b_spline_search_names=['Rotor_4_Support'])

# Rotor: rear right inner
rri_disk = geometry.declare_component(component_name='rri_disk', b_spline_search_names=['Rotor_6_disk'])
rri_blade_1 = geometry.declare_component(component_name='rri_blade_1', b_spline_search_names=['Rotor_6_blades, 0'])
rri_blade_2 = geometry.declare_component(component_name='rri_blade_2', b_spline_search_names=['Rotor_6_blades, 1'])
rri_hub = geometry.declare_component(component_name='rri_hub', b_spline_search_names=['Rotor_6_Hub'])
rri_boom = geometry.declare_component(component_name='rri_boom', b_spline_search_names=['Rotor_6_Support'])

# Rotor: rear right outer
rro_disk = geometry.declare_component(component_name='rro_disk', b_spline_search_names=['Rotor_8_disk'])
rro_blade_1 = geometry.declare_component(component_name='rro_blade_1', b_spline_search_names=['Rotor_8_blades, 0'])
rro_blade_2 = geometry.declare_component(component_name='rro_blade_2', b_spline_search_names=['Rotor_8_blades, 1'])
rro_hub = geometry.declare_component(component_name='rro_hub', b_spline_search_names=['Rotor_8_Hub'])
rro_boom = geometry.declare_component(component_name='rro_boom', b_spline_search_names=['Rotor_8_Support'])

# Rotor: front left outer
flo_disk = geometry.declare_component(component_name='flo_disk', b_spline_search_names=['Rotor_1_disk'])
flo_blade_1 = geometry.declare_component(component_name='flo_blade_1', b_spline_search_names=['Rotor_1_blades, 0'])
flo_blade_2 = geometry.declare_component(component_name='flo_blade_2', b_spline_search_names=['Rotor_1_blades, 1'])
flo_hub = geometry.declare_component(component_name='flo_hub', b_spline_search_names=['Rotor_1_Hub'])
flo_boom = geometry.declare_component(component_name='flo_boom', b_spline_search_names=['Rotor_1_Support'])

# Rotor: front left inner
fli_disk = geometry.declare_component(component_name='fli_disk', b_spline_search_names=['Rotor_3_disk'])
fli_blade_1 = geometry.declare_component(component_name='fli_blade_1', b_spline_search_names=['Rotor_3_blades, 0'])
fli_blade_2 = geometry.declare_component(component_name='fli_blade_2', b_spline_search_names=['Rotor_3_blades, 1'])
fli_hub = geometry.declare_component(component_name='fli_hub', b_spline_search_names=['Rotor_3_Hub'])
fli_boom = geometry.declare_component(component_name='fli_boom', b_spline_search_names=['Rotor_3_Support'])

# Rotor: front right inner
fri_disk = geometry.declare_component(component_name='fri_disk', b_spline_search_names=['Rotor_5_disk'])
fri_blade_1 = geometry.declare_component(component_name='fri_blade_1', b_spline_search_names=['Rotor_5_blades, 0'])
fri_blade_2 = geometry.declare_component(component_name='fri_blade_2', b_spline_search_names=['Rotor_5_blades, 1'])
fri_hub = geometry.declare_component(component_name='fri_hub', b_spline_search_names=['Rotor_5_Hub'])
fri_boom = geometry.declare_component(component_name='fri_boom', b_spline_search_names=['Rotor_5_Support'])

# Rotor: front right outer
fro_disk = geometry.declare_component(component_name='fro_disk', b_spline_search_names=['Rotor_7_disk'])
fro_blade_1 = geometry.declare_component(component_name='fro_blade_1', b_spline_search_names=['Rotor_7_blades, 0'])
fro_blade_2 = geometry.declare_component(component_name='fro_blade_2', b_spline_search_names=['Rotor_7_blades, 1'])
fro_hub = geometry.declare_component(component_name='fro_hub', b_spline_search_names=['Rotor_7_Hub'])
fro_boom = geometry.declare_component(component_name='fro_boom', b_spline_search_names=['Rotor_7_Support'])
# endregion

# Region geometric desing variables
# Wing
wing_area_input = system_model.create_input('wing_area_input', val=210., dv_flag=geometry_dv, lower=150, upper=250, scaler=8e-3)
wing_aspect_ratio_input = system_model.create_input('wing_aspect_ratio_input', val=12.12, dv_flag=geometry_dv, lower=8, upper=16, scaler=1e-1)
wing_taper_ratio_input = system_model.create_input('wing_taper_ratio_input', val=0.5)

wing_span_input = (wing_aspect_ratio_input * wing_area_input)**0.5
wing_root_chord_input = 2 * wing_area_input/((1 + wing_taper_ratio_input) * wing_span_input)
wing_tip_chord_left_input = wing_root_chord_input * wing_taper_ratio_input / 2.7
wing_tip_chord_right_input = wing_tip_chord_left_input * 1

# Tail
tail_area_input = system_model.create_input('tail_area_input', val=39.5, dv_flag=geometry_dv, lower=25, upper=55, scaler=1e-2)
tail_aspect_ratio_input = system_model.create_input('tail_aspect_ratio_input', val=4.3, dv_flag=geometry_dv, lower=3, upper=8, scaler=1e-1)
tail_taper_ratio_input = system_model.create_input('tail_taper_ratio_input', val=0.6)

tail_span_input = (tail_aspect_ratio_input * tail_area_input)**0.5
tail_root_chord_input = 2 * tail_area_input/((1 + tail_taper_ratio_input) * tail_span_input)
tail_tip_chord_left_input = tail_root_chord_input * tail_taper_ratio_input
tail_tip_chord_right_input = tail_tip_chord_left_input * 1

# Tail moment arm
tail_moment_arm_input = system_model.create_input(name='tail_moment_arm_input', val=17., dv_flag=False, lower=12., upper=22., scaler=1e-1)

# Radii
flo_radius = fro_radius = front_outer_radius = system_model.create_input(name='front_outer_radius', val=10/2, dv_flag=geometry_dv, lower=5/2, upper=15/2, scaler=1e-1)
fli_radius = fri_radius = front_inner_radius = system_model.create_input(name='front_inner_radius', val=10/2, dv_flag=geometry_dv, lower=5/2, upper=15/2, scaler=1e-1)
rlo_radius = rro_radius = rear_outer_radius = system_model.create_input(name='rear_outer_radius', val=10/2, dv_flag=geometry_dv, lower=5/2, upper=15/2, scaler=1e-1)
rli_radius = rri_radius = rear_inner_radius = system_model.create_input(name='rear_inner_radius', val=10/2, dv_flag=geometry_dv, lower=5/2, upper=15/2, scaler=1e-1)
dv_radius_list = [rlo_radius, rli_radius, rri_radius, rro_radius, flo_radius, fli_radius, fri_radius, fro_radius]

pusher_prop_radius = system_model.create_input(name='pusher_prop_radius', val=9/2, dv_flag=geometry_dv, lower=7/2, upper=11/2, scaler=1e-1)



wing_te_right = np.array([13.4, 25.250, 7.5])
wing_te_left = np.array([13.4, -25.250, 7.5])
wing_te_center = np.array([14.332, 0., 8.439])
wing_le_left = np.array([12.356, -25.25, 7.618])
wing_le_right = np.array([12.356, 25.25, 7.618])
wing_le_center = np.array([8.892, 0., 8.633])
wing_qc = np.array([10.25, 0., 8.5])

tail_te_right = np.array([31.5, 6.75, 6.])
tail_te_left = np.array([31.5, -6.75, 6.])
tail_le_right = np.array([26.5, 6.75, 6.])
tail_le_left = np.array([26.5, -6.75, 6.])
tail_te_center = np.array([31.187, 0., 8.009])
tail_le_center = np.array([27.428, 0., 8.009])
tail_qc = np.array([24.15, 0., 8.])


# region Making meshes
# Wing 
num_spanwise_vlm = 25
num_chordwise_vlm = 8


# wing_actuation_angle = system_model.create_input('wing_act_angle', val=0, dv_flag=geometry_dv, lower=-15, upper=15, scaler=1e-1)

wing_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=wing,
    num_spanwise=num_spanwise_vlm,
    num_chordwise=num_chordwise_vlm,
    te_right=wing_te_right,
    te_left=wing_te_left,
    te_center=wing_te_center,
    le_left=wing_le_left,
    le_right=wing_le_right,
    le_center=wing_le_center,
    grid_search_density_parameter=50,
    # actuation_axis=[
    #     0.75 * wing_te_right + 0.25 * wing_le_right,
    #     0.75 * wing_te_left + 0.25 * wing_le_left
    # ],
    # actuation_angle=wing_actuation_angle,
    off_set_x=0.2,
    bunching_cos=True,
    plot=False,
    mirror=True,
)


# tail mesh
num_spanwise_vlm_htail = 8
num_chordwise_vlm_htail = 4

actuation_axis=[
    0.5 * tail_te_right + 0.5 * tail_le_right,
    0.5 * tail_te_left + 0.5 * tail_le_left
]

axis_origin = geometry.evaluate(h_tail.project(actuation_axis[0]))
axis_vector = geometry.evaluate(h_tail.project(actuation_axis[1])) - axis_origin

tail_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=h_tail, 
    num_spanwise=num_spanwise_vlm_htail,
    num_chordwise=num_chordwise_vlm_htail,
    te_right=tail_te_right,
    te_left=tail_te_left,
    le_right=tail_le_right,
    le_left=tail_le_left,
    plot=False,
    mirror=True,
)

# v tail mesh
num_spanwise_vlm_vtail = 8
num_chordwise_vlm_vtail = 6
vtail_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=vtail,
    num_spanwise=num_spanwise_vlm_vtail,
    num_chordwise=num_chordwise_vlm_vtail,
    le_left=np.array([20.843, 0., 8.231,]),
    le_right=np.array([29.434, 0., 13.911,]),
    te_left=np.array([30.543, 0., 8.231]),
    te_right=np.array([32.065, 0., 13.911]),
    plot=False,
    orientation='vertical',
    zero_y=True,
)

# wing beam mesh
num_wing_beam = 21

box_beam_mesh = make_1d_box_beam_mesh(
    geometry=geometry,
    wing_component=wing,
    num_beam_nodes=num_wing_beam,
    te_right=wing_te_right,
    te_left=wing_te_left,
    te_center=wing_te_center,
    le_left=wing_le_left,
    le_right=wing_le_right,
    le_center=wing_le_center,
    beam_width=0.5,
    node_center=0.5,
    plot=False,
)

# Fuselage mesh
num_fuselage_len = 10
num_fuselage_height = 4
nose = np.array([2.464, 0., 5.113])
rear = np.array([31.889, 0., 7.798])
nose_points_parametric = fuselage.project(nose, grid_search_density_parameter=20)
rear_points_parametric = fuselage.project(rear)

nose_points_m3l = geometry.evaluate(nose_points_parametric)
rear_points_m3l = geometry.evaluate(rear_points_parametric)

fuselage_linspace = m3l.linspace(nose_points_m3l, rear_points_m3l, num_fuselage_len)

fueslage_top_points_parametric = fuselage.project(fuselage_linspace.value + np.array([0., 0., 3]), direction=np.array([0.,0.,-1.]), plot=False, grid_search_density_parameter=20)
fueslage_bottom_points_parametric = fuselage.project(fuselage_linspace.value - np.array([0., 0., 3]), direction=np.array([0.,0.,1.]), plot=False, grid_search_density_parameter=20)

fueslage_top_points_m3l = geometry.evaluate(fueslage_top_points_parametric)
fueslage_bottom_points_m3l = geometry.evaluate(fueslage_bottom_points_parametric)

fuesleage_mesh = m3l.linspace(fueslage_top_points_m3l.reshape((-1, 3)), fueslage_bottom_points_m3l.reshape((-1, 3)),  int(num_fuselage_height + 1))
fuesleage_mesh.description = 'zero_y'

# nose2 = np.array([3.439, 1.148, 6.589])
# rear2 = np.array([22.030, 1.148, 6.589])
# nose2_points = geometry.evaluate(fuselage.project(nose2, plot=False, grid_search_density_parameter=20))
# rear2_points = geometry.evaluate(fuselage.project(rear2, plot=False, grid_search_density_parameter=20))
# fuselage_linspace2 = m3l.linspace(nose2_points, rear2_points, num_fuselage_len)
# fuselage_left_points = geometry.evaluate(fuselage.project(fuselage_linspace2.value, direction=np.array([0., 1., 0.]), plot=False)).reshape((-1, 3))
# multiplier = np.ones(fuselage_left_points.shape)
# multiplier[:, 1] *= -1
# fuselage_right_points = fuselage_left_points * multiplier
# # fuselage_right_points = geometry.evaluate(fuselage.project(fuselage_linspace2.value - np.array([0., 5., 0.,]), direction=np.array([0., -1., 0.]), plot=False)).reshape((-1, 3))
# fuselage_mesh_2 = m3l.linspace(fuselage_left_points, fuselage_right_points, num_fuselage_height)# .reshape((num_fuselage_len, num_fuselage_height, 3))
# fuselage_mesh_2.description = 'average_z'

# region Rotor meshes
num_radial = 30
num_spanwise_vlm_rotor = 8
num_chord_vlm_rotor = 2

# Pusher prop
blade_1_params = BladeParameters(
    blade_component=pp_blade_1,
    point_on_leading_edge=np.array([31.649, 2.209, 7.411]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

blade_2_params = BladeParameters(
    blade_component=pp_blade_2,
    point_on_leading_edge=np.array([31.704, 0.421, 10.654]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

blade_3_params = BladeParameters(
    blade_component=pp_blade_3,
    point_on_leading_edge=np.array([31.853, -2.536, 8.270]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

blade_4_params = BladeParameters(
    blade_component=pp_blade_4,
    point_on_leading_edge=np.array([31.672, -0.408, 5.254]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

pp_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=pp_disk,
    origin=np.array([32.625, 0., 7.79]),
    y1=np.array([31.94, 0.00, 3.29]),
    y2=np.array([31.94, 0.00, 12.29]),
    z1=np.array([31.94, -4.50, 7.78]),
    z2=np.array([31.94, 4.45, 7.77]),
    # blade_geometry_parameters=[blade_1_params, blade_2_params, blade_3_params, blade_4_params],
    create_disk_mesh=False,
    plot=False,
    # radius=pusher_prop_radius,
)


# Rear left outer
rlo_blade_1_params = BladeParameters(
    blade_component=rlo_blade_1,
    point_on_leading_edge=np.array([22.018, -19.243, 9.236]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rlo_blade_2_params = BladeParameters(
    blade_component=rlo_blade_2,
    point_on_leading_edge=np.array([16.382, -18.257, 9.236]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rlo_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=rlo_disk,
    origin=np.array([19.2, -18.75, 9.01]),
    y1=np.array([19.2, -13.75, 9.01]),
    y2=np.array([19.2, -23.75, 9.01]),
    z1=np.array([14.2, -18.75, 9.01]),
    z2=np.array([24.2, -18.75, 9.01]),
    # blade_geometry_parameters=[rlo_blade_1_params, rlo_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=rlo_boom,
    # radius=rlo_radius,
)


# Rear right outer 
rro_blade_1_params = BladeParameters(
    blade_component=rro_blade_1,
    point_on_leading_edge=np.array([16.382, 18.257, 9.236]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rro_blade_2_params = BladeParameters(
    blade_component=rro_blade_2,
    point_on_leading_edge=np.array([22.018, 19.195, 9.248]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rro_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=rro_disk,
    origin=np.array([19.2, 18.75, 9.01]),
    y1=np.array([19.2, 23.75, 9.01]),
    y2=np.array([19.2, 13.75, 9.01]),
    z1=np.array([14.2, 18.75, 9.01]),
    z2=np.array([24.2, 18.75, 9.01]),
    # blade_geometry_parameters=[rro_blade_1_params, rro_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=rro_boom,
    # radius=rro_radius,
)


# Front left outer 
flo_blade_1_params = BladeParameters(
    blade_component=flo_blade_1,
    point_on_leading_edge=np.array([7.888, -19.243, 6.956]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

flo_blade_2_params = BladeParameters(
    blade_component=flo_blade_2,
    point_on_leading_edge=np.array([2.252, -18.257, 6.956]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

flo_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=flo_disk,
    origin=np.array([5.07, -18.75, 6.73]),
    y1=np.array([5.070, -13.750, 6.730]),
    y2=np.array([5.070, -23.750, 6.730]),
    z1=np.array([0.070, -18.750, 6.730]),
    z2=np.array([10.070, -18.750, 6.730]),
    # blade_geometry_parameters=[flo_blade_1_params, flo_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=flo_boom,
    # radius=flo_radius,
)


# Front right outer 
fro_blade_1_params = BladeParameters(
    blade_component=fro_blade_1,
    point_on_leading_edge=np.array([2.252, 18.257, 6.956]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

fro_blade_2_params = BladeParameters(
    blade_component=fro_blade_2,
    point_on_leading_edge=np.array([7.888, 19.243, 6.956]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

fro_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=fro_disk,
    origin=np.array([5.07, 18.75, 6.73]),
    y1=np.array([5.070, 23.750, 6.730]),
    y2=np.array([5.070, 13.750, 6.730]),
    z1=np.array([0.070, 18.750, 6.730]),
    z2=np.array([10.070, 18.750, 6.730]),
    # blade_geometry_parameters=[fro_blade_1_params, fro_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=fro_boom,
    # radius=fro_radius,
)

# Rear left inner
rli_blade_1_params = BladeParameters(
    blade_component=rli_blade_1,
    point_on_leading_edge=np.array([15.578, -8.969, 9.437]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rli_blade_2_params = BladeParameters(
    blade_component=rli_blade_2,
    point_on_leading_edge=np.array([21.578, -7.993, 9.593]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rli_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=rli_disk,
    origin=np.array([18.760, -8.537, 9.919]),
    y1=np.array([18.760, -3.499, 9.996]),
    y2=np.array([18.760, -13.401, 8.604]),
    z1=np.array([13.760, -8.450, 9.300]),
    z2=np.array([23.760, -8.450, 9.300]),
    # blade_geometry_parameters=[rli_blade_1_params, rli_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=rli_boom,
    # radius=rli_radius,
)


# Rear right inner
rri_blade_1_params = BladeParameters(
    blade_component=rri_blade_1,
    point_on_leading_edge=np.array([15.578, 8.969, 9.437]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rri_blade_2_params = BladeParameters(
    blade_component=rri_blade_2,
    point_on_leading_edge=np.array([21.942, 7.989, 9.575]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

rri_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=rri_disk,
    origin=np.array([18.760, 8.537, 9.919]),
    y1=np.array([18.760, 13.401, 8.604]),
    y2=np.array([18.760, 3.499, 9.996]),
    z1=np.array([13.760, 8.450, 9.300]),
    z2=np.array([23.760, 8.450, 9.300]),
    # blade_geometry_parameters=[rri_blade_1_params, rri_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=rri_boom,
    # radius=rri_radius,
)

# Front left inner
fli_blade_1_params = BladeParameters(
    blade_component=fli_blade_1,
    point_on_leading_edge=np.array([2.175, -8.634, 7.208]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

fli_blade_2_params = BladeParameters(
    blade_component=fli_blade_2,
    point_on_leading_edge=np.array([7.085, -7.692, 7.341]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

fli_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=fli_disk,
    origin=np.array([4.630, -8.217, 7.659]),
    y1=np.array([4.630, -3.179, 7.736]),
    y2=np.array([4.630, -13.081, 6.344]),
    z1=np.array([-0.370, -8.130, 7.040]),
    z2=np.array([9.630, -8.130, 7.040]),
    # blade_geometry_parameters=[fli_blade_1_params, fli_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=fli_boom,
    # radius=fli_radius,
)

# Front right inner
fri_blade_1_params = BladeParameters(
    blade_component=fri_blade_1,
    point_on_leading_edge=np.array([7.448, 7.673, 7.333]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

fri_blade_2_params = BladeParameters(
    blade_component=fri_blade_2,
    point_on_leading_edge=np.array([1.085, 8.626, 7.155]),
    num_spanwise_vlm=num_spanwise_vlm_rotor,
    num_chordwise_vlm=num_chord_vlm_rotor,
)

fri_mesh = make_rotor_mesh(
    geometry=geometry,
    num_radial=num_radial,
    disk_component=fri_disk,
    origin=np.array([4.630, 8.217, 7.659]), 
    y1=np.array([4.630, 13.081, 6.344]),
    y2=np.array([4.630, 3.179, 7.736]),
    z1=np.array([-0.370, 8.130, 7.040]),
    z2=np.array([9.630, 8.130, 7.040]),
    # blade_geometry_parameters=[fri_blade_1_params, fri_blade_2_params],
    create_disk_mesh=False,
    plot=False,
    # boom_is_thrust_origin=fri_boom,
    # radius=fri_radius,
)

radius_1_list = [flo_mesh.radius, fli_mesh.radius, fri_mesh.radius, fro_mesh.radius,
                   rlo_mesh.radius, rli_mesh.radius, rri_mesh.radius, rro_mesh.radius]

radius_2_list = [flo_mesh._radius_2, fli_mesh._radius_2, fri_mesh._radius_2, fro_mesh._radius_2,
                   rlo_mesh._radius_2, rli_mesh._radius_2, rri_mesh._radius_2, rro_mesh._radius_2]




# endregion
# endregion


# region projections for drag components
# Fuselage 
fuselage_l1_parametric = fuselage.project(np.array([1.889, 0., 4.249]))
fuselage_l2_parametric = fuselage.project(np.array([31.889, 0., 7.798]))

fuselage_d1_parametric = fuselage.project(np.array([10.916, -2.945, 5.736]))
fuselage_d2_parametric = fuselage.project(np.array([10.916, 2.945, 5.736]))

# wing
wing_mid_le_parametric = wing.project(np.array([8.892, 0., 8.633]))
wing_mid_te_parametric = wing.project(np.array([14.332, 0, 8.439]))

wing_le_right_parametric = wing.project(wing_le_right)
wing_le_left__parametric = wing.project(wing_le_left)

# htail
h_tail_mid_le_parametric = h_tail.project(np.array([27.806, -6.520, 8.008]))
h_tail_mid_te_parametric = h_tail.project(np.array([30.050, -6.520, 8.008]))

# vtail
vtail_mid_le_parametric = vtail.project(np.array([26.971, 0.0, 11.038]))
vtail_mid_te_parametric = vtail.project(np.array([31.302, 0.0, 11.038]))

# boom
boom_l1_parametric = rlo_boom.project(np.array([20.000, -18.750, 7.613]))
boom_l2_parametric = rlo_boom.project(np.array([12.000, -18.750, 7.613]))

boom_d1_parametric = rlo_boom.project(np.array([15.600, -19.250, 7.613]))
boom_d2_parametric = rlo_boom.project(np.array([15.600, -18.250, 7.613]))

# lift hubs
hub_l1_parametric = rlo_hub.project(np.array([18.075, -18.750,9.525]))
hub_l2_parametric = rlo_hub.project(np.array([20.325, -18.750,9.525]))

# blade 
blade_tip_parametric = rlo_blade_2.project(np.array([14.200, -18.626, 9.040]))
blade_hub_parametric = rlo_blade_2.project(np.array([18.200, -18.512, 9.197]))
# endregion

if FFD:
    # region FFD
    # Wing
    import time 
    constant_b_spline_curve_1_dof_space =  bsp.BSplineSpace(name='constant_b_spline_curve_1_dof_space', order=1, parametric_coefficients_shape=(1,))
    linear_b_spline_curve_2_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_2_dof_space', order=2, parametric_coefficients_shape=(2,))
    linear_b_spline_curve_3_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_3_dof_space', order=2, parametric_coefficients_shape=(3,))
    cubic_b_spline_curve_5_dof_space = bsp.BSplineSpace(name='cubic_b_spline_curve_5_dof_space', order=4, parametric_coefficients_shape=(5,))
    from lsdo_geo.core.parameterization.parameterization_solver import ParameterizationSolver
    lpc_param_solver = ParameterizationSolver()

    t1 = time.time()
    wing_ffd_block = lg.construct_ffd_block_around_entities(name='wing_ffd_block', entities=wing, num_coefficients=(2, 11, 2), order=(2, 4, 2))
    t2 = time.time()
    print(t2-t1)
    wing_ffd_block_sect_param = VolumeSectionalParameterization(name='wing_ffd_sect_param', principal_parametric_dimension=1, 
                                                                parameterized_points=wing_ffd_block.coefficients,
                                                                parameterized_points_shape=wing_ffd_block.coefficients_shape)
    t3 = time.time()
    print(t3-t2)


    wing_ffd_block_sect_param.add_sectional_translation(name='wing_span_stretch', axis=1)
    wing_ffd_block_sect_param.add_sectional_stretch(name='wing_chord_stretch', axis=0)


    wing_span_strech_coefficients = system_model.create_input('wing_span_stretch_coefficients', shape=(2, ), val=np.array([0., 0.]))
    wing_span_strech_b_spline = bsp.BSpline(name='wing_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=wing_span_strech_coefficients, num_physical_dimensions=1)
        
    wing_chord_stretch_coefficients = system_model.create_input('wing_chord_stretch_coefficients', shape=(3, ), val=np.array([0., 0., 0.]))
    wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_b_spline', space=linear_b_spline_curve_3_dof_space, coefficients=wing_chord_stretch_coefficients, 
                                            num_physical_dimensions=1)


    section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sect_param.num_sections).reshape((-1,1))
    wing_wingspan_stretch = wing_span_strech_b_spline.evaluate(section_parametric_coordinates)
    wing_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'wing_span_stretch': wing_wingspan_stretch,
            'wing_chord_stretch': wing_chord_stretch,
    }


    wing_ffd_block_coefficients = wing_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
    wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=False)

    # h-tail
    htail_ffd_block = lg.construct_ffd_block_around_entities(name='htail_ffd_block', entities=h_tail, num_coefficients=(2, 3, 2))
    htail_ffd_block_sect_param = VolumeSectionalParameterization(name='htail_ffd_sect_param', principal_parametric_dimension=1, 
                                                                parameterized_points=htail_ffd_block.coefficients,
                                                                parameterized_points_shape=htail_ffd_block.coefficients_shape)

    htail_ffd_block_sect_param.add_sectional_translation(name='htail_span_stretch', axis=1)
    htail_ffd_block_sect_param.add_sectional_stretch(name='htail_chord_stretch', axis=0)
    htail_ffd_block_sect_param.add_sectional_translation(name='htail_translation_x', axis=0)

    htail_span_stretch_coefficients = system_model.create_input('htail_span_stretch_coefficients', shape=(2, ), val=np.array([0., 0.]))
    htail_span_strech_b_spline = bsp.BSpline(name='htail_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=htail_span_stretch_coefficients, num_physical_dimensions=1)
        
    htail_chord_stretch_coefficients = system_model.create_input('htail_chord_stretch_coefficients', shape=(3, ), val=np.array([0., 0., 0.]))
    htail_chord_stretch_b_spline = bsp.BSpline(name='htail_chord_b_spline', space=linear_b_spline_curve_3_dof_space, coefficients=htail_chord_stretch_coefficients, 
                                            num_physical_dimensions=1)

    htail_chord_transl_x_coefficients = system_model.create_input('htail_translation_x_coefficients', shape=(1, ), val=np.array([0.]))
    htail_chord_transl_x_b_spline = bsp.BSpline(name='htail_translation_x_b_spline', space=constant_b_spline_curve_1_dof_space, coefficients=htail_chord_transl_x_coefficients, 
                                            num_physical_dimensions=1)


    section_parametric_coordinates = np.linspace(0., 1., htail_ffd_block_sect_param.num_sections).reshape((-1,1))
    htail_span_stretch = htail_span_strech_b_spline.evaluate(section_parametric_coordinates)
    htail_chord_stretch = htail_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
    htail_translation_x = htail_chord_transl_x_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'htail_span_stretch': htail_span_stretch,
            'htail_chord_stretch': htail_chord_stretch,
            'htail_translation_x': htail_translation_x,
    }

    htail_ffd_block_coefficients = htail_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
    htail_coefficients = htail_ffd_block.evaluate(htail_ffd_block_coefficients, plot=False)

    # fuselage
    fuselage_ffd_block = lg.construct_ffd_block_around_entities(name='fuselage_ffd_block', entities=fuselage, num_coefficients=(2, 2, 2))
    fuselage_ffd_block_sect_param = VolumeSectionalParameterization(name='fuselage_ffd_sect_param', principal_parametric_dimension=0,
                                                                    parameterized_points=fuselage_ffd_block.coefficients,
                                                                    parameterized_points_shape=fuselage_ffd_block.coefficients_shape)

    fuselage_ffd_block_sect_param.add_sectional_translation(name='fuselage_stretch', axis=0)
    fuselage_stretch_coeffs = system_model.create_input('fuselage_stretch_coefficients', shape=(2, ), val=np.array([0., 0.]))
    fuselage_stretch_b_spline = bsp.BSpline(name='fuselage_b_spline', space=linear_b_spline_curve_2_dof_space,
                                            coefficients=fuselage_stretch_coeffs, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., fuselage_ffd_block_sect_param.num_sections).reshape((-1, 1))
    fuselage_stretch = fuselage_stretch_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'fuselage_stretch' : fuselage_stretch
    }

    fuselage_ffd_block_coefficients = fuselage_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
    fuselage_coefficients = fuselage_ffd_block.evaluate(fuselage_ffd_block_coefficients, plot=False)


    # v-tail
    vtail_ffd_block = lg.construct_ffd_block_around_entities(name='vtail_ffd_block', entities=vtail, num_coefficients=(2, 2, 2))
    vtail_ffd_block_sect_param = VolumeSectionalParameterization(name='vtail_ffd_sect_param', principal_parametric_dimension=0,
                                                                parameterized_points=vtail_ffd_block.coefficients, parameterized_points_shape=vtail_ffd_block.coefficients_shape)

    vtail_ffd_block_sect_param.add_sectional_translation(name='vtail_translation_x', axis=0)
    vtail_transl_x_coeffs = system_model.create_input(name='vtail_transl_x_coefficients', shape=(1, ), val=np.array([0.]))
    vtail_transl_x_b_spline = bsp.BSpline(name='vtail_b_spline', space=constant_b_spline_curve_1_dof_space, coefficients=vtail_transl_x_coeffs, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., vtail_ffd_block_sect_param.num_sections).reshape((-1, 1))
    vtail_transl_x = vtail_transl_x_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'vtail_translation_x' : vtail_transl_x
    }

    vtail_ffd_block_coefficients = vtail_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
    vtail_coefficients = vtail_ffd_block.evaluate(vtail_ffd_block_coefficients, plot=False)


    # weird nose hub
    nose_hub_ffd_block = lg.construct_ffd_block_around_entities(name='weird_nose_hub_ffd_block', entities=nose_hub, num_coefficients=(2, 2, 2))
    nose_hub_ffd_block_sect_param = VolumeSectionalParameterization(name='weird_nose_hub_ffd_sect_param', principal_parametric_dimension=0,
                                                                    parameterized_points=nose_hub_ffd_block.coefficients, parameterized_points_shape=nose_hub_ffd_block.coefficients_shape)

    nose_hub_ffd_block_sect_param.add_sectional_translation(name='nose_hub_translation_x', axis=0)
    nose_hub_transl_x_coeffs = system_model.create_input(name='nose_hub_transl_x_coefficients', shape=(1, ), val=np.array([0.]))
    nose_hub_transl_x_b_spline = bsp.BSpline(name='nose_hub_b_spline', space=constant_b_spline_curve_1_dof_space, coefficients=nose_hub_transl_x_coeffs, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., nose_hub_ffd_block_sect_param.num_sections).reshape((-1, 1))
    nose_hub_transl_x = nose_hub_transl_x_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'nose_hub_translation_x' : nose_hub_transl_x
    }

    nose_hub_ffd_block_coefficients = nose_hub_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
    nose_hub_coefficients = nose_hub_ffd_block.evaluate(nose_hub_ffd_block_coefficients, plot=False)


    # Boom + hub + disk
    rlo_disk_pt = rlo_hub_pt = np.array([19.200, -18.750, 9.635])
    rro_disk_pt = rro_hub_pt = np.array([19.200, 18.750, 9.635])
    rlo_boom_pt = np.array([12.000, -18.750, 7.613])
    rro_boom_pt = np.array([12.000, 18.750, 7.613])

    flo_disk_pt = flo_hub_pt = np.array([5.070, -18.750, 7.355])
    fro_disk_pt = fro_hub_pt = np.array([5.070, 18.750, 7.355])
    flo_boom_pt = np.array([12.200, -18.750, 7.615])
    fro_boom_pt = np.array([12.200, 18.750, 7.615])

    rli_disk_pt = rli_hub_pt = np.array([18.760, -8.537, 9.919])
    rri_disk_pt = rri_hub_pt = np.array([18.760, 8.537, 9.919])
    rli_boom_pt = np.array([11.500, -8.250, 7.898])
    rri_boom_pt = np.array([11.500, 8.250, 7.898])

    fli_disk_pt = fli_hub_pt = np.array([4.630, -8.217, 7.659])
    fri_disk_pt = fri_hub_pt = np.array([4.630, 8.217, 7.659])
    fli_boom_pt = np.array([11.741, -8.250, 7.900])
    fri_boom_pt = np.array([11.741, 8.250, 7.900])

    hub_disk_components = [
        (rlo_disk, rlo_hub, rlo_blade_1, rlo_blade_2), (rli_disk, rli_hub, rli_blade_1, rli_blade_2), (rri_disk, rri_hub, rri_blade_1, rri_blade_2), (rro_disk, rro_hub, rro_blade_1, rro_blade_2), 
        (flo_disk, flo_hub, flo_blade_1, flo_blade_2), (fli_disk, fli_hub, fli_blade_1, fli_blade_2), (fri_disk, fri_hub, fri_blade_1, fri_blade_2), (fro_disk, fro_hub, fro_blade_1, fro_blade_2)
    ]

    points_on_disk = [rlo_disk_pt, rli_disk_pt, rri_disk_pt, rro_disk_pt, flo_disk_pt, fli_disk_pt, fri_disk_pt, fro_disk_pt]

    components_to_be_connected = [rlo_boom, rli_boom, rri_boom, rro_boom, flo_boom, fli_boom, fri_boom, fro_boom]
    # components_to_be_connected = [rlo_boom, rro_boom]
    # components_to_be_connected = [rro_boom]
    points_on_components = [rlo_boom_pt, rli_boom_pt, rri_boom_pt, rro_boom_pt, flo_boom_pt, fli_boom_pt, fri_boom_pt, fro_boom_pt]
    # points_on_components = [rlo_boom_pt, rro_boom_pt]
    # points_on_components = [rro_boom_pt]
    components_to_connect_to = wing
    prinicpal_parametric_dimension = 0
    prinicpal_parametric_dimension_booms = 0
    space = constant_b_spline_curve_1_dof_space
    plot = False

    boom_distances = []
    ffd_block_translation_x_sect_coeffs_list = []
    ffd_block_translation_y_sect_coeffs_list = []
    ffd_block_translation_z_sect_coeffs_list = []
    ffd_block_sect_param_list = []
    ffd_block_list = []
    booms_coefficients_list = []

    # Booms
    for i, comp in enumerate(components_to_be_connected):
        block_name = f'{comp.name}_ffd_block'
        ffd_block = lg.construct_ffd_block_around_entities(name=block_name, entities=comp, num_coefficients=(2, 2, 2))
        ffd_block_sect_param = VolumeSectionalParameterization(name=f'{block_name}_sect_param', principal_parametric_dimension=prinicpal_parametric_dimension_booms, 
                                                            parameterized_points=ffd_block.coefficients,
                                                            parameterized_points_shape=ffd_block.coefficients_shape)
        
        ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_x', axis=0)
        ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_y', axis=1)
        ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_z', axis=2)


        ffd_block_translation_x_sect_coeffs = system_model.create_input(f'{block_name}_translation_x_coefficients', shape=(1, ), val=np.array([0]))
        ffd_block_translation_y_sect_coeffs = system_model.create_input(f'{block_name}_translation_y_coefficients', shape=(1, ), val=np.array([0]))
        ffd_block_translation_z_sect_coeffs = system_model.create_input(f'{block_name}_translation_z_coefficients', shape=(1, ), val=np.array([0]))

        ffd_block_translation_x_sect_coeffs_list.append(ffd_block_translation_x_sect_coeffs)
        ffd_block_translation_y_sect_coeffs_list.append(ffd_block_translation_y_sect_coeffs)
        ffd_block_translation_z_sect_coeffs_list.append(ffd_block_translation_z_sect_coeffs)

        ffd_block_translation_x_b_spline = bsp.BSpline(name=f'{block_name}_translation_x_bspline', space=space, coefficients=ffd_block_translation_x_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        ffd_block_translation_y_b_spline = bsp.BSpline(name=f'{block_name}_translation_y_bspline', space=space, coefficients=ffd_block_translation_y_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        ffd_block_translation_z_b_spline = bsp.BSpline(name=f'{block_name}_translation_z_bspline', space=space, coefficients=ffd_block_translation_z_sect_coeffs, 
                                                        num_physical_dimensions=1)


        section_parametric_coordinates = np.linspace(0., 1., ffd_block_sect_param.num_sections).reshape((-1,1))
        comp_transl_x = ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_y = ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_z = ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)

        sectional_parameters = {
                f'{block_name}_translation_x': comp_transl_x,
                f'{block_name}_translation_y': comp_transl_y,
                f'{block_name}_translation_z': comp_transl_z,
        }

        comp_ffd_block_coefficients = ffd_block_sect_param.evaluate(sectional_parameters, plot=plot)
        comp_coefficients = ffd_block.evaluate(comp_ffd_block_coefficients, plot=plot)
        booms_coefficients_list.append(comp_coefficients)
        ffd_block_sect_param_list.append(ffd_block_sect_param)
        ffd_block_list.append(ffd_block)

    # Lift rotors disk + Hub + blades
    hub_disk_distances = []
    hub_wing_distances = []
    ffd_block_translation_x_sect_coeffs_list_2 = []
    ffd_block_translation_y_sect_coeffs_list_2 = []
    ffd_block_translation_z_sect_coeffs_list_2 = []
    ffd_block_sect_param_list_2 = []
    ffd_block_list_2 = []
    ffd_block_stretch_x_sect_coeffs_list = []
    ffd_block_stretch_y_sect_coeffs_list = []
    rotor_coefficients_list = []

    for i, comp in enumerate(hub_disk_components):
        block_name = f'{comp[0].name}_{comp[1].name}'
        disk = comp[0]
        hub = comp[1]
        blade_1 = comp[2]
        blade_2 = comp[3]
        boom = components_to_be_connected[i]
        ffd_block = lg.construct_ffd_block_around_entities(name=block_name, entities=list(comp), num_coefficients=(2, 2, 2))
        ffd_block_sect_param = VolumeSectionalParameterization(name=f'{block_name}_sect_param', principal_parametric_dimension=prinicpal_parametric_dimension, 
                                                            parameterized_points=ffd_block.coefficients,
                                                            parameterized_points_shape=ffd_block.coefficients_shape)
        
        # Allow the lift rotors to move/translate
        ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_x', axis=0)
        ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_y', axis=1)
        ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_z', axis=2)

        # Allow the lift rotors to change radius
        # ffd_block_sect_param.add_sectional_stretch(name=f'{block_name}_stretch_x', axis=0)
        ffd_block_sect_param.add_sectional_stretch(name=f'{block_name}_stretch_y', axis=1)
        # ffd_block_sect_param.add_sectional_stretch(name=f'{block_name}_stretch_z', axis=2)

        ffd_block_translation_x_sect_coeffs = system_model.create_input(f'{block_name}_translation_x_coefficients', shape=(2, ), val=np.array([0, 0]))
        ffd_block_translation_y_sect_coeffs = system_model.create_input(f'{block_name}_translation_y_coefficients', shape=(1, ), val=np.array([0]))
        ffd_block_translation_z_sect_coeffs = system_model.create_input(f'{block_name}_translation_z_coefficients', shape=(1, ), val=np.array([0]))
        # ffd_block_stretch_x_sect_coeffs = system_model.create_input(f'{block_name}_stretch_x_coefficients', shape=(1, ), val=np.array([0.]))
        ffd_block_stretch_y_sect_coeffs = system_model.create_input(f'{block_name}_stretch_y_coefficients', shape=(1, ), val=np.array([0]))
        # ffd_block_stretch_z_sect_coeffs = system_model.create_input(f'{block_name}_stretch_z_coefficients', shape=(1, ), val=np.array([10.]))

        ffd_block_translation_x_sect_coeffs_list_2.append(ffd_block_translation_x_sect_coeffs)
        ffd_block_translation_y_sect_coeffs_list_2.append(ffd_block_translation_y_sect_coeffs)
        ffd_block_translation_z_sect_coeffs_list_2.append(ffd_block_translation_z_sect_coeffs)
        ffd_block_stretch_y_sect_coeffs_list.append(ffd_block_stretch_y_sect_coeffs)

        time_b_spline_1 = time.time()
        ffd_block_translation_x_b_spline = bsp.BSpline(name=f'{block_name}_translation_x_bspline', space=linear_b_spline_curve_2_dof_space, coefficients=ffd_block_translation_x_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        ffd_block_translation_y_b_spline = bsp.BSpline(name=f'{block_name}_translation_y_bspline', space=space, coefficients=ffd_block_translation_y_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        ffd_block_translation_z_b_spline = bsp.BSpline(name=f'{block_name}_translation_z_bspline', space=space, coefficients=ffd_block_translation_z_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        # ffd_block_stretch_x_b_spline = bsp.BSpline(name=f'{block_name}_stretch_x_bspline', space=linear_b_spline_curve_2_dof_space, coefficients=ffd_block_stretch_x_sect_coeffs, 
        #                                                 num_physical_dimensions=1)
        
        ffd_block_stretch_y_b_spline = bsp.BSpline(name=f'{block_name}_stretch_y_bspline', space=space, coefficients=ffd_block_stretch_y_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        # ffd_block_stretch_z_b_spline = bsp.BSpline(name=f'{block_name}_stretch_z_bspline', space=space, coefficients=ffd_block_stretch_z_sect_coeffs, 
        #                                                 num_physical_dimensions=1)

        section_parametric_coordinates = np.linspace(0., 1., ffd_block_sect_param.num_sections).reshape((-1,1))
        comp_transl_x = ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_y = ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_z = ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)
        # comp_stretch_x = ffd_block_stretch_x_b_spline.evaluate(section_parametric_coordinates)
        comp_stretch_y = ffd_block_stretch_y_b_spline.evaluate(section_parametric_coordinates)
        # comp_stretch_z = ffd_block_stretch_y_b_spline.evaluate(section_parametric_coordinates)

        sectional_parameters = {
                f'{block_name}_translation_x': comp_transl_x,
                f'{block_name}_translation_y': comp_transl_y,
                f'{block_name}_translation_z': comp_transl_z,
                # f'{block_name}_stretch_x': comp_stretch_x,
                f'{block_name}_stretch_y': comp_stretch_y,
                # f'{block_name}_stretch_z': comp_stretch_z,
        }

        # ffd_block.plot()

        comp_ffd_block_coefficients = ffd_block_sect_param.evaluate(sectional_parameters, plot=plot)
        comp_coefficients = ffd_block.evaluate(comp_ffd_block_coefficients, plot=plot)
        rotor_coefficients_list.append(comp_coefficients)
        ffd_block_sect_param_list_2.append(ffd_block_sect_param)
        ffd_block_list_2.append(ffd_block)

    # Pusher prop disk + hub + blades
    pusher_prop_ffd_block = lg.construct_ffd_block_around_entities(name='pusher_prop_ffd_block', entities=[pp_disk, pp_hub, pp_blade_1, pp_blade_2, pp_blade_3, pp_blade_4], num_coefficients=(2, 2, 2))
    pusher_prop_ffd_block_sect_param = VolumeSectionalParameterization(name='pusher_prop_ffd_block_sect_param', principal_parametric_dimension=0, 
                                                            parameterized_points=pusher_prop_ffd_block.coefficients,
                                                            parameterized_points_shape=pusher_prop_ffd_block.coefficients_shape)

    pusher_prop_ffd_block_sect_param.add_sectional_translation(name='pusher_prop_ffd_block_translation_x', axis=0)
    pusher_prop_ffd_block_translation_x_sect_coeffs = system_model.create_input('pusher_prop_ffd_block_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    pusher_prop_ffd_block_translation_x_b_spline = bsp.BSpline(name='pusher_prop_ffd_block_translation_x_bspline', space=space, coefficients=pusher_prop_ffd_block_translation_x_sect_coeffs, 
                                                        num_physical_dimensions=1)

    pusher_prop_ffd_block_sect_param.add_sectional_stretch(name='pusher_prop_ffd_block_sectional_stretch_y', axis=1)
    pusher_prop_ffd_block_stretch_y_coefficients = system_model.create_input('pusher_prop_ffd_block_stretch_y_coefficients', shape=(1, ), val=np.array([0]))
    pusher_prop_ffd_block_stretch_y_b_spline = bsp.BSpline(name='pusher_prop_ffd_block_stretch_y_bspline', space=space, coefficients=pusher_prop_ffd_block_stretch_y_coefficients, 
                                                        num_physical_dimensions=1)

    pusher_prop_ffd_block_sect_param.add_sectional_stretch(name='pusher_prop_ffd_block_sectional_stretch_z', axis=2)
    pusher_prop_ffd_block_stretch_z_coefficients = system_model.create_input('pusher_prop_ffd_block_stretch_z_coefficients', shape=(1, ), val=np.array([0]))
    pusher_prop_ffd_block_stretch_z_b_spline = bsp.BSpline(name='pusher_prop_ffd_block_stretch_z_bspline', space=space, coefficients=pusher_prop_ffd_block_stretch_z_coefficients, 
                                                        num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., pusher_prop_ffd_block_sect_param.num_sections).reshape((-1,1))
    pusher_prop_transl_x = pusher_prop_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    pusher_prop_stretch_y = pusher_prop_ffd_block_stretch_y_b_spline.evaluate(section_parametric_coordinates)
    pusher_prop_stretch_z = pusher_prop_ffd_block_stretch_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'pusher_prop_ffd_block_translation_x': pusher_prop_transl_x,
        'pusher_prop_ffd_block_sectional_stretch_y': pusher_prop_stretch_y,
        'pusher_prop_ffd_block_sectional_stretch_z': pusher_prop_stretch_z,
    }

    pusher_prop_ffd_block_coefficients = pusher_prop_ffd_block_sect_param.evaluate(sectional_parameters, plot=plot)
    pusher_prop_coefficients = pusher_prop_ffd_block.evaluate(pusher_prop_ffd_block_coefficients, plot=plot)


    # Assigning coefficients + Setting up inner optimization inputs/constraints
    ################################################################

    # Assign coefficients
    coefficients_list = []
    coefficients_list.append(wing_coefficients)
    coefficients_list.append(fuselage_coefficients)
    coefficients_list.append(htail_coefficients)
    coefficients_list.append(vtail_coefficients)
    coefficients_list.append(nose_hub_coefficients)

    b_spline_names_list = []
    b_spline_names_list.append(wing.b_spline_names)
    b_spline_names_list.append(fuselage.b_spline_names)
    b_spline_names_list.append(h_tail.b_spline_names)
    b_spline_names_list.append(vtail.b_spline_names)
    b_spline_names_list.append(nose_hub.b_spline_names)

    geometry.assign_coefficients(coefficients=wing_coefficients, b_spline_names=wing.b_spline_names)
    geometry.assign_coefficients(coefficients=fuselage_coefficients, b_spline_names=fuselage.b_spline_names)
    geometry.assign_coefficients(coefficients=htail_coefficients, b_spline_names=h_tail.b_spline_names)
    geometry.assign_coefficients(coefficients=vtail_coefficients, b_spline_names=vtail.b_spline_names)
    geometry.assign_coefficients(coefficients=nose_hub_coefficients, b_spline_names=nose_hub.b_spline_names)

    for i, comp in enumerate(components_to_be_connected):
        # geometry.assign_coefficients(coefficients=comp_coefficients, b_spline_names=comp.b_spline_names)
        coefficients_list.append(booms_coefficients_list[i])
        b_spline_names_list.append(comp.b_spline_names)

    for i, comp in enumerate(hub_disk_components):
        disk = comp[0]
        hub = comp[1]
        blade_1 = comp[2]
        blade_2 = comp[3]
        boom = components_to_be_connected[i]

        coefficients_list.append(rotor_coefficients_list[i][disk.name + '_coefficients'])
        coefficients_list.append(rotor_coefficients_list[i][hub.name + '_coefficients'])
        coefficients_list.append(rotor_coefficients_list[i][blade_1.name + '_coefficients'])
        coefficients_list.append(rotor_coefficients_list[i][blade_2.name + '_coefficients'])
        
        b_spline_names_list.append(disk.b_spline_names)
        b_spline_names_list.append(hub.b_spline_names)
        b_spline_names_list.append(blade_1.b_spline_names)
        b_spline_names_list.append(blade_2.b_spline_names)

        # geometry.assign_coefficients(coefficients=comp_coefficients[disk.name + '_coefficients'], b_spline_names=disk.b_spline_names)
        # geometry.assign_coefficients(coefficients=comp_coefficients[hub.name + '_coefficients'], b_spline_names=hub.b_spline_names)
        # geometry.assign_coefficients(coefficients=comp_coefficients[blade_1.name + '_coefficients'], b_spline_names=blade_1.b_spline_names)
        # geometry.assign_coefficients(coefficients=comp_coefficients[blade_2.name + '_coefficients'], b_spline_names=blade_2.b_spline_names)

    geometry.assign_coefficients(coefficients=pusher_prop_coefficients[pp_disk.name + '_coefficients'], b_spline_names=pp_disk.b_spline_names)
    geometry.assign_coefficients(coefficients=pusher_prop_coefficients[pp_hub.name + '_coefficients'], b_spline_names=pp_hub.b_spline_names)
    geometry.assign_coefficients(coefficients=pusher_prop_coefficients[pp_blade_1.name + '_coefficients'], b_spline_names=pp_blade_1.b_spline_names)
    geometry.assign_coefficients(coefficients=pusher_prop_coefficients[pp_blade_2.name + '_coefficients'], b_spline_names=pp_blade_2.b_spline_names)
    geometry.assign_coefficients(coefficients=pusher_prop_coefficients[pp_blade_3.name + '_coefficients'], b_spline_names=pp_blade_3.b_spline_names)
    geometry.assign_coefficients(coefficients=pusher_prop_coefficients[pp_blade_4.name + '_coefficients'], b_spline_names=pp_blade_4.b_spline_names)

    coefficients_list.append(pusher_prop_coefficients[pp_disk.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_hub.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_1.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_2.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_3.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_4.name + '_coefficients'])

    b_spline_names_list.append(pp_disk.b_spline_names)
    b_spline_names_list.append(pp_hub.b_spline_names)
    b_spline_names_list.append(pp_blade_1.b_spline_names)
    b_spline_names_list.append(pp_blade_2.b_spline_names)
    b_spline_names_list.append(pp_blade_3.b_spline_names)
    b_spline_names_list.append(pp_blade_4.b_spline_names)

    geometry.assign_coefficients(coefficients=coefficients_list, b_spline_names=b_spline_names_list)

    # geometry.plot()

    # Evaluations 
    wing_te_right_m3l = geometry.evaluate(wing.project(wing_te_right))
    wing_te_left_m3l = geometry.evaluate(wing.project(wing_te_left))
    wing_te_center_m3l = geometry.evaluate(wing.project(wing_te_center))
    wing_le_left_m3l = geometry.evaluate(wing.project(wing_le_left))
    wing_le_right_m3l = geometry.evaluate(wing.project(wing_le_right))
    wing_le_center_m3l = geometry.evaluate(wing.project(wing_le_center))
    wing_span = m3l.norm(wing_te_right_m3l - wing_te_left_m3l)
    root_chord = m3l.norm(wing_te_center_m3l-wing_le_center_m3l)
    tip_chord_left = m3l.norm(wing_te_left_m3l-wing_le_left_m3l)
    tip_chord_right = m3l.norm(wing_te_right_m3l-wing_le_right_m3l)


    fuselage_front = geometry.evaluate(fuselage.project(points=np.array([1.889, 0., -0.175])))
    fuselage_rear = geometry.evaluate(fuselage.project(points=np.array([31.889, 0., 7.798])))
    fuselage_length = m3l.norm(fuselage_rear-fuselage_front)


    tail_te_right_m3l = geometry.evaluate(h_tail.project(tail_te_right))
    tail_te_left_m3l = geometry.evaluate(h_tail.project(tail_te_left))
    tail_le_right_m3l = geometry.evaluate(h_tail.project(tail_le_right))
    tail_le_left_m3l = geometry.evaluate(h_tail.project(tail_le_left))
    tail_te_center_m3l = geometry.evaluate(h_tail.project(tail_te_center))
    tail_le_center_m3l = geometry.evaluate(h_tail.project(tail_le_center))
    htail_span = m3l.norm(tail_te_right_m3l - tail_te_left_m3l)
    htail_tip_chord_left = m3l.norm(tail_te_left_m3l - tail_le_left_m3l)
    htail_tip_chord_right = m3l.norm(tail_te_right_m3l - tail_le_right_m3l)
    htail_root_chord = m3l.norm(tail_te_center_m3l - tail_le_center_m3l)

    htail_qc_m3l = geometry.evaluate(h_tail.project(tail_te_center))
    wing_qc_m3l = geometry.evaluate(wing.project(tail_te_center))

    tail_moment_arm = m3l.norm(htail_qc_m3l - wing_qc_m3l)
    # tail_moment_arm = htail_qc_m3l - wing_qc_m3l


    vtail_point_m3l = geometry.evaluate(vtail.project(np.array([30.543, 0., 8.231])))
    vtail_minus_fuselage = m3l.norm(vtail_point_m3l - fuselage_rear)

    wing_point_on_fuselage_m3l = geometry.evaluate(fuselage.project(wing_le_center))
    wing_point_on_wing_m3l = geometry.evaluate(wing.project(wing_le_center))
    wing_fuselage_connection = m3l.norm(wing_point_on_fuselage_m3l-wing_point_on_wing_m3l)
    # wing_fuselage_connection = (wing_point_on_fuselage_m3l-wing_point_on_wing_m3l)

    h_tail_point_on_fuselage_m3l = geometry.evaluate(fuselage.project(tail_te_center))
    h_tail_point_on_htail_m3l = geometry.evaluate(h_tail.project(tail_te_center))
    h_tail_fuselage_connection = m3l.norm(h_tail_point_on_fuselage_m3l-h_tail_point_on_htail_m3l)

    nose_hub_point_m3l = geometry.evaluate(nose_hub.project(np.array([2.250, 0., 4.150])))
    fuselage_minus_nose_hub = m3l.norm(fuselage_front-nose_hub_point_m3l)

    for i, comp in enumerate(components_to_be_connected):
        point_on_component = points_on_components[i]
        point_on_component_m3l = geometry.evaluate(comp.project(point_on_component, plot=False))
        point_on_component_to_be_connecte_to_m3l = geometry.evaluate(components_to_connect_to.project(point_on_component, plot=False))

        # euclidean_distance = m3l.norm(point_on_component_m3l-point_on_component_to_be_connecte_to_m3l)
        euclidean_distance = point_on_component_m3l-point_on_component_to_be_connecte_to_m3l
        boom_distances.append(euclidean_distance)

        # euclidean_distance = point_on_component_m3l-point_on_component_to_be_connecte_to_m3l
        # boom_distances.append(euclidean_distance)
        


    for i, comp in enumerate(hub_disk_components):
    #     disk = comp[0]
        hub = comp[1]
        blade_1 = comp[2]
        blade_2 = comp[3]
        boom = components_to_be_connected[i]

        point_on_disk = points_on_disk[i]
        point_on_hub_m3l = geometry.evaluate(hub.project(point_on_disk))
        # point_on_wing_m3l = geometry.evaluate(wing.project(point_on_disk))
        point_on_boom_m3l = geometry.evaluate(boom.project(point_on_disk))
        hub_minus_boom = point_on_hub_m3l - point_on_boom_m3l # or hub to boom

        hub_minus_boom_dist = m3l.norm(hub_minus_boom)

        # hub_disk_distances.append(hub_minus_disk_dist)
        # hub_wing_distances.append(hub_minus_boom_dist)
        
        hub_wing_distances.append(hub_minus_boom)
    
        
        # ffd_block_stretch_x_sect_coeffs_list.append(ffd_block_stretch_x_sect_coeffs)
        


    pusher_prop_point_m3l = geometry.evaluate(pp_disk.project(np.array([32.625, 0., 7.79])))
    pusher_prop_minus_fuselage = m3l.norm(pusher_prop_point_m3l - fuselage_rear)

    pp_mesh = pp_mesh.update(geometry=geometry)
    rlo_mesh = rlo_mesh.update(geometry=geometry)
    rro_mesh = rro_mesh.update(geometry=geometry)
    flo_mesh = flo_mesh.update(geometry=geometry)
    fro_mesh = fro_mesh.update(geometry=geometry)
    rli_mesh = rli_mesh.update(geometry=geometry)
    rri_mesh = rri_mesh.update(geometry=geometry)
    fli_mesh = fli_mesh.update(geometry=geometry)
    fri_mesh = fri_mesh.update(geometry=geometry)

    radius_1_list = [flo_mesh.radius, fli_mesh.radius, fri_mesh.radius, fro_mesh.radius,
                    rlo_mesh.radius, rli_mesh.radius, rri_mesh.radius, rro_mesh.radius]

    radius_2_list = [flo_mesh._radius_2, fli_mesh._radius_2, fri_mesh._radius_2, fro_mesh._radius_2,
                    rlo_mesh._radius_2, rli_mesh._radius_2, rri_mesh._radius_2, rro_mesh._radius_2]

    ################################################################

    # Declaring states and input to the inner optimization
    lpc_param_solver.declare_state('wing_span_stretch_coefficients', wing_span_strech_coefficients, penalty_factor=1000)
    lpc_param_solver.declare_state('wing_chord_stretch_coefficients', wing_chord_stretch_coefficients)
    lpc_param_solver.declare_state('htail_span_stretch_coefficients', htail_span_stretch_coefficients, penalty_factor=1000)
    lpc_param_solver.declare_state('htail_chord_stretch_coefficients', htail_chord_stretch_coefficients)
    lpc_param_solver.declare_state('fuselage_stretch_coefficients', fuselage_stretch_coeffs)
    lpc_param_solver.declare_state('htail_translation_x_coefficients', htail_chord_transl_x_coefficients, penalty_factor=1e-5)
    lpc_param_solver.declare_state('nose_hub_transl_x_coefficients', nose_hub_transl_x_coeffs)
    lpc_param_solver.declare_state('vtail_transl_x_coefficients', vtail_transl_x_coeffs, penalty_factor=1)
    lpc_param_solver.declare_state('pusher_prop_ffd_block_translation_x_coefficients', pusher_prop_ffd_block_translation_x_sect_coeffs)
    lpc_param_solver.declare_state('pusher_prop_ffd_block_stretch_y_coefficients', pusher_prop_ffd_block_stretch_y_coefficients)
    lpc_param_solver.declare_state('pusher_prop_ffd_block_stretch_z_coefficients', pusher_prop_ffd_block_stretch_z_coefficients)


    lpc_param_solver.declare_input(name=wing_span_input.name, input=wing_span)
    lpc_param_solver.declare_input(name=wing_tip_chord_left_input.name, input=tip_chord_left)
    lpc_param_solver.declare_input(name=wing_tip_chord_right_input.name, input=tip_chord_right)
    lpc_param_solver.declare_input(name=wing_root_chord_input.name, input=root_chord)
    lpc_param_solver.declare_input(name=tail_tip_chord_left_input.name, input=htail_tip_chord_left)
    lpc_param_solver.declare_input(name=tail_tip_chord_right_input.name, input=htail_tip_chord_right)
    lpc_param_solver.declare_input(name=tail_root_chord_input.name, input=htail_root_chord)
    lpc_param_solver.declare_input(name=tail_span_input.name, input=htail_span)
    # lpc_param_solver.declare_input(name=tail_moment_arm_input.name, input=tail_moment_arm)
    lpc_param_solver.declare_input(name='nose_hub_to_fuselage_connection', input=fuselage_minus_nose_hub)
    lpc_param_solver.declare_input(name='vtail_to_fuselage_connection', input=vtail_minus_fuselage)
    lpc_param_solver.declare_input(name='pusher_prop_to_fuselage_connection', input=pusher_prop_minus_fuselage)
    lpc_param_solver.declare_input(name='pusher_prop_r1', input=pp_mesh.radius)
    lpc_param_solver.declare_input(name='pusher_prop_r2', input=pp_mesh._radius_2)
    lpc_param_solver.declare_input(name='wing_fuselage_connection', input=wing_fuselage_connection)
    lpc_param_solver.declare_input(name='h_tail_fuselage_connection', input=h_tail_fuselage_connection)




    nose_hub_to_fuselage_connection_input = m3l.Variable(name='nose_hub_to_fuselage_connection', shape=(1, ), value=fuselage_minus_nose_hub.value)
    vtail_to_fuselage_connection_input = m3l.Variable(name='vtail_to_fuselage_connection', shape=(1, ), value=vtail_minus_fuselage.value)
    pusher_prop_to_fuselage_connection_input = m3l.Variable(name='pusher_prop_to_fuselage_connection', shape=(1, ), value=pusher_prop_minus_fuselage.value)
    pusher_prop_r1_input = m3l.Variable(name='pusher_prop_r1', shape=(1, ), value=pusher_prop_radius.value)
    pusher_prop_r2_input = m3l.Variable(name='pusher_prop_r2', shape=(1, ), value=pusher_prop_radius.value)
    wing_fuselage_connection_input = m3l.Variable(name='wing_fuselage_connection', shape=(1, ), value=wing_fuselage_connection.value)
    h_tail_fuselage_connection_input = m3l.Variable(name='h_tail_fuselage_connection', shape=(1, ), value=h_tail_fuselage_connection.value)


    parameterization_inputs = {
        wing_span_input.name : wing_span_input,
        wing_tip_chord_left_input.name : wing_tip_chord_left_input,
        wing_tip_chord_right_input.name : wing_tip_chord_right_input,
        wing_root_chord_input.name : wing_root_chord_input,
        tail_tip_chord_left_input.name : tail_tip_chord_left_input,
        tail_tip_chord_right_input.name : tail_tip_chord_right_input,
        tail_root_chord_input.name : tail_root_chord_input,
        tail_span_input.name : tail_span_input,
        # tail_moment_arm_input.name : tail_moment_arm_input,
        'nose_hub_to_fuselage_connection' : nose_hub_to_fuselage_connection_input,
        'vtail_to_fuselage_connection' : vtail_to_fuselage_connection_input,
        'pusher_prop_to_fuselage_connection' : pusher_prop_to_fuselage_connection_input,
        'pusher_prop_r1' : pusher_prop_r1_input,
        'pusher_prop_r2' : pusher_prop_r2_input,
        'wing_fuselage_connection' : wing_fuselage_connection_input,
        'h_tail_fuselage_connection' : h_tail_fuselage_connection_input,
    }

    # Booms
    for i, comp in enumerate(components_to_be_connected):
        block_name = f'{comp.name}_ffd_block'
        lpc_param_solver.declare_state(name=f'{block_name}_translation_x_coefficients', state=ffd_block_translation_x_sect_coeffs_list[i], penalty_factor=1)
        lpc_param_solver.declare_state(name=f'{block_name}_translation_y_coefficients', state=ffd_block_translation_y_sect_coeffs_list[i], penalty_factor=1)
        lpc_param_solver.declare_state(name=f'{block_name}_translation_z_coefficients', state=ffd_block_translation_z_sect_coeffs_list[i], penalty_factor=1)

        lpc_param_solver.declare_input(name=f'{block_name}_distance_to_be_enforced', input=boom_distances[i])
        component_distance_input = m3l.Variable(name=f'{block_name}_distance_to_be_enforced', shape=(3, ), value=boom_distances[i].value)
        # component_distance_input = m3l.Variable(name=f'{block_name}_distance_to_be_enforced', shape=(3, ), value=boom_distances[i].value)
        # system_model.create_input(name=f'{block_name}_distance_to_be_enforced', shape=(1, ), val=boom_distances[i].value)

        parameterization_inputs[f'{block_name}_distance_to_be_enforced'] = component_distance_input

    # Hub + Disk + blades
    for i, comp in enumerate(hub_disk_components):
        block_name = f'{comp[0].name}_{comp[1].name}'
        lpc_param_solver.declare_state(name=f'{block_name}_translation_x_coefficients', state=ffd_block_translation_x_sect_coeffs_list_2[i], penalty_factor=1)
        lpc_param_solver.declare_state(name=f'{block_name}_translation_y_coefficients', state=ffd_block_translation_y_sect_coeffs_list_2[i], penalty_factor=1)
        lpc_param_solver.declare_state(name=f'{block_name}_translation_z_coefficients', state=ffd_block_translation_z_sect_coeffs_list_2[i], penalty_factor=1)
        # lpc_param_solver.declare_state(name=f'{block_name}_stretch_x_coefficients', state=ffd_block_stretch_x_sect_coeffs_list[i], penalty_factor=1)
        lpc_param_solver.declare_state(name=f'{block_name}_stretch_y_coefficients', state=ffd_block_stretch_y_sect_coeffs_list[i], penalty_factor=1)

        lpc_param_solver.declare_input(name=f'{block_name}_hub_wing_distance_to_be_enforced', input=hub_wing_distances[i])
        lpc_param_solver.declare_input(name=f'{block_name}_r1', input=radius_1_list[i])
        lpc_param_solver.declare_input(name=f'{block_name}_r2', input=radius_2_list[i])
        
        component_distance_input_2 = m3l.Variable(name=f'{block_name}_hub_wing_distance_to_be_enforced', shape=(3, ), value=hub_wing_distances[i].value)
        r1_input = m3l.Variable(name=f'{block_name}_r1', shape=(1, ), value=dv_radius_list[i].value)
        r2_input = m3l.Variable(name=f'{block_name}_r2', shape=(1, ), value=dv_radius_list[i].value)

        parameterization_inputs[f'{block_name}_hub_wing_distance_to_be_enforced'] = component_distance_input_2
        parameterization_inputs[f'{block_name}_r1'] = r1_input
        parameterization_inputs[f'{block_name}_r2'] = r2_input




    # Evaluate parameterization solver
    outputs_dict = lpc_param_solver.evaluate(inputs=parameterization_inputs, plot=False)

    # RE-ASSIGNING COEFFICIENTS AFTER INNER OPTIMIZATION
 
    coefficients_list = []
    # b_spline_names_list = []
    # Wing
    wing_span_stretch_coefficients = outputs_dict['wing_span_stretch_coefficients']
    left_wing_wingspan_stretch_b_spline = bsp.BSpline(name='wingspan_stretch_b_spline', space=linear_b_spline_curve_2_dof_space,
                                                        coefficients=wing_span_stretch_coefficients, num_physical_dimensions=1)

    wing_chord_stretch_coefficients = outputs_dict['wing_chord_stretch_coefficients']
    wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space,
                                                    coefficients=wing_chord_stretch_coefficients, num_physical_dimensions=1)
    wing_twist_coefficients = m3l.Variable(name='wing_twist_coefficients', shape=(5,),
                                                value=np.array([0., 0., 0., 0., 0.]))
    wing_twist_b_spline = bsp.BSpline(name='wing_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
                                                coefficients=wing_twist_coefficients, num_physical_dimensions=1)
    # wing_ffd_block_sect_param.plot()

    section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sect_param.num_sections).reshape((-1,1))
    wing_wingspan_stretch = left_wing_wingspan_stretch_b_spline.evaluate(section_parametric_coordinates)
    wing_sectional_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
    wing_sectional_twist = wing_twist_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'wing_span_stretch': wing_wingspan_stretch,
            'wing_chord_stretch': wing_sectional_chord_stretch,
            'wing_twist': wing_sectional_twist,
                                }

    wing_ffd_block_coefficients = wing_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
    wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=False)
    b_spline_names_list.append(wing.b_spline_names)

    # h-tail
    htail_span_stretch_coefficients = outputs_dict['htail_span_stretch_coefficients']
    htail_span_stretch_b_spline = bsp.BSpline(name='htail_span_b_spline', space=linear_b_spline_curve_2_dof_space,
                                                        coefficients=htail_span_stretch_coefficients, num_physical_dimensions=1)

    htail_chord_stretch_coefficients = outputs_dict['htail_chord_stretch_coefficients']
    htail_chord_stretch_b_spline = bsp.BSpline(name='htail_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space,
                                                    coefficients=htail_chord_stretch_coefficients, num_physical_dimensions=1)

    htail_ffd_block_translation_x_sect_coeffs = outputs_dict['htail_translation_x_coefficients']
    htail_ffd_block_translation_x_b_spline = bsp.BSpline(name='htail_translation_x_b_spline', space=constant_b_spline_curve_1_dof_space, 
                                                coefficients=htail_ffd_block_translation_x_sect_coeffs, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., htail_ffd_block_sect_param.num_sections).reshape((-1,1))
    htail_span_stretch = htail_span_stretch_b_spline.evaluate(section_parametric_coordinates)
    htail_sectional_chord_stretch = htail_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
    htail_translation_x = htail_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)


    sectional_parameters = {
        'htail_span_stretch' : htail_span_stretch,
        'htail_chord_stretch' : htail_sectional_chord_stretch,
        'htail_translation_x' : htail_translation_x,
    }

    htail_ffd_block_coefficients = htail_ffd_block_sect_param.evaluate(sectional_parameters)
    htail_coefficients = htail_ffd_block.evaluate(htail_ffd_block_coefficients)

    # Fuselage
    fuselage_stretch_coefficients = outputs_dict['fuselage_stretch_coefficients']
    fuselage_stretch_b_spline = bsp.BSpline(name='fuselage_stretch_b_spline', space=linear_b_spline_curve_2_dof_space,
                                                        coefficients=fuselage_stretch_coefficients, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., fuselage_ffd_block_sect_param.num_sections).reshape((-1, 1))
    fuselage_stretch = fuselage_stretch_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'fuselage_stretch' : fuselage_stretch
    }

    fuselage_ffd_block_coefficients = fuselage_ffd_block_sect_param.evaluate(sectional_parameters)
    fuselage_coefficients = fuselage_ffd_block.evaluate(fuselage_ffd_block_coefficients)

    # Weird nose hub
    nose_hub_transl_x_coefficients = outputs_dict['nose_hub_transl_x_coefficients']
    nose_hub_transl_x_b_spline = bsp.BSpline(name='nose_hub_transl_x_b_spline', space=constant_b_spline_curve_1_dof_space,
                                                        coefficients=nose_hub_transl_x_coefficients, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., nose_hub_ffd_block_sect_param.num_sections).reshape((-1, 1))
    nose_hub_translation_x = nose_hub_transl_x_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'nose_hub_translation_x' : nose_hub_translation_x
    }

    nose_hub_ffd_block_coefficients = nose_hub_ffd_block_sect_param.evaluate(sectional_parameters)
    nose_hub_coefficients = nose_hub_ffd_block.evaluate(nose_hub_ffd_block_coefficients)

    # V-tail
    vtail_transl_x_coefficients = outputs_dict['vtail_transl_x_coefficients']
    vtail_transl_x_b_spline = bsp.BSpline(name='vtail_transl_x_b_spline', space=constant_b_spline_curve_1_dof_space,
                                                        coefficients=vtail_transl_x_coefficients, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., vtail_ffd_block_sect_param.num_sections).reshape((-1, 1))
    vtail_translation_x = vtail_transl_x_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'vtail_translation_x' : vtail_translation_x
    }

    vtail_ffd_block_coefficients = vtail_ffd_block_sect_param.evaluate(sectional_parameters)
    vtail_coefficients = vtail_ffd_block.evaluate(vtail_ffd_block_coefficients)


    # pusher prop
    pusher_prop_ffd_block_translation_x_sect_coeffs = outputs_dict['pusher_prop_ffd_block_translation_x_coefficients']
    pusher_prop_ffd_block_translation_x_b_spline = bsp.BSpline(name='pusher_prop_ffd_block_translation_x_bspline', space=space, coefficients=pusher_prop_ffd_block_translation_x_sect_coeffs, 
                                                        num_physical_dimensions=1)

    pusher_prop_ffd_block_stretch_y_coefficients = outputs_dict['pusher_prop_ffd_block_stretch_y_coefficients']
    pusher_prop_ffd_block_stretch_y_b_spline = bsp.BSpline(name='pusher_prop_ffd_block_stretch_y_bspline', space=space, coefficients=pusher_prop_ffd_block_stretch_y_coefficients, 
                                                        num_physical_dimensions=1)

    pusher_prop_ffd_block_stretch_z_coefficients = outputs_dict['pusher_prop_ffd_block_stretch_z_coefficients']
    pusher_prop_ffd_block_stretch_z_b_spline = bsp.BSpline(name='pusher_prop_ffd_block_stretch_z_bspline', space=space, coefficients=pusher_prop_ffd_block_stretch_z_coefficients, 
                                                        num_physical_dimensions=1)


    section_parametric_coordinates = np.linspace(0., 1., pusher_prop_ffd_block_sect_param.num_sections).reshape((-1,1))
    pusher_prop_transl_x = pusher_prop_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    pusher_prop_stretch_y = pusher_prop_ffd_block_stretch_y_b_spline.evaluate(section_parametric_coordinates)
    pusher_prop_stretch_z = pusher_prop_ffd_block_stretch_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
        'pusher_prop_ffd_block_translation_x': pusher_prop_transl_x,
        'pusher_prop_ffd_block_sectional_stretch_y': pusher_prop_stretch_y,
        'pusher_prop_ffd_block_sectional_stretch_z': pusher_prop_stretch_z,
    }

    pusher_prop_ffd_block_coefficients = pusher_prop_ffd_block_sect_param.evaluate(sectional_parameters, plot=plot)
    pusher_prop_coefficients = pusher_prop_ffd_block.evaluate(pusher_prop_ffd_block_coefficients, plot=plot)


    coefficients_list.append(wing_coefficients)
    coefficients_list.append(fuselage_coefficients)
    coefficients_list.append(htail_coefficients)
    coefficients_list.append(vtail_coefficients)
    coefficients_list.append(nose_hub_coefficients)
    

    for i, comp in enumerate(components_to_be_connected):
        block_name = f'{comp.name}_ffd_block'
        ffd_block_translation_x_sect_coeffs = outputs_dict[f'{block_name}_translation_x_coefficients']
        ffd_block_translation_x_b_spline = bsp.BSpline(name=f'{block_name}_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                coefficients=ffd_block_translation_x_sect_coeffs, num_physical_dimensions=1)
        ffd_block_translation_y_sect_coeffs = outputs_dict[f'{block_name}_translation_y_coefficients']
        ffd_block_translation_y_b_spline = bsp.BSpline(name=f'{block_name}_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                coefficients=ffd_block_translation_y_sect_coeffs, num_physical_dimensions=1)
        ffd_block_translation_z_sect_coeffs = outputs_dict[f'{block_name}_translation_z_coefficients']
        ffd_block_translation_z_b_spline = bsp.BSpline(name=f'{block_name}_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                coefficients=ffd_block_translation_z_sect_coeffs, num_physical_dimensions=1)

        section_parametric_coordinates = np.linspace(0., 1., ffd_block_sect_param_list[i].num_sections).reshape((-1,1))
        comp_transl_x = ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_y = ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_z = ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)

        sectional_parameters = {
                f'{block_name}_translation_x': comp_transl_x,
                f'{block_name}_translation_y': comp_transl_y,
                f'{block_name}_translation_z': comp_transl_z,
        }

        comp_ffd_block_coefficients = ffd_block_sect_param_list[i].evaluate(sectional_parameters, plot=plot)
        comp_coefficients = ffd_block_list[i].evaluate(comp_ffd_block_coefficients, plot=plot)
        
        
        coefficients_list.append(comp_coefficients)
        # b_spline_names_list.append(comp.b_spline_names)
        
        # geometry.assign_coefficients(coefficients=comp_coefficients, b_spline_names=comp.b_spline_names)


    for i, comp in enumerate(hub_disk_components):
        block_name = f'{comp[0].name}_{comp[1].name}'
        disk = comp[0]
        hub = comp[1]
        blade_1 = comp[2]
        blade_2 = comp[3]

        ffd_block_translation_x_sect_coeffs = outputs_dict[f'{block_name}_translation_x_coefficients']
        ffd_block_translation_x_b_spline = bsp.BSpline(name=f'{block_name}_translation_x_bspline', space=linear_b_spline_curve_2_dof_space, 
                                                coefficients=ffd_block_translation_x_sect_coeffs, num_physical_dimensions=1)
        ffd_block_translation_y_sect_coeffs = outputs_dict[f'{block_name}_translation_y_coefficients']
        ffd_block_translation_y_b_spline = bsp.BSpline(name=f'{block_name}_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                coefficients=ffd_block_translation_y_sect_coeffs, num_physical_dimensions=1)
        ffd_block_translation_z_sect_coeffs = outputs_dict[f'{block_name}_translation_z_coefficients']
        ffd_block_translation_z_b_spline = bsp.BSpline(name=f'{block_name}_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                coefficients=ffd_block_translation_z_sect_coeffs, num_physical_dimensions=1)

        ffd_block_stretch_y_sect_coeffs = outputs_dict[f'{block_name}_stretch_y_coefficients']

        ffd_block_stretch_y_b_spline = bsp.BSpline(name=f'{block_name}_stretch_y_bspline', space=space, coefficients=ffd_block_stretch_y_sect_coeffs, 
                                                        num_physical_dimensions=1)
        
        section_parametric_coordinates = np.linspace(0., 1., ffd_block_sect_param_list_2[i].num_sections).reshape((-1,1))
        comp_transl_x = ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_y = ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
        comp_transl_z = ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)
        comp_stretch_y = ffd_block_stretch_y_b_spline.evaluate(section_parametric_coordinates)

        sectional_parameters = {
                f'{block_name}_translation_x': comp_transl_x,
                f'{block_name}_translation_y': comp_transl_y,
                f'{block_name}_translation_z': comp_transl_z,
                f'{block_name}_stretch_y': comp_stretch_y,
        }


        comp_ffd_block_coefficients = ffd_block_sect_param_list_2[i].evaluate(sectional_parameters, plot=plot)
        comp_coefficients = ffd_block_list_2[i].evaluate(comp_ffd_block_coefficients, plot=plot)
        # geometry.assign_coefficients(coefficients=comp_coefficients, b_spline_names=disk.b_spline_names + hub.b_spline_names)

        coefficients_list.append(comp_coefficients[disk.name + '_coefficients'])
        coefficients_list.append(comp_coefficients[hub.name + '_coefficients'])
        coefficients_list.append(comp_coefficients[blade_1.name + '_coefficients'])
        coefficients_list.append(comp_coefficients[blade_2.name + '_coefficients'])

        # geometry.assign_coefficients(coefficients=comp_coefficients[disk.name + '_coefficients'], b_spline_names=disk.b_spline_names)
        # geometry.assign_coefficients(coefficients=comp_coefficients[hub.name + '_coefficients'], b_spline_names=hub.b_spline_names)
        # geometry.assign_coefficients(coefficients=comp_coefficients[blade_1.name + '_coefficients'], b_spline_names=blade_1.b_spline_names)
        # geometry.assign_coefficients(coefficients=comp_coefficients[blade_2.name + '_coefficients'], b_spline_names=blade_2.b_spline_names)


    # def enforce_kinematic_geometric_constraints(
    #         components_to_be_connected=[rlo_boom, rli_boom, rri_boom, rro_boom, flo_boom, fli_boom, fri_boom, fro_boom],
    #         points_on_components=[rlo_boom_pt, rli_boom_pt, rri_boom_pt, rro_boom_pt, flo_boom_pt, fli_boom_pt, fri_boom_pt, fro_boom_pt],
    #         components_to_connect_to=wing,
    #         prinicpal_parametric_dimension=0,
    #         geometry=geometry,
    #         m3l_model=system_model,
    #         space=constant_b_spline_curve_1_dof_space,
    #         parameterization_solver=lpc_param_solver,
    #         plot=False,
    #         parameterization_inputs : dict = parameterization_inputs,
    # ):
    #     for i, comp in enumerate(components_to_be_connected):
    #         block_name = f'{comp.name}_ffd_block'
    #         ffd_block = lg.construct_ffd_block_around_entities(name=block_name, entities=comp, num_coefficients=(2, 2, 2))
    #         ffd_block_sect_param = VolumeSectionalParameterization(name=f'{block_name}_sect_param', principal_parametric_dimension=prinicpal_parametric_dimension, 
    #                                                             parameterized_points=ffd_block.coefficients,
    #                                                             parameterized_points_shape=ffd_block.coefficients_shape)
            
    #         ffd_block_translation_x_sect_coeffs = m3l_model.create_input(f'{block_name}_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    #         ffd_block_translation_y_sect_coeffs = m3l_model.create_input(f'{block_name}_translation_y_coefficients', shape=(1, ), val=np.array([0]))
    #         ffd_block_translation_z_sect_coeffs = m3l_model.create_input(f'{block_name}_translation_z_coefficients', shape=(1, ), val=np.array([0]))

    #         ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_x', axis=0)
    #         ffd_block_translation_x_b_spline = bsp.BSpline(name=f'{block_name}_translation_x_bspline', space=space, coefficients=ffd_block_translation_x_sect_coeffs, 
    #                                                        num_physical_dimensions=1)
            
    #         ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_y', axis=0)
    #         ffd_block_translation_y_b_spline = bsp.BSpline(name=f'{block_name}_translation_y_bspline', space=space, coefficients=ffd_block_translation_y_sect_coeffs, 
    #                                                        num_physical_dimensions=1)
            
    #         ffd_block_sect_param.add_sectional_translation(name=f'{block_name}_translation_z', axis=0)
    #         ffd_block_translation_z_b_spline = bsp.BSpline(name=f'{block_name}_translation_z_bspline', space=space, coefficients=ffd_block_translation_z_sect_coeffs, 
    #                                                        num_physical_dimensions=1)


    #         section_parametric_coordinates = np.linspace(0., 1., ffd_block_sect_param.num_sections).reshape((-1,1))
    #         comp_transl_x = ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    #         comp_transl_y = ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
    #         comp_transl_z = ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)

    #         sectional_parameters = {
    #                 f'{block_name}_translation_x': comp_transl_x,
    #                 f'{block_name}_translation_y': comp_transl_y,
    #                 f'{block_name}_translation_z': comp_transl_z,
    #         }

    #         comp_ffd_block_coefficients = ffd_block_sect_param.evaluate(sectional_parameters, plot=plot)
    #         comp_coefficients = ffd_block.evaluate(comp_ffd_block_coefficients, plot=plot)
    #         geometry.assign_coefficients(coefficients=comp_coefficients, b_spline_names=comp.b_spline_names)

    #         point_on_component = points_on_components[i]
    #         point_on_component_m3l = geometry.evaluate(comp.project(point_on_component))
    #         point_on_component_to_be_connecte_to_m3l = geometry.evaluate(components_to_connect_to.project(point_on_component))

    #         euclidean_distance = m3l.norm(point_on_component_m3l-point_on_component_to_be_connecte_to_m3l)

    #         parameterization_solver.declare_state(name=f'{block_name}_translation_x_coefficients', state=ffd_block_translation_x_sect_coeffs, penalty_factor=1e-7)
    #         parameterization_solver.declare_state(name=f'{block_name}_translation_y_coefficients', state=ffd_block_translation_y_sect_coeffs, penalty_factor=1e-7)
    #         parameterization_solver.declare_state(name=f'{block_name}_translation_z_coefficients', state=ffd_block_translation_z_sect_coeffs, penalty_factor=1e-7)

    #         parameterization_solver.declare_input(name=f'{block_name}_distance_to_be_enforced', input=euclidean_distance)
    #         component_distance_input = m3l_model.create_input(name=f'{block_name}_distance_to_be_enforced', shape=(1, ), val=euclidean_distance.value)

    #         parameterization_inputs[f'{block_name}_distance_to_be_enforced'] = component_distance_input
            
            
    #         outputs_dict = parameterization_solver.evaluate(inputs=parameterization_inputs, plot=False)

    #         ffd_block_translation_x_sect_coeffs = outputs_dict[f'{block_name}_translation_x_coefficients']
    #         ffd_block_translation_x_b_spline = bsp.BSpline(name=f'{block_name}_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, 
    #                                                 coefficients=ffd_block_translation_x_sect_coeffs, num_physical_dimensions=1)
    #         ffd_block_translation_y_sect_coeffs = outputs_dict[f'{block_name}_translation_y_coefficients']
    #         ffd_block_translation_y_b_spline = bsp.BSpline(name=f'{block_name}_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, 
    #                                                 coefficients=ffd_block_translation_y_sect_coeffs, num_physical_dimensions=1)
    #         ffd_block_translation_z_sect_coeffs = outputs_dict[f'{block_name}_translation_z_coefficients']
    #         ffd_block_translation_z_b_spline = bsp.BSpline(name=f'{block_name}_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, 
    #                                                 coefficients=ffd_block_translation_z_sect_coeffs, num_physical_dimensions=1)

    #         section_parametric_coordinates = np.linspace(0., 1., ffd_block_sect_param.num_sections).reshape((-1,1))
    #         comp_transl_x = ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    #         comp_transl_y = ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
    #         comp_transl_z = ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)

    #         sectional_parameters = {
    #                 f'{block_name}_translation_x': comp_transl_x,
    #                 f'{block_name}_translation_y': comp_transl_y,
    #                 f'{block_name}_translation_z': comp_transl_z,
    #         }

    #         comp_ffd_block_coefficients = ffd_block_sect_param.evaluate(sectional_parameters, plot=plot)
    #         comp_coefficients = ffd_block.evaluate(comp_ffd_block_coefficients, plot=plot)
    #         geometry.assign_coefficients(coefficients=comp_coefficients, b_spline_names=comp.b_spline_names)
    
    coefficients_list.append(pusher_prop_coefficients[pp_disk.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_hub.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_1.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_2.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_3.name + '_coefficients'])
    coefficients_list.append(pusher_prop_coefficients[pp_blade_4.name + '_coefficients'])


    geometry.assign_coefficients(coefficients=coefficients_list, b_spline_names=b_spline_names_list)
    # geometry.assign_coefficients(coefficients=coefficients_list, b_spline_names=[wing.b_spline_names, ])


    # del outputs_dict
    # gc.collect()

    # enforce_kinematic_geometric_constraints()

    # endregion


# Update all the meshes
wing_meshes = wing_meshes.update(geometry=geometry)
tail_meshes = tail_meshes.update(geometry=geometry)
vtail_meshes =  vtail_meshes.update(geometry=geometry)
box_beam_mesh = box_beam_mesh.update(geometry=geometry)

fueslage_top_points_m3l = geometry.evaluate(fueslage_top_points_parametric)
fueslage_bottom_points_m3l = geometry.evaluate(fueslage_bottom_points_parametric)
fuesleage_mesh = m3l.linspace(fueslage_top_points_m3l.reshape((-1, 3)), fueslage_bottom_points_m3l.reshape((-1, 3)),  int(num_fuselage_height + 1))
fuesleage_mesh.description = 'zero_y'

if plot_meshes:
    geometry.plot_meshes(meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh, vtail_meshes.vlm_mesh, box_beam_mesh.beam_nodes, fuesleage_mesh])


pp_mesh = pp_mesh.update(geometry=geometry)
rlo_mesh = rlo_mesh.update(geometry=geometry)
rro_mesh = rro_mesh.update(geometry=geometry)
flo_mesh = flo_mesh.update(geometry=geometry)
fro_mesh = fro_mesh.update(geometry=geometry)
rli_mesh = rli_mesh.update(geometry=geometry)
rri_mesh = rri_mesh.update(geometry=geometry)
fli_mesh = fli_mesh.update(geometry=geometry)
fri_mesh = fri_mesh.update(geometry=geometry)

# chord/ twist profiles
chord_cps_numpy = np.array([0.122222, 0.213889, 0.188426, 0.050926]) * 5
chord_cps_numpy_pusher = np.array([0.122222, 0.213889, 0.188426, 0.050926]) * 4.5

twist_cps_numpy = np.deg2rad(np.linspace(35.000000, 15.000000, 4))
twist_cps_numpy_pusher = np.deg2rad(np.linspace(55.000000, 10.000000, 4))

lower_twist_hover = np.deg2rad(1)
upper_twist_hover = np.deg2rad(60)
lower_twist_pusher = np.deg2rad(5)
upper_twist_pusher = np.deg2rad(85)

flo_blade_chord_bsp_cps = fro_blade_chord_bsp_cps = system_model.create_input('front_outer_blade_chord_cps', val=chord_cps_numpy, dv_flag=True, lower=0.1, upper=1.1)
fli_blade_chord_bsp_cps = fri_blade_chord_bsp_cps = system_model.create_input('front_inner_blade_chord_cps', val=chord_cps_numpy, dv_flag=True, lower=0.1, upper=1.1)
rlo_blade_chord_bsp_cps = rro_blade_chord_bsp_cps = system_model.create_input('rear_outer_blade_chord_cps', val=chord_cps_numpy, dv_flag=True, lower=0.1, upper=1.1)
rli_blade_chord_bsp_cps = rri_blade_chord_bsp_cps = system_model.create_input('rear_inner_blade_chord_cps', val=chord_cps_numpy, dv_flag=True, lower=0.1, upper=1.1)

flo_blade_twist_bsp_cps = fro_blade_twist_bsp_cps = system_model.create_input('front_outer_blade_twist_cps', val=twist_cps_numpy, dv_flag=True, lower=lower_twist_hover, upper=upper_twist_hover)
fli_blade_twist_bsp_cps = fri_blade_twist_bsp_cps = system_model.create_input('front_inner_blade_twist_cps', val=twist_cps_numpy, dv_flag=True, lower=lower_twist_hover, upper=upper_twist_hover)
rlo_blade_twist_bsp_cps = rro_blade_twist_bsp_cps = system_model.create_input('rear_outer_blade_twist_cps', val=twist_cps_numpy, dv_flag=True, lower=lower_twist_hover, upper=upper_twist_hover)
rli_blade_twist_bsp_cps = rri_blade_twist_bsp_cps = system_model.create_input('rear_inner_blade_twist_cps', val=twist_cps_numpy, dv_flag=True, lower=lower_twist_hover, upper=upper_twist_hover)

pusher_chord_bsp_cps = system_model.create_input('pusher_chord_bsp_cps', val=chord_cps_numpy_pusher, dv_flag=True, lower=0.08, upper=1.4)
pusher_twist_bsp_cps = system_model.create_input('pusher_twist_bsp_cps', val=twist_cps_numpy_pusher, dv_flag=True, lower=lower_twist_pusher, upper=upper_twist_pusher)

rlo_mesh.chord_cps = rlo_blade_chord_bsp_cps
rlo_mesh.twist_cps = rlo_blade_twist_bsp_cps

rli_mesh.chord_cps = rli_blade_chord_bsp_cps
rli_mesh.twist_cps = rli_blade_twist_bsp_cps

rri_mesh.chord_cps = rri_blade_chord_bsp_cps
rri_mesh.twist_cps = rri_blade_twist_bsp_cps

rro_mesh.chord_cps = rro_blade_chord_bsp_cps
rro_mesh.twist_cps = rro_blade_twist_bsp_cps

flo_mesh.chord_cps = flo_blade_chord_bsp_cps
flo_mesh.twist_cps = flo_blade_twist_bsp_cps

fli_mesh.chord_cps = fli_blade_chord_bsp_cps
fli_mesh.twist_cps = fli_blade_twist_bsp_cps

fri_mesh.chord_cps = fri_blade_chord_bsp_cps
fri_mesh.twist_cps = fri_blade_twist_bsp_cps

fro_mesh.chord_cps = fro_blade_chord_bsp_cps
fro_mesh.twist_cps = fro_blade_twist_bsp_cps

pp_mesh.chord_cps = pusher_chord_bsp_cps
pp_mesh.twist_cps = pusher_twist_bsp_cps

print('New thrust origins')
print(pp_mesh.thrust_origin)
print(rlo_mesh.thrust_origin)
print(rro_mesh.thrust_origin)
print(flo_mesh.thrust_origin)
print(fro_mesh.thrust_origin)
print(rli_mesh.thrust_origin)
print(rri_mesh.thrust_origin)
print(fli_mesh.thrust_origin)
print(fri_mesh.thrust_origin)

print('New radii')
print(pp_mesh.radius)
print(rlo_mesh.radius)
print(rro_mesh.radius)
print(flo_mesh.radius)
print(fro_mesh.radius)
print(rli_mesh.radius)
print(rri_mesh.radius)
print(fli_mesh.radius)
print(fri_mesh.radius)


# region Drag components 

# Component surface areas
component_list = [rlo_boom, rlo_hub, fuselage, wing, h_tail, vtail, pp_hub, rlo_blade_1]
surface_area_list = compute_component_surface_area(
    component_list=component_list,
    geometry=geometry,
    parametric_mesh_grid_num=20,
    plot=False,
)


# Fuselage
fuselage_l1 = geometry.evaluate(fuselage_l1_parametric).reshape((-1, 3))
fuselage_l2 = geometry.evaluate(fuselage_l2_parametric).reshape((-1, 3))
fuselage_length = m3l.norm(fuselage_l2-fuselage_l1)

fuselage_d1 = geometry.evaluate(fuselage_d1_parametric).reshape((-1, 3))
fuselage_d2=  geometry.evaluate(fuselage_d2_parametric).reshape((-1, 3))
fuselage_diameter = m3l.norm(fuselage_d2-fuselage_d1)

fuselage_drag_comp = DragComponent(
    component_type='fuselage',
    wetted_area=surface_area_list[2],
    characteristic_length=fuselage_length,
    characteristic_diameter=fuselage_diameter,
    Q=2.9,
)


# wing
wing_mid_le = geometry.evaluate(wing_mid_le_parametric).reshape((-1, 3))
wing_mid_te = geometry.evaluate(wing_mid_te_parametric).reshape((-1, 3))
wing_mid_chord_length = m3l.norm(wing_mid_le-wing_mid_te)

wing_le_right_mapped_array = geometry.evaluate(wing_le_right_parametric).reshape((-1, 3))
wing_le_left_mapped_array = geometry.evaluate(wing_le_left__parametric).reshape((-1, 3))
wing_span = m3l.norm(wing_le_left_mapped_array - wing_le_right_mapped_array)

wing_drag_comp = DragComponent(
    component_type='wing',
    wetted_area=surface_area_list[3],
    characteristic_length=wing_mid_chord_length,
    thickness_to_chord=0.16,
    x_cm=0.45,
    Q=1.4,
)

# h tail
h_tail_mid_le = geometry.evaluate(h_tail_mid_le_parametric).reshape((-1, 3))
h_tail_mid_te = geometry.evaluate(h_tail_mid_te_parametric).reshape((-1, 3))
h_tail_chord_length = m3l.norm(h_tail_mid_le-h_tail_mid_te)

h_tail_drag_comp = DragComponent(
    component_type='wing',
    wetted_area=surface_area_list[4],
    characteristic_length=h_tail_chord_length,
    thickness_to_chord=0.15,
    x_cm=0.3,
    Q=1.5,

)

# v tail
vtail_mid_le = geometry.evaluate(vtail_mid_le_parametric).reshape((-1, 3))
vtail_mid_te = geometry.evaluate(vtail_mid_te_parametric).reshape((-1, 3))
vtail_chord_length = m3l.norm(vtail_mid_le-vtail_mid_te)

vtail_drag_comp = DragComponent(
    component_type='wing',
    wetted_area=surface_area_list[5],
    characteristic_length=vtail_chord_length,
    thickness_to_chord=0.115,
    x_cm=0.2,
    Q=1.5,
)

# boom
boom_l1 = geometry.evaluate(boom_l1_parametric).reshape((-1, 3))
boom_l2 = geometry.evaluate(boom_l2_parametric).reshape((-1, 3))
boom_length = m3l.norm(boom_l1-boom_l2)

boom_d1 = geometry.evaluate(boom_d1_parametric).reshape((-1, 3))
boom_d2 = geometry.evaluate(boom_d2_parametric).reshape((-1, 3))
boom_diameter = m3l.norm(boom_d1-boom_d2)

boom_drag_comp = DragComponent(
    component_type='boom',
    wetted_area=surface_area_list[0],
    characteristic_diameter=boom_diameter,
    characteristic_length=boom_length,
    multiplicity=8,
    Q=2.,
)

# lift hubs
hub_l1 = geometry.evaluate(hub_l1_parametric)
hub_l2 = geometry.evaluate(hub_l2_parametric)
hub_length = m3l.norm(hub_l1-hub_l2)

hub_drag_comp = DragComponent(
    component_type='nacelle',
    wetted_area=surface_area_list[1],
    characteristic_diameter=hub_length,
    characteristic_length=hub_length,
    multiplicity=8,
    Q=1,

)


# blade 
blade_tip = geometry.evaluate(blade_tip_parametric)
blade_hub = geometry.evaluate(blade_hub_parametric)

blade_length = m3l.norm(blade_tip-blade_hub)
blade_drag_comp = DragComponent(
    component_type='flat_plate',
    characteristic_length=blade_length,
    wetted_area=surface_area_list[-1],
    multiplicity=16,
    Q=2,

)

drag_comp_list = [wing_drag_comp, fuselage_drag_comp, h_tail_drag_comp, vtail_drag_comp,
                  blade_drag_comp, boom_drag_comp, hub_drag_comp]

S_ref = surface_area_list[3] / 2.1
print('wing area', S_ref.value)
h_tail_area = surface_area_list[4] / 2.1
print('tail area', h_tail_area.value)
v_tail_area = surface_area_list[5] / 2.
print('v_tail area', v_tail_area.value)
# 
wing_AR = wing_span**2 / S_ref
print('wing span', wing_span.value)
print('wing AR', wing_AR.value)
# endregion

# if geometry_dv:
if False:
    radii_front_wing_ratio = (front_outer_radius * 1 + front_inner_radius * 1) / (0.5 * wing_span_input)
    radii_rear_wing_ratio = (rear_outer_radius * 1 + rear_inner_radius * 1) / (0.5 * wing_span_input)
    system_model.register_output(radii_front_wing_ratio * 1)
    system_model.register_output(radii_rear_wing_ratio)
    system_model.add_constraint(radii_front_wing_ratio, equals=0.4)
    system_model.add_constraint(radii_rear_wing_ratio, equals=0.4)


# exit()

# 


# cruise_geometry = geometry.copy()




# 
# # TODO
# # 1) helper function for lines 28-255
# # 2) storing imports/projections: clean up code + location of where to store imports/projections
# # 3) FFDs + inner optimization: @Andrew: finalize API; formally move into lsdo_geo (xyz_to_uvw_indices); think about api changes for free vs prescribed varaibles for inner optimization + FFD

# # 5) Difference between get_primitives and get_geometry_primitives

# # 4) converting lsdo_geo to m3l

# t1 = time.time()

# # region create components
# # Fuselage
# fuselaga_primitive_names = list(spatial_rep.get_geometry_primitives(search_names=['Fuselage_***.main']))
# fuselage = cd.Component(name='fuselage', spatial_representation=spatial_rep, primitive_names=fuselaga_primitive_names)

# weird_nose_hub_primitive_names = list(spatial_rep.get_geometry_primitives(search_names=['EngineGroup_10']))
# weird_nose_hub = cd.Component(name='weird_nose_hub', spatial_representation=spatial_rep, primitive_names=weird_nose_hub_primitive_names)

# # Main wing
# wing_primitive_names = list(spatial_rep.get_geometry_primitives(search_names=['Wing']))
# wing = cd.LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

# # Horizontal tail
# tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
# htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)

# # Vertical tail
# vtail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_2']).keys())
# vtail = cd.LiftingSurface(name='vtail', spatial_representation=spatial_rep, primitive_names=vtail_primitive_names)

# # Rotor: pusher
# pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor-9-disk']).keys())
# pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)

# pp_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 0']).keys())
# pp_blade_1 = cd.Rotor(name='pp_blade_1', spatial_representation=spatial_rep, primitive_names=pp_blade_1_prim_names)

# pp_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 1']).keys())
# pp_blade_2 = cd.Rotor(name='pp_blade_2', spatial_representation=spatial_rep, primitive_names=pp_blade_2_prim_names)

# pp_blade_3_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 2']).keys())
# pp_blade_3 = cd.Rotor(name='pp_blade_3', spatial_representation=spatial_rep, primitive_names=pp_blade_3_prim_names)

# pp_blade_4_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 3']).keys())
# pp_blade_4 = cd.Rotor(name='pp_blade_4', spatial_representation=spatial_rep, primitive_names=pp_blade_4_prim_names)

# pp_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_Hub']).keys())
# pp_hub = cd.Component(name='pp_hub', spatial_representation=spatial_rep, primitive_names=pp_hub_prim_names)

# # Rotor: rear left outer
# rlo_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_disk']).keys())
# rlo_disk = cd.Rotor(name='rlo_disk', spatial_representation=spatial_rep, primitive_names=rlo_disk_prim_names)

# rlo_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_blades, 0']).keys())
# rlo_blade_1 = cd.Rotor(name='rlo_blade_1', spatial_representation=spatial_rep, primitive_names=rlo_blade_1_prim_names)

# rlo_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_blades, 1']).keys())
# rlo_blade_2 = cd.Rotor(name='rlo_blade_2', spatial_representation=spatial_rep, primitive_names=rlo_blade_2_prim_names)

# rlo_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_Hub']).keys())
# rlo_hub = cd.Rotor(name='rlo_hub', spatial_representation=spatial_rep, primitive_names=rlo_hub_prim_names)

# rlo_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_Support']).keys())
# rlo_boom = cd.Component(name='rlo_boom', spatial_representation=spatial_rep, primitive_names=rlo_boom_prim_names)

# # Rotor: rear left inner
# rli_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_disk']).keys())
# rli_disk = cd.Rotor(name='rli_disk', spatial_representation=spatial_rep, primitive_names=rli_disk_prim_names)

# rli_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_blades, 1']).keys())
# rli_blade_1 = cd.Rotor(name='rli_blade_1', spatial_representation=spatial_rep, primitive_names=rli_blade_1_prim_names)

# rli_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_blades, 0']).keys())
# rli_blade_2 = cd.Rotor(name='rli_blade_2', spatial_representation=spatial_rep, primitive_names=rli_blade_2_prim_names)

# rli_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_Hub']).keys())
# rli_hub = cd.Rotor(name='rli_hub', spatial_representation=spatial_rep, primitive_names=rli_hub_prim_names)

# rli_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_Support']).keys())
# rli_boom = cd.Component(name='rli_boom', spatial_representation=spatial_rep, primitive_names=rli_boom_prim_names)


# # Rotor: rear right inner
# rri_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_disk']).keys())
# rri_disk = cd.Rotor(name='rri_disk', spatial_representation=spatial_rep, primitive_names=rri_disk_prim_names)

# rri_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_blades, 0']).keys())
# rri_blade_1 = cd.Rotor(name='rri_blade_1', spatial_representation=spatial_rep, primitive_names=rri_blade_1_prim_names)

# rri_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_blades, 1']).keys())
# rri_blade_2 = cd.Rotor(name='rri_blade_2', spatial_representation=spatial_rep, primitive_names=rri_blade_2_prim_names)

# rri_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_Hub']).keys())
# rri_hub = cd.Rotor(name='rri_hub', spatial_representation=spatial_rep, primitive_names=rri_hub_prim_names)

# rri_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_Support']).keys())
# rri_boom = cd.Component(name='rri_boom', spatial_representation=spatial_rep, primitive_names=rri_boom_prim_names)

# # Rotor: rear right outer
# rro_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_disk']).keys())
# rro_disk = cd.Rotor(name='rro_disk', spatial_representation=spatial_rep, primitive_names=rro_disk_prim_names)

# rro_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_blades, 1']).keys())
# rro_blade_1 = cd.Rotor(name='rro_blade_1', spatial_representation=spatial_rep, primitive_names=rro_blade_1_prim_names)

# rro_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_blades, 0']).keys())
# rro_blade_2 = cd.Rotor(name='rro_blade_2', spatial_representation=spatial_rep, primitive_names=rro_blade_2_prim_names)

# rro_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_Hub']).keys())
# rro_hub = cd.Rotor(name='rro_hub', spatial_representation=spatial_rep, primitive_names=rro_hub_prim_names)

# rro_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_Support']).keys())
# rro_boom = cd.Component(name='rro_boom', spatial_representation=spatial_rep, primitive_names=rro_boom_prim_names)

# # Rotor: front left outer
# flo_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_disk']).keys())
# flo_disk = cd.Rotor(name='flo_disk', spatial_representation=spatial_rep, primitive_names=flo_disk_prim_names)

# flo_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_blades, 1']).keys())
# flo_blade_2 = cd.Rotor(name='flo_blade_1', spatial_representation=spatial_rep, primitive_names=flo_blade_2_prim_names)

# flo_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_blades, 0']).keys())
# flo_blade_1 = cd.Rotor(name='flo_blade_2', spatial_representation=spatial_rep, primitive_names=flo_blade_1_prim_names)

# flo_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_Hub']).keys())
# flo_hub = cd.Rotor(name='flo_hub', spatial_representation=spatial_rep, primitive_names=flo_hub_prim_names)

# flo_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_Support']).keys())
# flo_boom = cd.Component(name='flo_boom', spatial_representation=spatial_rep, primitive_names=flo_boom_prim_names)

# # Rotor: front left inner
# fli_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_disk']).keys())
# fli_disk = cd.Rotor(name='fli_disk', spatial_representation=spatial_rep, primitive_names=fli_disk_prim_names)

# fli_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_blades, 0']).keys())
# fli_blade_2 = cd.Rotor(name='fli_blade_1', spatial_representation=spatial_rep, primitive_names=fli_blade_2_prim_names)

# fli_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_blades, 1']).keys())
# fli_blade_1 = cd.Rotor(name='fli_blade_2', spatial_representation=spatial_rep, primitive_names=fli_blade_1_prim_names)

# fli_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_Hub']).keys())
# fli_hub = cd.Rotor(name='fli_hub', spatial_representation=spatial_rep, primitive_names=fli_hub_prim_names)

# fli_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_Support']).keys())
# fli_boom = cd.Component(name='fli_boom', spatial_representation=spatial_rep, primitive_names=fli_boom_prim_names)

# # Rotor: front right inner
# fri_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_disk']).keys())
# fri_disk = cd.Rotor(name='fri_disk', spatial_representation=spatial_rep, primitive_names=fri_disk_prim_names)

# fri_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_blades, 0']).keys())
# fri_blade_1 = cd.Rotor(name='fri_blade_1', spatial_representation=spatial_rep, primitive_names=fri_blade_1_prim_names)

# fri_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_blades, 1']).keys())
# fri_blade_2 = cd.Rotor(name='fri_blade_2', spatial_representation=spatial_rep, primitive_names=fri_blade_2_prim_names)

# fri_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_Hub']).keys())
# fri_hub = cd.Rotor(name='fri_hub', spatial_representation=spatial_rep, primitive_names=fri_hub_prim_names)

# fri_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_Support']).keys())
# fri_boom = cd.Component(name='fri_boom', spatial_representation=spatial_rep, primitive_names=fri_boom_prim_names)

# # Rotor: front right outer
# fro_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_disk']).keys())
# fro_disk = cd.Rotor(name='fro_disk', spatial_representation=spatial_rep, primitive_names=fro_disk_prim_names)

# fro_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_blades, 0']).keys())
# fro_blade_2 = cd.Rotor(name='fro_blade_1', spatial_representation=spatial_rep, primitive_names=fro_blade_2_prim_names)

# fro_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_blades, 1']).keys())
# fro_blade_1 = cd.Rotor(name='fro_blade_2', spatial_representation=spatial_rep, primitive_names=fro_blade_1_prim_names)

# fro_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_Hub']).keys())
# fro_hub = cd.Rotor(name='fro_hub', spatial_representation=spatial_rep, primitive_names=fro_hub_prim_names)

# fro_boom_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_Support']).keys())
# fro_boom = cd.Component(name='fro_boom', spatial_representation=spatial_rep, primitive_names=fro_boom_prim_names)
# # endregion

# # region add component
# # add components
# lpc_rep.add_component(wing)
# lpc_rep.add_component(htail)
# lpc_rep.add_component(vtail)
# lpc_rep.add_component(fuselage)
# lpc_rep.add_component(weird_nose_hub)

# lpc_rep.add_component(pp_disk)
# lpc_rep.add_component(pp_blade_1)
# lpc_rep.add_component(pp_blade_2)
# lpc_rep.add_component(pp_blade_3)
# lpc_rep.add_component(pp_blade_4)

# lpc_rep.add_component(rlo_disk)
# lpc_rep.add_component(rlo_hub)
# lpc_rep.add_component(rlo_boom)
# lpc_rep.add_component(rlo_blade_1)
# lpc_rep.add_component(rlo_blade_2)

# lpc_rep.add_component(rli_disk)
# lpc_rep.add_component(rli_hub)
# lpc_rep.add_component(rli_boom)
# lpc_rep.add_component(rli_blade_1)
# lpc_rep.add_component(rli_blade_2)

# lpc_rep.add_component(rri_disk)
# lpc_rep.add_component(rri_hub)
# lpc_rep.add_component(rri_boom)
# lpc_rep.add_component(rri_blade_1)
# lpc_rep.add_component(rri_blade_2)

# lpc_rep.add_component(rro_disk)
# lpc_rep.add_component(rro_hub)
# lpc_rep.add_component(rro_boom)
# lpc_rep.add_component(rro_blade_1)
# lpc_rep.add_component(rro_blade_2)

# lpc_rep.add_component(flo_disk)
# lpc_rep.add_component(flo_hub)
# lpc_rep.add_component(flo_boom)
# lpc_rep.add_component(flo_blade_1)
# lpc_rep.add_component(flo_blade_2)

# lpc_rep.add_component(fli_disk)
# lpc_rep.add_component(fli_hub)
# lpc_rep.add_component(fli_boom)
# lpc_rep.add_component(fli_blade_1)
# lpc_rep.add_component(fli_blade_2)

# lpc_rep.add_component(fri_disk)
# lpc_rep.add_component(fri_hub)
# lpc_rep.add_component(fri_boom)
# lpc_rep.add_component(fri_blade_1)
# lpc_rep.add_component(fri_blade_2)

# lpc_rep.add_component(fro_disk)
# lpc_rep.add_component(fro_hub)
# lpc_rep.add_component(fro_boom)
# lpc_rep.add_component(fro_blade_1)
# lpc_rep.add_component(fro_blade_2)
# # endregion

# # region free form deformation

# # region Wing FFD 
# wing_geometry_primitives = wing.get_geometry_primitives()
# wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
# wing_ffd_block = cd.SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
# wing_ffd_block.add_scale_v(name='wing_linear_taper', order=2, num_dof=3, cost_factor=1.)
# wing_ffd_block.add_rotation_u(name='wing_twist_distribution', connection_name='wing_twist_distribution', order=4, num_dof=10, value=np.zeros((10, )))
# wing_ffd_block.add_translation_u(name='wing_span_dof', order=2, num_dof=2, cost_factor=1000) # to use inner optimization, don't specify 'connection_name' and 'val'

# # mapped arrays to get wing span
# left_point = np.array([15., -26., 7.5])
# right_point= np.array([15., 26., 7.5])
# left_point_am = wing.project(left_point, direction=np.array([0., 0., -1.]))
# right_point_am = wing.project(right_point, direction=np.array([0., 0., -1.]))
# wing_span = am.norm(left_point_am - right_point_am)
# lpc_param.add_input('wing_span', wing_span) #, value=80)

# wing_root_chord_le = np.array([8.892, 0., 8.633+0.1])
# wing_root_chord_te = np.array([14.332, 0., 8.429+0.1])
# wing_root_chord_le_am = wing.project(wing_root_chord_le, direction=np.array([0., 0., -1.]), plot=False)
# wing_root_chord_te_am = wing.project(wing_root_chord_te, direction=np.array([0., 0., -1.]), plot=False)
# wing_root_chord = am.norm(am.subtract(wing_root_chord_le_am, wing_root_chord_te_am))

# wing_tip_chord_le_right = np.array([11.541, 24.647, 7.644 + 0.1])
# wing_tip_chord_le_left = np.array([11.541, -24.647, 7.644 + 0.1])
# wing_tip_chord_te_right = np.array([13.495, 24.647, 7.637 + 0.1])
# wing_tip_chord_te_left = np.array([13.495, -24.647, 7.637 + 0.1])
# wing_tip_chord_le_right_am = wing.project(wing_tip_chord_le_right, direction=np.array([0., 0., -1.]), plot=False)
# wing_tip_chord_le_left_am = wing.project(wing_tip_chord_le_left, direction=np.array([0., 0., -1.]), plot=False)
# wing_tip_chord_te_right_am = wing.project(wing_tip_chord_te_right, direction=np.array([0., 0., -1.]), plot=False)
# wing_tip_chord_te_left_am = wing.project(wing_tip_chord_te_left, direction=np.array([0., 0., -1.]), plot=False)

# wing_tip_chord_left = am.norm(wing_tip_chord_le_left_am - wing_tip_chord_te_left_am)
# wing_tip_chord_right = am.norm(wing_tip_chord_le_right_am - wing_tip_chord_te_right_am)

# lpc_param.add_input('wing_tip_chord_left', wing_tip_chord_left)
# lpc_param.add_input('wing_tip_chord_right', wing_tip_chord_right)
# lpc_param.add_input('wing_root_chord', wing_root_chord)
# # endregion

# # region Tail FFD
# htail_geometry_primitives = htail.get_geometry_primitives()
# htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(htail_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
# htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block', primitive=htail_ffd_bspline_volume, embedded_entities=htail_geometry_primitives)
# htail_ffd_block.add_scale_v(name='htail_linear_taper', order=2, num_dof=3, cost_factor=1)
# htail_ffd_block.add_translation_u(name='tail_span_dof', order=2, num_dof=2, cost_factor=1) # to use inner optimization, don't specify 'connection_name' and 'val'
# htail_ffd_block.add_translation_v(name='tail_tv', order=1, num_dof=1, cost_factor=1) # to use inner optimization, don't specify 'connection_name' and 'val'
# htail_ffd_block.add_translation_w(name='tail_tw', order=1, num_dof=1, cost_factor=1) # to use inner optimization, don't specify 'connection_name' and 'val'

# # mapped arrays to get tail span
# left_point = np.array([27., 6.75, 6.])
# right_point= np.array([27., -6.75, 6.])
# left_point_am = htail.project(left_point, direction=np.array([0., 0., -1.]))
# right_point_am = htail.project(right_point, direction=np.array([0., 0., -1.]))
# tail_span = am.norm(left_point_am - right_point_am)
# lpc_param.add_input('tail_span', tail_span) #, value=30)
# # NOTE: line above is performaing actuation- change when actuations are ready

# tail_root_chord_le = np.array([27.428, 0., 8.008])
# tail_root_chord_te = np.array([31.187, 0., 8.008])
# tail_root_chord_le_am = htail.project(tail_root_chord_le, direction=np.array([0., 0., -1.]), plot=False)
# tail_root_chord_te_am = htail.project(tail_root_chord_te, direction=np.array([0., 0., -1.]), plot=False)
# tail_root_chord = am.norm(am.subtract(tail_root_chord_le_am, tail_root_chord_te_am))

# tail_tip_chord_le_right = np.array([27.806, 6.520, 8.008])
# tail_tip_chord_le_left = np.array([27.806, -6.520, 8.008])
# tail_tip_chord_te_right = np.array([30.050, 6.520, 8.008])
# tail_tip_chord_te_left = np.array([30.050, -6.520, 8.008])
# tail_tip_chord_le_right_am = htail.project(tail_tip_chord_le_right, direction=np.array([0., 0., -1.]), plot=False)
# tail_tip_chord_le_left_am = htail.project(tail_tip_chord_le_left, direction=np.array([0., 0., -1.]), plot=False)
# tail_tip_chord_te_right_am = htail.project(tail_tip_chord_te_right, direction=np.array([0., 0., -1.]), plot=False)
# tail_tip_chord_te_left_am = htail.project(tail_tip_chord_te_left, direction=np.array([0., 0., -1.]), plot=False)

# tail_tip_chord_left = am.norm(tail_tip_chord_le_left_am - tail_tip_chord_te_left_am)
# tail_tip_chord_right = am.norm(tail_tip_chord_le_right_am - tail_tip_chord_te_right_am)

# lpc_param.add_input('tail_tip_chord_left', tail_tip_chord_left)
# lpc_param.add_input('tail_tip_chord_right', tail_tip_chord_right)
# lpc_param.add_input('tail_root_chord', tail_root_chord)

# # endregion

# # region fuselage + v-tail FFD + h-tail fuselage connection
# weird_nose_hub_geometry = primitives = weird_nose_hub.get_geometry_primitives()
# fuselage_geometry_primitives = fuselage.get_geometry_primitives()
# fuselage_weird_nose_geom_prims = {**weird_nose_hub_geometry, **fuselage_geometry_primitives}
# fuselage_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(fuselage_weird_nose_geom_prims, num_control_points=(2, 2, 2), order=(2,2,2), xyz_to_uvw_indices=(0,1,2))
# fuselage_ffd_block = cd.SRBGFFDBlock(name='fuselage_ffd_block', primitive=fuselage_ffd_bspline_volume, embedded_entities=fuselage_weird_nose_geom_prims)
# fuselage_ffd_block.add_translation_u(name='fuselage_stretch', order=2, num_dof=2, cost_factor=1) # to use inner optimization, don't specify 'connection_name' and 'val'
# fuselage_ffd_block.add_translation_v(name='fuselage_tv', order=1, num_dof=1) # to use inner optimization, don't specify 'connection_name' and 'val'
# fuselage_ffd_block.add_translation_w(name='fuselage_tw', order=1, num_dof=1) # to use inner optimization, don't specify 'connection_name' and 'val'

# vtail_geometry_primitives = vtail.get_geometry_primitives()
# vtail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(vtail_geometry_primitives, num_control_points=(2, 2, 2), order=(2,2,2), xyz_to_uvw_indices=(2,1,0))
# vtail_ffd_block = cd.SRBGFFDBlock(name='vtail_ffd_block', primitive=vtail_ffd_bspline_volume, embedded_entities=vtail_geometry_primitives)
# vtail_ffd_block.add_translation_u(name='vtail_tu', order=1, num_dof=1) # to use inner optimization, don't specify 'connection_name' and 'val'
# vtail_ffd_block.add_translation_v(name='vtail_tv', order=1, num_dof=1) # to use inner optimization, don't specify 'connection_name' and 'val'
# vtail_ffd_block.add_translation_w(name='vtail_tw', order=1, num_dof=1) # to use inner optimization, don't specify 'connection_name' and 'val'

# # vtail_root_1 = vtail.project(np.array([30.543, 0., 8.231]))
# # vtail_root_2 = vtail.project(np.array([20.843, 0., 8.231]))
# # vtail_root_chord = am.norm(vtail_root_1 - vtail_root_2)

# # vtail_tip_1 = vtail.project(np.array([32.065, 0., 13.911]))
# # vtail_tip_2 = vtail.project(np.array([29.434, 0., 13.911]))
# # vtail_tip_chord = am.norm(vtail_tip_1 - vtail_tip_2)

# # vtail_height_2 = am.norm(vtail_tip_1 - vtail_root_1)

# wing_te_fuselage_am = fuselage.project(np.array([14.332, 0.0, 8.429]))
# fuselage_vtail_le_am = fuselage.project(np.array([30.843, 0.000, 8.231]))

# wing_qc = 0.75 * wing_root_chord_te_am + 0.25 * wing_root_chord_le_am
# wing_mc = 0.50 * wing_root_chord_te_am + 0.50 * wing_root_chord_le_am
# tail_qc = 0.75 * tail_root_chord_te_am + 0.25 * tail_root_chord_le_am

# wing_mc_fuselage_am = fuselage.project(wing_mc.value)
# # print(am.norm(wing_qc-tail_qc).value)
# # 
# lpc_param.add_input('tail_moment_arm', am.norm(wing_qc-tail_qc))

# vtail_le_am = vtail.project(np.array([30.843, 0.000, 8.231]))
# htail_le_am = htail.project(np.array([30.428, 0.0, 8.008]))
# htail_le_fuselage_am = fuselage.project(np.array([27.428, 0.0, 8.008]))

# # lpc_param.add_input('wing_te_fuselage_connection', wing_root_chord_te_am-wing_te_fuselage_am)
# lpc_param.add_input('wing_te_fuselage_connection', wing_mc-wing_mc_fuselage_am)
# lpc_param.add_input('vtail_le_fuselage_connection', vtail_le_am-fuselage_vtail_le_am)
# # lpc_param.add_input('htail_le_fuselage_connection', htail_le_am-htail_le_fuselage_am)
# lpc_param.add_input('htail_le_fuselage_connection', htail_le_am-vtail_le_am)

# fuselage_front = fuselage.project(np.array([1.889, 0.000, 4.249]))
# fuselage_rear = fuselage.project(np.array([31.889, 0.000, 7.798]))
# feslage_length = am.norm(fuselage_front-fuselage_rear)
# lpc_rep.add_output('fuselage_length', feslage_length)

# # vtail_tip_chord_fus_ratio = vtail_tip_chord/feslage_length
# # vtail_root_chord_fus_ratio = vtail_root_chord/feslage_length
# # vtail_height_fus_ratio = vtail_height_2/feslage_length


# # lpc_param.add_input('vtail_tip_chord_fus_ratio', vtail_tip_chord/feslage_length)
# # lpc_param.add_input('vtail_tip_chord_fus_ratio', vtail_root_chord/feslage_length)
# # lpc_param.add_input('vtail_height_fus_ratio', vtail_height_2/feslage_length)

# # endregion

# # region Pusher prop
# pp_disk_geom_prim = pp_disk.get_geometry_primitives()
# pp_hub_geom_prim = pp_hub.get_geometry_primitives()
# pp_hub_disk_geom_prim = {**pp_disk_geom_prim, **pp_hub_geom_prim}
# pp_disk_bspline_vol = cd.create_cartesian_enclosure_volume(pp_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# pp_disk_ffd_block = cd.SRBGFFDBlock(name='pp_disk_ffd_block', primitive=pp_disk_bspline_vol, embedded_entities=pp_hub_disk_geom_prim)
# pp_disk_ffd_block.add_scale_v(name='pp_disk_r1', order=1, num_dof=1)
# pp_disk_ffd_block.add_scale_w(name='pp_disk_r2', order=1, num_dof=1)
# pp_disk_ffd_block.add_translation_u(name='pp_disk_tu', order=1, num_dof=1)
# pp_disk_ffd_block.add_translation_v(name='pp_disk_tv', order=1, num_dof=1)
# pp_disk_ffd_block.add_translation_w(name='pp_disk_tw', order=1, num_dof=1)
# # pp_disk_ffd_block.plot()

# pp_blade_1_geom_prim = pp_blade_1.get_geometry_primitives()
# pp_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1, 2, 0))
# pp_blade_1_ffd_block = cd.SRBGFFDBlock(name='pp_blade_1_ffd_block', primitive=pp_blade_1_bspline_vol, embedded_entities=pp_blade_1_geom_prim)
# pp_blade_1_ffd_block.add_scale_v(name='pp_blade_1_chord', connection_name='pp_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# pp_blade_1_ffd_block.add_rotation_u(name='pp_blade_1_twist', connection_name='pp_blade_1_twist', order=4, num_dof=5, value=np.deg2rad(np.array([0., 0., 0., 0., 0.])))
# pp_blade_1_ffd_block.add_translation_u(name='pp_blade_1_radius', order=2, num_dof=2)
# pp_blade_1_ffd_block.add_translation_v(name='pp_blade_1_transl_v', order=1, num_dof=1)
# pp_blade_1_ffd_block.add_translation_w(name='pp_blade_1_transl_w', order=1, num_dof=1)
# # pp_blade_1_ffd_block.plot()

# pp_blade_2_geom_prim = pp_blade_2.get_geometry_primitives()
# pp_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(2, 1, 0))
# pp_blade_2_ffd_block = cd.SRBGFFDBlock(name='pp_blade_2_ffd_block', primitive=pp_blade_2_bspline_vol, embedded_entities=pp_blade_2_geom_prim)
# pp_blade_2_ffd_block.add_scale_v(name='pp_blade_2_chord', connection_name='pp_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# pp_blade_2_ffd_block.add_rotation_u(name='pp_blade_2_twist', connection_name='pp_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# pp_blade_2_ffd_block.add_translation_u(name='pp_blade_2_radius', order=2, num_dof=2)
# pp_blade_2_ffd_block.add_translation_v(name='pp_blade_2_transl_v', order=1, num_dof=1)
# pp_blade_2_ffd_block.add_translation_w(name='pp_blade_2_transl_w', order=1, num_dof=1)
# # pp_blade_2_ffd_block.plot()


# pp_blade_3_geom_prim = pp_blade_3.get_geometry_primitives()
# pp_blade_3_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_3_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1, 2, 0))
# pp_blade_3_ffd_block = cd.SRBGFFDBlock(name='pp_blade_3_ffd_block', primitive=pp_blade_3_bspline_vol, embedded_entities=pp_blade_3_geom_prim)
# pp_blade_3_ffd_block.add_scale_v(name='pp_blade_3_chord', connection_name='pp_blade_3_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# pp_blade_3_ffd_block.add_rotation_u(name='pp_blade_3_twist', connection_name='pp_blade_3_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# pp_blade_3_ffd_block.add_translation_u(name='pp_blade_3_radius', order=2, num_dof=2)
# pp_blade_3_ffd_block.add_translation_v(name='pp_blade_3_transl_v', order=1, num_dof=1)
# pp_blade_3_ffd_block.add_translation_w(name='pp_blade_3_transl_w', order=1, num_dof=1)
# # pp_blade_3_ffd_block.plot()

# pp_blade_4_geom_prim = pp_blade_4.get_geometry_primitives()
# pp_blade_4_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_4_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(2, 1, 0))
# pp_blade_4_ffd_block = cd.SRBGFFDBlock(name='pp_blade_4_ffd_block', primitive=pp_blade_4_bspline_vol, embedded_entities=pp_blade_4_geom_prim)
# pp_blade_4_ffd_block.add_scale_v(name='pp_blade_4_chord', connection_name='pp_blade_4_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# pp_blade_4_ffd_block.add_rotation_u(name='pp_blade_4_twist', connection_name='pp_blade_4_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# pp_blade_4_ffd_block.add_translation_u(name='pp_blade_4_radius', order=2, num_dof=2)
# pp_blade_4_ffd_block.add_translation_v(name='pp_blade_4_transl_v', order=1, num_dof=1)
# pp_blade_4_ffd_block.add_translation_w(name='pp_blade_4_transl_w', order=1, num_dof=1)
# # pp_blade_4_ffd_block.plot()
# # along z
# y11 = pp_disk.project(np.array([31.94, 0.00, 3.29]), direction=np.array([-1., 0., 0.]), plot=False)
# y12 = pp_disk.project(np.array([31.94, 0.00, 12.29]), direction=np.array([-1., 0., 0.]), plot=False)

# # along y
# y21 = pp_disk.project(np.array([31.94, -4.50, 7.78]), direction=np.array([-1., 0., 0.]), plot=False)
# y22 = pp_disk.project(np.array([31.94, 4.45, 7.77]), direction=np.array([-1., 0., 0.]), plot=False)

# pp_disk_in_plane_y = am.subtract(y11, y12)
# pp_disk_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('pp_in_plane_r1', am.norm(pp_disk_in_plane_y / 2))#, value=2)
# lpc_param.add_input('pp_in_plane_r2', am.norm(pp_disk_in_plane_x / 2))#, value=2)

# pp_hub_center = pp_hub.project(np.array([32.625, 0., 7.79]), direction=np.array([1.1, 0., 0.]), grid_search_n=50, plot=False)

# pp_blade_4_root = pp_blade_4.project(np.array([31.940, 0.0, 6.890]), plot=False)
# pp_blade_4_tip = pp_blade_4.project(np.array([31.940, 0.0, 3.288]),  plot=False)

# pp_blade_3_root = pp_blade_3.project(np.array([31.940, -0.9, 7.790]), plot=False)
# pp_blade_3_tip = pp_blade_3.project(np.array([31.941, -4.500, 7.790]),  plot=False)

# pp_blade_2_root = pp_blade_2.project(np.array([31.940, 0.0, 8.690]), plot=False)
# pp_blade_2_tip = pp_blade_2.project(np.array([31.940, 0.0, 12.292]),  plot=False)

# pp_blade_1_root = pp_blade_1.project(np.array([31.940, 0.900, 7.790]), plot=False)
# pp_blade_1_tip = pp_blade_1.project(np.array([31.941, 4.500, 7.790]), plot=False)

# pp_hub_4_root = pp_hub.project(np.array([31.940, 0.0, 6.890]), plot=False)
# pp_hub_3_root = pp_hub.project(np.array([31.940, -0.9, 7.790]), plot=False)
# pp_hub_2_root = pp_hub.project(np.array([31.940, 0.0, 8.690]), plot=False)
# pp_hub_1_root = pp_hub.project(np.array([31.940, 0.900, 7.790]), plot=False)

# lpc_param.add_input('pp_in_plane_r3', am.norm(pp_blade_4_tip-pp_hub_center)) #, value=2)
# lpc_param.add_input('pp_in_plane_r4', am.norm(pp_blade_3_tip-pp_hub_center)) #, value=2)
# lpc_param.add_input('pp_in_plane_r5', am.norm(pp_blade_2_tip-pp_hub_center)) #, value=2)
# lpc_param.add_input('pp_in_plane_r6', am.norm(pp_blade_1_tip-pp_hub_center)) #, value=2)

# pp_disk_center = pp_disk.project(np.array([32.625, 0., 7.79]),  grid_search_n=50, plot=False)
# lpc_param.add_input('pp_blade_1_hub_connection', pp_hub_1_root-pp_blade_1_root)
# lpc_param.add_input('pp_blade_2_hub_connection', pp_hub_2_root-pp_blade_2_root)
# lpc_param.add_input('pp_blade_3_hub_connection', pp_hub_3_root-pp_blade_3_root)
# lpc_param.add_input('pp_blade_4_hub_connection', pp_hub_4_root-pp_blade_4_root)


# pp_disk_fuselage_connection_am = fuselage.project(pp_disk_center.value) - pp_disk_center
# lpc_param.add_input('pusher_prop_fuselage_connection', pp_disk_fuselage_connection_am)

# # pp_blade_1_ffd_block.setup()
# # affine_section_properties = pp_blade_1_ffd_block.evaluate_affine_section_properties()
# # rotational_section_properties = pp_blade_1_ffd_block.evaluate_rotational_section_properties()
# # affine_ffd_control_points_local_frame = pp_blade_1_ffd_block.evaluate_affine_block_deformations(plot=False)
# # ffd_control_points_local_frame = pp_blade_1_ffd_block.evaluate_rotational_block_deformations(plot=False)
# # ffd_control_points = pp_blade_1_ffd_block.evaluate_control_points(plot=False)
# # updated_geometry = pp_blade_1_ffd_block.evaluate_embedded_entities(plot=False)
# # updated_primitives_names = pp_blade_1_ffd_block.copy()
# # pp_blade_1_ffd_block.plot()

# # 

# # endregion

# # region Rotor: rear left outer
# rlo_disk_geom_prim = rlo_disk.get_geometry_primitives()
# rlo_hub_geom_prim = rlo_hub.get_geometry_primitives()
# rlo_hub_disk_geom_prim = {**rlo_disk_geom_prim, **rlo_hub_geom_prim}
# rlo_disk_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# rlo_disk_ffd_block = cd.SRBGFFDBlock(name='rlo_disk_ffd_block', primitive=rlo_disk_bspline_vol, embedded_entities=rlo_hub_disk_geom_prim)
# rlo_disk_ffd_block.add_scale_v(name='rlo_disk_r1', order=1, num_dof=1)
# rlo_disk_ffd_block.add_scale_w(name='rlo_disk_r2', order=1, num_dof=1)
# rlo_disk_ffd_block.add_translation_u(name='rlo_disk_tu', order=1, num_dof=1)
# rlo_disk_ffd_block.add_translation_v(name='rlo_disk_tv', order=1, num_dof=1)
# rlo_disk_ffd_block.add_translation_w(name='rlo_disk_tw', order=1, num_dof=1)

# # rlo_disk_ffd_block.plot()

# rlo_blade_1_geom_prim = rlo_blade_1.get_geometry_primitives()
# rlo_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rlo_blade_1_ffd_block = cd.SRBGFFDBlock(name='rlo_blade_1_ffd_block', primitive=rlo_blade_1_bspline_vol, embedded_entities=rlo_blade_1_geom_prim)
# rlo_blade_1_ffd_block.add_scale_v(name='rlo_blade_1_chord', connection_name='rlo_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rlo_blade_1_ffd_block.add_rotation_u(name='rlo_blade_1_twist', connection_name='rlo_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rlo_blade_1_ffd_block.add_translation_u(name='rlo_blade_1_radius', order=2, num_dof=2)
# rlo_blade_1_ffd_block.add_translation_v(name='rlo_blade_1_transl_v', order=1, num_dof=1)
# rlo_blade_1_ffd_block.add_translation_w(name='rlo_blade_1_transl_w', order=1, num_dof=1)
# # rlo_blade_1_ffd_block.plot()

# rlo_blade_2_geom_prim = rlo_blade_2.get_geometry_primitives()
# rlo_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rlo_blade_2_ffd_block = cd.SRBGFFDBlock(name='rlo_blade_2_ffd_block', primitive=rlo_blade_2_bspline_vol, embedded_entities=rlo_blade_2_geom_prim)
# rlo_blade_2_ffd_block.add_scale_v(name='rlo_blade_2_chord', connection_name='rlo_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rlo_blade_2_ffd_block.add_rotation_u(name='rlo_blade_2_twist', connection_name='rlo_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rlo_blade_2_ffd_block.add_translation_u(name='rlo_blade_2_radius', order=2, num_dof=2)
# rlo_blade_2_ffd_block.add_translation_v(name='rlo_blade_2_transl_v', order=1, num_dof=1)
# rlo_blade_2_ffd_block.add_translation_w(name='rlo_blade_2_transl_w', order=1, num_dof=1)
# # rlo_blade_2_ffd_block.plot()

# # along y
# y11 = rlo_disk.project(np.array([19.2, -13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rlo_disk.project(np.array([19.2, -23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rlo_disk.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rlo_disk.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rlo_in_plane_y = am.subtract(y12, y11)
# rlo_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('rlo_in_plane_r1', am.norm(rlo_in_plane_y / 2))#, value=2)
# lpc_param.add_input('rlo_in_plane_r2', am.norm(rlo_in_plane_x / 2))#, value=2)

# rlo_hub_center = rlo_hub.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rlo_blade_2_root = rlo_blade_2.project(np.array([18.150, -18.964, 8.972]), direction=np.array([0., 0., -1.]),  plot=False)
# rlo_blade_2_tip = rlo_blade_2.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rlo_blade_1_root = rlo_blade_1.project(np.array([20.325,-18.750, 9.135]), direction=np.array([0., 0., -1.]), plot=False)
# rlo_blade_1_tip = rlo_blade_1.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rlo_hub_2_root = rlo_hub.project(np.array([18.150, -18.964, 8.972]), direction=np.array([0., 0., -1.]), plot=False)
# rlo_hub_1_root = rlo_hub.project(np.array([20.325,-18.750, 9.135]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('rlo_in_plane_r3', am.norm(rlo_blade_2_tip-rlo_hub_center)) #, value=2)
# lpc_param.add_input('rlo_in_plane_r4', am.norm(rlo_blade_1_tip-rlo_hub_center)) #, value=2)

# rlo_disk_center = rlo_disk.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('rlo_blade_1_hub_connection', rlo_hub_1_root-rlo_blade_1_root)
# lpc_param.add_input('rlo_blade_2_hub_connection', rlo_hub_2_root-rlo_blade_2_root)

# # boom
# rlo_boom_geom_prim = rlo_boom.get_geometry_primitives()
# rlo_boom_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# rlo_boom_ffd_block = cd.SRBGFFDBlock(name='rlo_boom_ffd_block', primitive=rlo_boom_bspline_vol, embedded_entities=rlo_boom_geom_prim)
# rlo_boom_ffd_block.add_translation_u(name='rlo_boom_tu', order=1, num_dof=1)
# rlo_boom_ffd_block.add_translation_v(name='rlo_disk_tv', order=1, num_dof=1)
# rlo_boom_ffd_block.add_translation_w(name='rlo_disk_tw', order=1, num_dof=1)


# rlo_boom_am = rlo_boom.project(np.array([12.000, -18.750, 7.613]))
# wing_boom_am = wing.project(np.array([12.000, -18.750, 7.613]))
# wing_boom_connection_am = rlo_boom_am - wing_boom_am

# hub_boom_connection_am = rlo_boom.project(rlo_hub_center.value) - rlo_hub_center

# lpc_param.add_input('rlo_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('rlo_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: rear left inner
# rli_disk_geom_prim = rli_disk.get_geometry_primitives()
# rli_hub_geom_prim = rli_hub.get_geometry_primitives()
# rli_hub_disk_geom_prim = {**rli_disk_geom_prim, **rli_hub_geom_prim}
# rli_disk_bspline_vol = cd.create_cartesian_enclosure_volume(rli_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# rli_disk_ffd_block = cd.SRBGFFDBlock(name='rli_disk_ffd_block', primitive=rli_disk_bspline_vol, embedded_entities=rli_hub_disk_geom_prim)
# rli_disk_ffd_block.add_scale_v(name='rli_disk_r1', order=1, num_dof=1)
# rli_disk_ffd_block.add_scale_w(name='rli_disk_r2', order=1, num_dof=1)
# rli_disk_ffd_block.add_translation_u(name='rli_disk_tu', order=1, num_dof=1)
# rli_disk_ffd_block.add_translation_v(name='rli_disk_tv', order=1, num_dof=1)
# rli_disk_ffd_block.add_translation_w(name='rli_disk_tw', order=1, num_dof=1)

# rli_blade_1_geom_prim = rli_blade_1.get_geometry_primitives()
# rli_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rli_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rli_blade_1_ffd_block = cd.SRBGFFDBlock(name='rli_blade_1_ffd_block', primitive=rli_blade_1_bspline_vol, embedded_entities=rli_blade_1_geom_prim)
# rli_blade_1_ffd_block.add_scale_v(name='rli_blade_1_chord', connection_name='rli_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rli_blade_1_ffd_block.add_rotation_u(name='rli_blade_1_twist', connection_name='rli_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rli_blade_1_ffd_block.add_translation_u(name='rli_blade_1_radius', order=2, num_dof=2)
# rli_blade_1_ffd_block.add_translation_v(name='rli_blade_1_transl_v', order=1, num_dof=1)
# rli_blade_1_ffd_block.add_translation_w(name='rli_blade_1_transl_w', order=1, num_dof=1)
# # rli_blade_1_ffd_block.plot()

# rli_blade_2_geom_prim = rli_blade_2.get_geometry_primitives()
# rli_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rli_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rli_blade_2_ffd_block = cd.SRBGFFDBlock(name='rli_blade_2_ffd_block', primitive=rli_blade_2_bspline_vol, embedded_entities=rli_blade_2_geom_prim)
# rli_blade_2_ffd_block.add_scale_v(name='rli_blade_2_chord', connection_name='rli_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rli_blade_2_ffd_block.add_rotation_u(name='rli_blade_2_twist', connection_name='rli_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rli_blade_2_ffd_block.add_translation_u(name='rli_blade_2_radius', order=2, num_dof=2)
# rli_blade_2_ffd_block.add_translation_v(name='rli_blade_2_transl_v', order=1, num_dof=1)
# rli_blade_2_ffd_block.add_translation_w(name='rli_blade_2_transl_w', order=1, num_dof=1)
# # rli_blade_2_ffd_block.plot()

# # along y
# y11 = rli_disk.project(np.array([18.760, -3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rli_disk.project(np.array([18.760, -13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rli_disk.project(np.array([13.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rli_disk.project(np.array([23.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)

# rli_in_plane_y = am.subtract(y12, y11)
# rli_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('rli_in_plane_r1', am.norm(rli_in_plane_y / 2)) #, value=2.5)
# lpc_param.add_input('rli_in_plane_r2', am.norm(rli_in_plane_x / 2)) #, value=2.5)

# rli_hub_center = rli_hub.project(np.array([18.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rli_blade_2_tip = rli_blade_2.project(np.array([13.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# rli_blade_2_root = rli_blade_2.project(np.array([17.635, -8.478, 9.501]), direction=np.array([0., 0., -1.]),  plot=False)
# rli_blade_1_root = rli_blade_1.project(np.array([19.810,-8.658, 9.237]), direction=np.array([0., 0., -1.]), plot=False)
# rli_blade_1_tip = rli_blade_1.project(np.array([23.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)

# rli_hub_2_root = rli_hub.project(np.array([17.635, -8.478, 9.501]), direction=np.array([0., 0., -1.]), plot=False)
# rli_hub_1_root = rli_hub.project(np.array([19.810,-8.658, 9.237]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('rli_in_plane_r3', am.norm(rli_blade_1_tip-rli_hub_center)) #, value=2.5)
# lpc_param.add_input('rli_in_plane_r4', am.norm(rli_blade_2_tip-rli_hub_center)) #, value=2.5)

# rli_disk_center = rli_disk.project(np.array([18.760, -8.450, 9.996]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('rli_blade_1_hub_connection', rli_hub_2_root-rli_blade_2_root)
# lpc_param.add_input('rli_blade_2_hub_connection', rli_hub_1_root-rli_blade_1_root)

# #boom
# rli_boom_geom_prim = rli_boom.get_geometry_primitives()
# rli_boom_bspline_vol = cd.create_cartesian_enclosure_volume(rli_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# rli_boom_ffd_block = cd.SRBGFFDBlock(name='rli_boom_ffd_block', primitive=rli_boom_bspline_vol, embedded_entities=rli_boom_geom_prim)
# rli_boom_ffd_block.add_translation_u(name='rli_boom_tu', order=1, num_dof=1)
# rli_boom_ffd_block.add_translation_v(name='rli_disk_tv', order=1, num_dof=1)
# rli_boom_ffd_block.add_translation_w(name='rli_disk_tw', order=1, num_dof=1)


# rli_boom_am = rli_boom.project(np.array([11.500, -8.250, 7.898]))
# wing_boom_am = wing.project(np.array([11.500, -8.250, 7.898]))
# wing_boom_connection_am = rli_boom_am - wing_boom_am

# hub_boom_connection_am = rli_boom.project(rli_hub_center.value) - rli_hub_center

# lpc_param.add_input('rli_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('rli_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: rear right inner
# rri_disk_geom_prim = rri_disk.get_geometry_primitives()
# rri_hub_geom_prim = rri_hub.get_geometry_primitives()
# rri_hub_disk_geom_prim = {**rri_disk_geom_prim, **rri_hub_geom_prim}
# rri_disk_bspline_vol = cd.create_cartesian_enclosure_volume(rri_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# rri_disk_ffd_block = cd.SRBGFFDBlock(name='rri_disk_ffd_block', primitive=rri_disk_bspline_vol, embedded_entities=rri_hub_disk_geom_prim)
# rri_disk_ffd_block.add_scale_v(name='rri_disk_r1', order=1, num_dof=1)
# rri_disk_ffd_block.add_scale_w(name='rri_disk_r2', order=1, num_dof=1)
# rri_disk_ffd_block.add_translation_u(name='rri_disk_tu', order=1, num_dof=1)
# rri_disk_ffd_block.add_translation_v(name='rri_disk_tv', order=1, num_dof=1)
# rri_disk_ffd_block.add_translation_w(name='rri_disk_tw', order=1, num_dof=1)

# rri_blade_1_geom_prim = rri_blade_1.get_geometry_primitives()
# rri_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rri_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rri_blade_1_ffd_block = cd.SRBGFFDBlock(name='rri_blade_1_ffd_block', primitive=rri_blade_1_bspline_vol, embedded_entities=rri_blade_1_geom_prim)
# rri_blade_1_ffd_block.add_scale_v(name='rri_blade_1_chord', connection_name='rri_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rri_blade_1_ffd_block.add_rotation_u(name='rri_blade_1_twist', connection_name='rri_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rri_blade_1_ffd_block.add_translation_u(name='rri_blade_1_stretch', order=2, num_dof=2)
# rri_blade_1_ffd_block.add_translation_v(name='rri_blade_1_transl_v', order=1, num_dof=1)
# rri_blade_1_ffd_block.add_translation_w(name='rri_blade_1_transl_w', order=1, num_dof=1)
# # rri_blade_1_ffd_block.plot()

# rri_blade_2_geom_prim = rri_blade_2.get_geometry_primitives()
# rri_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rri_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rri_blade_2_ffd_block = cd.SRBGFFDBlock(name='rri_blade_2_ffd_block', primitive=rri_blade_2_bspline_vol, embedded_entities=rri_blade_2_geom_prim)
# rri_blade_2_ffd_block.add_scale_v(name='rri_blade_2_chord', connection_name='rri_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rri_blade_2_ffd_block.add_rotation_u(name='rri_blade_2_twist', connection_name='rri_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rri_blade_2_ffd_block.add_translation_u(name='rri_blade_2_stretch', order=2, num_dof=2)
# rri_blade_2_ffd_block.add_translation_v(name='rri_blade_2_transl_v', order=1, num_dof=1)
# rri_blade_2_ffd_block.add_translation_w(name='rri_blade_2_transl_w', order=1, num_dof=1)
# # rri_blade_2_ffd_block.plot()
# # along y
# y11 = rri_disk.project(np.array([18.760, 13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rri_disk.project(np.array([18.760, 3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rri_disk.project(np.array([13.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rri_disk.project(np.array([23.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)

# rri_in_plane_y = am.subtract(y12, y11)
# rri_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('rri_in_plane_r1', am.norm(rri_in_plane_y / 2)) #, value=2.5)
# lpc_param.add_input('rri_in_plane_r2', am.norm(rri_in_plane_x / 2)) #, value=2.5)

# rri_hub_center = rri_hub.project(np.array([18.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rri_blade_2_tip = rri_blade_2.project(np.array([13.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# rri_blade_2_root = rri_blade_2.project(np.array([17.635, 8.478, 9.501]), direction=np.array([0., 0., -1.]),  plot=False)
# rri_blade_1_root = rri_blade_1.project(np.array([19.810, 8.658, 9.237]), direction=np.array([0., 0., -1.]), plot=False)
# rri_blade_1_tip = rri_blade_1.project(np.array([23.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)

# rri_hub_2_root = rri_hub.project(np.array([17.635, 8.478, 9.501]), direction=np.array([0., 0., -1.]), plot=False)
# rri_hub_1_root = rri_hub.project(np.array([19.810, 8.658, 9.237]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('rri_in_plane_r3', am.norm(rri_blade_1_tip-rri_hub_center)) #, value=2.5)
# lpc_param.add_input('rri_in_plane_r4', am.norm(rri_blade_2_tip-rri_hub_center)) #, value=2.5)

# rri_disk_center = rri_disk.project(np.array([18.760, 8.450, 9.996]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('rri_blade_1_hub_connection', rri_hub_2_root-rri_blade_2_root)
# lpc_param.add_input('rri_blade_2_hub_connection', rri_hub_1_root-rri_blade_1_root)

# # boom
# rri_boom_geom_prim = rri_boom.get_geometry_primitives()
# rri_boom_bspline_vol = cd.create_cartesian_enclosure_volume(rri_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# rri_boom_ffd_block = cd.SRBGFFDBlock(name='rri_boom_ffd_block', primitive=rri_boom_bspline_vol, embedded_entities=rri_boom_geom_prim)
# rri_boom_ffd_block.add_translation_u(name='rri_boom_tu', order=1, num_dof=1)
# rri_boom_ffd_block.add_translation_v(name='rri_disk_tv', order=1, num_dof=1)
# rri_boom_ffd_block.add_translation_w(name='rri_disk_tw', order=1, num_dof=1)


# rri_boom_am = rri_boom.project(np.array([11.500, 8.250, 8.000]))
# wing_boom_am = wing.project(np.array([11.500, 8.250, 8.000]))
# wing_boom_connection_am = rri_boom_am - wing_boom_am

# hub_boom_connection_am = rri_boom.project(rri_hub_center.value) - rri_hub_center

# lpc_param.add_input('rri_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('rri_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: rear right outer
# rro_disk_geom_prim = rro_disk.get_geometry_primitives()
# rro_hub_geom_prim = rro_hub.get_geometry_primitives()
# rro_hub_disk_geom_prim = {**rro_disk_geom_prim, **rro_hub_geom_prim}
# rro_disk_bspline_vol = cd.create_cartesian_enclosure_volume(rro_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# rro_disk_ffd_block = cd.SRBGFFDBlock(name='rro_disk_ffd_block', primitive=rro_disk_bspline_vol, embedded_entities=rro_hub_disk_geom_prim)
# rro_disk_ffd_block.add_scale_v(name='rro_disk_r1', order=1, num_dof=1)
# rro_disk_ffd_block.add_scale_w(name='rro_disk_r2', order=1, num_dof=1)
# rro_disk_ffd_block.add_translation_u(name='rro_disk_tu', order=1, num_dof=1)
# rro_disk_ffd_block.add_translation_v(name='rro_disk_tv', order=1, num_dof=1)
# rro_disk_ffd_block.add_translation_w(name='rro_disk_tw', order=1, num_dof=1)

# rro_blade_1_geom_prim = rro_blade_1.get_geometry_primitives()
# rro_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rro_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rro_blade_1_ffd_block = cd.SRBGFFDBlock(name='rro_blade_1_ffd_block', primitive=rro_blade_1_bspline_vol, embedded_entities=rro_blade_1_geom_prim)
# rro_blade_1_ffd_block.add_scale_v(name='rro_blade_1_chord', connection_name='rro_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rro_blade_1_ffd_block.add_rotation_u(name='rro_blade_1_twist', connection_name='rro_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rro_blade_1_ffd_block.add_translation_u(name='rro_blade_1_stretch', order=2, num_dof=2)
# rro_blade_1_ffd_block.add_translation_v(name='rro_blade_1_transl_v', order=1, num_dof=1)
# rro_blade_1_ffd_block.add_translation_w(name='rro_blade_1_transl_w', order=1, num_dof=1)

# rro_blade_2_geom_prim = rro_blade_2.get_geometry_primitives()
# rro_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rro_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# rro_blade_2_ffd_block = cd.SRBGFFDBlock(name='rro_blade_2_ffd_block', primitive=rro_blade_2_bspline_vol, embedded_entities=rro_blade_2_geom_prim)
# rro_blade_2_ffd_block.add_scale_v(name='rro_blade_2_chord', connection_name='rro_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# rro_blade_2_ffd_block.add_rotation_u(name='rro_blade_2_twist', connection_name='rro_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# rro_blade_2_ffd_block.add_translation_u(name='rro_blade_2_stretch', order=2, num_dof=2)
# rro_blade_2_ffd_block.add_translation_v(name='rro_blade_2_transl_v', order=1, num_dof=1)
# rro_blade_2_ffd_block.add_translation_w(name='rro_blade_2_transl_w', order=1, num_dof=1)

# rro_origin = rro_disk.project(np.array([19.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]))
# y11 = rro_disk.project(np.array([19.2, 23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rro_disk.project(np.array([19.2, 13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rro_disk.project(np.array([14.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rro_disk.project(np.array([24.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rro_in_plane_y = am.subtract(y12, y11)
# rro_in_plane_x = am.subtract(y21, y22)

# # num_tangential = 25
# # radius = np.linspace(0.2 * 5, 5, 25)
# # angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# # cartesian = np.zeros((25, num_tangential, 3))

# # for i in range(25):
# #     for j in range(num_tangential):
# #         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
# #         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
# #         cartesian[i, j, 2] = 0

# # cartesian_plus_origin = cartesian + rro_origin.value

# # rro_disk_mesh = rro_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)


# # # along y
# # y11 = rro_disk.project(np.array([19.2, 23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # y12 = rro_disk.project(np.array([19.2, 13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # # along x
# # y21 = rro_disk.project(np.array([14.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # y22 = rro_disk.project(np.array([24.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# # rro_in_plane_y = am.subtract(y12, y11)
# # rro_in_plane_x = am.subtract(y21, y22)
# # lpc_param.add_input('rro_disk_mesh', rro_disk_mesh) #, value=2.5)

# lpc_param.add_input('rro_in_plane_r1', am.norm(rro_in_plane_y / 2)) #, value=2.5)
# lpc_param.add_input('rro_in_plane_r2', am.norm(rro_in_plane_x / 2)) #, value=2.5)

# rro_hub_center = rro_hub.project(np.array([19.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rro_blade_2_root = rro_blade_2.project(np.array([18.150, 18.964, 8.972]), direction=np.array([0., 0., -1.]),  plot=False)
# rro_blade_2_tip = rro_blade_2.project(np.array([14.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rro_blade_1_root = rro_blade_1.project(np.array([20.325, 18.750, 9.135]), direction=np.array([0., 0., -1.]), plot=False)
# rro_blade_1_tip = rro_blade_1.project(np.array([24.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rro_hub_2_root = rro_hub.project(np.array([18.150, 18.964, 8.972]), direction=np.array([0., 0., -1.]), plot=False)
# rro_hub_1_root = rro_hub.project(np.array([20.325, 18.750, 9.135]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('rro_in_plane_r3', am.norm(rro_blade_2_tip-rro_hub_center)) #, value=2)
# lpc_param.add_input('rro_in_plane_r4', am.norm(rro_blade_1_tip-rro_hub_center)) #, value=2)

# rro_disk_center = rro_disk.project(np.array([19.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('rro_blade_1_hub_connection', rro_hub_1_root-rro_blade_1_root)
# lpc_param.add_input('rro_blade_2_hub_connection', rro_hub_2_root-rro_blade_2_root)

# # boom
# rro_boom_geom_prim = rro_boom.get_geometry_primitives()
# rro_boom_bspline_vol = cd.create_cartesian_enclosure_volume(rro_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# rro_boom_ffd_block = cd.SRBGFFDBlock(name='rro_boom_ffd_block', primitive=rro_boom_bspline_vol, embedded_entities=rro_boom_geom_prim)
# rro_boom_ffd_block.add_translation_u(name='rro_boom_tu', order=1, num_dof=1)
# rro_boom_ffd_block.add_translation_v(name='rro_disk_tv', order=1, num_dof=1)
# rro_boom_ffd_block.add_translation_w(name='rro_disk_tw', order=1, num_dof=1)


# rro_boom_am = rro_boom.project(np.array([12.000, 18.750, 7.613]))
# wing_boom_am = wing.project(np.array([12.000, 18.750, 7.613]))
# wing_boom_connection_am = rro_boom_am - wing_boom_am

# hub_boom_connection_am = rro_boom.project(rro_hub_center.value) - rro_hub_center

# lpc_param.add_input('rro_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('rro_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: front left outer
# flo_disk_geom_prim = flo_disk.get_geometry_primitives()
# flo_hub_geom_prim = flo_hub.get_geometry_primitives()
# flo_hub_disk_geom_prim = {**flo_disk_geom_prim, **flo_hub_geom_prim}
# flo_disk_bspline_vol = cd.create_cartesian_enclosure_volume(flo_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# flo_disk_ffd_block = cd.SRBGFFDBlock(name='flo_disk_ffd_block', primitive=flo_disk_bspline_vol, embedded_entities=flo_hub_disk_geom_prim)
# flo_disk_ffd_block.add_scale_v(name='flo_disk_r1', order=1, num_dof=1)
# flo_disk_ffd_block.add_scale_w(name='flo_disk_r2', order=1, num_dof=1)
# flo_disk_ffd_block.add_translation_u(name='flo_disk_tu', order=1, num_dof=1)
# flo_disk_ffd_block.add_translation_v(name='flo_disk_tv', order=1, num_dof=1)
# flo_disk_ffd_block.add_translation_w(name='flo_disk_tw', order=1, num_dof=1)

# flo_blade_1_geom_prim = flo_blade_1.get_geometry_primitives()
# flo_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(flo_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# flo_blade_1_ffd_block = cd.SRBGFFDBlock(name='flo_blade_1_ffd_block', primitive=flo_blade_1_bspline_vol, embedded_entities=flo_blade_1_geom_prim)
# flo_blade_1_ffd_block.add_scale_v(name='flo_blade_1_chord', connection_name='flo_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# flo_blade_1_ffd_block.add_rotation_u(name='flo_blade_1_twist', connection_name='flo_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# flo_blade_1_ffd_block.add_translation_u(name='flo_blade_1_radius', order=2, num_dof=2)
# flo_blade_1_ffd_block.add_translation_v(name='flo_blade_1_transl_v', order=1, num_dof=1)
# flo_blade_1_ffd_block.add_translation_w(name='flo_blade_1_transl_w', order=1, num_dof=1)

# flo_blade_2_geom_prim = flo_blade_2.get_geometry_primitives()
# flo_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(flo_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# flo_blade_2_ffd_block = cd.SRBGFFDBlock(name='flo_blade_2_ffd_block', primitive=flo_blade_2_bspline_vol, embedded_entities=flo_blade_2_geom_prim)
# flo_blade_2_ffd_block.add_scale_v(name='flo_blade_2_chord', connection_name='flo_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# flo_blade_2_ffd_block.add_rotation_u(name='flo_blade_2_twist', connection_name='flo_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# flo_blade_2_ffd_block.add_translation_u(name='flo_blade_2_radius', order=2, num_dof=2)
# flo_blade_2_ffd_block.add_translation_v(name='flo_blade_2_transl_v', order=1, num_dof=1)
# flo_blade_2_ffd_block.add_translation_w(name='flo_blade_2_transl_w', order=1, num_dof=1)

# # along y
# y11 = flo_disk.project(np.array([5.070, -13.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = flo_disk.project(np.array([5.070, -23.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = flo_disk.project(np.array([0.070, -18.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = flo_disk.project(np.array([10.070, -18.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)

# flo_in_plane_y = am.subtract(y12, y11)
# flo_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('flo_in_plane_r1', am.norm(flo_in_plane_y / 2))#, value=2)
# lpc_param.add_input('flo_in_plane_r2', am.norm(flo_in_plane_x / 2))#, value=2)

# flo_hub_center = flo_hub.project(np.array([5.070, -18.75, 6.730]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# flo_blade_2_root = flo_blade_2.project(np.array([4.020, -18.764, 6.719]), direction=np.array([0., 0., -1.]),  plot=False)
# flo_blade_2_tip = flo_blade_2.project(np.array([0.044, -18.602, 6.742]), direction=np.array([0., 0., -1.]), plot=False)

# flo_blade_1_root = flo_blade_1.project(np.array([6.120,-18.750, 6.767]), direction=np.array([0., 0., -1.]), plot=False)
# flo_blade_1_tip = flo_blade_1.project(np.array([10.070, -18.75, 6.769]), direction=np.array([0., 0., -1.]), plot=False)

# flo_hub_2_root = flo_hub.project(np.array([4.020, -18.964, 6.719]), direction=np.array([0., 0., -1.]), plot=False)
# flo_hub_1_root = flo_hub.project(np.array([6.120,-18.750, 6.767]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('flo_in_plane_r3', am.norm(flo_blade_2_tip-flo_hub_center)) #, value=2)
# lpc_param.add_input('flo_in_plane_r4', am.norm(flo_blade_1_tip-flo_hub_center)) #, value=2)

# flo_disk_center = flo_disk.project(np.array([5.070, -18.75, 6.730]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('flo_blade_1_hub_connection', flo_hub_1_root-flo_blade_1_root)
# lpc_param.add_input('flo_blade_2_hub_connection', flo_hub_2_root-flo_blade_2_root)

# # boom
# flo_boom_geom_prim = flo_boom.get_geometry_primitives()
# flo_boom_bspline_vol = cd.create_cartesian_enclosure_volume(flo_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# flo_boom_ffd_block = cd.SRBGFFDBlock(name='flo_boom_ffd_block', primitive=flo_boom_bspline_vol, embedded_entities=flo_boom_geom_prim)
# flo_boom_ffd_block.add_translation_u(name='flo_boom_tu', order=1, num_dof=1)
# flo_boom_ffd_block.add_translation_v(name='flo_disk_tv', order=1, num_dof=1)
# flo_boom_ffd_block.add_translation_w(name='flo_disk_tw', order=1, num_dof=1)


# flo_boom_am = flo_boom.project(np.array([12.000, -18.750, 7.613]))
# wing_boom_am = wing.project(np.array([12.000, -18.750, 7.613]))
# wing_boom_connection_am = flo_boom_am - wing_boom_am

# hub_boom_connection_am = flo_boom.project(flo_hub_center.value) - flo_hub_center

# lpc_param.add_input('flo_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('flo_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: front left inner
# fli_disk_geom_prim = fli_disk.get_geometry_primitives()
# fli_hub_geom_prim = fli_hub.get_geometry_primitives()
# fli_hub_disk_geom_prim = {**fli_disk_geom_prim, **fli_hub_geom_prim}
# fli_disk_bspline_vol = cd.create_cartesian_enclosure_volume(fli_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# fli_disk_ffd_block = cd.SRBGFFDBlock(name='fli_disk_ffd_block', primitive=fli_disk_bspline_vol, embedded_entities=fli_hub_disk_geom_prim)
# fli_disk_ffd_block.add_scale_v(name='fli_disk_r1', order=1, num_dof=1)
# fli_disk_ffd_block.add_scale_w(name='fli_disk_r2', order=1, num_dof=1)
# fli_disk_ffd_block.add_translation_u(name='fli_disk_tu', order=1, num_dof=1)
# fli_disk_ffd_block.add_translation_v(name='fli_disk_tv', order=1, num_dof=1)
# fli_disk_ffd_block.add_translation_w(name='fli_disk_tw', order=1, num_dof=1)

# fli_blade_1_geom_prim = fli_blade_1.get_geometry_primitives()
# fli_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(fli_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# fli_blade_1_ffd_block = cd.SRBGFFDBlock(name='fli_blade_1_ffd_block', primitive=fli_blade_1_bspline_vol, embedded_entities=fli_blade_1_geom_prim)
# fli_blade_1_ffd_block.add_scale_v(name='fli_blade_1_chord', connection_name='fli_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# fli_blade_1_ffd_block.add_rotation_u(name='fli_blade_1_twist', connection_name='fli_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# fli_blade_1_ffd_block.add_translation_u(name='fli_blade_1_stretch', order=2, num_dof=2)
# fli_blade_1_ffd_block.add_translation_v(name='fli_blade_1_transl_v', order=1, num_dof=1)
# fli_blade_1_ffd_block.add_translation_w(name='fli_blade_1_transl_w', order=1, num_dof=1)

# fli_blade_2_geom_prim = fli_blade_2.get_geometry_primitives()
# fli_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(fli_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# fli_blade_2_ffd_block = cd.SRBGFFDBlock(name='fli_blade_2_ffd_block', primitive=fli_blade_2_bspline_vol, embedded_entities=fli_blade_2_geom_prim)
# fli_blade_2_ffd_block.add_scale_v(name='fli_blade_2_chord', connection_name='fli_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# fli_blade_2_ffd_block.add_rotation_u(name='fli_blade_2_twist', connection_name='fli_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# fli_blade_2_ffd_block.add_translation_u(name='fli_blade_2_stretch', order=2, num_dof=2)
# fli_blade_2_ffd_block.add_translation_v(name='fli_blade_2_transl_v', order=1, num_dof=1)
# fli_blade_2_ffd_block.add_translation_w(name='fli_blade_2_transl_w', order=1, num_dof=1)

# # along y
# y11 = fli_disk.project(np.array([4.630, -3.179, 7.736]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = fli_disk.project(np.array([4.630, -13.081, 6.344]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = fli_disk.project(np.array([-0.370, -8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = fli_disk.project(np.array([9.630, -8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)

# fli_in_plane_y = am.subtract(y12, y11)
# fli_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('fli_in_plane_r1', am.norm(fli_in_plane_y / 2)) #, value=2.5)
# lpc_param.add_input('fli_in_plane_r2', am.norm(fli_in_plane_x / 2)) #, value=2.5)

# fli_hub_center = fli_hub.project(np.array([4.630, -8.130, 6.669]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fli_blade_2_tip = fli_blade_2.project(np.array([-0.388, -8.280, 7.050]), direction=np.array([0., 0., -1.]), plot=False)
# fli_blade_2_root = fli_blade_2.project(np.array([3.580, -8.169, 7.071]), direction=np.array([0., 0., -1.]),  plot=False)
# fli_blade_1_root = fli_blade_1.project(np.array([5.680,-7.912, 7.059]), direction=np.array([0., 0., -1.]), plot=False)
# fli_blade_1_tip = fli_blade_1.project(np.array([9.656, -7.985, 7.073]), direction=np.array([0., 0., -1.]), plot=False)

# fli_hub_2_root = fli_hub.project(np.array([3.580, -8.169, 7.071]), direction=np.array([0., 0., -1.]), plot=False)
# fli_hub_1_root = fli_hub.project(np.array([5.680,-7.912, 7.059]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('fli_in_plane_r3', am.norm(fli_blade_1_tip-fli_hub_center)) #, value=2.5)
# lpc_param.add_input('fli_in_plane_r4', am.norm(fli_blade_2_tip-fli_hub_center)) #, value=2.5)

# fli_disk_center = fli_disk.project(np.array([4.630, -8.130, 6.669]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('fli_blade_1_hub_connection', fli_hub_2_root-fli_blade_2_root)
# lpc_param.add_input('fli_blade_2_hub_connection', fli_hub_1_root-fli_blade_1_root)

# # boom
# fli_boom_geom_prim = fli_boom.get_geometry_primitives()
# fli_boom_bspline_vol = cd.create_cartesian_enclosure_volume(fli_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# fli_boom_ffd_block = cd.SRBGFFDBlock(name='fli_boom_ffd_block', primitive=fli_boom_bspline_vol, embedded_entities=fli_boom_geom_prim)
# fli_boom_ffd_block.add_translation_u(name='fli_boom_tu', order=1, num_dof=1)
# fli_boom_ffd_block.add_translation_v(name='fli_disk_tv', order=1, num_dof=1)
# fli_boom_ffd_block.add_translation_w(name='fli_disk_tw', order=1, num_dof=1)


# fli_boom_am = fli_boom.project(np.array([11.500, -8.250, 7.898]))
# wing_boom_am = wing.project(np.array([11.500, -8.250, 7.898]))
# wing_boom_connection_am = fli_boom_am - wing_boom_am

# hub_boom_connection_am = fli_boom.project(fli_hub_center.value) - fli_hub_center

# lpc_param.add_input('fli_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('fli_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: front right inner
# fri_disk_geom_prim = fri_disk.get_geometry_primitives()
# fri_hub_geom_prim = fri_hub.get_geometry_primitives()
# fri_hub_disk_geom_prim = {**fri_disk_geom_prim, **fri_hub_geom_prim}
# fri_disk_bspline_vol = cd.create_cartesian_enclosure_volume(fri_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# fri_disk_ffd_block = cd.SRBGFFDBlock(name='fri_disk_ffd_block', primitive=fri_disk_bspline_vol, embedded_entities=fri_hub_disk_geom_prim)
# fri_disk_ffd_block.add_scale_v(name='fri_disk_r1', order=1, num_dof=1)
# fri_disk_ffd_block.add_scale_w(name='fri_disk_r2', order=1, num_dof=1)
# fri_disk_ffd_block.add_translation_u(name='fri_disk_tu', order=1, num_dof=1)
# fri_disk_ffd_block.add_translation_v(name='fri_disk_tv', order=1, num_dof=1)
# fri_disk_ffd_block.add_translation_w(name='fri_disk_tw', order=1, num_dof=1)

# fri_blade_1_geom_prim = fri_blade_1.get_geometry_primitives()
# fri_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(fri_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# fri_blade_1_ffd_block = cd.SRBGFFDBlock(name='fri_blade_1_ffd_block', primitive=fri_blade_1_bspline_vol, embedded_entities=fri_blade_1_geom_prim)
# fri_blade_1_ffd_block.add_scale_v(name='fri_blade_1_chord', connection_name='fri_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# fri_blade_1_ffd_block.add_rotation_u(name='fri_blade_1_twist', connection_name='fri_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# fri_blade_1_ffd_block.add_translation_u(name='fri_blade_1_stretch', order=2, num_dof=2)
# fri_blade_1_ffd_block.add_translation_v(name='fri_blade_1_transl_v', order=1, num_dof=1)
# fri_blade_1_ffd_block.add_translation_w(name='fri_blade_1_transl_w', order=1, num_dof=1)

# fri_blade_2_geom_prim = fri_blade_2.get_geometry_primitives()
# fri_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(fri_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# fri_blade_2_ffd_block = cd.SRBGFFDBlock(name='fri_blade_2_ffd_block', primitive=fri_blade_2_bspline_vol, embedded_entities=fri_blade_2_geom_prim)
# fri_blade_2_ffd_block.add_scale_v(name='fri_blade_2_chord', connection_name='fri_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# fri_blade_2_ffd_block.add_rotation_u(name='fri_blade_2_twist', connection_name='fri_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# fri_blade_2_ffd_block.add_translation_u(name='fri_blade_2_stretch', order=2, num_dof=2)
# fri_blade_2_ffd_block.add_translation_v(name='fri_blade_2_transl_v', order=1, num_dof=1)
# fri_blade_2_ffd_block.add_translation_w(name='fri_blade_2_transl_w', order=1, num_dof=1)


# # along y
# y11 = fri_disk.project(np.array([4.630, 13.081, 6.344]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = fri_disk.project(np.array([4.630, 3.179, 7.736]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = fri_disk.project(np.array([-0.370, 8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = fri_disk.project(np.array([9.630, 8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)

# fri_in_plane_y = am.subtract(y12, y11)
# fri_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('fri_in_plane_r1', am.norm(fri_in_plane_y / 2)) #, value=2.5)
# lpc_param.add_input('fri_in_plane_r2', am.norm(fri_in_plane_x / 2)) #, value=2.5)

# fri_hub_center = fri_hub.project(np.array([4.630, 8.130, 6.669]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fri_blade_2_tip = fri_blade_2.project(np.array([-0.388, 8.280, 7.050]), direction=np.array([0., 0., -1.]), plot=False)
# fri_blade_2_root = fri_blade_2.project(np.array([3.580, 8.169, 7.071]), direction=np.array([0., 0., -1.]),  plot=False)
# fri_blade_1_root = fri_blade_1.project(np.array([5.680,7.912, 7.059]), direction=np.array([0., 0., -1.]), plot=False)
# fri_blade_1_tip = fri_blade_1.project(np.array([9.656, 7.985, 7.073]), direction=np.array([0., 0., -1.]), plot=False)

# fri_hub_2_root = fri_hub.project(np.array([3.580, 8.169, 7.071]), direction=np.array([0., 0., -1.]), plot=False)
# fri_hub_1_root = fri_hub.project(np.array([5.680,7.912, 7.059]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('fri_in_plane_r3', am.norm(fri_blade_1_tip-fri_hub_center)) #, value=2.5)
# lpc_param.add_input('fri_in_plane_r4', am.norm(fri_blade_2_tip-fri_hub_center)) #, value=2.5)

# fri_disk_center = fri_disk.project(np.array([4.630, 8.130, 6.669]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('fri_blade_1_hub_connection', fri_hub_2_root-fri_blade_2_root)
# lpc_param.add_input('fri_blade_2_hub_connection', fri_hub_1_root-fri_blade_1_root)

# # boom
# fri_boom_geom_prim = fri_boom.get_geometry_primitives()
# fri_boom_bspline_vol = cd.create_cartesian_enclosure_volume(fri_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# fri_boom_ffd_block = cd.SRBGFFDBlock(name='fri_boom_ffd_block', primitive=fri_boom_bspline_vol, embedded_entities=fri_boom_geom_prim)
# fri_boom_ffd_block.add_translation_u(name='fri_boom_tu', order=1, num_dof=1)
# fri_boom_ffd_block.add_translation_v(name='fri_disk_tv', order=1, num_dof=1)
# fri_boom_ffd_block.add_translation_w(name='fri_disk_tw', order=1, num_dof=1)


# fri_boom_am = fri_boom.project(np.array([11.500, 8.250, 8.000]))
# wing_boom_am = wing.project(np.array([11.500, 8.250, 8.000]))
# wing_boom_connection_am = fri_boom_am - wing_boom_am

# hub_boom_connection_am = fri_boom.project(fri_hub_center.value) - fri_hub_center

# lpc_param.add_input('fri_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('fri_hub_boom_connection', hub_boom_connection_am)
# # endregion

# # region Rotor: front right outer
# fro_disk_geom_prim = fro_disk.get_geometry_primitives()
# fro_hub_geom_prim = fro_hub.get_geometry_primitives()
# fro_hub_disk_geom_prim = {**fro_disk_geom_prim, **fro_hub_geom_prim}
# fro_disk_bspline_vol = cd.create_cartesian_enclosure_volume(fro_hub_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# fro_disk_ffd_block = cd.SRBGFFDBlock(name='fro_disk_ffd_block', primitive=fro_disk_bspline_vol, embedded_entities=fro_hub_disk_geom_prim)
# fro_disk_ffd_block.add_scale_v(name='fro_disk_r1', order=1, num_dof=1)
# fro_disk_ffd_block.add_scale_w(name='fro_disk_r2', order=1, num_dof=1)
# fro_disk_ffd_block.add_translation_u(name='fro_disk_tu', order=1, num_dof=1)
# fro_disk_ffd_block.add_translation_v(name='fro_disk_tv', order=1, num_dof=1)
# fro_disk_ffd_block.add_translation_w(name='fro_disk_tw', order=1, num_dof=1)

# fro_blade_1_geom_prim = fro_blade_1.get_geometry_primitives()
# fro_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(fro_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# fro_blade_1_ffd_block = cd.SRBGFFDBlock(name='fro_blade_1_ffd_block', primitive=fro_blade_1_bspline_vol, embedded_entities=fro_blade_1_geom_prim)
# fro_blade_1_ffd_block.add_scale_v(name='fro_blade_1_chord', connection_name='fro_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# fro_blade_1_ffd_block.add_rotation_u(name='fro_blade_1_twist', connection_name='fro_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# fro_blade_1_ffd_block.add_translation_u(name='fro_blade_1_radius', order=2, num_dof=2)
# fro_blade_1_ffd_block.add_translation_v(name='fro_blade_1_transl_v', order=1, num_dof=1)
# fro_blade_1_ffd_block.add_translation_w(name='fro_blade_1_transl_w', order=1, num_dof=1)

# fro_blade_2_geom_prim = fro_blade_2.get_geometry_primitives()
# fro_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(fro_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
# fro_blade_2_ffd_block = cd.SRBGFFDBlock(name='fro_blade_2_ffd_block', primitive=fro_blade_2_bspline_vol, embedded_entities=fro_blade_2_geom_prim)
# fro_blade_2_ffd_block.add_scale_v(name='fro_blade_2_chord', connection_name='fro_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
# fro_blade_2_ffd_block.add_rotation_u(name='fro_blade_2_twist', connection_name='fro_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# fro_blade_2_ffd_block.add_translation_u(name='fro_blade_2_radius', order=2, num_dof=2)
# fro_blade_2_ffd_block.add_translation_v(name='fro_blade_2_transl_v', order=1, num_dof=1)
# fro_blade_2_ffd_block.add_translation_w(name='fro_blade_2_transl_w', order=1, num_dof=1)

# # along y
# y11 = fro_disk.project(np.array([5.07, 23.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = fro_disk.project(np.array([5.07, 13.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = fro_disk.project(np.array([0.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = fro_disk.project(np.array([10.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)

# fro_in_plane_y = am.subtract(y12, y11)
# fro_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('fro_in_plane_r1', am.norm(fro_in_plane_y / 2))#, value=2)
# lpc_param.add_input('fro_in_plane_r2', am.norm(fro_in_plane_x / 2))#, value=2)

# fro_hub_center = fro_hub.project(np.array([5.070, 18.75, 6.730]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fro_blade_2_root = fro_blade_2.project(np.array([4.020, 18.764, 6.719]), direction=np.array([0., 0., -1.]),  plot=False)
# fro_blade_2_tip = fro_blade_2.project(np.array([0.044, 18.602, 6.742]), direction=np.array([0., 0., -1.]), plot=False)

# fro_blade_1_root = fro_blade_1.project(np.array([6.120, 18.750, 6.767]), direction=np.array([0., 0., -1.]), plot=False)
# fro_blade_1_tip = fro_blade_1.project(np.array([10.070, 18.75, 6.769]), direction=np.array([0., 0., -1.]), plot=False)

# fro_hub_2_root = fro_hub.project(np.array([4.020, 18.964, 6.719]), direction=np.array([0., 0., -1.]), plot=False)
# fro_hub_1_root = fro_hub.project(np.array([6.120, 18.750, 6.767]), direction=np.array([0., 0., -1.]), plot=False)

# lpc_param.add_input('fro_in_plane_r3', am.norm(fro_blade_2_tip-fro_hub_center)) #, value=2)
# lpc_param.add_input('fro_in_plane_r4', am.norm(fro_blade_1_tip-fro_hub_center)) #, value=2)

# fro_disk_center = fro_disk.project(np.array([5.070, 18.75, 6.730]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('fro_blade_1_hub_connection', fro_hub_1_root-fro_blade_1_root)
# lpc_param.add_input('fro_blade_2_hub_connection', fro_hub_2_root-fro_blade_2_root)

# # boom
# fro_boom_geom_prim = fro_boom.get_geometry_primitives()
# fro_boom_bspline_vol = cd.create_cartesian_enclosure_volume(fro_boom_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(0, 1, 2))
# fro_boom_ffd_block = cd.SRBGFFDBlock(name='fro_boom_ffd_block', primitive=fro_boom_bspline_vol, embedded_entities=fro_boom_geom_prim)
# fro_boom_ffd_block.add_translation_u(name='fro_boom_tu', order=1, num_dof=1)
# fro_boom_ffd_block.add_translation_v(name='fro_disk_tv', order=1, num_dof=1)
# fro_boom_ffd_block.add_translation_w(name='fro_disk_tw', order=1, num_dof=1)


# fro_boom_am = fro_boom.project(np.array([12.000, 18.750, 7.613]))
# wing_boom_am = wing.project(np.array([12.000, 18.750, 7.613]))
# wing_boom_connection_am = fro_boom_am - wing_boom_am

# hub_boom_connection_am = fro_boom.project(fro_hub_center.value) - fro_hub_center

# lpc_param.add_input('fro_wing_boom_connection', wing_boom_connection_am)
# lpc_param.add_input('fro_hub_boom_connection', hub_boom_connection_am)
# # endregion


# ffd_set = cd.SRBGFFDSet(
#     name='ffd_set', 
#     ffd_blocks={
#         wing_ffd_block.name : wing_ffd_block, 
#         htail_ffd_block.name : htail_ffd_block,
#         vtail_ffd_block.name : vtail_ffd_block,
#         fuselage_ffd_block.name : fuselage_ffd_block,
#         pp_disk_ffd_block.name : pp_disk_ffd_block,
#         pp_blade_1_ffd_block.name: pp_blade_1_ffd_block,
#         pp_blade_2_ffd_block.name: pp_blade_2_ffd_block,
#         pp_blade_3_ffd_block.name: pp_blade_3_ffd_block,
#         pp_blade_4_ffd_block.name: pp_blade_4_ffd_block,
#         rlo_blade_1_ffd_block.name : rlo_blade_1_ffd_block,
#         rlo_blade_2_ffd_block.name : rlo_blade_2_ffd_block,
#         rlo_disk_ffd_block.name : rlo_disk_ffd_block,
#         rlo_boom_ffd_block.name : rlo_boom_ffd_block,
#         rli_disk_ffd_block.name : rli_disk_ffd_block,
#         rli_boom_ffd_block.name : rli_boom_ffd_block,
#         rri_disk_ffd_block.name : rri_disk_ffd_block,
#         rri_boom_ffd_block.name : rri_boom_ffd_block,
#         rro_disk_ffd_block.name : rro_disk_ffd_block,
#         rro_boom_ffd_block.name : rro_boom_ffd_block,
#         flo_disk_ffd_block.name : flo_disk_ffd_block,
#         flo_boom_ffd_block.name : flo_boom_ffd_block,
#         fro_disk_ffd_block.name : fro_disk_ffd_block,
#         fro_boom_ffd_block.name : fro_boom_ffd_block,
#         fli_disk_ffd_block.name : fli_disk_ffd_block,
#         fli_boom_ffd_block.name : fli_boom_ffd_block,
#         fri_disk_ffd_block.name : fri_disk_ffd_block,
#         fri_boom_ffd_block.name : fri_boom_ffd_block,
#         rli_blade_1_ffd_block.name : rli_blade_1_ffd_block,
#         rli_blade_2_ffd_block.name : rli_blade_2_ffd_block,
#         rri_blade_1_ffd_block.name : rri_blade_1_ffd_block,
#         rri_blade_2_ffd_block.name : rri_blade_2_ffd_block,
#         rro_blade_1_ffd_block.name : rro_blade_1_ffd_block,
#         rro_blade_2_ffd_block.name : rro_blade_2_ffd_block,
#         flo_blade_1_ffd_block.name : flo_blade_1_ffd_block,
#         flo_blade_2_ffd_block.name : flo_blade_2_ffd_block,
#         fli_blade_1_ffd_block.name : fli_blade_1_ffd_block,
#         fli_blade_2_ffd_block.name : fli_blade_2_ffd_block,
#         fri_blade_1_ffd_block.name : fri_blade_1_ffd_block,
#         fri_blade_2_ffd_block.name : fri_blade_2_ffd_block,
#         fro_blade_1_ffd_block.name : fro_blade_1_ffd_block,
#         fro_blade_2_ffd_block.name : fro_blade_2_ffd_block,
#     }
# )



# # rlo_blade_1_ffd_block.setup()
# # affine_section_properties = rlo_blade_1_ffd_block.evaluate_affine_section_properties()
# # rotational_section_properties = rlo_blade_1_ffd_block.evaluate_rotational_section_properties()
# # affine_ffd_control_points_local_frame = rlo_blade_1_ffd_block.evaluate_affine_block_deformations(plot=False)
# # ffd_control_points_local_frame = rlo_blade_1_ffd_block.evaluate_rotational_block_deformations(plot=False)
# # ffd_control_points = rlo_blade_1_ffd_block.evaluate_control_points(plot=False)
# # updated_geometry = rlo_blade_1_ffd_block.evaluate_embedded_entities(plot=False)
# # updated_primitives_names = rlo_blade_1_ffd_block.copy()
# # rlo_blade_1_ffd_block.plot()
# # rlo_disk_ffd_block

# # ffd_set.setup()
# # affine_section_properties = ffd_set.evaluate_affine_section_properties()
# # rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
# # affine_ffd_control_points_local_frame = ffd_set.evaluate_affine_block_deformations(plot=False)
# # ffd_control_points_local_frame = ffd_set.evaluate_rotational_block_deformations(plot=False)
# # ffd_control_points = ffd_set.evaluate_control_points(plot=False)
# # updated_geometry = ffd_set.evaluate_embedded_entities(plot=False)
# # updated_primitives_names = htail.primitive_names.copy() + wing.primitive_names.copy()

# # spatial_rep.update(updated_geometry, updated_primitives_names)
# # spatial_rep.plot(opacity=0.8, point_types=['control_points'], plot_types=['point_cloud'])
# # spatial_rep.plot(opacity=0.8)


# # endregion

# # region meshes
# num_radial = 25
# num_lifting_line = 10
# off_set = 1
# off_set_long_le = 0.2
# off_set_long_le_tip = 0.40
# off_set_long_te_root = 0.25
# off_set_long_te_tip = 0.285

# # region wing mesh
# plot_wing_mesh = False
# num_spanwise_vlm = 25
# num_spanwise_ml = num_spanwise_vlm - 1
# num_chordwise_vlm = 2

# wing_surface_offset = np.zeros((num_spanwise_vlm, 3))
# wing_surface_offset[2:-2, 0] = 5.5
# wing_surface_offset[[0, -1], 0] = 1.1
# wing_surface_offset[[1, -2], 0] = 3
# wing_surface_offset[:, 2] = -1

# wing_surface_offset_ml = np.zeros((num_spanwise_ml, 3))
# wing_surface_offset_ml[2:-2, 0] = 5.5
# wing_surface_offset_ml[[0, -1], 0] = 2.2
# wing_surface_offset_ml[[1, -2], 0] = 3.2
# wing_surface_offset_ml[:, 2] = -1

# wing_surface_offset_ml_2 = np.zeros((num_spanwise_vlm, 3))
# wing_surface_offset_ml_2[2:-2, 0] = 5.5
# wing_surface_offset_ml_2[[0, -1], 0] = 1.1
# wing_surface_offset_ml_2[[1, -2], 0] = 3
# wing_surface_offset_ml_2[:, 2] = -1

# wing_trailing_edge = wing.project(np.linspace(np.array([15., -26., 7.5]), np.array([15., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), plot=plot_wing_mesh)  
# wing_leading_edge = wing.project(wing_trailing_edge.evaluate() - wing_surface_offset, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=plot_wing_mesh)

# wing_chord_surface = am.linspace(wing_leading_edge, wing_trailing_edge, num_chordwise_vlm)
# wing_upper_surface_wireframe = wing.project(wing_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=plot_wing_mesh)
# wing_lower_surface_wireframe = wing.project(wing_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=50, plot=plot_wing_mesh)
# wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
# wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))

# # OML mesh for ML pressures wing
# wing_trailing_edge_ml_2 = wing.project(np.linspace(np.array([15., -26., 7.5]), np.array([15., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), plot=False)  
# wing_leading_edge_ml_2 = wing.project(wing_trailing_edge_ml_2.evaluate() - wing_surface_offset_ml_2, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# wing_chord_surface_ml_2 = am.linspace(wing_leading_edge_ml_2, wing_trailing_edge_ml_2, num_chordwise_vlm)

# print(wing_trailing_edge.value)
# wing_trailing_edge_array = wing_trailing_edge.value
# wing_trailing_edge_array_ml = np.zeros((num_spanwise_ml, 3))
# for i in range(num_spanwise_vlm-1):
#     x = wing_trailing_edge_array[i, 0] + (wing_trailing_edge_array[i+1, 0] - wing_trailing_edge_array[i, 0])/2
#     y = wing_trailing_edge_array[i, 1] + (wing_trailing_edge_array[i+1, 1] - wing_trailing_edge_array[i, 1])/2
#     z = wing_trailing_edge_array[i, 2] + (wing_trailing_edge_array[i+1, 2] - wing_trailing_edge_array[i, 2])/2
#     wing_trailing_edge_array_ml[i, 0] = x
#     wing_trailing_edge_array_ml[i, 1] = y
#     wing_trailing_edge_array_ml[i, 2] = z

# print(wing_trailing_edge_array_ml)

# wing_trailing_edge_ml = wing.project(wing_trailing_edge_array_ml, direction=np.array([0., 0., -1.]), plot=False)
# wing_leading_edge_ml = wing.project(wing_trailing_edge_ml.evaluate() - wing_surface_offset_ml, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# wing_chord_surface_ml = am.linspace(wing_leading_edge_ml, wing_trailing_edge_ml, num_chordwise_vlm)


# num_ml_points = 100
# chord_surface_ml = am.linspace(wing_leading_edge_ml, wing_trailing_edge_ml, num_ml_points)
# i_vec = np.arange(0, len(chord_surface_ml.value))
# x_range = np.linspace(0, 1, num_ml_points)

# x_interp_x = wing_chord_surface_ml.value[1,:, 0].reshape(num_spanwise_ml, 1) - ((wing_chord_surface_ml.value[1, :, 0] - wing_chord_surface_ml.value[0, :, 0]).reshape(num_spanwise_ml, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
# x_interp_y = wing_chord_surface_ml.value[1,:, 1].reshape(num_spanwise_ml, 1) - ((wing_chord_surface_ml.value[1, :, 1] - wing_chord_surface_ml.value[0, :, 1]).reshape(num_spanwise_ml, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
# x_interp_z = wing_chord_surface_ml.value[1,:, 2].reshape(num_spanwise_ml, 1) - ((wing_chord_surface_ml.value[1, :, 2] - wing_chord_surface_ml.value[0, :, 2]).reshape(num_spanwise_ml, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))

# x_interp_x_2 = wing_chord_surface_ml_2.value[1,:, 0].reshape(num_spanwise_vlm, 1) - ((wing_chord_surface_ml_2.value[1, :, 0] - wing_chord_surface_ml_2.value[0, :, 0]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
# x_interp_y_2 = wing_chord_surface_ml_2.value[1,:, 1].reshape(num_spanwise_vlm, 1) - ((wing_chord_surface_ml_2.value[1, :, 1] - wing_chord_surface_ml_2.value[0, :, 1]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
# x_interp_z_2 = wing_chord_surface_ml_2.value[1,:, 2].reshape(num_spanwise_vlm, 1) - ((wing_chord_surface_ml_2.value[1, :, 2] - wing_chord_surface_ml_2.value[0, :, 2]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))

# new_chord_surface = np.zeros((num_ml_points, num_spanwise_ml, 3))
# new_chord_surface[:, :, 0] = x_interp_x.T
# new_chord_surface[:, :, 1] = x_interp_y.T
# new_chord_surface[:, :, 2] = x_interp_z.T

# new_chord_surface_2 = np.zeros((num_ml_points, num_spanwise_vlm, 3))
# new_chord_surface_2[:, :, 0] = x_interp_x_2.T
# new_chord_surface_2[:, :, 1] = x_interp_y_2.T
# new_chord_surface_2[:, :, 2] = x_interp_z_2.T

# wing_upper_surface_ml = wing.project(new_chord_surface + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=75, plot=False, max_iterations=200)
# wing_lower_surface_ml = wing.project(new_chord_surface - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=100, plot=False, max_iterations=200)

# # wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
# # import copy
# # # Manual surface identification
# # if True:
# #     for key in wing_primitive_names:
# #         surfaces = copy.deepcopy(wing_primitive_names)
# #         surfaces.remove(key)
# #         print(key)
# #         spatial_rep.plot(primitives=surfaces)
# # 
# wing_upper_surface_ml_2 = wing.project(new_chord_surface_2 + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=75, plot=False, max_iterations=200)
# wing_lower_surface_ml_2 = wing.project(new_chord_surface_2 - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=100, plot=False, max_iterations=200)

# print(wing_lower_surface_ml_2.value.shape)
# # 
# # wing_upper_surface_np_array = wing_upper_surface_ml_2.value
# # for i in range(num_spanwise_vlm-1):
# #     for j in range(100):
# #         if j==0:
# #             dy = np.linalg.norm(wing_upper_surface_np_array[j, i+1, :] - wing_upper_surface_np_array[j, i, :])
# #             dx = np.linalg.norm(wing_upper_surface_np_array[j, i, :] +  (wing_upper_surface_np_array[j, i, :] + wing_upper_surface_np_array[j, i, :])/2)
# #             area = dy * dx

# # 
# wing_oml_mesh_name_ml = 'wing_oml_mesh_ML'
# wing_oml_mesh_ml = am.vstack((wing_upper_surface_ml, wing_lower_surface_ml))
# # spatial_rep.plot_meshes([wing_camber_surface])
# # endregion

# # ml_nodes = wing_oml_mesh_ml.value.reshape((num_ml_points*2*num_spanwise_ml, 3), order='F')
# # ml_nodes_correct = np.zeros(ml_nodes.shape)
# # for i in range(num_spanwise_ml):
# #     ml_nodes_correct[i*100:i*100+100] = ml_nodes[i*200:i*200+100]
# #     ml_nodes_correct[i*100+100*num_spanwise_ml:i*100+100*num_spanwise_ml+100] = ml_nodes[i*200+100:i*200+200]
# # ml_nodes = ml_nodes_correct
# # ml_nodes_parametric = wing.project(ml_nodes, properties=['parametric_coordinates'], force_reprojection=False)
# # import pickle

# # with open(PROJECTIONS_FOLDER /  'wing_cp_projections.pcikle', 'wb+') as handle:
# #     pickle.dump(ml_nodes_parametric, handle, protocol=pickle.HIGHEST_PROTOCOL)
# # 
# # region wing beam mesh
# point00 = np.array([12.356, 25.250, 7.618 + 0.1]) 
# point01 = np.array([13.400, 25.250, 7.617 + 0.1]) 
# point10 = np.array([8.892,    0.000, 8.633 + 0.1]) 
# point11 = np.array([14.332,   0.000, 8.439 + 0.1]) 
# point20 = np.array([12.356, -25.250, 7.618 + 0.1]) 
# point21 = np.array([13.400, -25.250, 7.617 + 0.1]) 
# do_plots = False
# num_wing_beam = 21
# leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_beam/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_beam/2+1))), axis=0)
# trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_beam/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_beam/2+1))), axis=0)

# leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots, grid_search_n=50)
# trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots, grid_search_n=50)
# wing_beam = am.linear_combination(leading_edge, trailing_edge, 1, start_weights=np.ones((num_wing_beam, ))*0.75, stop_weights=np.ones((num_wing_beam, ))*0.25)
# width = am.norm((leading_edge - trailing_edge)*0.5)
# # width = am.subtract(leading_edge, trailing_edge)

# if do_plots:
#     spatial_rep.plot_meshes([wing_beam])

# wing_beam = wing_beam.reshape((num_wing_beam , 3))#*0.304
# offset = np.array([0,0,0.5])
# top = wing.project(wing_beam.value+offset, direction=np.array([0., 0., -1.]), plot=do_plots).reshape((num_wing_beam, 3))
# bot = wing.project(wing_beam.value-offset, direction=np.array([0., 0., 1.]), plot=do_plots).reshape((num_wing_beam, 3))
# height = am.norm((top - bot)*1)
# # endregion

# # region tail mesh
# plot_tail_mesh = False
# num_spanwise_vlm = 8
# num_chordwise_vlm = 2
# leading_edge = htail.project(np.linspace(np.array([27., -6.75, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)  # returns MappedArray
# trailing_edge = htail.project(np.linspace(np.array([31.5, -6.75, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)   # returns MappedArray
# tail_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
# htail_upper_surface_wireframe = htail.project(tail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
# htail_lower_surface_wireframe = htail.project(tail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)
# htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1) 
# htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))


# # OML mesh for ML pressures tail
# x_interp_x = tail_chord_surface.value[1,:, 0].reshape(num_spanwise_vlm, 1) - ((tail_chord_surface.value[1, :, 0] - tail_chord_surface.value[0, :, 0]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
# x_interp_y = tail_chord_surface.value[1,:, 1].reshape(num_spanwise_vlm, 1) - ((tail_chord_surface.value[1, :, 1] - tail_chord_surface.value[0, :, 1]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
# x_interp_z = tail_chord_surface.value[1,:, 2].reshape(num_spanwise_vlm, 1) - ((tail_chord_surface.value[1, :, 2] - tail_chord_surface.value[0, :, 2]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))

# new_chord_surface = np.zeros((num_ml_points, num_spanwise_vlm, 3))
# new_chord_surface[:, :, 0] = x_interp_x.T
# new_chord_surface[:, :, 1] = x_interp_y.T
# new_chord_surface[:, :, 2] = x_interp_z.T

# htail_upper_surface_ml = htail.project(new_chord_surface + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=75, plot=False, max_iterations=200)
# htail_lower_surface_ml = htail.project(new_chord_surface - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=100, plot=False, max_iterations=200)

# htail_oml_mesh_name_ml = 'htail_oml_mesh_ML'
# tail_oml_mesh_ml = am.vstack((htail_upper_surface_ml, htail_lower_surface_ml))

# # endregion

# # region pusher prop (pp) meshes
# # disk
# pp_disk_origin = pp_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

# # lifting line mesh
# # blade 2
# b2_le_low_res_numpy =np.linspace(np.array([31.813 - off_set, -0.155 + off_set, 8.735 - 2 * off_set_long_le]), np.array([31.953 - off_set, 0.125 + off_set, 12.290 + 3 * off_set_long_le]), num_lifting_line)
# b2_te_low_res_numpy = np.linspace(np.array([32.322 + off_set, -0.465 - off_set, 8.735 - 2 * off_set_long_te_root]), np.array([31.903 + off_set, -0.376 - off_set, 12.291 + 3 * off_set_long_te_tip]), num_lifting_line)

# pp_blade_2_le_low_res = pp_blade_2.project(b2_le_low_res_numpy, direction=np.array([1., 0., 0.]), grid_search_n=50, plot=False)
# pp_blade_2_te_low_res = pp_blade_2.project(b2_te_low_res_numpy, direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)

# pp_blade_2_chord_surface = am.linspace(pp_blade_2_le_low_res, pp_blade_2_te_low_res, 2)
# pp_blade_2_upper_surface_wireframe = pp_blade_2.project(pp_blade_2_chord_surface.value + np.array([1.01, 0., 0.]), direction=np.array([-1., 0., 0.]), grid_search_n=25)
# pp_blade_2_lower_surface_wireframe = pp_blade_2.project(pp_blade_2_chord_surface.value - np.array([-1.02, 0., 0.]), direction=np.array([1., 0., 0.]), grid_search_n=25)
# pp_blade_2_ll_mesh = am.linspace(pp_blade_2_upper_surface_wireframe, pp_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([pp_blade_2_ll_mesh])

# # blade 4
# b4_le_low_res_numpy = np.linspace(np.array([31.757 - off_set, -0.179 - off_set, 6.890 + 2 * off_set_long_le]), np.array([31.910 - off_set, -0.111 - off_set, 3.290 - 3 * off_set_long_le]), num_lifting_line)
# b4_te_low_res_numpy = np.linspace(np.array([32.123 + off_set, 0.179 + off_set, 6.890 + 2 * off_set_long_le]), np.array([31.970 + off_set, 0.111 + off_set, 3.290 - 3 * off_set_long_le]), num_lifting_line)

# pp_blade_4_le_low_res = pp_blade_4.project(b4_le_low_res_numpy, direction=np.array([1.02, 0., 0.]), grid_search_n=50, plot=False)
# pp_blade_4_te_low_res = pp_blade_4.project(b4_te_low_res_numpy, direction=np.array([-1.01, 0., 0.]), grid_search_n=50, plot=False)

# pp_blade_4_chord_surface = am.linspace(pp_blade_4_le_low_res, pp_blade_4_te_low_res, 2)
# pp_blade_4_upper_surface_wireframe = pp_blade_4.project(pp_blade_4_chord_surface.value + np.array([1.01, 0., 0.]), direction=np.array([-1., 0., 0.]), grid_search_n=25)
# pp_blade_4_lower_surface_wireframe = pp_blade_4.project(pp_blade_4_chord_surface.value - np.array([-1.02, 0., 0.]), direction=np.array([1., 0., 0.]), grid_search_n=25)
# pp_blade_4_ll_mesh = am.linspace(pp_blade_4_upper_surface_wireframe, pp_blade_4_lower_surface_wireframe, 1) 


# # chord 
# b4_le_high_res_numpy = np.linspace(np.array([31.757 - off_set, -0.179 - off_set, 6.890 + 2 * off_set_long_le]), np.array([31.910 - off_set, -0.111 - off_set, 3.290 - 3 * off_set_long_le]), num_radial)
# b4_te_high_res_numpy = np.linspace(np.array([32.123 + off_set, 0.179 + off_set, 6.890 + 2 * off_set_long_le]), np.array([31.970 + off_set, 0.111 + off_set, 3.290 - 3 * off_set_long_le]), num_radial)
# pp_blade_4_le_high_res = pp_blade_4.project(b4_le_high_res_numpy, direction=np.array([1., 0., 0.]), grid_search_n=50, plot=False)
# pp_blade_4_te_high_res = pp_blade_4.project(b4_te_high_res_numpy, direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)
# # pp_chord_length = am.norm(am.subtract(pp_blade_4_le_high_res, pp_blade_4_te_high_res), axes=(1, ))
# pp_chord_length = am.subtract(pp_blade_4_le_high_res, pp_blade_4_te_high_res)

# # twist
# pp_le_proj_disk = pp_disk.project(pp_blade_4_le_high_res.evaluate(), direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)
# pp_te_proj_disk = pp_disk.project(pp_blade_4_te_high_res.evaluate(), direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)

# pp_v_dist_le = am.subtract(pp_blade_4_le_high_res, pp_le_proj_disk)
# pp_v_dist_te = am.subtract(pp_blade_4_te_high_res, pp_te_proj_disk)
# pp_tot_v_dist = am.subtract(pp_v_dist_te, pp_v_dist_le)
# # endregion

# # region rear left outer (rlo) rotor meshes
# # disk
# rlo_origin = rlo_disk.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]))
# # along y
# y11 = rlo_disk.project(np.array([19.2, -13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rlo_disk.project(np.array([19.2, -23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rlo_disk.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rlo_disk.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rlo_in_plane_y = am.subtract(y12, y11)
# rlo_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + rlo_origin.value

# rlo_disk_mesh = rlo_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)


# # lifting line mesh
# # blade 1
# b1_le_low_res_numpy = np.linspace(np.array([20.20 - off_set_long_le, -18.967 - off_set, 9.062 + off_set]), np.array([24.200 + off_set_long_le_tip, -18.903 - off_set, 9.003 + off_set]), num_lifting_line)
# b1_te_low_res_numpy = np.linspace(np.array([20.20 - off_set_long_te_root, -18.099 + off_set, 8.857 + off_set]), np.array([24.201 + off_set_long_te_tip, -18.292 + off_set, 9.031 + off_set]), num_lifting_line)

# rlo_blade_1_le_low_res = rlo_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rlo_blade_1_te_low_res = rlo_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rlo_blade_1_chord_surface = am.linspace(rlo_blade_1_le_low_res, rlo_blade_1_te_low_res, 2)
# rlo_blade_1_upper_surface_wireframe = rlo_blade_1.project(rlo_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rlo_blade_1_lower_surface_wireframe = rlo_blade_1.project(rlo_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# rlo_blade_1_ll_mesh = am.linspace(rlo_blade_1_upper_surface_wireframe, rlo_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_le_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, -18.533 + off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le_tip, -18.597 + off_set, 9.003 + off_set]), num_lifting_line)
# b2_te_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_te_root, -19.401 - off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, -19.208 - off_set, 9.032 + off_set]), num_lifting_line)

# rlo_blade_2_le_low_res = rlo_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rlo_blade_2_te_low_res = rlo_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rlo_blade_2_chord_surface = am.linspace(rlo_blade_2_le_low_res, rlo_blade_2_te_low_res, 2)
# rlo_blade_2_upper_surface_wireframe = rlo_blade_2.project(rlo_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rlo_blade_2_lower_surface_wireframe = rlo_blade_2.project(rlo_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# rlo_blade_2_ll_mesh = am.linspace(rlo_blade_2_upper_surface_wireframe, rlo_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([rlo_blade_1_ll_mesh, rlo_blade_2_ll_mesh])

# # chord 
# b2_le_high_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, -18.533 + off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le_tip, -18.597 + off_set, 9.003 + off_set]), num_radial)
# b2_te_high_res_numpy = np.linspace(np.array([18.150 , -19.401 - off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, -19.208 - off_set, 9.032 + off_set]), num_radial)
# rlo_blade_2_le_high_res = rlo_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rlo_blade_2_te_high_res = rlo_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.01]), grid_search_n=200, plot=False)
# # rlo_chord_length = am.norm(am.subtract(rlo_blade_2_le_high_res, rlo_blade_2_te_high_res), axes=(1, ))
# rlo_chord_length = am.subtract(rlo_blade_2_le_high_res, rlo_blade_2_te_high_res)

# # twist
# rlo_le_proj_disk = rlo_disk.project(rlo_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rlo_te_proj_disk = rlo_disk.project(rlo_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rlo_v_dist_le = am.subtract(rlo_blade_2_le_high_res, rlo_le_proj_disk)
# rlo_v_dist_te = am.subtract(rlo_blade_2_te_high_res, rlo_te_proj_disk)
# rlo_tot_v_dist = am.subtract(rlo_v_dist_le, rlo_v_dist_te)


# # 
# # endregion

# # region rear right outer (rro) rotor meshes
# # disk
# rro_origin = rro_disk.project(np.array([19.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]))
# y11 = rro_disk.project(np.array([19.2, 23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rro_disk.project(np.array([19.2, 13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rro_disk.project(np.array([14.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rro_disk.project(np.array([24.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

# rro_in_plane_y = am.subtract(y12, y11)
# rro_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + rro_origin.value

# rro_disk_mesh = rro_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)

# # lifting line mesh
# # blade 1
# b1_le_low_res_numpy = np.linspace(np.array([20.250 - off_set_long_le, 18.967 + off_set, 9.062 + off_set]), np.array([24.200 + off_set_long_le, 18.903 + off_set, 9.003 + off_set]), num_lifting_line)
# b1_te_low_res_numpy = np.linspace(np.array([20.250 - off_set_long_te_root, 18.099 - off_set, 8.857 + off_set]), np.array([24.201 + off_set_long_te_tip, 18.292 - off_set, 9.031 + off_set]), num_lifting_line)

# rro_blade_1_le_low_res = rro_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rro_blade_1_te_low_res = rro_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rro_blade_1_chord_surface = am.linspace(rro_blade_1_le_low_res, rro_blade_1_te_low_res, 2)
# rro_blade_1_upper_surface_wireframe = rro_blade_1.project(rro_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rro_blade_1_lower_surface_wireframe = rro_blade_1.project(rro_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# rro_blade_1_ll_mesh = am.linspace(rro_blade_1_upper_surface_wireframe, rro_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_le_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, 18.533 - off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le, 18.597 - off_set, 9.003 + off_set]), num_lifting_line)
# b2_te_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_te_root, 19.401 + off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, 19.208 + off_set, 9.032 + off_set]), num_lifting_line)

# rro_blade_2_le_low_res = rro_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rro_blade_2_te_low_res = rro_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rro_blade_2_chord_surface = am.linspace(rro_blade_2_le_low_res, rro_blade_2_te_low_res, 2)
# rro_blade_2_upper_surface_wireframe = rro_blade_2.project(rro_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rro_blade_2_lower_surface_wireframe = rro_blade_2.project(rro_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# rro_blade_2_ll_mesh = am.linspace(rro_blade_2_upper_surface_wireframe, rro_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([rro_blade_1_ll_mesh, rro_blade_2_ll_mesh, rlo_blade_1_ll_mesh, rlo_blade_2_ll_mesh])

# # chord 
# b2_le_high_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, 18.533 - off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le, 18.597 - off_set, 9.003 + off_set]), num_radial)
# b2_te_high_res_numpy = np.linspace(np.array([18.150 + 0., 19.401 + off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, 19.208 + off_set, 9.032 + off_set]), num_radial)
# rro_blade_2_le_high_res = rro_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rro_blade_2_te_high_res = rro_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# # rro_chord_length = am.norm(am.subtract(rro_blade_2_le_high_res, rro_blade_2_te_high_res), axes=(1, ))
# rro_chord_length = am.subtract(rro_blade_2_le_high_res, rro_blade_2_te_high_res)

# # twist
# rro_te_proj_disk = rro_disk.project(rro_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rro_le_proj_disk = rro_disk.project(rro_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rro_v_dist_le = am.subtract(rro_blade_2_le_high_res, rro_le_proj_disk)
# rro_v_dist_te = am.subtract(rro_blade_2_te_high_res, rro_te_proj_disk)
# rro_tot_v_dist = am.subtract(rro_v_dist_le, rro_v_dist_te)
# # 

# # endregion

# # region front left outer (flo) rotor meshes
# # disk
# flo_origin = flo_disk.project(np.array([5.07, -18.75, 6.73]), direction=np.array([0., 0., -1.]))
# # along y
# y11 = flo_disk.project(np.array([5.070, -13.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = flo_disk.project(np.array([5.070, -23.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = flo_disk.project(np.array([0.070, -18.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = flo_disk.project(np.array([10.070, -18.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)

# flo_in_plane_y = am.subtract(y12, y11)
# flo_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + flo_origin.value

# flo_disk_mesh = flo_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)

# # lifting line mesh
# # blade 1
# b1_te_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_le, -18.533 + off_set, 6.782 + off_set]), np.array([10.070 + off_set_long_le, -18.597 + off_set, 6.723 + off_set]), num_lifting_line)
# b1_le_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_te_root, -19.401 - off_set, 6.577 + off_set]), np.array([10.071 + off_set_long_te_tip, -19.208 - off_set, 6.751 + off_set]), num_lifting_line)

# flo_blade_1_te_low_res = flo_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# flo_blade_1_le_low_res = flo_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# flo_blade_1_chord_surface = am.linspace(flo_blade_1_le_low_res, flo_blade_1_te_low_res, 2)
# flo_blade_1_upper_surface_wireframe = flo_blade_1.project(flo_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# flo_blade_1_lower_surface_wireframe = flo_blade_1.project(flo_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# flo_blade_1_ll_mesh = am.linspace(flo_blade_1_upper_surface_wireframe, flo_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_te_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, -18.967 - off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, -18.903 - off_set, 6.723 + off_set]), num_lifting_line)
# b2_le_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, -18.099 + off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, -18.292 + off_set, 6.752 + off_set]), num_lifting_line)

# flo_blade_2_le_low_res = flo_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# flo_blade_2_te_low_res = flo_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# flo_blade_2_chord_surface = am.linspace(flo_blade_2_le_low_res, flo_blade_2_te_low_res, 2)
# flo_blade_2_upper_surface_wireframe = flo_blade_2.project(flo_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# flo_blade_2_lower_surface_wireframe = flo_blade_2.project(flo_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# flo_blade_2_ll_mesh = am.linspace(flo_blade_2_upper_surface_wireframe, flo_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([flo_blade_1_ll_mesh, flo_blade_2_ll_mesh])

# # chord 
# b2_te_high_res_numpy = np.linspace(np.array([4.020 - 0.1, -18.967 - off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, -18.903 - off_set, 6.723 + off_set]), num_radial)
# b2_le_high_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, -18.099 + off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, -18.292 + off_set, 6.752 + off_set]), num_radial)
# flo_blade_2_le_high_res = flo_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# flo_blade_2_te_high_res = flo_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# # flo_chord_length = am.norm(am.subtract(flo_blade_2_le_high_res, flo_blade_2_te_high_res), axes=(1, ))
# flo_chord_length = am.subtract(flo_blade_2_le_high_res, flo_blade_2_te_high_res)

# # twist
# flo_le_proj_disk = flo_disk.project(flo_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# flo_te_proj_disk = flo_disk.project(flo_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# flo_v_dist_le = am.subtract(flo_blade_2_le_high_res, flo_le_proj_disk)
# flo_v_dist_te = am.subtract(flo_blade_2_te_high_res, flo_te_proj_disk)
# flo_tot_v_dist = am.subtract(flo_v_dist_le, flo_v_dist_te)
# # endregion

# # region front right outer (fro) rotor meshes
# # disk

# fro_origin = fro_disk.project(np.array([5.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]))

# # along y
# y11 = fro_disk.project(np.array([5.07, 23.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = fro_disk.project(np.array([5.07, 13.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = fro_disk.project(np.array([0.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = fro_disk.project(np.array([10.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)

# fro_in_plane_y = am.subtract(y12, y11)
# fro_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + fro_origin.value

# fro_disk_mesh = fro_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)

# # lifting line mesh
# # blade 1
# b1_te_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_le, 18.533 - off_set, 6.782 + off_set]), np.array([10.070 + off_set_long_le, 18.597 - off_set, 6.723 + off_set]), num_lifting_line)
# b1_le_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_te_root, 19.401 + off_set, 6.577 + off_set]), np.array([10.071 + off_set_long_te_tip, 19.208 + off_set, 6.751 + off_set]), num_lifting_line)

# fro_blade_1_le_low_res = fro_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fro_blade_1_te_low_res = fro_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fro_blade_1_chord_surface = am.linspace(fro_blade_1_le_low_res, fro_blade_1_te_low_res, 2)
# fro_blade_1_upper_surface_wireframe = fro_blade_1.project(fro_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# fro_blade_1_lower_surface_wireframe = fro_blade_1.project(fro_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# fro_blade_1_ll_mesh = am.linspace(fro_blade_1_upper_surface_wireframe, fro_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_te_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, 18.967 + off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, 18.903 + off_set, 6.723 + off_set]), num_lifting_line)
# b2_le_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, 18.099 - off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, 18.292 - off_set, 6.752 + off_set]), num_lifting_line)

# fro_blade_2_le_low_res = fro_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fro_blade_2_te_low_res = fro_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fro_blade_2_chord_surface = am.linspace(fro_blade_2_le_low_res, fro_blade_2_te_low_res, 2)
# fro_blade_2_upper_surface_wireframe = fro_blade_2.project(fro_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# fro_blade_2_lower_surface_wireframe = fro_blade_2.project(fro_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# fro_blade_2_ll_mesh = am.linspace(fro_blade_2_upper_surface_wireframe, fro_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([fro_blade_1_ll_mesh, fro_blade_2_ll_mesh])

# # chord 
# b2_te_high_res_numpy = np.linspace(np.array([4.020 - 0.1, 18.967 + off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, 18.903 + off_set, 6.723 + off_set]), num_radial)
# b2_le_high_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, 18.099 - off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, 18.292 - off_set, 6.752 + off_set]), num_radial)
# fro_blade_2_le_high_res = fro_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fro_blade_2_te_high_res = fro_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# # fro_chord_length = am.norm(am.subtract(fro_blade_2_le_high_res, fro_blade_2_te_high_res), axes=(1, ))
# fro_chord_length = am.subtract(fro_blade_2_le_high_res, fro_blade_2_te_high_res)

# # twist
# fro_le_proj_disk = fro_disk.project(fro_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fro_te_proj_disk = fro_disk.project(fro_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fro_v_dist_le = am.subtract(fro_blade_2_le_high_res, fro_le_proj_disk)
# fro_v_dist_te = am.subtract(fro_blade_2_te_high_res, fro_te_proj_disk)
# fro_tot_v_dist = am.subtract(fro_v_dist_le, fro_v_dist_te)
# # endregion

# # region rear left inner (rli) rotor meshes
# # disk
# y11 = rli_disk.project(np.array([18.760, -3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rli_disk.project(np.array([18.760, -13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
# y21 = rli_disk.project(np.array([13.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rli_disk.project(np.array([23.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# rli_in_plane_y = am.subtract(y12, y11)
# rli_in_plane_x = am.subtract(y21, y22)
# rli_origin = rli_disk.project(np.array([18.760, -8.537, 9.919]), direction=np.array([0., 0., -1.]))

# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + rli_origin.value

# rli_disk_mesh = rli_disk.project(cartesian_plus_origin, plot=False)
# # lifting line mesh
# # blade 1
# b1_le_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_le, -8.243 + off_set, 9.381 + off_set]), np.array([23.760 + off_set_long_le, -8.298 + off_set, 9.315 + off_set]), num_lifting_line)
# b1_te_low_res_numpy = np.linspace(np.array([19.810 - 0, -9.073 - off_set, 9.058 + off_set]), np.array([23.761 + off_set_long_te_tip, -8.906 - off_set, 9.257 + off_set]), num_lifting_line)

# rli_blade_1_le_low_res = rli_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rli_blade_1_te_low_res = rli_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rli_blade_1_chord_surface = am.linspace(rli_blade_1_le_low_res, rli_blade_1_te_low_res, 2)
# rli_blade_1_upper_surface_wireframe = rli_blade_1.project(rli_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rli_blade_1_lower_surface_wireframe = rli_blade_1.project(rli_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# rli_blade_1_ll_mesh = am.linspace(rli_blade_1_upper_surface_wireframe, rli_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_le_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, -8.672 - off_set, 9.321+ off_set]), np.array([13.760 - off_set_long_le, -8.600 - off_set, 9.003 + off_set]), num_lifting_line)
# b2_te_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_te_root, -7.784 + off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, -8.000 + off_set, 9.385 + off_set]), num_lifting_line)

# rli_blade_2_le_low_res = rli_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rli_blade_2_te_low_res = rli_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rli_blade_2_chord_surface = am.linspace(rli_blade_2_le_low_res, rli_blade_2_te_low_res, 2)
# rli_blade_2_upper_surface_wireframe = rli_blade_2.project(rli_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rli_blade_2_lower_surface_wireframe = rli_blade_2.project(rli_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# rli_blade_2_ll_mesh = am.linspace(rli_blade_2_upper_surface_wireframe, rli_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([rli_blade_1_ll_mesh, rli_blade_2_ll_mesh])

# # chord 
# b2_le_high_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, -8.672 - off_set, 9.321+ off_set]), np.array([13.760 - off_set_long_le, -8.600 - off_set, 9.003 + off_set]), num_radial)
# b2_te_high_res_numpy = np.linspace(np.array([17.710 + 0, -7.784 + off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, -8.000 + off_set, 9.385 + off_set]), num_radial)
# rli_blade_2_le_high_res = rli_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rli_blade_2_te_high_res = rli_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rli_chord_length = am.subtract(rli_blade_2_le_high_res, rli_blade_2_te_high_res)

# # twist
# rli_le_proj_disk = rli_disk.project(rli_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rli_te_proj_disk = rli_disk.project(rli_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rli_v_dist_le = am.subtract(rli_blade_2_le_high_res, rli_le_proj_disk)
# rli_v_dist_te = am.subtract(rli_blade_2_te_high_res, rli_te_proj_disk)
# rli_tot_v_dist = am.subtract(rli_v_dist_le, rli_v_dist_te)
# # endregion

# # region rear right inner (rri) rotor meshes
# # disk
# rri_origin = rri_disk.project(np.array([18.760, 8.537, 9.919]), direction=np.array([0., 0., -1.]))

# # along y
# y11 = rri_disk.project(np.array([18.760, 13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rri_disk.project(np.array([18.760, 3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rri_disk.project(np.array([13.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rri_disk.project(np.array([23.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)

# rri_in_plane_y = am.subtract(y12, y11)
# rri_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + rri_origin.value

# rri_disk_mesh = rri_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)

# # lifting line mesh
# # blade 1
# b1_le_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_le, 8.243 - off_set, 9.381 + off_set]), np.array([23.760 + off_set_long_le, 8.298 - off_set, 9.315 + off_set]), num_lifting_line)
# b1_te_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_te_root, 9.073 + off_set, 9.058 + off_set]), np.array([23.761 + off_set_long_te_tip, 8.906 + off_set, 9.257 + off_set]), num_lifting_line)

# rri_blade_1_le_low_res = rri_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rri_blade_1_te_low_res = rri_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rri_blade_1_chord_surface = am.linspace(rri_blade_1_le_low_res, rri_blade_1_te_low_res, 2)
# rri_blade_1_upper_surface_wireframe = rri_blade_1.project(rri_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rri_blade_1_lower_surface_wireframe = rri_blade_1.project(rri_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# rri_blade_1_ll_mesh = am.linspace(rri_blade_1_upper_surface_wireframe, rri_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_le_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, 8.672 + off_set, 9.321 + off_set]), np.array([13.760 - off_set_long_le, 8.600 + off_set, 9.003 + off_set]), num_lifting_line)
# b2_te_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_te_root, 7.784 - off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, 8.000 - off_set, 9.385 + off_set]), num_lifting_line)

# rri_blade_2_le_low_res = rri_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rri_blade_2_te_low_res = rri_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rri_blade_2_chord_surface = am.linspace(rri_blade_2_le_low_res, rri_blade_2_te_low_res, 2)
# rri_blade_2_upper_surface_wireframe = rri_blade_2.project(rri_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# rri_blade_2_lower_surface_wireframe = rri_blade_2.project(rri_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# rri_blade_2_ll_mesh = am.linspace(rri_blade_2_upper_surface_wireframe, rri_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([rri_blade_1_ll_mesh, rri_blade_2_ll_mesh])

# # chord 
# b2_le_high_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, 8.672 + off_set, 9.321 + off_set]), np.array([13.760 - off_set_long_le, 8.600 + off_set, 9.003 + off_set]), num_radial)
# b2_te_high_res_numpy = np.linspace(np.array([17.710 + 0., 7.784 - off_set, 9.239 + off_set]), np.array([13.760 - 0.3, 8.000 - off_set, 9.385 + off_set]), num_radial)
# rri_blade_2_le_high_res = rri_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rri_blade_2_te_high_res = rri_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# # rri_chord_length = am.norm(am.subtract(rri_blade_2_le_high_res, rri_blade_2_te_high_res), axes=(1, ))
# rri_chord_length = am.subtract(rri_blade_2_le_high_res, rri_blade_2_te_high_res)

# # twist
# rri_le_proj_disk = rri_disk.project(rri_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rri_te_proj_disk = rri_disk.project(rri_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# rri_v_dist_le = am.subtract(rri_blade_2_le_high_res, rri_le_proj_disk)
# rri_v_dist_te = am.subtract(rri_blade_2_te_high_res, rri_te_proj_disk)
# rri_tot_v_dist = am.subtract(rri_v_dist_le, rri_v_dist_te)

# # endregion

# # region front left inner (fli) rotor meshes
# # disk

# fli_origin = fli_disk.project(np.array([4.630, -8.217, 7.659]), direction=np.array([0., 0., -1.]), plot=False)

# # along y
# y11 = fli_disk.project(np.array([4.630, -3.179, 7.736]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = fli_disk.project(np.array([4.630, -13.081, 6.344]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = fli_disk.project(np.array([-0.370, -8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = fli_disk.project(np.array([9.630, -8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)

# fli_in_plane_y = am.subtract(y12, y11)
# fli_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + fli_origin.value

# fli_disk_mesh = fli_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)

# # lifting line mesh
# # blade 1
# b1_te_low_res_numpy = np.linspace(np.array([5.680 - 0., -8.352 - off_set, 7.061 + off_set]), np.array([9.630 + off_set_long_le, -8.280 - off_set, 7.012 + off_set]), num_lifting_line)
# b1_le_low_res_numpy = np.linspace(np.array([5.680 - off_set_long_le_tip, -7.464 + off_set, 6.798 + off_set]), np.array([9.630 + off_set_long_te_tip, -7.680 + off_set, 7.125 + off_set]), num_lifting_line)

# fli_blade_1_le_low_res = fli_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fli_blade_1_te_low_res = fli_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fli_blade_1_chord_surface = am.linspace(fli_blade_1_le_low_res, fli_blade_1_te_low_res, 2)
# fli_blade_1_upper_surface_wireframe = fli_blade_1.project(fli_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# fli_blade_1_lower_surface_wireframe = fli_blade_1.project(fli_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# fli_blade_1_ll_mesh = am.linspace(fli_blade_1_upper_surface_wireframe, fli_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_te_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_le, -7.923 + off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, -7.978 + off_set, 7.055 + off_set]), num_lifting_line)
# b2_le_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, -8.753 - off_set, 6.577 + off_set]), np.array([-0.370 - off_set_long_te_tip, -8.586 - off_set, 6.998 + off_set]), num_lifting_line)

# fli_blade_2_le_low_res = fli_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fli_blade_2_te_low_res = fli_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fli_blade_2_chord_surface = am.linspace(fli_blade_2_le_low_res, fli_blade_2_te_low_res, 2)
# fli_blade_2_upper_surface_wireframe = fli_blade_2.project(fli_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# fli_blade_2_lower_surface_wireframe = fli_blade_2.project(fli_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# fli_blade_2_ll_mesh = am.linspace(fli_blade_2_upper_surface_wireframe, fli_blade_2_lower_surface_wireframe, 1) 
# # spatial_rep.plot_meshes([fli_blade_1_ll_mesh, fli_blade_2_ll_mesh])

# # chord 
# b2_te_high_res_numpy = np.linspace(np.array([3.580 + 0.0, -7.923 + off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, -7.978 + off_set, 7.055 + off_set]), num_radial)
# b2_le_high_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, -8.753 - off_set, 6.577 + off_set]), np.array([-0.370 - off_set_long_te_tip, -8.586 - off_set, 6.998 + off_set]), num_radial)
# fli_blade_2_le_high_res = fli_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fli_blade_2_te_high_res = fli_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# # fli_chord_length = am.norm(am.subtract(fli_blade_2_le_high_res, fli_blade_2_te_high_res), axes=(1, ))
# fli_chord_length = am.subtract(fli_blade_2_le_high_res, fli_blade_2_te_high_res)

# # twist
# fli_le_proj_disk = fli_disk.project(fli_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fli_te_proj_disk = fli_disk.project(fli_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fli_v_dist_le = am.subtract(fli_blade_2_le_high_res, fli_le_proj_disk)
# fli_v_dist_te = am.subtract(fli_blade_2_te_high_res, fli_te_proj_disk)
# fli_tot_v_dist = am.subtract(fli_v_dist_le, fli_v_dist_te)
# # endregion

# # region front right inner (fri) rotor meshes
# # disk

# fri_origin = fri_disk.project(np.array([4.630, 8.217, 7.659]), direction=np.array([0., 0., -1.]), plot=False)

# # along y
# y11 = fri_disk.project(np.array([4.630, 13.081, 6.344]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = fri_disk.project(np.array([4.630, 3.179, 7.736]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = fri_disk.project(np.array([-0.370, 8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = fri_disk.project(np.array([9.630, 8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)

# fri_in_plane_y = am.subtract(y12, y11)
# fri_in_plane_x = am.subtract(y21, y22)

# num_tangential = 25
# radius = np.linspace(0.2 * 5, 5, 25)
# angles = np.linspace(0, 2*np.pi, num_tangential, endpoint=False)


# cartesian = np.zeros((25, num_tangential, 3))

# for i in range(25):
#     for j in range(num_tangential):
#         cartesian[i, j, 0] = radius[i] * np.cos(angles[j])
#         cartesian[i, j, 1] = radius[i] * np.sin(angles[j])
#         cartesian[i, j, 2] = 0

# cartesian_plus_origin = cartesian + fri_origin.value

# fri_disk_mesh = fri_disk.project(cartesian_plus_origin, direction=np.array([0., 0., -1.]) ,plot=False)

# # lifting line mesh
# # blade 1
# b1_te_low_res_numpy = np.linspace(np.array([5.680 - 0.0, 8.672 + off_set, 7.061 + off_set]), np.array([9.630 + off_set_long_le, 8.600 + off_set, 7.012 + off_set]), num_lifting_line)
# b1_le_low_res_numpy = np.linspace(np.array([5.680 - off_set_long_te_root, 7.784 - off_set, 6.979 + off_set]), np.array([9.630 + off_set_long_te_tip, 8.000 - off_set, 7.125 + off_set]), num_lifting_line)

# fri_blade_1_le_low_res = fri_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fri_blade_1_te_low_res = fri_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fri_blade_1_chord_surface = am.linspace(fri_blade_1_le_low_res, fri_blade_1_te_low_res, 2)
# fri_blade_1_upper_surface_wireframe = fri_blade_1.project(fri_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# fri_blade_1_lower_surface_wireframe = fri_blade_1.project(fri_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
# fri_blade_1_ll_mesh = am.linspace(fri_blade_1_upper_surface_wireframe, fri_blade_1_lower_surface_wireframe, 1) 


# # blade 2
# b2_te_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_le, 8.243 - off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, 8.298 - off_set, 7.055 + off_set]), num_lifting_line)
# b2_le_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, 9.073 + off_set, 6.798 + off_set]), np.array([-0.370 - off_set_long_te_tip, 8.906 + off_set, 6.998 + off_set]), num_lifting_line)

# fri_blade_2_le_low_res = fri_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fri_blade_2_te_low_res = fri_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fri_blade_2_chord_surface = am.linspace(fri_blade_2_le_low_res, fri_blade_2_te_low_res, 2)
# fri_blade_2_upper_surface_wireframe = fri_blade_2.project(fri_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# fri_blade_2_lower_surface_wireframe = fri_blade_2.project(fri_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# fri_blade_2_ll_mesh = am.linspace(fri_blade_2_upper_surface_wireframe, fri_blade_2_lower_surface_wireframe, 1) 

# # chord 
# b2_te_high_res_numpy = np.linspace(np.array([3.580 - 0.1, 8.243 - off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, 8.298 - off_set, 7.055 + off_set]), num_radial)
# b2_le_high_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, 9.073 + off_set, 6.798 + off_set]), np.array([-0.370 - 0.32, 8.906 + off_set, 6.998 + off_set]), num_radial)
# fri_blade_2_le_high_res = fri_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fri_blade_2_te_high_res = fri_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# # fri_chord_length = am.norm(am.subtract(fri_blade_2_le_high_res, fri_blade_2_te_high_res), axes=(1, ))
# fri_chord_length = am.subtract(fri_blade_2_le_high_res, fri_blade_2_te_high_res)

# # twist
# fri_le_proj_disk = fri_disk.project(fri_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fri_te_proj_disk = fri_disk.project(fri_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

# fri_v_dist_le = am.subtract(fri_blade_2_le_high_res, fri_le_proj_disk)
# fri_v_dist_te = am.subtract(fri_blade_2_te_high_res, fri_te_proj_disk)
# fri_tot_v_dist = am.subtract(fri_v_dist_le, fri_v_dist_te)
# # 
# # endregion

# # spatial_rep.plot_meshes([
# #     wing_camber_surface, htail_camber_surface,
# #     flo_blade_1_ll_mesh, flo_blade_2_ll_mesh,
# #     fli_blade_1_ll_mesh, fli_blade_2_ll_mesh,
# #     fri_blade_1_ll_mesh, fri_blade_2_ll_mesh,
# #     fro_blade_1_ll_mesh, fro_blade_2_ll_mesh,
# #     rlo_blade_1_ll_mesh, rlo_blade_2_ll_mesh,
# #     rli_blade_1_ll_mesh, rli_blade_2_ll_mesh,
# #     rri_blade_1_ll_mesh, rri_blade_2_ll_mesh,
# #     rro_blade_1_ll_mesh, rro_blade_2_ll_mesh,
# #     ]
# # )
# # endregion

# # region actuations
# configuration_names = [
#     "minus_1g_configuration", 
#     "plus_3g_configuration",
#     "hover_1_configuration", 
#     "cruise_configuration", 
#     "climb_configuration",
#     "descent_configuration",
#     "quasi_steady_transition_1",    
#     "quasi_steady_transition_2",    
#     "quasi_steady_transition_3",    
#     "quasi_steady_transition_4",    
#     "quasi_steady_transition_5",    
#     "quasi_steady_transition_6",    
#     "quasi_steady_transition_7",    
#     "quasi_steady_transition_8",    
#     "quasi_steady_transition_9",    
#     "quasi_steady_transition_10",    
#     "hover_configuration_oei_flo", 
#     "hover_configuration_oei_fli",
#     "quasi_steady_transition_1_oei_flo",    
#     "quasi_steady_transition_2_oei_flo",
#     "quasi_steady_transition_3_oei_flo",
#     "quasi_steady_transition_4_oei_flo",
# ]
# system_configurations = lpc_rep.declare_configurations(names=configuration_names)

# # region Projections
# horizontal_stabilizer_quarter_chord_port = htail.project(np.array([28.5, -10., 8.]))
# horizontal_stabilizer_quarter_chord_starboard = htail.project(np.array([28.5, 10., 8.]))
# horizontal_stabilizer_actuation_axis = horizontal_stabilizer_quarter_chord_starboard - horizontal_stabilizer_quarter_chord_port

# wing_quarter_chord_port = wing.project(np.array([28.5, -10., 8.]))
# wing_quarter_chord_starboard = wing.project(np.array([28.5, 10., 8.]))
# wing_actuation_axis = wing_quarter_chord_starboard - wing_quarter_chord_port
# # endregion

# wing_vlm_mesh_name = f"{wing.parameters['name']}_vlm_mesh"
# htail_vlm_mesh_name =  f"{htail.parameters['name']}_vlm_mesh"

# # region Cruise
# cruise_configuration = system_configurations['cruise_configuration']
# cruise_configuration.set_num_nodes(num_nodes=1)
# cruise_configuration.add_output(f"{htail_vlm_mesh_name}_cruise", htail_camber_surface)
# cruise_configuration.add_output(f"{wing_vlm_mesh_name}_cruise", wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='cruise_tail_actuation', value=0, units='radians')
# cruise_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='cruise_wing_actuation', value=np.deg2rad(0) , units='radians')
# cruise_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region Climb 
# climb_configuration = system_configurations['climb_configuration']
# climb_configuration.set_num_nodes(num_nodes=1)
# climb_configuration.add_output(f"{htail_vlm_mesh_name}_climb", htail_camber_surface)
# climb_configuration.add_output(f'{wing_vlm_mesh_name}_climb', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='climb_tail_actuation', value=0, units='radians')
# climb_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='climb_wing_actuation', value=np.deg2rad(4) , units='radians')
# climb_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region Descent 
# descent_configuration = system_configurations['descent_configuration']
# descent_configuration.set_num_nodes(num_nodes=1)
# descent_configuration.add_output(f"{htail_vlm_mesh_name}_descent", htail_camber_surface)
# descent_configuration.add_output(f'{wing_vlm_mesh_name}_descent', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='descent_tail_actuation', value=0, units='radians')
# descent_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='descent_wing_actuation', value=np.deg2rad(4) , units='radians')
# descent_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region +3g 
# plus_3g_configuration = system_configurations['plus_3g_configuration']
# plus_3g_configuration.set_num_nodes(num_nodes=1)
# plus_3g_configuration.add_output(f'{htail_vlm_mesh_name}_plus_3g', htail_camber_surface)
# plus_3g_configuration.add_output(f'{wing_vlm_mesh_name}_plus_3g', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='plus_3g_tail_actuation', value=0, units='radians')
# plus_3g_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='plus_3g_wing_actuation', value=np.deg2rad(0) , units='radians')
# plus_3g_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region -1g 
# minus_1g_configuration = system_configurations['minus_1g_configuration']
# minus_1g_configuration.set_num_nodes(num_nodes=1)
# minus_1g_configuration.add_output(f'{htail_vlm_mesh_name}_minus_1g', htail_camber_surface)
# minus_1g_configuration.add_output(f'{wing_vlm_mesh_name}_minus_1g', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='minus_1g_tail_actuation', value=0, units='radians')
# minus_1g_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='minus_1g_wing_actuation', value=np.deg2rad(0) , units='radians')
# minus_1g_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region hover 1
# hover_1_configuration = system_configurations['hover_1_configuration']
# hover_1_configuration.set_num_nodes(num_nodes=1)

# hover_1_configuration.add_output(f"hover_1_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{rlo_disk.parameters['name']}_origin", rlo_origin)

# hover_1_configuration.add_output(f"hover_1_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{rli_disk.parameters['name']}_origin", rli_origin)

# hover_1_configuration.add_output(f"hover_1_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{rri_disk.parameters['name']}_origin", rri_origin)

# hover_1_configuration.add_output(f"hover_1_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{rro_disk.parameters['name']}_origin", rro_origin)

# hover_1_configuration.add_output(f"hover_1_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{flo_disk.parameters['name']}_origin", flo_origin)

# hover_1_configuration.add_output(f"hover_1_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{fli_disk.parameters['name']}_origin", fli_origin)

# hover_1_configuration.add_output(f"hover_1_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{fri_disk.parameters['name']}_origin", fri_origin)

# hover_1_configuration.add_output(f"hover_1_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# hover_1_configuration.add_output(f"hover_1_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# hover_1_configuration.add_output(f"hover_1_{fro_disk.parameters['name']}_origin", fro_origin)


# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='hover_1_rlo_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='hover_1_rlo_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='hover_1_rli_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='hover_1_rli_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='hover_1_rri_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='hover_1_rri_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='hover_1_rro_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='hover_1_rro_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='hover_1_flo_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='hover_1_flo_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='hover_1_fli_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='hover_1_fli_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='hover_1_fri_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='hover_1_fri_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='hover_1_fro_disk_actuation_1', value=0, units='radians')
# hover_1_configuration.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='hover_1_fro_disk_actuation_2', value=0, units='radians')
# hover_1_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion 

# # region hover 1 oei flo
# hover_configuration_oei_flo = system_configurations['hover_configuration_oei_flo']
# hover_configuration_oei_flo.set_num_nodes(num_nodes=1)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rlo_disk.parameters['name']}_origin", rlo_origin)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rli_disk.parameters['name']}_origin", rli_origin)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rri_disk.parameters['name']}_origin", rri_origin)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{rro_disk.parameters['name']}_origin", rro_origin)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fli_disk.parameters['name']}_origin", fli_origin)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fri_disk.parameters['name']}_origin", fri_origin)

# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# hover_configuration_oei_flo.add_output(f"hover_1_oei_flo_{fro_disk.parameters['name']}_origin", fro_origin)


# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_rlo_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_rlo_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_rli_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_rli_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_rri_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_rri_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_rro_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_rro_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=rro_disk_actuator_solver_2)


# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_fli_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_fli_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_fri_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_fri_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='hover_1_oei_flo_fro_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='hover_1_oei_flo_fro_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_flo.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion 

# # region hover 1 oei fli
# hover_configuration_oei_fli = system_configurations['hover_configuration_oei_fli']
# hover_configuration_oei_fli.set_num_nodes(num_nodes=1)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rlo_disk.parameters['name']}_origin", rlo_origin)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rli_disk.parameters['name']}_origin", rli_origin)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rri_disk.parameters['name']}_origin", rri_origin)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{rro_disk.parameters['name']}_origin", rro_origin)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{flo_disk.parameters['name']}_origin", flo_origin)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{fri_disk.parameters['name']}_origin", fri_origin)

# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# hover_configuration_oei_fli.add_output(f"hover_1_oei_fli_{fro_disk.parameters['name']}_origin", fro_origin)


# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_rlo_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_rlo_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_rli_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_rli_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_rri_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_rri_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_rro_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_rro_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=rro_disk_actuator_solver_2)


# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_flo_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_flo_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=flo_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_fri_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_fri_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='hover_1_oei_fli_fro_disk_actuation_1', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='hover_1_oei_fli_fro_disk_actuation_2', value=0, units='radians')
# hover_configuration_oei_fli.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion 

# # region quasi_steady_transition_1
# qst_1_configuration = system_configurations['quasi_steady_transition_1']
# qst_1_configuration.set_num_nodes(num_nodes=1)
# qst_1_configuration.add_output(f'{htail_vlm_mesh_name}_qst_1', htail_camber_surface)
# qst_1_configuration.add_output(f'{wing_vlm_mesh_name}_qst_1', wing_camber_surface)

# qst_1_configuration.add_output(f"qst_1_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{rlo_disk.parameters['name']}_origin", rlo_origin)

# qst_1_configuration.add_output(f"qst_1_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{rli_disk.parameters['name']}_origin", rli_origin)

# qst_1_configuration.add_output(f"qst_1_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{rri_disk.parameters['name']}_origin", rri_origin)

# qst_1_configuration.add_output(f"qst_1_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{rro_disk.parameters['name']}_origin", rro_origin)

# qst_1_configuration.add_output(f"qst_1_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{flo_disk.parameters['name']}_origin", flo_origin)

# qst_1_configuration.add_output(f"qst_1_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{fli_disk.parameters['name']}_origin", fli_origin)

# qst_1_configuration.add_output(f"qst_1_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{fri_disk.parameters['name']}_origin", fri_origin)

# qst_1_configuration.add_output(f"qst_1_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# qst_1_configuration.add_output(f"qst_1_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# qst_1_configuration.add_output(f"qst_1_{fro_disk.parameters['name']}_origin", fro_origin)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_1_tail_actuation', value=0, units='radians')
# qst_1_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_1_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_1_configuration.actuate(transformation=wing_actuator_solver)

# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='qst_1_rlo_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='qst_1_rlo_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='qst_1_rli_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='qst_1_rli_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='qst_1_rri_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='qst_1_rri_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='qst_1_rro_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='qst_1_rro_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='qst_1_flo_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='qst_1_flo_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='qst_1_fli_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='qst_1_fli_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='qst_1_fri_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='qst_1_fri_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='qst_1_fro_disk_actuation_1', value=0, units='radians')
# qst_1_configuration.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='qst_1_fro_disk_actuation_2', value=0, units='radians')
# qst_1_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion

# # region quasi_steady_transition_2
# qst_2_configuration = system_configurations['quasi_steady_transition_2']
# qst_2_configuration.set_num_nodes(num_nodes=1)
# qst_2_configuration.add_output(f'{htail_vlm_mesh_name}_qst_2', htail_camber_surface)
# qst_2_configuration.add_output(f'{wing_vlm_mesh_name}_qst_2', wing_camber_surface)

# qst_2_configuration.add_output(f"qst_2_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{rlo_disk.parameters['name']}_origin", rlo_origin)

# qst_2_configuration.add_output(f"qst_2_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{rli_disk.parameters['name']}_origin", rli_origin)

# qst_2_configuration.add_output(f"qst_2_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{rri_disk.parameters['name']}_origin", rri_origin)

# qst_2_configuration.add_output(f"qst_2_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{rro_disk.parameters['name']}_origin", rro_origin)

# qst_2_configuration.add_output(f"qst_2_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{flo_disk.parameters['name']}_origin", flo_origin)

# qst_2_configuration.add_output(f"qst_2_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{fli_disk.parameters['name']}_origin", fli_origin)

# qst_2_configuration.add_output(f"qst_2_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{fri_disk.parameters['name']}_origin", fri_origin)

# qst_2_configuration.add_output(f"qst_2_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# qst_2_configuration.add_output(f"qst_2_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# qst_2_configuration.add_output(f"qst_2_{fro_disk.parameters['name']}_origin", fro_origin)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_2_tail_actuation', value=0, units='radians')
# qst_2_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_2_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_2_configuration.actuate(transformation=wing_actuator_solver)

# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='qst_2_rlo_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='qst_2_rlo_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='qst_2_rli_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='qst_2_rli_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='qst_2_rri_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='qst_2_rri_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='qst_2_rro_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='qst_2_rro_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='qst_2_flo_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='qst_2_flo_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='qst_2_fli_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='qst_2_fli_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='qst_2_fri_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='qst_2_fri_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='qst_2_fro_disk_actuation_1', value=0, units='radians')
# qst_2_configuration.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='qst_2_fro_disk_actuation_2', value=0, units='radians')
# qst_2_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion

# # region quasi_steady_transition_3
# qst_3_configuration = system_configurations['quasi_steady_transition_3']
# qst_3_configuration.set_num_nodes(num_nodes=1)
# qst_3_configuration.add_output(f'{htail_vlm_mesh_name}_qst_3', htail_camber_surface)
# qst_3_configuration.add_output(f'{wing_vlm_mesh_name}_qst_3', wing_camber_surface)

# qst_3_configuration.add_output(f"qst_3_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{rlo_disk.parameters['name']}_origin", rlo_origin)

# qst_3_configuration.add_output(f"qst_3_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{rli_disk.parameters['name']}_origin", rli_origin)

# qst_3_configuration.add_output(f"qst_3_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{rri_disk.parameters['name']}_origin", rri_origin)

# qst_3_configuration.add_output(f"qst_3_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{rro_disk.parameters['name']}_origin", rro_origin)

# qst_3_configuration.add_output(f"qst_3_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{flo_disk.parameters['name']}_origin", flo_origin)

# qst_3_configuration.add_output(f"qst_3_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{fli_disk.parameters['name']}_origin", fli_origin)

# qst_3_configuration.add_output(f"qst_3_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{fri_disk.parameters['name']}_origin", fri_origin)

# qst_3_configuration.add_output(f"qst_3_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# qst_3_configuration.add_output(f"qst_3_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# qst_3_configuration.add_output(f"qst_3_{fro_disk.parameters['name']}_origin", fro_origin)


# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_3_tail_actuation', value=0, units='radians')
# qst_3_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_3_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_3_configuration.actuate(transformation=wing_actuator_solver)

# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='qst_3_rlo_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='qst_3_rlo_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='qst_3_rli_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='qst_3_rli_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='qst_3_rri_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='qst_3_rri_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='qst_3_rro_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='qst_3_rro_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='qst_3_flo_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='qst_3_flo_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='qst_3_fli_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='qst_3_fli_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='qst_3_fri_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='qst_3_fri_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='qst_3_fro_disk_actuation_1', value=0, units='radians')
# qst_3_configuration.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='qst_3_fro_disk_actuation_2', value=0, units='radians')
# qst_3_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion

# # region quasi_steady_transition_4
# qst_4_configuration = system_configurations['quasi_steady_transition_4']
# qst_4_configuration.set_num_nodes(num_nodes=1)
# qst_4_configuration.add_output(f'{htail_vlm_mesh_name}_qst_4', htail_camber_surface)
# qst_4_configuration.add_output(f'{wing_vlm_mesh_name}_qst_4', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_4_tail_actuation', value=0, units='radians')
# qst_4_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_4_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_4_configuration.actuate(transformation=wing_actuator_solver)

# qst_4_configuration.add_output(f"qst_4_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{rlo_disk.parameters['name']}_origin", rlo_origin)

# qst_4_configuration.add_output(f"qst_4_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{rli_disk.parameters['name']}_origin", rli_origin)

# qst_4_configuration.add_output(f"qst_4_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{rri_disk.parameters['name']}_origin", rri_origin)

# qst_4_configuration.add_output(f"qst_4_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{rro_disk.parameters['name']}_origin", rro_origin)

# qst_4_configuration.add_output(f"qst_4_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{flo_disk.parameters['name']}_origin", flo_origin)

# qst_4_configuration.add_output(f"qst_4_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{fli_disk.parameters['name']}_origin", fli_origin)

# qst_4_configuration.add_output(f"qst_4_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{fri_disk.parameters['name']}_origin", fri_origin)

# qst_4_configuration.add_output(f"qst_4_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# qst_4_configuration.add_output(f"qst_4_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# qst_4_configuration.add_output(f"qst_4_{fro_disk.parameters['name']}_origin", fro_origin)

# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='qst_4_rlo_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='qst_4_rlo_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='qst_4_rli_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='qst_4_rli_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='qst_4_rri_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='qst_4_rri_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='qst_4_rro_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='qst_4_rro_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='qst_4_flo_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='qst_4_flo_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='qst_4_fli_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='qst_4_fli_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='qst_4_fri_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='qst_4_fri_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='qst_4_fro_disk_actuation_1', value=0, units='radians')
# qst_4_configuration.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='qst_4_fro_disk_actuation_2', value=0, units='radians')
# qst_4_configuration.actuate(transformation=fro_disk_actuator_solver_2)

# # endregion

# # region quasi_steady_transition_5
# qst_5_configuration = system_configurations['quasi_steady_transition_5']
# qst_5_configuration.set_num_nodes(num_nodes=1)
# qst_5_configuration.add_output(f'{htail_vlm_mesh_name}_qst_5', htail_camber_surface)
# qst_5_configuration.add_output(f'{wing_vlm_mesh_name}_qst_5', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_5_tail_actuation', value=0, units='radians')
# qst_5_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_5_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_5_configuration.actuate(transformation=wing_actuator_solver)

# qst_5_configuration.add_output(f"qst_5_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{rlo_disk.parameters['name']}_origin", rlo_origin)

# qst_5_configuration.add_output(f"qst_5_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{rli_disk.parameters['name']}_origin", rli_origin)

# qst_5_configuration.add_output(f"qst_5_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{rri_disk.parameters['name']}_origin", rri_origin)

# qst_5_configuration.add_output(f"qst_5_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{rro_disk.parameters['name']}_origin", rro_origin)

# qst_5_configuration.add_output(f"qst_5_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{flo_disk.parameters['name']}_origin", flo_origin)

# qst_5_configuration.add_output(f"qst_5_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{fli_disk.parameters['name']}_origin", fli_origin)

# qst_5_configuration.add_output(f"qst_5_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{fri_disk.parameters['name']}_origin", fri_origin)

# qst_5_configuration.add_output(f"qst_5_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
# qst_5_configuration.add_output(f"qst_5_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
# qst_5_configuration.add_output(f"qst_5_{fro_disk.parameters['name']}_origin", fro_origin)

# # rlo
# rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
# rlo_disk_actuator_solver_1.set_rotation(name='qst_5_rlo_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

# rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
# rlo_disk_actuator_solver_2.set_rotation(name='qst_5_rlo_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# # rli
# rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
# rli_disk_actuator_solver_1.set_rotation(name='qst_5_rli_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rli_disk_actuator_solver_1)

# rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
# rli_disk_actuator_solver_2.set_rotation(name='qst_5_rli_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# # rri
# rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
# rri_disk_actuator_solver_1.set_rotation(name='qst_5_rri_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rri_disk_actuator_solver_1)

# rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
# rri_disk_actuator_solver_2.set_rotation(name='qst_5_rri_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# # rro
# rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
# rro_disk_actuator_solver_1.set_rotation(name='qst_5_rro_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rro_disk_actuator_solver_1)

# rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
# rro_disk_actuator_solver_2.set_rotation(name='qst_5_rro_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# # flo
# flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
# flo_disk_actuator_solver_1.set_rotation(name='qst_5_flo_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=flo_disk_actuator_solver_1)

# flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
# flo_disk_actuator_solver_2.set_rotation(name='qst_5_flo_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# # fli
# fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
# fli_disk_actuator_solver_1.set_rotation(name='qst_5_fli_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=fli_disk_actuator_solver_1)

# fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
# fli_disk_actuator_solver_2.set_rotation(name='qst_5_fli_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# # fri
# fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
# fri_disk_actuator_solver_1.set_rotation(name='qst_5_fri_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=fri_disk_actuator_solver_1)

# fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
# fri_disk_actuator_solver_2.set_rotation(name='qst_5_fri_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# # fro
# fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
# fro_disk_actuator_solver_1.set_rotation(name='qst_5_fro_disk_actuation_1', value=0, units='radians')
# qst_5_configuration.actuate(transformation=fro_disk_actuator_solver_1)

# fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
# fro_disk_actuator_solver_2.set_rotation(name='qst_5_fro_disk_actuation_2', value=0, units='radians')
# qst_5_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# # endregion

# # region quasi_steady_transition_6
# qst_6_configuration = system_configurations['quasi_steady_transition_6']
# qst_6_configuration.set_num_nodes(num_nodes=1)
# qst_6_configuration.add_output(f'{htail_vlm_mesh_name}_qst_6', htail_camber_surface)
# qst_6_configuration.add_output(f'{wing_vlm_mesh_name}_qst_6', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_6_tail_actuation', value=0, units='radians')
# qst_6_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_6_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_6_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region quasi_steady_transition_7
# qst_7_configuration = system_configurations['quasi_steady_transition_7']
# qst_7_configuration.set_num_nodes(num_nodes=1)
# qst_7_configuration.add_output(f'{htail_vlm_mesh_name}_qst_7', htail_camber_surface)
# qst_7_configuration.add_output(f'{wing_vlm_mesh_name}_qst_7', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_7_tail_actuation', value=0, units='radians')
# qst_7_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_7_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_7_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region quasi_steady_transition_8
# qst_8_configuration = system_configurations['quasi_steady_transition_8']
# qst_8_configuration.set_num_nodes(num_nodes=1)
# qst_8_configuration.add_output(f'{htail_vlm_mesh_name}_qst_8', htail_camber_surface)
# qst_8_configuration.add_output(f'{wing_vlm_mesh_name}_qst_8', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_8_tail_actuation', value=0, units='radians')
# qst_8_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_8_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_8_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region quasi_steady_transition_9
# qst_9_configuration = system_configurations['quasi_steady_transition_9']
# qst_9_configuration.set_num_nodes(num_nodes=1)
# qst_9_configuration.add_output(f'{htail_vlm_mesh_name}_qst_9', htail_camber_surface)
# qst_9_configuration.add_output(f'{wing_vlm_mesh_name}_qst_9', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_9_tail_actuation', value=0, units='radians')
# qst_9_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_9_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_9_configuration.actuate(transformation=wing_actuator_solver)
# # endregion

# # region quasi_steady_transition_10
# qst_10_configuration = system_configurations['quasi_steady_transition_10']
# qst_10_configuration.set_num_nodes(num_nodes=1)
# qst_10_configuration.add_output(f'{htail_vlm_mesh_name}_qst_10', htail_camber_surface)
# qst_10_configuration.add_output(f'{wing_vlm_mesh_name}_qst_10', wing_camber_surface)

# horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
# horizontal_stabilizer_actuator_solver.set_rotation(name='qst_10_tail_actuation', value=0, units='radians')
# qst_10_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

# wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
# wing_actuator_solver.set_rotation(name='qst_10_wing_actuation', value=np.deg2rad(4) , units='radians')
# qst_10_configuration.actuate(transformation=wing_actuator_solver)
# # endregion
# # endregion

# lpc_param.add_geometry_parameterization(ffd_set)
# lpc_param.setup()


# lpc_rep.add_output(name=wing_vlm_mesh_name, quantity=wing_camber_surface)
# # lpc_rep.add_output(name=f"{wing_vlm_mesh_name}_cruise", quantity=wing_camber_surface)

# lpc_rep.add_output(name=htail_vlm_mesh_name, quantity=htail_camber_surface)
# lpc_rep.add_output(htail_oml_mesh_name_ml, tail_oml_mesh_ml)

# # lpc_rep.add_output(name=f"{htail_vlm_mesh_name}_cruise", quantity=htail_camber_surface)

# lpc_rep.add_output(name=f"{wing.parameters['name']}_oml_mesh", quantity=wing_oml_mesh)
# lpc_rep.add_output(wing_oml_mesh_name_ml, wing_oml_mesh_ml)

# # lpc_rep.add_output(name=f"{htail.parameters['name']}_oml_mesh", quantity=htail_camber_surface)

# lpc_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
# lpc_rep.add_output(name='wing_beam_width', quantity=width)
# lpc_rep.add_output(name='wing_beam_height', quantity=height)

# lpc_rep.add_output(name=f"{pp_disk.parameters['name']}_in_plane_1", quantity=pp_disk_in_plane_y)
# lpc_rep.add_output(name=f"{pp_disk.parameters['name']}_in_plane_2", quantity=pp_disk_in_plane_x)
# lpc_rep.add_output(name=f"{pp_disk.parameters['name']}_origin", quantity=pp_disk_origin)
# lpc_rep.add_output(name="pp_chord_length", quantity=pp_chord_length)
# lpc_rep.add_output(name='pp_twist', quantity=pp_tot_v_dist)

# lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_mesh", quantity=rlo_disk_mesh)
# lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_in_plane_1", quantity=rlo_in_plane_y)
# lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_in_plane_2", quantity=rlo_in_plane_x)
# lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_origin", quantity=rlo_origin)
# lpc_rep.add_output(name="rlo_chord_length", quantity=rlo_chord_length)
# lpc_rep.add_output(name='rlo_twist', quantity=rlo_tot_v_dist)

# lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_mesh", quantity=rli_disk_mesh)
# lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_in_plane_1", quantity=rli_in_plane_y)
# lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_in_plane_2", quantity=rli_in_plane_x)
# lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_origin", quantity=rli_origin)
# lpc_rep.add_output(name="rli_chord_length", quantity=rli_chord_length)
# lpc_rep.add_output(name='rli_twist', quantity=rli_tot_v_dist)

# lpc_rep.add_output(name=f"{rri_disk.parameters['name']}_in_plane_1", quantity=rri_in_plane_y)
# lpc_rep.add_output(name=f"{rri_disk.parameters['name']}_in_plane_2", quantity=rri_in_plane_x)
# lpc_rep.add_output(name=f"{rri_disk.parameters['name']}_origin", quantity=rri_origin)
# lpc_rep.add_output(name="rri_chord_length", quantity=rri_chord_length)
# lpc_rep.add_output(name='rri_twist', quantity=rri_tot_v_dist)

# lpc_rep.add_output(name=f"{rro_disk.parameters['name']}_in_plane_1", quantity=rro_in_plane_y)
# lpc_rep.add_output(name=f"{rro_disk.parameters['name']}_in_plane_2", quantity=rro_in_plane_x)
# lpc_rep.add_output(name=f"{rro_disk.parameters['name']}_origin", quantity=rro_origin)
# lpc_rep.add_output(name="rro_chord_length", quantity=rro_chord_length)
# lpc_rep.add_output(name='rro_twist', quantity=rro_tot_v_dist)

# lpc_rep.add_output(name=f"{flo_disk.parameters['name']}_in_plane_1", quantity=flo_in_plane_y)
# lpc_rep.add_output(name=f"{flo_disk.parameters['name']}_in_plane_2", quantity=flo_in_plane_x)
# lpc_rep.add_output(name=f"{flo_disk.parameters['name']}_origin", quantity=flo_origin)
# lpc_rep.add_output(name="flo_chord_length", quantity=flo_chord_length)
# lpc_rep.add_output(name='flo_twist', quantity=flo_tot_v_dist)

# lpc_rep.add_output(name=f"{fli_disk.parameters['name']}_in_plane_1", quantity=fli_in_plane_y)
# lpc_rep.add_output(name=f"{fli_disk.parameters['name']}_in_plane_2", quantity=fli_in_plane_x)
# lpc_rep.add_output(name=f"{fli_disk.parameters['name']}_origin", quantity=fli_origin)
# lpc_rep.add_output(name="fli_chord_length", quantity=fli_chord_length)
# lpc_rep.add_output(name='fli_twist', quantity=fli_tot_v_dist)

# lpc_rep.add_output(name=f"{fri_disk.parameters['name']}_in_plane_1", quantity=fri_in_plane_y)
# lpc_rep.add_output(name=f"{fri_disk.parameters['name']}_in_plane_2", quantity=fri_in_plane_x)
# lpc_rep.add_output(name=f"{fri_disk.parameters['name']}_origin", quantity=fri_origin)
# lpc_rep.add_output(name="fri_chord_length", quantity=fri_chord_length)
# lpc_rep.add_output(name='fri_twist', quantity=fri_tot_v_dist)

# lpc_rep.add_output(name=f"{fro_disk.parameters['name']}_in_plane_1", quantity=fro_in_plane_y)
# lpc_rep.add_output(name=f"{fro_disk.parameters['name']}_in_plane_2", quantity=fro_in_plane_x)
# lpc_rep.add_output(name=f"{fro_disk.parameters['name']}_origin", quantity=fro_origin)
# lpc_rep.add_output(name="fro_chord_length", quantity=fro_chord_length)
# lpc_rep.add_output(name='fro_twist', quantity=fro_tot_v_dist)


