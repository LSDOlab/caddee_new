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

# Instantiate system model
system_model = m3l.Model()


# Importing and refitting the geometry
geometry = lg.import_geometry(GEOMETRY_FILES_FOLDER / 'LPC_final_custom_blades_3.stp', parallelize=True)
geometry.refit(parallelize=True, order=(4, 4))
# geometry.plot()


# region Declaring all components
# Wing, tails, fuselage
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['Wing'])
h_tail = geometry.declare_component(component_name='h_tail', b_spline_search_names=['Tail_1'])
v_tail = geometry.declare_component(component_name='v_tail', b_spline_search_names=['Tail_2'])
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

# region FFD
# Wing
import time 

t2 = time.time()
wing_ffd_block = lg.construct_ffd_block_around_entities(name='wing_ffd_block', entities=wing, num_coefficients=(3, 10, 2))
t1 = time.time()
print(t1-t2)
wing_ffd_block_sect_param = VolumeSectionalParameterization(name='wing_ffd_sect_param', principal_parametric_dimension=1, 
                                                            parameterized_points=wing_ffd_block.coefficients,
                                                            parameterized_points_shape=wing_ffd_block.coefficients_shape)

wing_ffd_block_sect_param.add_sectional_translation(name='wing_span_stretch', axis=1)
wing_ffd_block_sect_param.add_sectional_stretch(name='wing_chord_stretch', axis=0)
wing_ffd_block_sect_param.add_sectional_rotation(name='wing_twist', axis=1)

linear_b_spline_curve_2_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_2_dof_space', order=2, parametric_coefficients_shape=(2,))
linear_b_spline_curve_3_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_3_dof_space', order=2, parametric_coefficients_shape=(3,))
cubic_b_spline_curve_5_dof_space = bsp.BSplineSpace(name='cubic_b_spline_curve_5_dof_space', order=4, parametric_coefficients_shape=(5,))

wing_span_strech_coefficients = system_model.create_input('wing_span_stretch_coefficients', shape=(2, ), val=np.array([1., 0.]))
wing_span_strech_b_spline = bsp.BSpline(name='wing_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                           coefficients=wing_span_strech_coefficients, num_physical_dimensions=1)
    
wing_chord_stretch_coefficients = system_model.create_input('wing_chord_stretch_coefficients', shape=(3, ), val=np.array([-0.5, -0.1, 0]))
wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_b_spline', space=linear_b_spline_curve_3_dof_space, coefficients=wing_chord_stretch_coefficients, 
                                          num_physical_dimensions=1)

wing_twist_coefficients = system_model.create_input('wing_twist_coefficients', shape=(5,), val=np.array([0., 30., 20., 10., 0.]))
wing_twist_b_spline = bsp.BSpline(name='wing_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
                                            coefficients=wing_twist_coefficients, num_physical_dimensions=1)




section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sect_param.num_sections).reshape((-1,1))
wing_wingspan_stretch = wing_span_strech_b_spline.evaluate(section_parametric_coordinates)
wing_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
wing_sectional_twist = wing_twist_b_spline.evaluate(section_parametric_coordinates)


sectional_parameters = {
        'wing_span_stretch': wing_wingspan_stretch,
        'wing_chord_stretch': wing_chord_stretch,
        'wing_twist': wing_sectional_twist,
}


wing_ffd_block_coefficients = wing_ffd_block_sect_param.evaluate(sectional_parameters, plot=False)
wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=False)
geometry.assign_coefficients(coefficients=wing_coefficients, b_spline_names=wing.b_spline_names)


wing_te_right = geometry.evaluate(wing.project(np.array([13.4, 25.250, 7.5])))
wing_te_left = geometry.evaluate(wing.project(np.array([13.4, -25.250, 7.5])))
wing_te_center = geometry.evaluate(wing.project(np.array([14.332, 0., 8.439])))
wing_le_left = geometry.evaluate(wing.project(np.array([12.356, -25.25, 7.618])))
wing_le_right = geometry.evaluate(wing.project(np.array([12.356, 25.25, 7.618])))
wing_le_center = geometry.evaluate(wing.project(np.array([8.892, 0., 8.633])))

wing_span = m3l.norm(wing_te_right - wing_te_left)
root_chord = m3l.norm(wing_te_center-wing_le_center)
tip_chord_left = m3l.norm(wing_te_left-wing_le_left)
tip_chord_right = m3l.norm(wing_te_right-wing_le_right)

from lsdo_geo.core.parameterization.parameterization_solver import ParameterizationSolver
wing_parameterization_solver = ParameterizationSolver()

wing_parameterization_solver.declare_state('wing_span_stretch_coefficients', wing_span_strech_coefficients)
wing_parameterization_solver.declare_state('wing_chord_stretch_coefficients', wing_chord_stretch_coefficients)

wing_parameterization_solver.declare_input(name='wing_span', input=wing_span)
wing_parameterization_solver.declare_input(name='tip_chord_left', input=tip_chord_left)
wing_parameterization_solver.declare_input(name='tip_chord_right', input=tip_chord_right)
wing_parameterization_solver.declare_input(name='root_chord', input=root_chord)

wing_span_input = m3l.Variable(name='wing_span', shape=(1, ), value=np.array([30]))
root_chord_input = m3l.Variable(name='root_chord', shape=(1, ), value=np.array([5]))
tip_chord_right_input = m3l.Variable(name='tip_chord_right', shape=(1, ), value=np.array([2]))
tip_chord_left_input = m3l.Variable(name='tip_chord_left', shape=(1, ), value=np.array([1]))

parameterization_inputs = {
    'wing_span' : wing_span,
    'root_chord' : root_chord_input,
    'tip_chord_left' : tip_chord_left_input,
    'tip_chord_right' : tip_chord_right_input,
}
outputs_dict = wing_parameterization_solver.evaluate(inputs=parameterization_inputs, plot=True)

# wing_ffd_block_sect_param.plot()

exit()

# H-tail
h_tail_ffd_blcok = lg.construct_ffd_block_around_entities(name='h_tail_ffd_block', entities=h_tail, num_coefficients=(2, 5, 2))

# V-tail
v_tail_ffd_block = lg.construct_ffd_block_around_entities(name='v_tail_ffd_block', entities=v_tail, num_coefficients=(2, 5, 2))
# endregion

# region Making meshes
# Wing 
num_spanwise_vlm = 25
num_chordwise_vlm = 8


wing_actuation_angle = system_model.create_input('wing_act_angle', val=0, dv_flag=False, lower=-15, upper=15, scaler=1e-1)

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
    actuation_axis=[
        0.75 * wing_te_right + 0.25 * wing_le_right,
        0.75 * wing_te_left + 0.25 * wing_le_left
    ],
    actuation_angle=wing_actuation_angle,
    off_set_x=0.2,
    bunching_cos=True,
    plot=False,
    mirror=True,
)
# 
# tail mesh
num_spanwise_vlm_htail = 8
num_chordwise_vlm_htail = 4

# Create copies of the central geometries for different design conditions for which there are component actuations
tail_te_right = np.array([31.5, 6.75, 6.])
tail_te_left = np.array([31.5, -6.75, 6.])
tail_le_right = np.array([26.5, 6.75, 6.])
tail_le_left = np.array([26.5, -6.75, 6.])


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

# axis_origin = cruise_geometry_2.evaluate(h_tail_cruise_2.project(actuation_axis[0]))
# axis_vector = cruise_geometry_2.evaluate(h_tail_cruise_2.project(actuation_axis[1])) - axis_origin


# v tail mesh
num_spanwise_vlm_vtail = 8
num_chordwise_vlm_vtail = 6

v_tail_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=v_tail,
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
nose_points = geometry.evaluate(fuselage.project(nose, grid_search_density_parameter=20))
rear_points = geometry.evaluate(fuselage.project(rear))

fuselage_linspace = m3l.linspace(nose_points, rear_points, num_fuselage_len)
fueslage_top_points = geometry.evaluate(fuselage.project(fuselage_linspace.value + np.array([0., 0., 3]), direction=np.array([0.,0.,-1.]), plot=False, grid_search_density_parameter=20))
fueslage_bottom_points = geometry.evaluate(fuselage.project(fuselage_linspace.value - np.array([0., 0., 3]), direction=np.array([0.,0.,1.]), plot=False, grid_search_density_parameter=20))
fuesleage_mesh = m3l.linspace(fueslage_top_points.reshape((-1, 3)), fueslage_bottom_points.reshape((-1, 3)),  int(num_fuselage_height + 1))
fuesleage_mesh.description = 'zero_y'


nose2 = np.array([3.439, 1.148, 6.589])
rear2 = np.array([22.030, 1.148, 6.589])
nose2_points = geometry.evaluate(fuselage.project(nose2, plot=False, grid_search_density_parameter=20))
rear2_points = geometry.evaluate(fuselage.project(rear2, plot=False, grid_search_density_parameter=20))
fuselage_linspace2 = m3l.linspace(nose2_points, rear2_points, num_fuselage_len)
fuselage_left_points = geometry.evaluate(fuselage.project(fuselage_linspace2.value, direction=np.array([0., 1., 0.]), plot=False)).reshape((-1, 3))
multiplier = np.ones(fuselage_left_points.shape)
multiplier[:, 1] *= -1
fuselage_right_points = fuselage_left_points * multiplier
# fuselage_right_points = geometry.evaluate(fuselage.project(fuselage_linspace2.value - np.array([0., 5., 0.,]), direction=np.array([0., -1., 0.]), plot=False)).reshape((-1, 3))
fuselage_mesh_2 = m3l.linspace(fuselage_left_points, fuselage_right_points, num_fuselage_height)# .reshape((num_fuselage_len, num_fuselage_height, 3))
fuselage_mesh_2.description = 'average_z'
# geometry.plot_meshes([fuselage_mesh_2, fuesleage_mesh])
# 

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
    blade_geometry_parameters=[blade_1_params, blade_2_params, blade_3_params, blade_4_params],
    create_disk_mesh=False,
    plot=False,
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
    blade_geometry_parameters=[rlo_blade_1_params, rlo_blade_2_params],
    create_disk_mesh=True,
    plot=False,
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
    blade_geometry_parameters=[rro_blade_1_params, rro_blade_2_params],
    create_disk_mesh=False,
    plot=False,
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
    blade_geometry_parameters=[flo_blade_1_params, flo_blade_2_params],
    create_disk_mesh=False,
    plot=False,
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
    blade_geometry_parameters=[fro_blade_1_params, fro_blade_2_params],
    create_disk_mesh=False,
    plot=False,
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
    blade_geometry_parameters=[rli_blade_1_params, rli_blade_2_params],
    create_disk_mesh=True,
    plot=False,
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
    blade_geometry_parameters=[rri_blade_1_params, rri_blade_2_params],
    create_disk_mesh=False,
    plot=False,
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
    blade_geometry_parameters=[fli_blade_1_params, fli_blade_2_params],
    create_disk_mesh=False,
    plot=False,
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
    blade_geometry_parameters=[fri_blade_1_params, fri_blade_2_params],
    create_disk_mesh=False,
    plot=False,
)
# endregion


# endregion


# region Component surface areas
component_list = [rlo_boom, rlo_hub, fuselage, wing, h_tail, v_tail, pp_hub, rlo_blade_1]
surface_area_list = compute_component_surface_area(
    component_list=component_list,
    geometry=geometry,
    parametric_mesh_grid_num=20,
    plot=False,
)

# 
# Drag components 
# Fuselage
fuselage_l1 = geometry.evaluate(fuselage.project(np.array([1.889, 0., 4.249]))).reshape((-1, 3))
fuselage_l2 = geometry.evaluate(fuselage.project(np.array([31.889, 0., 7.798]))).reshape((-1, 3))
fuselage_length = m3l.norm(fuselage_l2-fuselage_l1)

fuselage_d1 = geometry.evaluate(fuselage.project(np.array([10.916, -2.945, 5.736]))).reshape((-1, 3))
fuselage_d2=  geometry.evaluate(fuselage.project(np.array([10.916, 2.945, 5.736]))).reshape((-1, 3))
fuselage_diameter = m3l.norm(fuselage_d2-fuselage_d1)

fuselage_drag_comp = DragComponent(
    component_type='fuselage',
    wetted_area=surface_area_list[2],
    characteristic_length=fuselage_length,
    characteristic_diameter=fuselage_diameter,
    Q=2.9,
)


# wing
wing_mid_le = geometry.evaluate(wing.project(np.array([8.892, 0., 8.633]))).reshape((-1, 3))
wing_mid_te = geometry.evaluate(wing.project(np.array([14.332, 0, 8.439]))).reshape((-1, 3))
wing_mid_chord_length = m3l.norm(wing_mid_le-wing_mid_te)

wing_le_right_mapped_array = geometry.evaluate(wing.project(wing_le_right)).reshape((-1, 3))
wing_le_left_mapped_array = geometry.evaluate(wing.project(wing_le_left)).reshape((-1, 3))
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
h_tail_mid_le = geometry.evaluate(h_tail.project(np.array([27.806, -6.520, 8.008]))).reshape((-1, 3))
h_tail_mid_te = geometry.evaluate(h_tail.project(np.array([30.050, -6.520, 8.008]))).reshape((-1, 3))
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
v_tail_mid_le = geometry.evaluate(v_tail.project(np.array([26.971, 0.0, 11.038]))).reshape((-1, 3))
v_tail_mid_te = geometry.evaluate(v_tail.project(np.array([31.302, 0.0, 11.038]))).reshape((-1, 3))
v_tail_chord_length = m3l.norm(v_tail_mid_le-v_tail_mid_te)

v_tail_drag_comp = DragComponent(
    component_type='wing',
    wetted_area=surface_area_list[5],
    characteristic_length=v_tail_chord_length,
    thickness_to_chord=0.115,
    x_cm=0.2,
    Q=1.5,
)

# boom
boom_l1 = geometry.evaluate(rlo_boom.project(np.array([20.000, -18.750, 7.613]))).reshape((-1, 3))
boom_l2 = geometry.evaluate(rlo_boom.project(np.array([12.000, -18.750, 7.613]))).reshape((-1, 3))
boom_length = m3l.norm(boom_l1-boom_l2)

boom_d1 = geometry.evaluate(rlo_boom.project(np.array([15.600, -19.250, 7.613]))).reshape((-1, 3))
boom_d2 = geometry.evaluate(rlo_boom.project(np.array([15.600, -18.250, 7.613]))).reshape((-1, 3))
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
hub_l1 = geometry.evaluate(rlo_hub.project(np.array([18.075, -18.750,9.525]))).reshape((-1, 3))
hub_l2 = geometry.evaluate(rlo_hub.project(np.array([20.325, -18.750,9.525]))).reshape((-1, 3))
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
blade_tip = geometry.evaluate(rlo_blade_2.project(np.array([14.200, -18.626, 9.040]))).reshape((-1, 3))
blade_hub = geometry.evaluate(rlo_blade_2.project(np.array([18.200, -18.512, 9.197]))).reshape((-1, 3))

blade_length = m3l.norm(blade_tip-blade_hub)
blade_drag_comp = DragComponent(
    component_type='flat_plate',
    characteristic_length=blade_length,
    wetted_area=surface_area_list[-1],
    multiplicity=16,
    Q=2,

)

drag_comp_list = [wing_drag_comp, fuselage_drag_comp, h_tail_drag_comp, v_tail_drag_comp,
                  blade_drag_comp, boom_drag_comp, hub_drag_comp]

S_ref = surface_area_list[3] / 2.1
# print(S_ref.value)
# 
wing_AR = wing_span**2 / S_ref
# endregion

# 

# region actuations
# cruise_geometry = geometry.copy()

# endregion



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
# vtail = cd.LiftingSurface(name='v_tail', spatial_representation=spatial_rep, primitive_names=vtail_primitive_names)

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


