# region Imports
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem

# Geometry
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
import array_mapper as am

# Solvers
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from caddee.utils.aircraft_models.pav.pav_weight import PavMassProperties

from caddee import GEOMETRY_FILES_FOLDER

import numpy as np
# endregion


ft2m = 0.3048

debug_geom_flag = False
visualize_flag = False

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

# region Geometry
file_name = 'mk27.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.plot()

# region Lifting surfaces

# Main Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Horizontal Wing']).keys())
main_wing = LiftingSurface(name='Horizontal Wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    main_wing.plot()
sys_rep.add_component(main_wing)

# Lower Frame Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Frame']).keys())
lower_wing = LiftingSurface(name='Lower Frame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    lower_wing.plot()
sys_rep.add_component(lower_wing)

# Upper Frame Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Top of Frame']).keys())
upper_wing = LiftingSurface(name='Top of Frame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    upper_wing.plot()
sys_rep.add_component(upper_wing)

# endregion

# region Rotors

# Prop pair #1 (upper)
pp1_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props']).keys())
pp1_disk = cd.Rotor(name='pp1_disk', spatial_representation=spatial_rep, primitive_names=pp1_disk_prim_names)
if debug_geom_flag:
    pp1_disk.plot()
sys_rep.add_component(pp1_disk)

# Prop pair #2 (middle)
pp2_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props']).keys())
pp2_disk = cd.Rotor(name='pp2_disk', spatial_representation=spatial_rep, primitive_names=pp2_disk_prim_names)
if debug_geom_flag:
    pp2_disk.plot()
sys_rep.add_component(pp2_disk)

# Prop pair #3 (lower)
pp3_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props']).keys())
pp3_disk = cd.Rotor(name='pp3_disk', spatial_representation=spatial_rep, primitive_names=pp3_disk_prim_names)
if debug_geom_flag:
    pp3_disk.plot()
sys_rep.add_component(pp3_disk)

# endregion

# endregion


# region Actuations
# # Tail FFD
# htail_geometry_primitives = htail.get_geometry_primitives()
# htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
#     htail_geometry_primitives,
#     num_control_points=(11, 2, 2), order=(4,2,2),
#     xyz_to_uvw_indices=(1,0,2)
# )
# htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block',
#                                   primitive=htail_ffd_bspline_volume,
#                                   embedded_entities=htail_geometry_primitives)
# htail_ffd_block.add_scale_v(name='htail_linear_taper',
#                             order=2, num_dof=3, value=np.array([0., 0., 0.]),
#                             cost_factor=1.)
# htail_ffd_block.add_rotation_u(name='htail_twist_distribution',
#                                connection_name='h_tail_act', order=1,
#                                num_dof=1, value=np.array([np.deg2rad(1.75)]))
# ffd_set = cd.SRBGFFDSet(
#     name='ffd_set',
#     ffd_blocks={htail_ffd_block.name : htail_ffd_block}
# )
# sys_param.add_geometry_parameterization(ffd_set)
# sys_param.setup()
# endregion


# region Meshes

# region Main Wing
num_wing_vlm = 21
num_chordwise_vlm = 5
point00 = np.array([2.862, 3.250,  0.000]) # * ft2m # Right tip leading edge (x,y,z)
point01 = np.array([3.080, 3.250,  0.000]) # * ft2m # Right tip trailing edge
point10 = np.array([2.651, 0.000,  0.000]) # * ft2m # Center leading Edge
point11 = np.array([3.200, 0.000,  0.000]) # * ft2m # Center trailing edge
point20 = np.array([2.862, -3.250, 0.000]) # * ft2m # Left tip leading edge
point21 = np.array([3.080, -3.250, 0.000]) # * ft2m # Left tip trailing edge

leading_edge_points = np.concatenate(
    (np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point10, point20, int(num_wing_vlm/2+1))),
    axis=0)
trailing_edge_points = np.concatenate(
    (np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point11, point21, int(num_wing_vlm/2+1))),
    axis=0)

leading_edge = main_wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
trailing_edge = main_wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

# Chord Surface
main_wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if debug_geom_flag:
    spatial_rep.plot_meshes([main_wing_chord_surface])

# Upper and lower surface
main_wing_upper_surface_wireframe = main_wing.project(main_wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)
main_wing_lower_surface_wireframe = main_wing.project(main_wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)

# Chamber surface
main_wing_camber_surface = am.linspace(main_wing_upper_surface_wireframe, main_wing_lower_surface_wireframe, 1)
main_wing_vlm_mesh_name = 'wing_vlm_mesh'
sys_rep.add_output(main_wing_vlm_mesh_name, main_wing_camber_surface)
if debug_geom_flag:
    spatial_rep.plot_meshes([main_wing_camber_surface])

# OML mesh
main_wing_oml_mesh = am.vstack((main_wing_upper_surface_wireframe, main_wing_lower_surface_wireframe))
main_wing_oml_mesh_name = 'main_wing_oml_mesh'
sys_rep.add_output(main_wing_oml_mesh_name, main_wing_oml_mesh)
if debug_geom_flag:
    spatial_rep.plot_meshes([main_wing_oml_mesh])
# endregion

# region Lower Wing
num_wing_vlm = 21
num_chordwise_vlm = 5
point00 = np.array([0.400,  1.500, -2.710]) # * ft2m # Right tip leading edge (x,y,z)
point01 = np.array([1.100, 1.500,  -2.710]) # * ft2m # Right tip trailing edge
point10 = np.array([0.400,  0.000, -2.710]) # * ft2m # Center Leading Edge
point11 = np.array([1.100, 0.000,  -2.710]) # * ft2m # Center Trailing edge
point20 = np.array([0.400, -1.500, -2.710]) # * ft2m # Left tip leading edge
point21 = np.array([1.100, -1.500, -2.710]) # * ft2m # Left tip

leading_edge_points = np.concatenate(
    (np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point10, point20, int(num_wing_vlm/2+1))),
    axis=0)
trailing_edge_points = np.concatenate(
    (np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point11, point21, int(num_wing_vlm/2+1))),
    axis=0)

leading_edge = lower_wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
trailing_edge = lower_wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

# Chord Surface
lower_wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if debug_geom_flag:
    spatial_rep.plot_meshes([lower_wing_chord_surface])

# Upper and lower surface
lower_wing_upper_surface_wireframe = main_wing.project(lower_wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)
lower_wing_lower_surface_wireframe = main_wing.project(lower_wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)

# Chamber surface
lower_wing_camber_surface = am.linspace(lower_wing_upper_surface_wireframe, lower_wing_lower_surface_wireframe, 1)
lower_wing_vlm_mesh_name = 'wing_vlm_mesh'
sys_rep.add_output(lower_wing_vlm_mesh_name, lower_wing_camber_surface)
if debug_geom_flag:
    spatial_rep.plot_meshes([lower_wing_camber_surface])

# OML mesh
lower_wing_oml_mesh = am.vstack((lower_wing_upper_surface_wireframe, lower_wing_lower_surface_wireframe))
lower_wing_oml_mesh_name = 'lower_wing_oml_mesh'
sys_rep.add_output(lower_wing_oml_mesh_name, lower_wing_oml_mesh)
if debug_geom_flag:
    spatial_rep.plot_meshes([lower_wing_oml_mesh])
# endregion

# region Upper Wing
num_wing_vlm = 21
num_chordwise_vlm = 5
point00 = np.array([4.033,  1.800, 2.720]) # * ft2m # Right tip leading edge (x,y,z)
point01 = np.array([4.725, 1.800,  2.720]) # * ft2m # Right tip trailing edge
point10 = np.array([4.033,  0.000, 2.720]) # * ft2m # Center Leading Edge
point11 = np.array([4.725, 0.000,  2.720]) # * ft2m # Center Trailing edge
point20 = np.array([4.033, -1.800, 2.720]) # * ft2m # Left tip leading edge
point21 = np.array([4.725, -1.800, 2.720]) # * ft2m # Left tip

leading_edge_points = np.concatenate(
    (np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point10, point20, int(num_wing_vlm/2+1))),
    axis=0)
trailing_edge_points = np.concatenate(
    (np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point11, point21, int(num_wing_vlm/2+1))),
    axis=0)

leading_edge = upper_wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
trailing_edge = upper_wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

# Chord Surface
upper_wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if debug_geom_flag:
    spatial_rep.plot_meshes([upper_wing_chord_surface])

# Upper and lower surface
upper_wing_upper_surface_wireframe = main_wing.project(upper_wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)
upper_wing_lower_surface_wireframe = main_wing.project(upper_wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)

# Chamber surface
upper_wing_camber_surface = am.linspace(upper_wing_upper_surface_wireframe, upper_wing_lower_surface_wireframe, 1)
upper_wing_vlm_mesh_name = 'wing_vlm_mesh'
sys_rep.add_output(upper_wing_vlm_mesh_name, upper_wing_camber_surface)
if debug_geom_flag:
    spatial_rep.plot_meshes([upper_wing_camber_surface])

# OML mesh
upper_wing_oml_mesh = am.vstack((upper_wing_upper_surface_wireframe, upper_wing_lower_surface_wireframe))
upper_wing_oml_mesh_name = 'wing_oml_mesh'
sys_rep.add_output(upper_wing_oml_mesh_name, upper_wing_oml_mesh)
if debug_geom_flag:
    spatial_rep.plot_meshes([upper_wing_oml_mesh])
# endregion

if visualize_flag:
    spatial_rep.plot_meshes([main_wing_camber_surface, lower_wing_camber_surface, upper_wing_camber_surface])

# region Pusher prop (upper pair) (coordinates are not set yet)
# Right
y11_right1 = pp1_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
y12_right1 = pp1_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
y21_right1 = pp1_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
y22_right1 = pp1_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)

pp_right1_disk_in_plane_y = am.subtract(y11_right1, y12_right1)
pp_right1_disk_in_plane_x = am.subtract(y21_right1, y22_right1)
pp_right1_disk_origin = pp1_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

sys_rep.add_output(f"{pp1_disk.parameters['name']}_in_plane_1", pp_right1_disk_in_plane_y)
sys_rep.add_output(f"{pp1_disk.parameters['name']}_in_plane_2", pp_right1_disk_in_plane_x)
sys_rep.add_output(f"{pp1_disk.parameters['name']}_origin", pp_right1_disk_origin)

# Left
y11_left1 = pp1_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
y12_left1 = pp1_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
y21_left1 = pp1_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
y22_left1 = pp1_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)

pp_left1_disk_in_plane_y = am.subtract(y11_left1, y12_left1)
pp_left1_disk_in_plane_x = am.subtract(y21_left1, y22_left1)
pp_left1_disk_origin = pp1_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

sys_rep.add_output(f"{pp1_disk.parameters['name']}_in_plane_1", pp_left1_disk_in_plane_y)
sys_rep.add_output(f"{pp1_disk.parameters['name']}_in_plane_2", pp_left1_disk_in_plane_x)
sys_rep.add_output(f"{pp1_disk.parameters['name']}_origin", pp_left1_disk_origin)
# endregion

# region Pusher prop (middle pair) (coordinates are not set yet)
# Right
y11_right2 = pp2_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
y12_right2 = pp2_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
y21_right2 = pp2_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
y22_right2 = pp2_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)

pp_right2_disk_in_plane_y = am.subtract(y11_right2, y12_right2)
pp_right2_disk_in_plane_x = am.subtract(y21_right2, y22_right2)
pp_right2_disk_origin = pp2_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

sys_rep.add_output(f"{pp2_disk.parameters['name']}_in_plane_1", pp_right2_disk_in_plane_y)
sys_rep.add_output(f"{pp2_disk.parameters['name']}_in_plane_2", pp_right2_disk_in_plane_x)
sys_rep.add_output(f"{pp2_disk.parameters['name']}_origin", pp_right2_disk_origin)

# Left
y11_left2 = pp2_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
y12_left2 = pp2_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
y21_left2 = pp2_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
y22_left2 = pp2_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)

pp_left2_disk_in_plane_y = am.subtract(y11_left2, y12_left2)
pp_left2_disk_in_plane_x = am.subtract(y21_left2, y22_left2)
pp_left2_disk_origin = pp2_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

sys_rep.add_output(f"{pp2_disk.parameters['name']}_in_plane_1", pp_left2_disk_in_plane_y)
sys_rep.add_output(f"{pp2_disk.parameters['name']}_in_plane_2", pp_left2_disk_in_plane_x)
sys_rep.add_output(f"{pp2_disk.parameters['name']}_origin", pp_left2_disk_origin)
# endregion

# region Pusher prop (lower pair) (coordinates are not set yet)
# Right
y11_right3 = pp3_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
y12_right3 = pp3_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
y21_right3 = pp3_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
y22_right3 = pp3_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)

pp_right3_disk_in_plane_y = am.subtract(y11_right3, y12_right3)
pp_right3_disk_in_plane_x = am.subtract(y21_right3, y22_right3)
pp_right3_disk_origin = pp3_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

sys_rep.add_output(f"{pp3_disk.parameters['name']}_in_plane_1", pp_right3_disk_in_plane_y)
sys_rep.add_output(f"{pp3_disk.parameters['name']}_in_plane_2", pp_right3_disk_in_plane_x)
sys_rep.add_output(f"{pp3_disk.parameters['name']}_origin", pp_right3_disk_origin)

# Left
y11_left3 = pp3_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
y12_left3 = pp3_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
y21_left3 = pp3_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
y22_left3 = pp3_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)

pp_left3_disk_in_plane_y = am.subtract(y11_left3, y12_left3)
pp_left3_disk_in_plane_x = am.subtract(y21_left3, y22_left3)
pp_left3_disk_origin = pp3_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

sys_rep.add_output(f"{pp3_disk.parameters['name']}_in_plane_1", pp_left3_disk_in_plane_y)
sys_rep.add_output(f"{pp3_disk.parameters['name']}_in_plane_2", pp_left3_disk_in_plane_x)
sys_rep.add_output(f"{pp3_disk.parameters['name']}_origin", pp_left3_disk_origin)
# endregion

# endregion


# region Sizing
mk27_wt = PavMassProperties() # mk27 mass properties??
mass, cg, I = mk27_wt.evaluate()

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
# endregion


# region Mission

design_scenario = cd.DesignScenario(name='aircraft_trim')

# region Cruise condition 

# The proposed concept of operations (CONOPS) for the Model MK27-2 identifies a maximum operating
# altitude of 400 feet above ground level (AGL), a maximum cruise speed of 60 knots, operations beyond
# visual line of sight (BVLOS) of the pilot, and operations over human beings. cruise at ~50 mph

cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=400*ft2m)
cruise_condition.set_module_input(name='mach_number', val=0.0651662)  # 50 mph = 0.0651662 Mach
cruise_condition.set_module_input(name='range', val=14484.1)  # 9 miles = 14484.1 m
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='wind_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600*ft2m]))

cruise_ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(cruise_ac_states)

# region Propulsion
pusher_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=5,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=False,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=4000)
bem_model.set_module_input('propeller_radius', val=3.97727/2*ft2m)
bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625]))
bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
cruise_model.register_output(bem_forces)
cruise_model.register_output(bem_moments)
# endregion

# region Inertial loads
inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=cruise_ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)
# endregion

# region Aerodynamics
vlm_model = VASTFluidSover(
    surface_names=[
        wing_vlm_mesh_name,
        htail_vlm_mesh_name,
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
        ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.55, 0.0]
)
vlm_panel_forces, vlm_forces, vlm_moments  = vlm_model.evaluate(ac_states=cruise_ac_states)
cruise_model.register_output(vlm_forces)
cruise_model.register_output(vlm_moments)
# endregion

# Total loads
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    inertial_forces, inertial_moments,
    vlm_forces, vlm_moments,
    bem_forces, bem_moments
)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# Equations of motions
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass,
    total_cg_vector=total_cg,
    total_inertia_tensor=total_inertia,
    total_forces=total_forces,
    total_moments=total_moments,
    ac_states=cruise_ac_states
)

cruise_model.register_output(trim_residual)

# Add cruise m3l model to cruise condition
cruise_condition.add_m3l_model('cruise_model', cruise_model)

# Add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)
# endregion

system_model.add_design_scenario(design_scenario=design_scenario)
# endregion

caddee_csdl_model = caddee.assemble_csdl()

caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1.)

# region Optimization Setup
caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
caddee_csdl_model.add_constraint(
    name='system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta',
    equals=0.8)
caddee_csdl_model.add_constraint(
    name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D',
    equals=8.,
    scaler=1e-1
)
# endregion

# Create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()
# sim.compute_total_derivatives()
# sim.check_totals()

print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])

# system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.FOM

prob = CSDLProblem(problem_name='lpc', simulator=sim)
optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
optimizer.solve()
optimizer.print_results()

print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
print('Pitch: ', np.rad2deg(sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
print('RPM: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm'])
print('Horizontal tail actuation: ', np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))

print(sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta'])
print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])