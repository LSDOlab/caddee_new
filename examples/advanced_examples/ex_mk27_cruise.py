# region Imports
import caddee.api as cd
import m3l
import csdl
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
file_name = 'AmazonPrime.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.plot()

# region Lifting surfaces

# Main Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Horizontal Wing']).keys())
main_wing = LiftingSurface(name='HorizontalWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    main_wing.plot()
sys_rep.add_component(main_wing)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Upper Wing']).keys())
TopWing = LiftingSurface(name='TopWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Wing']).keys())
BotWing = LiftingSurface(name='BotWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Vertical Stabilizer']).keys())
VertStab = LiftingSurface(name='VertStab', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

sys_rep.add_component(TopWing)
sys_rep.add_component(BotWing)
sys_rep.add_component(VertStab)

# Lower Frame Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Frame']).keys())
low_wing = LiftingSurface(name='LowerWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    low_wing.plot()
sys_rep.add_component(low_wing)

# Upper Frame Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Top of Frame']).keys())
top_wing = LiftingSurface(name='UpperWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    top_wing.plot()
sys_rep.add_component(top_wing)

# endregion

# region Rotors
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props, 0', 'Middle Props, 1', 'Middle Props, 2', 'Middle Props, 3']).keys())
ppm_left = cd.Rotor(name='ppm_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppm_left)
#ppm_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props, 4', 'Middle Props, 5', 'Middle Props, 6', 'Middle Props, 7']).keys())
ppm_right = cd.Rotor(name='ppm_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppm_right)
#ppm_right.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props, 4', 'Upper Props, 5', 'Upper Props, 6', 'Upper Props, 7']).keys())
ppu_left = cd.Rotor(name='ppu_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppu_left)
#ppu_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props, 0', 'Upper Props, 1', 'Upper Props, 2', 'Upper Props, 3']).keys())
ppu_right = cd.Rotor(name='ppu_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppu_right)
#ppu_right.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props, 4', 'Lower Props, 5', 'Lower Props, 6', 'Lower Props, 7']).keys())
ppl_left = cd.Rotor(name='ppl_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppl_left)
#ppl_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props, 0', 'Lower Props, 1', 'Lower Props, 2', 'Lower Props, 3']).keys())
ppl_right = cd.Rotor(name='ppl_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppl_right)

# endregion

# endregion

# region FFD
ppm_left_geometry_primitives = ppm_left.get_geometry_primitives()
ppm_left_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppm_left_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppm_left_ffd_block = cd.SRBGFFDBlock(name='ppm_left_ffd_block',
                                  primitive=ppm_left_ffd_bspline_volume,
                                  embedded_entities=ppm_left_geometry_primitives)
ppm_left_ffd_block.add_scale_v(name='ppm_left_scale_v',order=1, num_dof=1, cost_factor=1.)
ppm_left_ffd_block.add_scale_w(name='ppm_left_scale_w', order=1, num_dof=1)
# ppm_left_ffd_block.plot()

ppm_right_geometry_primitives = ppm_right.get_geometry_primitives()
ppm_right_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppm_right_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppm_right_ffd_block = cd.SRBGFFDBlock(name='ppm_right_ffd_block',
                                  primitive=ppm_right_ffd_bspline_volume,
                                  embedded_entities=ppm_right_geometry_primitives)
ppm_right_ffd_block.add_scale_v(name='ppm_right_scale_v',order=1, num_dof=1, cost_factor=1.)
ppm_right_ffd_block.add_scale_w(name='ppm_right_scale_w', order=1, num_dof=1)

ppu_left_geometry_primitives = ppu_left.get_geometry_primitives()
ppu_left_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppu_left_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppu_left_ffd_block = cd.SRBGFFDBlock(name='ppu_left_ffd_block',
                                  primitive=ppu_left_ffd_bspline_volume,
                                  embedded_entities=ppu_left_geometry_primitives)
ppu_left_ffd_block.add_scale_v(name='ppu_left_scale_v',order=1, num_dof=1, cost_factor=1.)
ppu_left_ffd_block.add_scale_w(name='ppu_left_scale_w', order=1, num_dof=1)
# ppu_left_ffd_block.plot()

ppu_right_geometry_primitives = ppu_right.get_geometry_primitives()
ppu_right_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppu_right_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppu_right_ffd_block = cd.SRBGFFDBlock(name='ppu_right_ffd_block',
                                  primitive=ppu_right_ffd_bspline_volume,
                                  embedded_entities=ppu_right_geometry_primitives)
ppu_right_ffd_block.add_scale_v(name='ppu_right_scale_v',order=1, num_dof=1, cost_factor=1.)
ppu_right_ffd_block.add_scale_w(name='ppu_right_scale_w', order=1, num_dof=1)


ppl_left_geometry_primitives = ppl_left.get_geometry_primitives()
ppl_left_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppl_left_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppl_left_ffd_block = cd.SRBGFFDBlock(name='ppl_left_ffd_block',
                                  primitive=ppl_left_ffd_bspline_volume,
                                  embedded_entities=ppl_left_geometry_primitives)
ppl_left_ffd_block.add_scale_v(name='ppl_left_scale_v',order=1, num_dof=1, cost_factor=1.)
ppl_left_ffd_block.add_scale_w(name='ppl_left_scale_w', order=1, num_dof=1)


ppl_right_geometry_primitives = ppl_right.get_geometry_primitives()
ppl_right_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppl_right_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppl_right_ffd_block = cd.SRBGFFDBlock(name='ppl_right_ffd_block',
                                  primitive=ppl_right_ffd_bspline_volume,
                                  embedded_entities=ppl_right_geometry_primitives)
ppl_right_ffd_block.add_scale_v(name='ppl_right_scale_v',order=1, num_dof=1, cost_factor=1.)
ppl_right_ffd_block.add_scale_w(name='ppl_right_scale_w', order=1, num_dof=1)

# endregion


# region Meshes

vlm_main_wing_mesh_name = 'vlm_main_wing_mesh'
vlm_top_wing_mesh_name = 'vlm_top_wing_mesh'
vlm_low_wing_mesh_name = 'vlm_bot_wing_mesh'

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
main_wing_vlm_mesh_name = vlm_main_wing_mesh_name
sys_rep.add_output(main_wing_vlm_mesh_name, main_wing_camber_surface)
if debug_geom_flag:
    spatial_rep.plot_meshes([main_wing_camber_surface])

# # OML mesh
# main_wing_oml_mesh = am.vstack((main_wing_upper_surface_wireframe, main_wing_lower_surface_wireframe))
# main_wing_oml_mesh_name = 'main_wing_oml_mesh'
# sys_rep.add_output(main_wing_oml_mesh_name, main_wing_oml_mesh)
# if debug_geom_flag:
#     spatial_rep.plot_meshes([main_wing_oml_mesh])

main_wing_area = am.wireframe_area(main_wing_chord_surface)

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

leading_edge = low_wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
trailing_edge = low_wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

# Chord Surface
lower_wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if debug_geom_flag:
    spatial_rep.plot_meshes([lower_wing_chord_surface])

# Upper and lower surface
lower_wing_upper_surface_wireframe = low_wing.project(lower_wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)
lower_wing_lower_surface_wireframe = low_wing.project(lower_wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)

# Chamber surface
lower_wing_camber_surface = am.linspace(lower_wing_upper_surface_wireframe, lower_wing_lower_surface_wireframe, 1)
lower_wing_vlm_mesh_name = vlm_low_wing_mesh_name
sys_rep.add_output(lower_wing_vlm_mesh_name, lower_wing_camber_surface)
if debug_geom_flag:
    spatial_rep.plot_meshes([lower_wing_camber_surface])

# # OML mesh
# lower_wing_oml_mesh = am.vstack((lower_wing_upper_surface_wireframe, lower_wing_lower_surface_wireframe))
# lower_wing_oml_mesh_name = 'lower_wing_oml_mesh'
# sys_rep.add_output(lower_wing_oml_mesh_name, lower_wing_oml_mesh)
# if debug_geom_flag:
#     spatial_rep.plot_meshes([lower_wing_oml_mesh])

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

leading_edge = top_wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
trailing_edge = top_wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

# Chord Surface
upper_wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if debug_geom_flag:
    spatial_rep.plot_meshes([upper_wing_chord_surface])

# Upper and lower surface
upper_wing_upper_surface_wireframe = top_wing.project(upper_wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)
upper_wing_lower_surface_wireframe = top_wing.project(upper_wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=debug_geom_flag, max_iterations=200)

# Chamber surface
upper_wing_camber_surface = am.linspace(upper_wing_upper_surface_wireframe, upper_wing_lower_surface_wireframe, 1)
upper_wing_vlm_mesh_name = vlm_top_wing_mesh_name
sys_rep.add_output(upper_wing_vlm_mesh_name, upper_wing_camber_surface)
if debug_geom_flag:
    spatial_rep.plot_meshes([upper_wing_camber_surface])

# # OML mesh
# upper_wing_oml_mesh = am.vstack((upper_wing_upper_surface_wireframe, upper_wing_lower_surface_wireframe))
# upper_wing_oml_mesh_name = 'wing_oml_mesh'
# sys_rep.add_output(upper_wing_oml_mesh_name, upper_wing_oml_mesh)
# if debug_geom_flag:
#     spatial_rep.plot_meshes([upper_wing_oml_mesh])

# endregion

if visualize_flag:
    spatial_rep.plot_meshes([main_wing_camber_surface, lower_wing_camber_surface, upper_wing_camber_surface])

# region pusher prop (pp) meshes

# disk: middle 
#left
y11 = ppm_left.project(np.array([2.5,-.93,0]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = ppm_left.project(np.array([2.5,-2.57,0]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = ppm_left.project(np.array([2.5,-1.75,0.82]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = ppm_left.project(np.array([2.5,-1.75,-0.82]), direction=np.array([-1., 0., 0.]), plot=False)
ppm_left_plane_y = am.subtract(y11, y12)
ppm_left_plane_x = am.subtract(y21, y22)
ppm_left_origin = ppm_left.project(np.array([2.5,-1.75,0]), direction=np.array([-1., 0., 0.]))
sys_rep.add_output(f"{ppm_left.parameters['name']}_in_plane_1", ppm_left_plane_y)
sys_rep.add_output(f"{ppm_left.parameters['name']}_in_plane_2", ppm_left_plane_x)
sys_rep.add_output(f"{ppm_left.parameters['name']}_origin", ppm_left_origin)

#right
y11 = ppm_right.project(np.array([2.5,0.93,0]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = ppm_right.project(np.array([2.5,2.57,0]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = ppm_right.project(np.array([2.5,1.75,0.82]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = ppm_right.project(np.array([2.5,1.75,-0.82]), direction=np.array([-1., 0., 0.]), plot=False)
ppm_right_plane_y = am.subtract(y11, y12)
ppm_right_plane_x = am.subtract(y21, y22)
ppm_right_origin = ppm_right.project(np.array([2.5,1.75,0]), direction=np.array([-1., 0., 0.]))
sys_rep.add_output(f"{ppm_right.parameters['name']}_in_plane_1", ppm_right_plane_y)
sys_rep.add_output(f"{ppm_right.parameters['name']}_in_plane_2", ppm_right_plane_x)
sys_rep.add_output(f"{ppm_right.parameters['name']}_origin", ppm_right_origin)

# disk: uppers
#left
y11 = ppu_left.project(np.array([3.976,-1.458,2.193]), direction=np.array([-.683,0.259,0.683]), plot=False)
y12 = ppu_left.project(np.array([3.024,-0.842,1.007]), direction=np.array([-.683,0.259,0.683]), plot=False)
y21 = ppu_left.project(np.array([3.136,-1.88,1.513]), direction=np.array([-.683,0.259,0.683]), plot=False)
y22 = ppu_left.project(np.array([3.846,-0.42,1.687]), direction=np.array([-.683,0.259,0.683]), plot=False)
ppu_left_plane_y = am.subtract(y11, y12)
ppu_left_plane_x = am.subtract(y21, y22)
ppu_left_origin = ppu_left.project(np.array([3.5,-1.15,1.6]), direction=np.array([-.683,0.259,0.683]))
sys_rep.add_output(f"{ppu_left.parameters['name']}_in_plane_1", ppu_left_plane_y)
sys_rep.add_output(f"{ppu_left.parameters['name']}_in_plane_2", ppu_left_plane_x)
sys_rep.add_output(f"{ppu_left.parameters['name']}_origin", ppu_left_origin)

#right
y11 = ppu_right.project(np.array([3.976,1.458,2.193]), direction=np.array([-.683,-0.259,0.683]), plot=False)
y12 = ppu_right.project(np.array([3.024,0.842,1.007]), direction=np.array([-.683,-0.259,0.683]), plot=False)
y21 = ppu_right.project(np.array([3.136,1.88,1.513]), direction=np.array([-.683,-0.259,0.683]), plot=False)
y22 = ppu_right.project(np.array([3.846,0.42,1.687]), direction=np.array([-.683,-0.259,0.683]), plot=False)
ppu_right_plane_y = am.subtract(y11, y12)
ppu_right_plane_x = am.subtract(y21, y22)
ppu_right_origin = ppu_right.project(np.array([3.5,1.15,1.6]), direction=np.array([-.683,-0.259,0.683]))
sys_rep.add_output(f"{ppu_right.parameters['name']}_in_plane_1", ppu_right_plane_y)
sys_rep.add_output(f"{ppu_right.parameters['name']}_in_plane_2", ppu_right_plane_x)
sys_rep.add_output(f"{ppu_right.parameters['name']}_origin", ppu_right_origin)

# disk: lowers
#left
y11 = ppl_left.project(np.array([1.66,-0.775,-1.03]), direction=np.array([-0.75,0.5,0.433]), plot=False)
y12 = ppl_left.project(np.array([0.84,-0.775,-2.45]), direction=np.array([-0.75,0.5,0.433]), plot=False)
y21 = ppl_left.project(np.array([0.895,-0.065,-1.535]), direction=np.array([-0.75,0.5,0.433]), plot=False)
y22 = ppl_left.project(np.array([1.605,-1.485,-1.945]), direction=np.array([-0.75,0.5,0.433]), plot=False)
ppl_left_plane_y = am.subtract(y11, y12)
ppl_left_plane_x = am.subtract(y21, y22)
ppl_left_origin = ppl_left.project(np.array([1.25,0.775,-1.74]), direction=np.array([-0.75,0.5,0.433]))
sys_rep.add_output(f"{ppl_left.parameters['name']}_in_plane_1", ppl_left_plane_y)
sys_rep.add_output(f"{ppl_left.parameters['name']}_in_plane_2", ppl_left_plane_x)
sys_rep.add_output(f"{ppl_left.parameters['name']}_origin", ppl_left_origin)

#right
y11 = ppl_right.project(np.array([1.66,0.775,-1.03]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y12 = ppl_right.project(np.array([0.84,0.775,-2.45]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y21 = ppl_right.project(np.array([0.895,0.065,-1.535]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y22 = ppl_right.project(np.array([1.605,1.485,-1.945]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
ppl_right_plane_y = am.subtract(y11, y12)
ppl_right_plane_x = am.subtract(y21, y22)
ppl_right_origin = ppl_right.project(np.array([1.25,-0.775,-1.74]), direction=np.array([-0.75,-0.5,0.433]))
sys_rep.add_output(f"{ppl_right.parameters['name']}_in_plane_1", ppl_right_plane_y)
sys_rep.add_output(f"{ppl_right.parameters['name']}_in_plane_2", ppl_right_plane_x)
sys_rep.add_output(f"{ppl_right.parameters['name']}_origin", ppl_right_origin)

# endregion

# endregion
ppm_left_radius_1 = am.norm(ppm_left_plane_x/2)
ppm_left_radius_2 = am.norm(ppm_left_plane_y/2)
sys_param.add_input(name='ppm_left_radius_1', quantity=ppm_left_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppm_left_radius_2', quantity=ppm_left_radius_2, value=np.array([0.5]))

ppm_right_radius_1 = am.norm(ppm_right_plane_x/2)
ppm_right_radius_2 = am.norm(ppm_right_plane_y/2)
sys_param.add_input(name='ppm_right_radius_1', quantity=ppm_right_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppm_right_radius_2', quantity=ppm_right_radius_2, value=np.array([0.5]))

ppu_left_radius_1 = am.norm(ppu_left_plane_x/2)
ppu_left_radius_2 = am.norm(ppu_left_plane_y/2)
sys_param.add_input(name='ppu_left_radius_1', quantity=ppu_left_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppu_left_radius_2', quantity=ppu_left_radius_2, value=np.array([0.5]))

ppu_right_radius_1 = am.norm(ppu_right_plane_x/2)
ppu_right_radius_2 = am.norm(ppu_right_plane_y/2)
sys_param.add_input(name='ppu_right_radius_1', quantity=ppu_right_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppu_right_radius_2', quantity=ppu_right_radius_2, value=np.array([0.5]))

ppl_left_radius_1 = am.norm(ppl_left_plane_x/2)
ppl_left_radius_2 = am.norm(ppl_left_plane_y/2)
sys_param.add_input(name='ppl_left_radius_1', quantity=ppl_left_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppl_left_radius_2', quantity=ppl_left_radius_2, value=np.array([0.5]))

ppl_right_radius_1 = am.norm(ppl_right_plane_x/2)
ppl_right_radius_2 = am.norm(ppl_right_plane_y/2)
sys_param.add_input(name='ppl_right_radius_1', quantity=ppl_right_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppl_right_radius_2', quantity=ppl_right_radius_2, value=np.array([0.5]))

# region setup parameterization
ffd_set = cd.SRBGFFDSet(
    name='ffd_set',
    ffd_blocks={ppm_left_ffd_block.name : ppm_left_ffd_block,
                ppm_right_ffd_block.name : ppm_right_ffd_block,
                ppu_left_ffd_block.name : ppu_left_ffd_block,
                ppu_right_ffd_block.name : ppu_right_ffd_block,
                ppl_left_ffd_block.name : ppl_left_ffd_block,
                ppl_right_ffd_block.name : ppl_right_ffd_block,
                }
)
sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()

# # TEMPORARY
# system_representation_model = sys_rep.assemble_csdl()
# system_parameterization_model = sys_param.assemble_csdl()

# my_model = csdl.Model()
# my_model.add(system_parameterization_model, 'system_parameterization')
# my_model.add(system_representation_model, 'system_representation')

# sim = Simulator(my_model, analytics=True, display_scripts=True)
# sim.run()
# exit()
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
# cruise_model.register_output(cruise_ac_states)

# region Propulsion
ppm_left_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=3,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=True,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
disk_prefix = 'ppm_left'
ppm_left_bem_model = BEM(disk_prefix=disk_prefix, blade_prefix=disk_prefix, component=ppm_left, mesh=ppm_left_bem_mesh)
ppm_left_bem_model.set_module_input('rpm', val=4000)
ppm_left_bem_model.set_module_input(f'{disk_prefix}_in_plane_1', val=ppm_left_plane_y.value)
ppm_left_bem_model.set_module_input(f'{disk_prefix}_in_plane_2', val=ppm_left_plane_x.value)
ppm_left_bem_model.set_module_input(f'{disk_prefix}_origin', val=ppm_left_origin.value)
ppm_left_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
ppm_left_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
ppm_left_bem_forces, ppm_left_bem_moments, _, _, _, _, _, _ = ppm_left_bem_model.evaluate(ac_states=cruise_ac_states)
cruise_model.register_output(ppm_left_bem_forces)
cruise_model.register_output(ppm_left_bem_moments)

ppm_right_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=3,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=True,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
disk_prefix = 'ppm_right'
ppm_right_bem_model = BEM(disk_prefix=disk_prefix, blade_prefix=disk_prefix, component=ppm_right, mesh=ppm_right_bem_mesh)
ppm_right_bem_model.set_module_input('rpm', val=4000)
ppm_right_bem_model.set_module_input(f'{disk_prefix}_in_plane_1', val=ppm_left_plane_y.value)
ppm_right_bem_model.set_module_input(f'{disk_prefix}_in_plane_2', val=ppm_left_plane_x.value)
ppm_right_bem_model.set_module_input(f'{disk_prefix}_origin', val=ppm_left_origin.value)
ppm_right_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
ppm_right_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
ppm_right_bem_forces, ppm_right_bem_moments, _, _, _, _, _, _ = ppm_right_bem_model.evaluate(ac_states=cruise_ac_states)
# cruise_model.register_output(ppm_right_bem_forces)
# cruise_model.register_output(ppm_right_bem_moments)

ppu_left_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=3,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=True,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
disk_prefix = 'ppu_left'
ppu_left_bem_model = BEM(disk_prefix=disk_prefix, blade_prefix=disk_prefix, component=ppu_left, mesh=ppu_left_bem_mesh)
ppu_left_bem_model.set_module_input('rpm', val=4000)
ppu_left_bem_model.set_module_input(f'{disk_prefix}_in_plane_1', val=ppm_left_plane_y.value)
ppu_left_bem_model.set_module_input(f'{disk_prefix}_in_plane_2', val=ppm_left_plane_x.value)
ppu_left_bem_model.set_module_input(f'{disk_prefix}_origin', val=ppm_left_origin.value)
ppu_left_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
ppu_left_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
ppu_left_bem_forces, ppu_left_bem_moments, _, _, _, _, _, _ = ppu_left_bem_model.evaluate(ac_states=cruise_ac_states)
# cruise_model.register_output(ppu_left_bem_forces)
# cruise_model.register_output(ppu_left_bem_moments)

ppu_right_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=3,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=True,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
disk_prefix = 'ppu_right'
ppu_right_bem_model = BEM(disk_prefix=disk_prefix, blade_prefix=disk_prefix, component=ppu_right, mesh=ppu_right_bem_mesh)
ppu_right_bem_model.set_module_input('rpm', val=4000)
ppu_right_bem_model.set_module_input(f'{disk_prefix}_in_plane_1', val=ppm_left_plane_y.value)
ppu_right_bem_model.set_module_input(f'{disk_prefix}_in_plane_2', val=ppm_left_plane_x.value)
ppu_right_bem_model.set_module_input(f'{disk_prefix}_origin', val=ppm_left_origin.value)
ppu_right_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
ppu_right_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
ppu_right_bem_forces, ppu_right_bem_moments, _, _, _, _, _, _ = ppu_right_bem_model.evaluate(ac_states=cruise_ac_states)
# cruise_model.register_output(ppu_right_bem_forces)
# cruise_model.register_output(ppu_right_bem_moments)

ppl_left_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=3,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=True,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
disk_prefix = 'ppl_left'
ppl_left_bem_model = BEM(disk_prefix=disk_prefix, blade_prefix=disk_prefix, component=ppl_left, mesh=ppl_left_bem_mesh)
ppl_left_bem_model.set_module_input('rpm', val=4000)
ppl_left_bem_model.set_module_input(f'{disk_prefix}_in_plane_1', val=ppm_left_plane_y.value)
ppl_left_bem_model.set_module_input(f'{disk_prefix}_in_plane_2', val=ppm_left_plane_x.value)
ppl_left_bem_model.set_module_input(f'{disk_prefix}_origin', val=ppm_left_origin.value)
ppl_left_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
ppl_left_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
ppl_left_bem_forces, ppl_left_bem_moments, _, _, _, _, _, _ = ppl_left_bem_model.evaluate(ac_states=cruise_ac_states)
# cruise_model.register_output(ppl_left_bem_forces)
# cruise_model.register_output(ppl_left_bem_moments)

ppl_right_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=3,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=True,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)
disk_prefix = 'ppl_right'
ppl_right_bem_model = BEM(disk_prefix=disk_prefix, blade_prefix=disk_prefix, component=ppl_right, mesh=ppl_right_bem_mesh)
ppl_right_bem_model.set_module_input('rpm', val=4000)
ppl_right_bem_model.set_module_input(f'{disk_prefix}_in_plane_1', val=ppm_left_plane_y.value)
ppl_right_bem_model.set_module_input(f'{disk_prefix}_in_plane_2', val=ppm_left_plane_x.value)
ppl_right_bem_model.set_module_input(f'{disk_prefix}_origin', val=ppm_left_origin.value)
ppl_right_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
ppl_right_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )
ppl_right_bem_forces, ppl_right_bem_moments, _, _, _, _, _, _ = ppl_right_bem_model.evaluate(ac_states=cruise_ac_states)
# cruise_model.register_output(ppl_right_bem_forces)
# cruise_model.register_output(ppl_right_bem_moments)
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
        vlm_main_wing_mesh_name,
        vlm_top_wing_mesh_name,
        vlm_low_wing_mesh_name
    ],
    surface_shapes=[
        (1, ) + main_wing_camber_surface.evaluate().shape[1:],
        (1, ) + upper_wing_camber_surface.evaluate().shape[1:],
        (1, ) + lower_wing_camber_surface.evaluate().shape[1:],
        ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.55, 0.0, 0.0]
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
    ppm_left_bem_forces, ppm_left_bem_moments,
    # ppm_right_bem_forces, ppm_right_bem_moments,
    # ppu_left_bem_forces, ppu_left_bem_moments,
    # ppu_right_bem_forces, ppu_right_bem_moments,
    # ppl_left_bem_forces, ppl_left_bem_moments,
    # ppl_right_bem_forces, ppl_right_bem_moments,
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

# caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
# caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1.)

# # region Optimization Setup
caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
# caddee_csdl_model.add_constraint(
#     name='system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta',
#     equals=0.8)
# # endregion

objective_model = csdl.Model()
# Whoops, this is for hover.
# upper_fom = objective_model.declare_variable('upper_fom')
# mid_fom = objective_model.declare_variable('mid_fom')
# lower_fom = objective_model.declare_variable('lower_fom')

upper_prop_efficiency = objective_model.declare_variable('upper_prop_efficiency')
mid_prop_efficiency = objective_model.declare_variable('mid_prop_efficiency')
lower_prop_efficiency = objective_model.declare_variable('lower_prop_efficiency')

prop_efficiencies = objective_model.create_output(name='prop_efficiencies', shape=(3,))
prop_efficiencies[0] = upper_prop_efficiency
prop_efficiencies[1] = mid_prop_efficiency
prop_efficiencies[2] = lower_prop_efficiency
cruise_objective = csdl.pnorm(prop_efficiencies)
objective_model.register_output('cruise_objective', cruise_objective)

mk27_model = csdl.Model()
mk27_model.add(caddee_csdl_model, 'caddee_model')
mk27_model.add(objective_model, 'objective_model')

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