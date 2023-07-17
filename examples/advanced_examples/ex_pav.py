# region Imports
import caddee.api as cd

# Geometry
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
import array_mapper as am

from caddee import GEOMETRY_FILES_FOLDER

import numpy as np
# endregion


plots_flag = True

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

# region Geometry
file_name = 'pav.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

# region Lifting surfaces
# Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
wing = LiftingSurface(name='Wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if plots_flag:
    wing.plot()
sys_rep.add_component(wing)

# Horizontal tail
tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Stabilizer']).keys())
htail = cd.LiftingSurface(name='HTail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
if plots_flag:
    htail.plot()
sys_rep.add_component(htail)
# endregion

# region Rotors
# Pusher prop
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropPusher']).keys())
pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
if plots_flag:
    pp_disk.plot()
sys_rep.add_component(pp_disk)
# endregion

# endregion

# region Actuations
# Tail FFD
htail_geometry_primitives = htail.get_geometry_primitives()
htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    htail_geometry_primitives,
    num_control_points=(11, 2, 2), order=(4,2,2),
    xyz_to_uvw_indices=(1,0,2)
)
htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block',
                                  primitive=htail_ffd_bspline_volume,
                                  embedded_entities=htail_geometry_primitives)
htail_ffd_block.add_scale_v(name='htail_linear_taper',
                            order=2, num_dof=3, value=np.array([0., 0., 0.]),
                            cost_factor=1.)
htail_ffd_block.add_rotation_u(name='htail_twist_distribution',
                               connection_name='h_tail_act', order=1,
                               num_dof=1, value=np.array([np.deg2rad(1.75)]))
ffd_set = cd.SRBGFFDSet(
    name='ffd_set',
    ffd_blocks={htail_ffd_block.name : htail_ffd_block}
)
sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()
# endregion

# region Meshes

# region Wing
num_wing_vlm = 21
num_chordwise_vlm = 5
point00 = np.array([10.261, 17.596,  2.500 + 0.1]) # * ft2m # Right tip leading edge
point01 = np.array([13.276, 17.596,  2.500]) # * ft2m # Right tip trailing edge
point10 = np.array([10.278, 0.0000,  2.500 + 0.1]) # * ft2m # Center Leading Edge
point11 = np.array([17.030, 0.0000,  2.500]) # * ft2m # Center Trailing edge
point20 = np.array([10.261, -17.596, 2.500 + 0.1]) # * ft2m # Left tip leading edge
point21 = np.array([13.276, -17.596, 2.500]) # * ft2m # Left tip trailing edge

leading_edge_points = np.concatenate(
    (np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point10, point20, int(num_wing_vlm/2+1))),
    axis=0)
trailing_edge_points = np.concatenate(
    (np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point11, point21, int(num_wing_vlm/2+1))),
    axis=0)

leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=plots_flag)
trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=plots_flag)

# Chord Surface
wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if plots_flag:
    spatial_rep.plot_meshes([wing_chord_surface])

# Upper and lower surface
wing_upper_surface_wireframe = wing.project(wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=plots_flag, max_iterations=200)
wing_lower_surface_wireframe = wing.project(wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=plots_flag, max_iterations=200)

# Chamber surface
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
wing_vlm_mesh_name = 'wing_vlm_mesh'
sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
if plots_flag:
    spatial_rep.plot_meshes([wing_camber_surface])

# OML mesh
wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
wing_oml_mesh_name = 'wing_oml_mesh'
sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
if plots_flag:
    spatial_rep.plot_meshes([wing_oml_mesh])
# endregion

# region Tail
# endregion

# region Canard
# endregion

# endregion