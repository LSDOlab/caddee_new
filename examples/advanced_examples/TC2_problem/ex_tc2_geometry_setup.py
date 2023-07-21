import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee.core.caddee_core.system_representation.prescribed_actuations import PrescribedRotation
from caddee import IMPORTS_FILES_FOLDER
import array_mapper as am
import time


lpc_rep = cd.SystemRepresentation()
lpc_param = cd.SystemParameterization(system_representation=lpc_rep)

file_name = IMPORTS_FILES_FOLDER / 'lift_plus_cruise_final.stp'
spatial_rep = lpc_rep.spatial_representation
spatial_rep.import_file(file_name=file_name)
spatial_rep.refit_geometry(file_name=file_name)

t1 = time.time()

# region create components
# Main wing
wing_primitive_names = list(spatial_rep.get_geometry_primitives(search_names=['Wing']))
wing = cd.LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

# Horizontal tail
tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)

# Rotor: pusher
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor-9-disk']).keys())
pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)

pp_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 0']).keys())
pp_blade_1 = cd.Rotor(name='pp_blade_1', spatial_representation=spatial_rep, primitive_names=pp_blade_1_prim_names)

pp_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 1']).keys())
pp_blade_2 = cd.Rotor(name='pp_blade_2', spatial_representation=spatial_rep, primitive_names=pp_blade_2_prim_names)

pp_blade_3_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 2']).keys())
pp_blade_3 = cd.Rotor(name='pp_blade_3', spatial_representation=spatial_rep, primitive_names=pp_blade_3_prim_names)

pp_blade_4_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_9_blades, 3']).keys())
pp_blade_4 = cd.Rotor(name='pp_blade_4', spatial_representation=spatial_rep, primitive_names=pp_blade_4_prim_names)

# Rotor: rear left outer
rlo_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_disk']).keys())
rlo_disk = cd.Rotor(name='rlo_disk', spatial_representation=spatial_rep, primitive_names=rlo_disk_prim_names)

rlo_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_blades, 0']).keys())
rlo_blade_1 = cd.Rotor(name='rlo_blade_1', spatial_representation=spatial_rep, primitive_names=rlo_blade_1_prim_names)

rlo_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_blades, 1']).keys())
rlo_blade_2 = cd.Rotor(name='rlo_blade_2', spatial_representation=spatial_rep, primitive_names=rlo_blade_2_prim_names)

rlo_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_2_Hub']).keys())
rlo_hub = cd.Rotor(name='rlo_hub', spatial_representation=spatial_rep, primitive_names=rlo_hub_prim_names)

# Rotor: rear left inner
rli_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_disk']).keys())
rli_disk = cd.Rotor(name='rli_disk', spatial_representation=spatial_rep, primitive_names=rli_disk_prim_names)

rli_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_blades, 1']).keys())
rli_blade_1 = cd.Rotor(name='rli_blade_1', spatial_representation=spatial_rep, primitive_names=rli_blade_1_prim_names)

rli_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_blades, 0']).keys())
rli_blade_2 = cd.Rotor(name='rli_blade_2', spatial_representation=spatial_rep, primitive_names=rli_blade_2_prim_names)

rli_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_4_Hub']).keys())
rli_hub = cd.Rotor(name='rli_hub', spatial_representation=spatial_rep, primitive_names=rli_hub_prim_names)

# Rotor: rear right inner
rri_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_disk']).keys())
rri_disk = cd.Rotor(name='rri_disk', spatial_representation=spatial_rep, primitive_names=rri_disk_prim_names)

rri_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_blades, 0']).keys())
rri_blade_1 = cd.Rotor(name='rri_blade_1', spatial_representation=spatial_rep, primitive_names=rri_blade_1_prim_names)

rri_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_blades, 1']).keys())
rri_blade_2 = cd.Rotor(name='rri_blade_2', spatial_representation=spatial_rep, primitive_names=rri_blade_2_prim_names)

rri_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_6_Hub']).keys())
rri_hub = cd.Rotor(name='rri_hub', spatial_representation=spatial_rep, primitive_names=rri_hub_prim_names)

# Rotor: rear right outer
rro_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_disk']).keys())
rro_disk = cd.Rotor(name='rro_disk', spatial_representation=spatial_rep, primitive_names=rro_disk_prim_names)

rro_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_blades, 1']).keys())
rro_blade_1 = cd.Rotor(name='rro_blade_1', spatial_representation=spatial_rep, primitive_names=rro_blade_1_prim_names)

rro_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_blades, 0']).keys())
rro_blade_2 = cd.Rotor(name='rro_blade_2', spatial_representation=spatial_rep, primitive_names=rro_blade_2_prim_names)

rro_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_8_Hub']).keys())
rro_hub = cd.Rotor(name='rro_hub', spatial_representation=spatial_rep, primitive_names=rro_hub_prim_names)

# Rotor: front left outer
flo_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_disk']).keys())
flo_disk = cd.Rotor(name='flo_disk', spatial_representation=spatial_rep, primitive_names=flo_disk_prim_names)

flo_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_blades, 1']).keys())
flo_blade_1 = cd.Rotor(name='flo_blade_1', spatial_representation=spatial_rep, primitive_names=flo_blade_1_prim_names)

flo_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_blades, 0']).keys())
flo_blade_2 = cd.Rotor(name='flo_blade_2', spatial_representation=spatial_rep, primitive_names=flo_blade_2_prim_names)

flo_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_Hub']).keys())
flo_hub = cd.Rotor(name='flo_hub', spatial_representation=spatial_rep, primitive_names=flo_hub_prim_names)

# Rotor: front left inner
fli_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_disk']).keys())
fli_disk = cd.Rotor(name='fli_disk', spatial_representation=spatial_rep, primitive_names=fli_disk_prim_names)

fli_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_blades, 0']).keys())
fli_blade_1 = cd.Rotor(name='fli_blade_1', spatial_representation=spatial_rep, primitive_names=fli_blade_1_prim_names)

fli_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_3_blades, 1']).keys())
fli_blade_2 = cd.Rotor(name='fli_blade_2', spatial_representation=spatial_rep, primitive_names=fli_blade_2_prim_names)

# flo_hub_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_1_Hub']).keys())
# flo_hub = cd.Rotor(name='flo_hub', spatial_representation=spatial_rep, primitive_names=flo_hub_prim_names)
# Rotor: front right inner
fri_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_disk']).keys())
fri_disk = cd.Rotor(name='fri_disk', spatial_representation=spatial_rep, primitive_names=fri_disk_prim_names)

fri_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_blades, 1']).keys())
fri_blade_1 = cd.Rotor(name='fri_blade_1', spatial_representation=spatial_rep, primitive_names=fri_blade_1_prim_names)

fri_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_5_blades, 0']).keys())
fri_blade_2 = cd.Rotor(name='fri_blade_2', spatial_representation=spatial_rep, primitive_names=fri_blade_2_prim_names)

# Rotor: front right outer
fro_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_disk']).keys())
fro_disk = cd.Rotor(name='fro_disk', spatial_representation=spatial_rep, primitive_names=fro_disk_prim_names)

fro_blade_1_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_blades, 0']).keys())
fro_blade_1 = cd.Rotor(name='fro_blade_1', spatial_representation=spatial_rep, primitive_names=fro_blade_1_prim_names)

fro_blade_2_prim_names = list(spatial_rep.get_primitives(search_names=['Rotor_7_blades, 1']).keys())
fro_blade_2 = cd.Rotor(name='fro_blade_2', spatial_representation=spatial_rep, primitive_names=fro_blade_2_prim_names)
# endregion

# region add component
# add components
lpc_rep.add_component(wing)
lpc_rep.add_component(htail)

lpc_rep.add_component(pp_disk)
lpc_rep.add_component(pp_blade_1)
lpc_rep.add_component(pp_blade_2)
lpc_rep.add_component(pp_blade_3)
lpc_rep.add_component(pp_blade_4)

lpc_rep.add_component(rlo_disk)
lpc_rep.add_component(rlo_blade_1)
lpc_rep.add_component(rlo_blade_2)

lpc_rep.add_component(rli_disk)
lpc_rep.add_component(rli_blade_1)
lpc_rep.add_component(rli_blade_2)

lpc_rep.add_component(rri_disk)
lpc_rep.add_component(rri_blade_1)
lpc_rep.add_component(rri_blade_2)

lpc_rep.add_component(rro_disk)
lpc_rep.add_component(rro_blade_1)
lpc_rep.add_component(rro_blade_2)

lpc_rep.add_component(flo_disk)
lpc_rep.add_component(flo_blade_1)
lpc_rep.add_component(flo_blade_2)

lpc_rep.add_component(fli_disk)
lpc_rep.add_component(fli_blade_1)
lpc_rep.add_component(fli_blade_2)

lpc_rep.add_component(fri_disk)
lpc_rep.add_component(fri_blade_1)
lpc_rep.add_component(fri_blade_2)

lpc_rep.add_component(fro_disk)
lpc_rep.add_component(fro_blade_1)
lpc_rep.add_component(fro_blade_2)
# endregion

# region free form deformation
# Wing FFD 
wing_geometry_primitives = wing.get_geometry_primitives()
wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
wing_ffd_block = cd.SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
wing_ffd_block.add_scale_v(name='wing_linear_taper', order=2, num_dof=3, value=np.array([0., 0., 0.]), cost_factor=1.)
wing_ffd_block.add_rotation_u(name='wing_twist_distribution', connection_name='wing_twist', order=4, num_dof=10, value=np.zeros((10, )))
wing_ffd_block.add_translation_u(name='wing_span_dof', order=2, num_dof=2) # to use inner optimization, don't specify 'connection_name' and 'val'

# mapped arrays to get wing span
left_point = np.array([15., -26., 7.5])
right_point= np.array([15., 26., 7.5])
left_point_am = wing.project(left_point, direction=np.array([0., 0., -1.]))
right_point_am = wing.project(right_point, direction=np.array([0., 0., -1.]))
wing_span = am.norm(left_point_am - right_point_am)
lpc_param.add_input('wing_span', wing_span, value=80)

# Tail FFD
htail_geometry_primitives = htail.get_geometry_primitives()
htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(htail_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block', primitive=htail_ffd_bspline_volume, embedded_entities=htail_geometry_primitives)
htail_ffd_block.add_scale_v(name='htail_linear_taper', order=2, num_dof=3, value=np.array([0., 0., 0.]), cost_factor=1.)
htail_ffd_block.add_rotation_u(name='htail_twist_distribution', connection_name='h_tail_act', order=1, num_dof=1, value=np.array([np.deg2rad(1.75)]))
htail_ffd_block.add_translation_u(name='tail_span_dof', order=2, num_dof=2) # to use inner optimization, don't specify 'connection_name' and 'val'

# mapped arrays to get wing span
left_point = np.array([27., 6.75, 6.])
right_point= np.array([27., -6.75, 6.])
left_point_am = htail.project(left_point, direction=np.array([0., 0., -1.]))
right_point_am = htail.project(right_point, direction=np.array([0., 0., -1.]))
tail_span = am.norm(left_point_am - right_point_am)
lpc_param.add_input('tail_span', tail_span, value=15)
# NOTE: line above is performaing actuation- change when actuations are ready

# region Pusher prop
pp_blade_1_geom_prim = pp_blade_1.get_geometry_primitives()
pp_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1, 2, 0))
pp_blade_1_ffd_block = cd.SRBGFFDBlock(name='pp_blade_1_ffd_block', primitive=pp_blade_1_bspline_vol, embedded_entities=pp_blade_1_geom_prim)
pp_blade_1_ffd_block.add_scale_v(name='pp_blade_1_chord', connection_name='pp_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
pp_blade_1_ffd_block.add_rotation_u(name='pp_blade_1_twist', connection_name='pp_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

pp_blade_2_geom_prim = pp_blade_2.get_geometry_primitives()
pp_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(2, 1, 0))
pp_blade_2_ffd_block = cd.SRBGFFDBlock(name='pp_blade_2_ffd_block', primitive=pp_blade_2_bspline_vol, embedded_entities=pp_blade_2_geom_prim)
pp_blade_2_ffd_block.add_scale_v(name='pp_blade_2_chord', connection_name='pp_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
pp_blade_2_ffd_block.add_rotation_u(name='pp_blade_2_twist', connection_name='pp_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

pp_blade_3_geom_prim = pp_blade_3.get_geometry_primitives()
pp_blade_3_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_3_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1, 2, 0))
pp_blade_3_ffd_block = cd.SRBGFFDBlock(name='pp_blade_3_ffd_block', primitive=pp_blade_3_bspline_vol, embedded_entities=pp_blade_3_geom_prim)
pp_blade_3_ffd_block.add_scale_v(name='pp_blade_3_chord', connection_name='pp_blade_3_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
pp_blade_3_ffd_block.add_rotation_u(name='pp_blade_3_twist', connection_name='pp_blade_3_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

pp_blade_4_geom_prim = pp_blade_4.get_geometry_primitives()
pp_blade_4_bspline_vol = cd.create_cartesian_enclosure_volume(pp_blade_4_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(2, 1, 0))
pp_blade_4_ffd_block = cd.SRBGFFDBlock(name='pp_blade_4_ffd_block', primitive=pp_blade_4_bspline_vol, embedded_entities=pp_blade_4_geom_prim)
pp_blade_4_ffd_block.add_scale_v(name='pp_blade_4_chord', connection_name='pp_blade_4_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
pp_blade_4_ffd_block.add_rotation_u(name='pp_blade_4_twist', connection_name='pp_blade_4_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))
# endregion

# region Rotor: rear left outer
# rlo_disk_geom_prim = rlo_disk.get_geometry_primitives()
# rlo_disk_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# rlo_disk_ffd_block = cd.SRBGFFDBlock(name='rlo_blade_1_ffd_block', primitive=rlo_disk_bspline_vol, embedded_entities=rlo_disk_geom_prim)
# rlo_disk_ffd_block.add_scale_v(name='rlo_disk_r1', connection_name='rlo_disk_r1', order=1, num_dof=1, value=0)
# rlo_disk_ffd_block.add_scale_w(name='rlo_disk_r2', connection_name='rlo_disk_r2', order=1, num_dof=1, value=0)

rlo_blade_1_geom_prim = rlo_blade_1.get_geometry_primitives()
rlo_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rlo_blade_1_ffd_block = cd.SRBGFFDBlock(name='rlo_blade_1_ffd_block', primitive=rlo_blade_1_bspline_vol, embedded_entities=rlo_blade_1_geom_prim)
rlo_blade_1_ffd_block.add_scale_v(name='rlo_blade_1_chord', connection_name='rlo_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rlo_blade_1_ffd_block.add_rotation_u(name='rlo_blade_1_twist', connection_name='rlo_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

rlo_blade_2_geom_prim = rlo_blade_2.get_geometry_primitives()
rlo_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rlo_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rlo_blade_2_ffd_block = cd.SRBGFFDBlock(name='rlo_blade_2_ffd_block', primitive=rlo_blade_2_bspline_vol, embedded_entities=rlo_blade_2_geom_prim)
rlo_blade_2_ffd_block.add_scale_v(name='rlo_blade_2_chord', connection_name='rlo_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rlo_blade_2_ffd_block.add_rotation_u(name='rlo_blade_2_twist', connection_name='rlo_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# along y
y11 = rlo_disk.project(np.array([19.2, -13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rlo_disk.project(np.array([19.2, -23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# along x
y21 = rlo_disk.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rlo_disk.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)

rlo_in_plane_y = am.subtract(y12, y11)
rlo_in_plane_x = am.subtract(y21, y22)

lpc_param.add_input('rlo_in_plane_d1', am.norm(rlo_in_plane_y))
lpc_param.add_input('rlo_in_plane_d2', am.norm(rlo_in_plane_x))

rlo_hub_center = rlo_hub.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rlo_blade_2_tip = rlo_blade_2.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
rlo_blade_1_tip = rlo_blade_1.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
lpc_param.add_input('rlo_in_plane_r1', am.norm(rlo_blade_2_tip-rlo_hub_center))
lpc_param.add_input('rlo_in_plane_r2', am.norm(rlo_blade_2_tip-rlo_hub_center))

rlo_disk_center = rlo_disk.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
lpc_param.add_input('hub_disk_connection', rlo_hub_center-rlo_disk_center)

# exit()
# endregion

# region Rotor: rear left inner
# rli_disk_geom_prim = rli_disk.get_geometry_primitives()
# rli_disk_bspline_vol = cd.create_cartesian_enclosure_volume(rli_disk_geom_prim, num_control_points=(2, 2, 2), order=(2, 2, 2), xyz_to_uvw_indices=(2, 1, 0))
# rli_disk_ffd_block = cd.SRBGFFDBlock(name='rli_blade_1_ffd_block', primitive=rli_disk_bspline_vol, embedded_entities=rli_disk_geom_prim)
# rli_disk_ffd_block.add_scale_v(name='rli_disk_r1', connection_name='rli_disk_r1', order=1, num_dof=1, value=0)
# rli_disk_ffd_block.add_scale_w(name='rli_disk_r2', connection_name='rli_disk_r2', order=1, num_dof=1, value=0)

rli_blade_1_geom_prim = rli_blade_1.get_geometry_primitives()
rli_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rli_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rli_blade_1_ffd_block = cd.SRBGFFDBlock(name='rli_blade_1_ffd_block', primitive=rli_blade_1_bspline_vol, embedded_entities=rli_blade_1_geom_prim)
rli_blade_1_ffd_block.add_scale_v(name='rli_blade_1_chord', connection_name='rli_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rli_blade_1_ffd_block.add_rotation_u(name='rli_blade_1_twist', connection_name='rli_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

rli_blade_2_geom_prim = rli_blade_2.get_geometry_primitives()
rli_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rli_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rli_blade_2_ffd_block = cd.SRBGFFDBlock(name='rli_blade_2_ffd_block', primitive=rli_blade_2_bspline_vol, embedded_entities=rli_blade_2_geom_prim)
rli_blade_2_ffd_block.add_scale_v(name='rli_blade_2_chord', connection_name='rli_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rli_blade_2_ffd_block.add_rotation_u(name='rli_blade_2_twist', connection_name='rli_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# # along y
# y11 = rli_disk.project(np.array([18.760, -3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
# y12 = rli_disk.project(np.array([18.760, -13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
# # along x
# y21 = rli_disk.project(np.array([13.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
# y22 = rli_disk.project(np.array([23.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)


# rli_in_plane_y = am.subtract(y12, y11)
# rli_in_plane_x = am.subtract(y21, y22)

# lpc_param.add_input('rli_in_plane_d1', am.norm(rli_in_plane_y))
# lpc_param.add_input('rli_in_plane_d2', am.norm(rli_in_plane_x))

# rli_hub_center = rli_hub.project(np.array([18.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rli_blade_2_tip = rli_blade_2.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# rli_blade_1_tip = rli_blade_1.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
# lpc_param.add_input('rli_in_plane_r1', am.norm(rli_blade_2_tip-rli_hub_center))
# lpc_param.add_input('rli_in_plane_r2', am.norm(rli_blade_2_tip-rli_hub_center))

# rli_disk_center = rli_disk.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# lpc_param.add_input('hub_disk_connection', rli_hub_center-rli_disk_center)

# endregion

# Rotor: rear right inner
rri_blade_1_geom_prim = rri_blade_1.get_geometry_primitives()
rri_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rri_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rri_blade_1_ffd_block = cd.SRBGFFDBlock(name='rri_blade_1_ffd_block', primitive=rri_blade_1_bspline_vol, embedded_entities=rri_blade_1_geom_prim)
rri_blade_1_ffd_block.add_scale_v(name='rri_blade_1_chord', connection_name='rri_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rri_blade_1_ffd_block.add_rotation_u(name='rri_blade_1_twist', connection_name='rri_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

rri_blade_2_geom_prim = rri_blade_2.get_geometry_primitives()
rri_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rri_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rri_blade_2_ffd_block = cd.SRBGFFDBlock(name='rri_blade_2_ffd_block', primitive=rri_blade_2_bspline_vol, embedded_entities=rri_blade_2_geom_prim)
rri_blade_2_ffd_block.add_scale_v(name='rri_blade_2_chord', connection_name='rri_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rri_blade_2_ffd_block.add_rotation_u(name='rri_blade_2_twist', connection_name='rri_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# Rotor: rear right outer
rro_blade_1_geom_prim = rro_blade_1.get_geometry_primitives()
rro_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(rro_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rro_blade_1_ffd_block = cd.SRBGFFDBlock(name='rro_blade_1_ffd_block', primitive=rro_blade_1_bspline_vol, embedded_entities=rro_blade_1_geom_prim)
rro_blade_1_ffd_block.add_scale_v(name='rro_blade_1_chord', connection_name='rro_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rro_blade_1_ffd_block.add_rotation_u(name='rro_blade_1_twist', connection_name='rro_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

rro_blade_2_geom_prim = rro_blade_2.get_geometry_primitives()
rro_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(rro_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
rro_blade_2_ffd_block = cd.SRBGFFDBlock(name='rro_blade_2_ffd_block', primitive=rro_blade_2_bspline_vol, embedded_entities=rro_blade_2_geom_prim)
rro_blade_2_ffd_block.add_scale_v(name='rro_blade_2_chord', connection_name='rro_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
rro_blade_2_ffd_block.add_rotation_u(name='rro_blade_2_twist', connection_name='rro_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# Rotor: front left outer
flo_blade_1_geom_prim = flo_blade_1.get_geometry_primitives()
flo_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(flo_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
flo_blade_1_ffd_block = cd.SRBGFFDBlock(name='flo_blade_1_ffd_block', primitive=flo_blade_1_bspline_vol, embedded_entities=flo_blade_1_geom_prim)
flo_blade_1_ffd_block.add_scale_v(name='flo_blade_1_chord', connection_name='flo_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
flo_blade_1_ffd_block.add_rotation_u(name='flo_blade_1_twist', connection_name='flo_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

flo_blade_2_geom_prim = flo_blade_2.get_geometry_primitives()
flo_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(flo_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
flo_blade_2_ffd_block = cd.SRBGFFDBlock(name='flo_blade_2_ffd_block', primitive=flo_blade_2_bspline_vol, embedded_entities=flo_blade_2_geom_prim)
flo_blade_2_ffd_block.add_scale_v(name='flo_blade_2_chord', connection_name='flo_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
flo_blade_2_ffd_block.add_rotation_u(name='flo_blade_2_twist', connection_name='flo_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# Rotor: front left inner
fli_blade_1_geom_prim = fli_blade_1.get_geometry_primitives()
fli_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(fli_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
fli_blade_1_ffd_block = cd.SRBGFFDBlock(name='fli_blade_1_ffd_block', primitive=fli_blade_1_bspline_vol, embedded_entities=fli_blade_1_geom_prim)
fli_blade_1_ffd_block.add_scale_v(name='fli_blade_1_chord', connection_name='fli_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
fli_blade_1_ffd_block.add_rotation_u(name='fli_blade_1_twist', connection_name='fli_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

fli_blade_2_geom_prim = fli_blade_2.get_geometry_primitives()
fli_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(fli_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
fli_blade_2_ffd_block = cd.SRBGFFDBlock(name='fli_blade_2_ffd_block', primitive=fli_blade_2_bspline_vol, embedded_entities=fli_blade_2_geom_prim)
fli_blade_2_ffd_block.add_scale_v(name='fli_blade_2_chord', connection_name='fli_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
fli_blade_2_ffd_block.add_rotation_u(name='fli_blade_2_twist', connection_name='fli_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# Rotor: front right inner
fri_blade_1_geom_prim = fri_blade_1.get_geometry_primitives()
fri_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(fri_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
fri_blade_1_ffd_block = cd.SRBGFFDBlock(name='fri_blade_1_ffd_block', primitive=fri_blade_1_bspline_vol, embedded_entities=fri_blade_1_geom_prim)
fri_blade_1_ffd_block.add_scale_v(name='fri_blade_1_chord', connection_name='fri_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
fri_blade_1_ffd_block.add_rotation_u(name='fri_blade_1_twist', connection_name='fri_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

fri_blade_2_geom_prim = fri_blade_2.get_geometry_primitives()
fri_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(fri_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
fri_blade_2_ffd_block = cd.SRBGFFDBlock(name='fri_blade_2_ffd_block', primitive=fri_blade_2_bspline_vol, embedded_entities=fri_blade_2_geom_prim)
fri_blade_2_ffd_block.add_scale_v(name='fri_blade_2_chord', connection_name='fri_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
fri_blade_2_ffd_block.add_rotation_u(name='fri_blade_2_twist', connection_name='fri_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

# Rotor: front right outer
fro_blade_1_geom_prim = fro_blade_1.get_geometry_primitives()
fro_blade_1_bspline_vol = cd.create_cartesian_enclosure_volume(fro_blade_1_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
fro_blade_1_ffd_block = cd.SRBGFFDBlock(name='fro_blade_1_ffd_block', primitive=fro_blade_1_bspline_vol, embedded_entities=fro_blade_1_geom_prim)
fro_blade_1_ffd_block.add_scale_v(name='fro_blade_1_chord', connection_name='fro_blade_1_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
fro_blade_1_ffd_block.add_rotation_u(name='fro_blade_1_twist', connection_name='fro_blade_1_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

fro_blade_2_geom_prim = fro_blade_2.get_geometry_primitives()
fro_blade_2_bspline_vol = cd.create_cartesian_enclosure_volume(fro_blade_2_geom_prim, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(0, 1, 2))
fro_blade_2_ffd_block = cd.SRBGFFDBlock(name='fro_blade_2_ffd_block', primitive=fro_blade_2_bspline_vol, embedded_entities=fro_blade_2_geom_prim)
fro_blade_2_ffd_block.add_scale_v(name='fro_blade_2_chord', connection_name='fro_blade_2_chord', order=4, num_dof=4, value=np.array([0., 0., 0., 0.]))
fro_blade_2_ffd_block.add_rotation_u(name='fro_blade_2_twist', connection_name='fro_blade_2_twist', order=4, num_dof=5, value=np.array([0., 0., 0., 0., 0.]))

ffd_set = cd.SRBGFFDSet(
    name='ffd_set', 
    ffd_blocks={
        wing_ffd_block.name : wing_ffd_block, 
        htail_ffd_block.name : htail_ffd_block,
        pp_blade_1_ffd_block.name: pp_blade_1_ffd_block,
        pp_blade_2_ffd_block.name: pp_blade_2_ffd_block,
        pp_blade_3_ffd_block.name: pp_blade_3_ffd_block,
        pp_blade_4_ffd_block.name: pp_blade_4_ffd_block,
        rlo_blade_1_ffd_block.name : rlo_blade_1_ffd_block,
        rlo_blade_2_ffd_block.name : rlo_blade_2_ffd_block,
        # rlo_disk_ffd_block.name : rlo_disk_ffd_block,
        rli_blade_1_ffd_block.name : rli_blade_1_ffd_block,
        rli_blade_2_ffd_block.name : rli_blade_2_ffd_block,
        rri_blade_1_ffd_block.name : rri_blade_1_ffd_block,
        rri_blade_2_ffd_block.name : rri_blade_2_ffd_block,
        rro_blade_1_ffd_block.name : rro_blade_1_ffd_block,
        rro_blade_2_ffd_block.name : rro_blade_2_ffd_block,
        flo_blade_1_ffd_block.name : flo_blade_1_ffd_block,
        flo_blade_2_ffd_block.name : flo_blade_2_ffd_block,
        fli_blade_1_ffd_block.name : fli_blade_1_ffd_block,
        fli_blade_2_ffd_block.name : fli_blade_2_ffd_block,
        fri_blade_1_ffd_block.name : fri_blade_1_ffd_block,
        fri_blade_2_ffd_block.name : fri_blade_2_ffd_block,
        fro_blade_1_ffd_block.name : fro_blade_1_ffd_block,
        fro_blade_2_ffd_block.name : fro_blade_2_ffd_block,
    }
)

# rlo_ffd_block.setup()
# affine_section_properties = rlo_ffd_block.evaluate_affine_section_properties()
# rotational_section_properties = rlo_ffd_block.evaluate_rotational_section_properties()
# affine_ffd_control_points_local_frame = rlo_ffd_block.evaluate_affine_block_deformations(plot=False)
# ffd_control_points_local_frame = rlo_ffd_block.evaluate_rotational_block_deformations(plot=False)
# ffd_control_points = rlo_ffd_block.evaluate_control_points(plot=False)
# updated_geometry = rlo_ffd_block.evaluate_embedded_entities(plot=False)

# ffd_set.setup()
# affine_section_properties = ffd_set.evaluate_affine_section_properties()
# rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
# affine_ffd_control_points_local_frame = ffd_set.evaluate_affine_block_deformations(plot=False)
# ffd_control_points_local_frame = ffd_set.evaluate_rotational_block_deformations(plot=False)
# ffd_control_points = ffd_set.evaluate_control_points(plot=False)
# updated_geometry = ffd_set.evaluate_embedded_entities(plot=False)
# updated_primitives_names = htail.primitive_names.copy() + wing.primitive_names.copy()

# spatial_rep.update(updated_geometry, updated_primitives_names)
# spatial_rep.plot()


# endregion

# region meshes
num_radial = 25
num_lifting_line = 10
off_set = 1
off_set_long_le = 0.1
off_set_long_te_root = 0.35
off_set_long_te_tip = 0.25

# region wing mesh
plot_wing_mesh = False
num_spanwise_vlm = 25
num_chordwise_vlm = 2

wing_surface_offset = np.zeros((num_spanwise_vlm, 3))
wing_surface_offset[2:-2, 0] = 5.5
wing_surface_offset[[0, -1], 0] = 1.1
wing_surface_offset[[1, -2], 0] = 3
wing_surface_offset[:, 2] = -1

wing_trailing_edge = wing.project(np.linspace(np.array([15., -26., 7.5]), np.array([15., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), plot=plot_wing_mesh)  
wing_leading_edge = wing.project(wing_trailing_edge.evaluate() - wing_surface_offset, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=plot_wing_mesh)

wing_chord_surface = am.linspace(wing_leading_edge, wing_trailing_edge, num_chordwise_vlm)
wing_upper_surface_wireframe = wing.project(wing_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=plot_wing_mesh)
wing_lower_surface_wireframe = wing.project(wing_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=50, plot=plot_wing_mesh)
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))


# OML mesh for ML pressures wing
num_ml_points = 100
chord_surface_ml = am.linspace(wing_leading_edge, wing_trailing_edge, num_ml_points)
i_vec = np.arange(0, len(chord_surface_ml.value))
x_range = np.linspace(0, 1, num_ml_points)

x_interp_x = wing_chord_surface.value[1,:, 0].reshape(num_spanwise_vlm, 1) - ((wing_chord_surface.value[1, :, 0] - wing_chord_surface.value[0, :, 0]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
x_interp_y = wing_chord_surface.value[1,:, 1].reshape(num_spanwise_vlm, 1) - ((wing_chord_surface.value[1, :, 1] - wing_chord_surface.value[0, :, 1]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
x_interp_z = wing_chord_surface.value[1,:, 2].reshape(num_spanwise_vlm, 1) - ((wing_chord_surface.value[1, :, 2] - wing_chord_surface.value[0, :, 2]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))

new_chord_surface = np.zeros((num_ml_points, num_spanwise_vlm, 3))
new_chord_surface[:, :, 0] = x_interp_x.T
new_chord_surface[:, :, 1] = x_interp_y.T
new_chord_surface[:, :, 2] = x_interp_z.T

wing_upper_surface_ml = wing.project(new_chord_surface + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=75, plot=False, max_iterations=200)
wing_lower_surface_ml = wing.project(new_chord_surface - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=100, plot=False, max_iterations=200)

wing_oml_mesh_name_ml = 'wing_oml_mesh_ML'
wing_oml_mesh_ml = am.vstack((wing_upper_surface_ml, wing_lower_surface_ml))
# spatial_rep.plot_meshes([wing_camber_surface])
# endregion

# region wing beam mesh
point00 = np.array([12.356, 25.250, 7.618 + 0.1]) 
point01 = np.array([13.400, 25.250, 7.617 + 0.1]) 
point10 = np.array([8.892,    0.000, 8.633 + 0.1]) 
point11 = np.array([14.332,   0.000, 8.439 + 0.1]) 
point20 = np.array([12.356, -25.250, 7.618 + 0.1]) 
point21 = np.array([13.400, -25.250, 7.617 + 0.1]) 
do_plots = False
num_wing_beam = 11
leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_beam/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_beam/2+1))), axis=0)
trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_beam/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_beam/2+1))), axis=0)

leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)
wing_beam = am.linear_combination(leading_edge, trailing_edge, 1, start_weights=np.ones((num_wing_beam, ))*0.75, stop_weights=np.ones((num_wing_beam, ))*0.25)
width = am.norm((leading_edge - trailing_edge)*0.5)
# width = am.subtract(leading_edge, trailing_edge)

if do_plots:
    spatial_rep.plot_meshes([wing_beam])

wing_beam = wing_beam.reshape((11,3))#*0.304

offset = np.array([0,0,0.5])
top = wing.project(wing_beam.value+offset, direction=np.array([0., 0., -1.]), plot=do_plots)
bot = wing.project(wing_beam.value-offset, direction=np.array([0., 0., 1.]), plot=do_plots)
height = am.norm((top - bot)*1)
# endregion

# region tail mesh
plot_tail_mesh = False
num_spanwise_vlm = 8
num_chordwise_vlm = 2
leading_edge = htail.project(np.linspace(np.array([27., -6.75, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)  # returns MappedArray
trailing_edge = htail.project(np.linspace(np.array([31.5, -6.75, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)   # returns MappedArray
tail_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
htail_upper_surface_wireframe = htail.project(tail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
htail_lower_surface_wireframe = htail.project(tail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)
htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1) 


# OML mesh for ML pressures tail
x_interp_x = tail_chord_surface.value[1,:, 0].reshape(num_spanwise_vlm, 1) - ((tail_chord_surface.value[1, :, 0] - tail_chord_surface.value[0, :, 0]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
x_interp_y = tail_chord_surface.value[1,:, 1].reshape(num_spanwise_vlm, 1) - ((tail_chord_surface.value[1, :, 1] - tail_chord_surface.value[0, :, 1]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))
x_interp_z = tail_chord_surface.value[1,:, 2].reshape(num_spanwise_vlm, 1) - ((tail_chord_surface.value[1, :, 2] - tail_chord_surface.value[0, :, 2]).reshape(num_spanwise_vlm, 1) * np.cos(np.pi/(2 * len(x_range)) * i_vec).reshape(1,100))

new_chord_surface = np.zeros((num_ml_points, num_spanwise_vlm, 3))
new_chord_surface[:, :, 0] = x_interp_x.T
new_chord_surface[:, :, 1] = x_interp_y.T
new_chord_surface[:, :, 2] = x_interp_z.T

htail_upper_surface_ml = htail.project(new_chord_surface + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=75, plot=False, max_iterations=200)
htail_lower_surface_ml = htail.project(new_chord_surface - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=100, plot=False, max_iterations=200)

htail_oml_mesh_name_ml = 'htail_oml_mesh_ML'
tail_oml_mesh_ml = am.vstack((htail_upper_surface_ml, htail_lower_surface_ml))

# endregion

# region pusher prop (pp) meshes
# disk
y11 = pp_disk.project(np.array([31.94, 0.00, 3.29]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = pp_disk.project(np.array([31.94, 0.00, 12.29]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = pp_disk.project(np.array([31.94, -4.50, 7.78]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = pp_disk.project(np.array([31.94, 4.45, 7.77]), direction=np.array([-1., 0., 0.]), plot=False)
pp_disk_in_plane_y = am.subtract(y11, y12)
pp_disk_in_plane_x = am.subtract(y21, y22)
pp_disk_origin = pp_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))

# lifting line mesh
# blade 2
b2_le_low_res_numpy =np.linspace(np.array([31.813 - off_set, -0.155 + off_set, 8.735 - 2 * off_set_long_le]), np.array([31.953 - off_set, 0.125 + off_set, 12.290 + 3 * off_set_long_le]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([32.322 + off_set, -0.465 - off_set, 8.735 - 2 * off_set_long_te_root]), np.array([31.903 + off_set, -0.376 - off_set, 12.291 + 3 * off_set_long_te_tip]), num_lifting_line)

pp_blade_2_le_low_res = pp_blade_2.project(b2_le_low_res_numpy, direction=np.array([1., 0., 0.]), grid_search_n=50, plot=False)
pp_blade_2_te_low_res = pp_blade_2.project(b2_te_low_res_numpy, direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)

pp_blade_2_chord_surface = am.linspace(pp_blade_2_le_low_res, pp_blade_2_te_low_res, 2)
pp_blade_2_upper_surface_wireframe = pp_blade_2.project(pp_blade_2_chord_surface.value + np.array([1.01, 0., 0.]), direction=np.array([-1., 0., 0.]), grid_search_n=25)
pp_blade_2_lower_surface_wireframe = pp_blade_2.project(pp_blade_2_chord_surface.value - np.array([-1.02, 0., 0.]), direction=np.array([1., 0., 0.]), grid_search_n=25)
pp_blade_2_ll_mesh = am.linspace(pp_blade_2_upper_surface_wireframe, pp_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([pp_blade_2_ll_mesh])

# blade 4
b4_le_low_res_numpy = np.linspace(np.array([31.757 - off_set, -0.179 - off_set, 6.890 + 2 * off_set_long_le]), np.array([31.910 - off_set, -0.111 - off_set, 3.290 - 3 * off_set_long_le]), num_lifting_line)
b4_te_low_res_numpy = np.linspace(np.array([32.123 + off_set, 0.179 + off_set, 6.890 + 2 * off_set_long_le]), np.array([31.970 + off_set, 0.111 + off_set, 3.290 - 3 * off_set_long_le]), num_lifting_line)

pp_blade_4_le_low_res = pp_blade_4.project(b4_le_low_res_numpy, direction=np.array([1.02, 0., 0.]), grid_search_n=50, plot=False)
pp_blade_4_te_low_res = pp_blade_4.project(b4_te_low_res_numpy, direction=np.array([-1.01, 0., 0.]), grid_search_n=50, plot=False)

pp_blade_4_chord_surface = am.linspace(pp_blade_4_le_low_res, pp_blade_4_te_low_res, 2)
pp_blade_4_upper_surface_wireframe = pp_blade_4.project(pp_blade_4_chord_surface.value + np.array([1.01, 0., 0.]), direction=np.array([-1., 0., 0.]), grid_search_n=25)
pp_blade_4_lower_surface_wireframe = pp_blade_4.project(pp_blade_4_chord_surface.value - np.array([-1.02, 0., 0.]), direction=np.array([1., 0., 0.]), grid_search_n=25)
pp_blade_4_ll_mesh = am.linspace(pp_blade_4_upper_surface_wireframe, pp_blade_4_lower_surface_wireframe, 1) 


# chord 
b4_le_high_res_numpy = np.linspace(np.array([31.757 - off_set, -0.179 - off_set, 6.890 + 2 * off_set_long_le]), np.array([31.910 - off_set, -0.111 - off_set, 3.290 - 3 * off_set_long_le]), num_radial)
b4_te_high_res_numpy = np.linspace(np.array([32.123 + off_set, 0.179 + off_set, 6.890 + 2 * off_set_long_le]), np.array([31.970 + off_set, 0.111 + off_set, 3.290 - 3 * off_set_long_le]), num_radial)
pp_blade_4_le_high_res = pp_blade_4.project(b4_le_high_res_numpy, direction=np.array([1., 0., 0.]), grid_search_n=50, plot=False)
pp_blade_4_te_high_res = pp_blade_4.project(b4_te_high_res_numpy, direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)
# pp_chord_length = am.norm(am.subtract(pp_blade_4_le_high_res, pp_blade_4_te_high_res), axes=(1, ))
pp_chord_length = am.subtract(pp_blade_4_le_high_res, pp_blade_4_te_high_res)

# twist
pp_le_proj_disk = pp_disk.project(pp_blade_4_le_high_res.evaluate(), direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)
pp_te_proj_disk = pp_disk.project(pp_blade_4_te_high_res.evaluate(), direction=np.array([-1., 0., 0.]), grid_search_n=50, plot=False)

pp_v_dist_le = am.subtract(pp_blade_4_le_high_res, pp_le_proj_disk)
pp_v_dist_te = am.subtract(pp_blade_4_te_high_res, pp_te_proj_disk)
pp_tot_v_dist = am.subtract(pp_v_dist_te, pp_v_dist_le)
# endregion

# region rear left outer (rlo) rotor meshes
# disk
rlo_origin = rlo_disk.project(np.array([19.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]))

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([20.250 - off_set_long_le, -18.967 - off_set, 9.062 + off_set]), np.array([24.200 + off_set_long_le, -18.903 - off_set, 9.003 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([20.250 - off_set_long_te_root, -18.099 + off_set, 8.857 + off_set]), np.array([24.201 + off_set_long_te_tip, -18.292 + off_set, 9.031 + off_set]), num_lifting_line)

rlo_blade_1_le_low_res = rlo_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rlo_blade_1_te_low_res = rlo_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rlo_blade_1_chord_surface = am.linspace(rlo_blade_1_le_low_res, rlo_blade_1_te_low_res, 2)
rlo_blade_1_upper_surface_wireframe = rlo_blade_1.project(rlo_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rlo_blade_1_lower_surface_wireframe = rlo_blade_1.project(rlo_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
rlo_blade_1_ll_mesh = am.linspace(rlo_blade_1_upper_surface_wireframe, rlo_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, -18.533 + off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le, -18.597 + off_set, 9.003 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_te_root, -19.401 - off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, -19.208 - off_set, 9.032 + off_set]), num_lifting_line)

rlo_blade_2_le_low_res = rlo_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rlo_blade_2_te_low_res = rlo_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rlo_blade_2_chord_surface = am.linspace(rlo_blade_2_le_low_res, rlo_blade_2_te_low_res, 2)
rlo_blade_2_upper_surface_wireframe = rlo_blade_2.project(rlo_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rlo_blade_2_lower_surface_wireframe = rlo_blade_2.project(rlo_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
rlo_blade_2_ll_mesh = am.linspace(rlo_blade_2_upper_surface_wireframe, rlo_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([rlo_blade_1_ll_mesh, rlo_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, -18.533 + off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le, -18.597 + off_set, 9.003 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([18.150 + off_set_long_te_root, -19.401 - off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, -19.208 - off_set, 9.032 + off_set]), num_radial)
rlo_blade_2_le_high_res = rlo_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rlo_blade_2_te_high_res = rlo_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rlo_chord_length = am.norm(am.subtract(rlo_blade_2_le_high_res, rlo_blade_2_te_high_res), axes=(1, ))
rlo_chord_length = am.subtract(rlo_blade_2_le_high_res, rlo_blade_2_te_high_res)


# twist
rlo_le_proj_disk = rlo_disk.project(rlo_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rlo_te_proj_disk = rlo_disk.project(rlo_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rlo_v_dist_le = am.subtract(rlo_blade_2_le_high_res, rlo_le_proj_disk)
rlo_v_dist_te = am.subtract(rlo_blade_2_te_high_res, rlo_te_proj_disk)
rlo_tot_v_dist = am.subtract(rlo_v_dist_le, rlo_v_dist_te)
# endregion

# region rear right outer (rro) rotor meshes
# disk
y11 = rro_disk.project(np.array([19.2, 23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rro_disk.project(np.array([19.2, 13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rro_disk.project(np.array([14.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rro_disk.project(np.array([24.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
rro_in_plane_y = am.subtract(y12, y11)
rro_in_plane_x = am.subtract(y21, y22)
rro_origin = rro_disk.project(np.array([19.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]))

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([20.250 - off_set_long_le, 18.967 + off_set, 9.062 + off_set]), np.array([24.200 + off_set_long_le, 18.903 + off_set, 9.003 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([20.250 - off_set_long_te_root, 18.099 - off_set, 8.857 + off_set]), np.array([24.201 + off_set_long_te_tip, 18.292 - off_set, 9.031 + off_set]), num_lifting_line)

rro_blade_1_le_low_res = rro_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rro_blade_1_te_low_res = rro_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rro_blade_1_chord_surface = am.linspace(rro_blade_1_le_low_res, rro_blade_1_te_low_res, 2)
rro_blade_1_upper_surface_wireframe = rro_blade_1.project(rro_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rro_blade_1_lower_surface_wireframe = rro_blade_1.project(rro_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
rro_blade_1_ll_mesh = am.linspace(rro_blade_1_upper_surface_wireframe, rro_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, 18.533 - off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le, 18.597 - off_set, 9.003 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([18.150 + off_set_long_te_root, 19.401 + off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, 19.208 + off_set, 9.032 + off_set]), num_lifting_line)

rro_blade_2_le_low_res = rro_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rro_blade_2_te_low_res = rro_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rro_blade_2_chord_surface = am.linspace(rro_blade_2_le_low_res, rro_blade_2_te_low_res, 2)
rro_blade_2_upper_surface_wireframe = rro_blade_2.project(rro_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rro_blade_2_lower_surface_wireframe = rro_blade_2.project(rro_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
rro_blade_2_ll_mesh = am.linspace(rro_blade_2_upper_surface_wireframe, rro_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([rro_blade_1_ll_mesh, rro_blade_2_ll_mesh, rlo_blade_1_ll_mesh, rlo_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([18.150 + off_set_long_le, 18.533 - off_set, 9.062 + off_set]), np.array([14.200 - off_set_long_le, 18.597 - off_set, 9.003 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([18.150 + off_set_long_te_root, 19.401 + off_set, 8.857 + off_set]), np.array([14.200 - off_set_long_te_tip, 19.208 + off_set, 9.032 + off_set]), num_radial)
rro_blade_2_le_high_res = rro_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rro_blade_2_te_high_res = rro_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rro_chord_length = am.norm(am.subtract(rro_blade_2_le_high_res, rro_blade_2_te_high_res), axes=(1, ))
rro_chord_length = am.subtract(rro_blade_2_le_high_res, rro_blade_2_te_high_res)

# twist
rro_te_proj_disk = rro_disk.project(rro_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rro_le_proj_disk = rro_disk.project(rro_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rro_v_dist_le = am.subtract(rro_blade_2_le_high_res, rro_le_proj_disk)
rro_v_dist_te = am.subtract(rro_blade_2_te_high_res, rro_te_proj_disk)
rro_tot_v_dist = am.subtract(rro_v_dist_le, rro_v_dist_te)
# endregion

# region front left outer (flo) rotor meshes
# disk
y11 = flo_disk.project(np.array([5.070, -13.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
y12 = flo_disk.project(np.array([5.070, -23.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
y21 = flo_disk.project(np.array([0.070, -18.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
y22 = flo_disk.project(np.array([10.070, -18.750, 6.730]), direction=np.array([0., 0., -1.]), plot=False)
flo_in_plane_y = am.subtract(y12, y11)
flo_in_plane_x = am.subtract(y21, y22)
flo_origin = flo_disk.project(np.array([5.07, -18.75, 6.73]), direction=np.array([0., 0., -1.]))

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_le, -18.533 + off_set, 6.782 + off_set]), np.array([10.070 + off_set_long_le, -18.597 + off_set, 6.723 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_te_root, -19.401 - off_set, 6.577 + off_set]), np.array([10.071 + off_set_long_te_tip, -19.208 - off_set, 6.751 + off_set]), num_lifting_line)

flo_blade_1_le_low_res = flo_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
flo_blade_1_te_low_res = flo_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

flo_blade_1_chord_surface = am.linspace(flo_blade_1_le_low_res, flo_blade_1_te_low_res, 2)
flo_blade_1_upper_surface_wireframe = flo_blade_1.project(flo_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
flo_blade_1_lower_surface_wireframe = flo_blade_1.project(flo_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
flo_blade_1_ll_mesh = am.linspace(flo_blade_1_upper_surface_wireframe, flo_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, -18.967 - off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, -18.903 - off_set, 6.723 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, -18.099 + off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, -18.292 + off_set, 6.752 + off_set]), num_lifting_line)

flo_blade_2_le_low_res = flo_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
flo_blade_2_te_low_res = flo_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

flo_blade_2_chord_surface = am.linspace(flo_blade_2_le_low_res, flo_blade_2_te_low_res, 2)
flo_blade_2_upper_surface_wireframe = flo_blade_2.project(flo_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
flo_blade_2_lower_surface_wireframe = flo_blade_2.project(flo_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
flo_blade_2_ll_mesh = am.linspace(flo_blade_2_upper_surface_wireframe, flo_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([flo_blade_1_ll_mesh, flo_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, -18.967 - off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, -18.903 - off_set, 6.723 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, -18.099 + off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, -18.292 + off_set, 6.752 + off_set]), num_radial)
flo_blade_2_le_high_res = flo_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
flo_blade_2_te_high_res = flo_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# flo_chord_length = am.norm(am.subtract(flo_blade_2_le_high_res, flo_blade_2_te_high_res), axes=(1, ))
flo_chord_length = am.subtract(flo_blade_2_le_high_res, flo_blade_2_te_high_res)

# twist
flo_le_proj_disk = flo_disk.project(flo_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
flo_te_proj_disk = flo_disk.project(flo_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

flo_v_dist_le = am.subtract(flo_blade_2_le_high_res, flo_le_proj_disk)
flo_v_dist_te = am.subtract(flo_blade_2_te_high_res, flo_te_proj_disk)
flo_tot_v_dist = am.subtract(flo_v_dist_le, flo_v_dist_te)
# endregion

# region front right outer (fro) rotor meshes
# disk
y11 = fro_disk.project(np.array([5.07, 23.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y12 = fro_disk.project(np.array([5.07, 13.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y21 = fro_disk.project(np.array([0.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y22 = fro_disk.project(np.array([10.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
fro_in_plane_y = am.subtract(y12, y11)
fro_in_plane_x = am.subtract(y21, y22)
fro_origin = fro_disk.project(np.array([5.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]))

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_le, 18.533 - off_set, 6.782 + off_set]), np.array([10.070 + off_set_long_le, 18.597 - off_set, 6.723 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([6.120 - off_set_long_te_root, 19.401 + off_set, 6.577 + off_set]), np.array([10.071 + off_set_long_te_tip, 19.208 + off_set, 6.751 + off_set]), num_lifting_line)

fro_blade_1_le_low_res = fro_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fro_blade_1_te_low_res = fro_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fro_blade_1_chord_surface = am.linspace(fro_blade_1_le_low_res, fro_blade_1_te_low_res, 2)
fro_blade_1_upper_surface_wireframe = fro_blade_1.project(fro_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
fro_blade_1_lower_surface_wireframe = fro_blade_1.project(fro_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
fro_blade_1_ll_mesh = am.linspace(fro_blade_1_upper_surface_wireframe, fro_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, 18.967 + off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, 18.903 + off_set, 6.723 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, 18.099 - off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, 18.292 - off_set, 6.752 + off_set]), num_lifting_line)

fro_blade_2_le_low_res = fro_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fro_blade_2_te_low_res = fro_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fro_blade_2_chord_surface = am.linspace(fro_blade_2_le_low_res, fro_blade_2_te_low_res, 2)
fro_blade_2_upper_surface_wireframe = fro_blade_2.project(fro_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
fro_blade_2_lower_surface_wireframe = fro_blade_2.project(fro_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
fro_blade_2_ll_mesh = am.linspace(fro_blade_2_upper_surface_wireframe, fro_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([fro_blade_1_ll_mesh, fro_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([4.020 + off_set_long_le, 18.967 + off_set, 6.782 + off_set]), np.array([0.070 - off_set_long_le, 18.903 + off_set, 6.723 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([4.020 + off_set_long_te_root, 18.099 - off_set, 6.577 + off_set]), np.array([0.070 - off_set_long_te_tip, 18.292 - off_set, 6.752 + off_set]), num_radial)
fro_blade_2_le_high_res = fro_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fro_blade_2_te_high_res = fro_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fro_chord_length = am.norm(am.subtract(fro_blade_2_le_high_res, fro_blade_2_te_high_res), axes=(1, ))
fro_chord_length = am.subtract(fro_blade_2_le_high_res, fro_blade_2_te_high_res)

# twist
fro_le_proj_disk = fro_disk.project(fro_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fro_te_proj_disk = fro_disk.project(fro_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fro_v_dist_le = am.subtract(fro_blade_2_le_high_res, fro_le_proj_disk)
fro_v_dist_te = am.subtract(fro_blade_2_te_high_res, fro_te_proj_disk)
fro_tot_v_dist = am.subtract(fro_v_dist_le, fro_v_dist_te)
# endregion

# region rear left inner (rli) rotor meshes
# disk
y11 = rli_disk.project(np.array([18.760, -3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rli_disk.project(np.array([18.760, -13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rli_disk.project(np.array([13.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rli_disk.project(np.array([23.760, -8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
rli_in_plane_y = am.subtract(y12, y11)
rli_in_plane_x = am.subtract(y21, y22)
rli_origin = rli_disk.project(np.array([18.760, -8.537, 9.919]), direction=np.array([0., 0., -1.]))

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_le, -8.243 + off_set, 9.381 + off_set]), np.array([23.760 + off_set_long_le, -8.298 + off_set, 9.315 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_te_root, -9.073 - off_set, 9.058 + off_set]), np.array([23.761 + off_set_long_te_tip, -8.906 - off_set, 9.257 + off_set]), num_lifting_line)

rli_blade_1_le_low_res = rli_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rli_blade_1_te_low_res = rli_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rli_blade_1_chord_surface = am.linspace(rli_blade_1_le_low_res, rli_blade_1_te_low_res, 2)
rli_blade_1_upper_surface_wireframe = rli_blade_1.project(rli_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rli_blade_1_lower_surface_wireframe = rli_blade_1.project(rli_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
rli_blade_1_ll_mesh = am.linspace(rli_blade_1_upper_surface_wireframe, rli_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, -8.672 - off_set, 9.321+ off_set]), np.array([13.760 - off_set_long_le, -8.600 - off_set, 9.003 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_te_root, -7.784 + off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, -8.000 + off_set, 9.385 + off_set]), num_lifting_line)

rli_blade_2_le_low_res = rli_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rli_blade_2_te_low_res = rli_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rli_blade_2_chord_surface = am.linspace(rli_blade_2_le_low_res, rli_blade_2_te_low_res, 2)
rli_blade_2_upper_surface_wireframe = rli_blade_2.project(rli_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rli_blade_2_lower_surface_wireframe = rli_blade_2.project(rli_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
rli_blade_2_ll_mesh = am.linspace(rli_blade_2_upper_surface_wireframe, rli_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([rli_blade_1_ll_mesh, rli_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, -8.672 - off_set, 9.321+ off_set]), np.array([13.760 - off_set_long_le, -8.600 - off_set, 9.003 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([17.710 + off_set_long_te_root, -7.784 + off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, -8.000 + off_set, 9.385 + off_set]), num_radial)
rli_blade_2_le_high_res = rli_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rli_blade_2_te_high_res = rli_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rli_chord_length = am.subtract(rli_blade_2_le_high_res, rli_blade_2_te_high_res)

# twist
rli_le_proj_disk = rli_disk.project(rli_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rli_te_proj_disk = rli_disk.project(rli_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rli_v_dist_le = am.subtract(rli_blade_2_le_high_res, rli_le_proj_disk)
rli_v_dist_te = am.subtract(rli_blade_2_te_high_res, rli_te_proj_disk)
rli_tot_v_dist = am.subtract(rli_v_dist_le, rli_v_dist_te)
# endregion

# region rear right inner (rri) rotor meshes
# disk
y11 = rri_disk.project(np.array([18.760, 13.401, 8.604]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rri_disk.project(np.array([18.760, 3.499, 9.996]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rri_disk.project(np.array([13.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rri_disk.project(np.array([23.760, 8.450, 9.300]), direction=np.array([0., 0., -1.]), plot=False)
rri_in_plane_y = am.subtract(y12, y11)
rri_in_plane_x = am.subtract(y21, y22)
rri_origin = rri_disk.project(np.array([18.760, 8.537, 9.919]), direction=np.array([0., 0., -1.]))

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_le, 8.243 - off_set, 9.381 + off_set]), np.array([23.760 + off_set_long_le, 8.298 - off_set, 9.315 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([19.810 - off_set_long_te_root, 9.073 + off_set, 9.058 + off_set]), np.array([23.761 + off_set_long_te_tip, 8.906 + off_set, 9.257 + off_set]), num_lifting_line)

rri_blade_1_le_low_res = rri_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rri_blade_1_te_low_res = rri_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rri_blade_1_chord_surface = am.linspace(rri_blade_1_le_low_res, rri_blade_1_te_low_res, 2)
rri_blade_1_upper_surface_wireframe = rri_blade_1.project(rri_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rri_blade_1_lower_surface_wireframe = rri_blade_1.project(rri_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
rri_blade_1_ll_mesh = am.linspace(rri_blade_1_upper_surface_wireframe, rri_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, 8.672 + off_set, 9.321 + off_set]), np.array([13.760 - off_set_long_le, 8.600 + off_set, 9.003 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([17.710 + off_set_long_te_root, 7.784 - off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, 8.000 - off_set, 9.385 + off_set]), num_lifting_line)

rri_blade_2_le_low_res = rri_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rri_blade_2_te_low_res = rri_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rri_blade_2_chord_surface = am.linspace(rri_blade_2_le_low_res, rri_blade_2_te_low_res, 2)
rri_blade_2_upper_surface_wireframe = rri_blade_2.project(rri_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
rri_blade_2_lower_surface_wireframe = rri_blade_2.project(rri_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
rri_blade_2_ll_mesh = am.linspace(rri_blade_2_upper_surface_wireframe, rri_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([rri_blade_1_ll_mesh, rri_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([17.710 + off_set_long_le, 8.672 + off_set, 9.321 + off_set]), np.array([13.760 - off_set_long_le, 8.600 + off_set, 9.003 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([17.710 + off_set_long_te_root, 7.784 - off_set, 9.239 + off_set]), np.array([13.760 - off_set_long_te_tip, 8.000 - off_set, 9.385 + off_set]), num_radial)
rri_blade_2_le_high_res = rri_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rri_blade_2_te_high_res = rri_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# rri_chord_length = am.norm(am.subtract(rri_blade_2_le_high_res, rri_blade_2_te_high_res), axes=(1, ))
rri_chord_length = am.subtract(rri_blade_2_le_high_res, rri_blade_2_te_high_res)

# twist
rri_le_proj_disk = rri_disk.project(rri_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
rri_te_proj_disk = rri_disk.project(rri_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

rri_v_dist_le = am.subtract(rri_blade_2_le_high_res, rri_le_proj_disk)
rri_v_dist_te = am.subtract(rri_blade_2_te_high_res, rri_te_proj_disk)
rri_tot_v_dist = am.subtract(rri_v_dist_le, rri_v_dist_te)

# endregion

# region front left inner (fli) rotor meshes
# disk
y11 = fli_disk.project(np.array([4.630, -3.179, 7.736]), direction=np.array([0., 0., -1.]), plot=False)
y12 = fli_disk.project(np.array([4.630, -13.081, 6.344]), direction=np.array([0., 0., -1.]), plot=False)
y21 = fli_disk.project(np.array([-0.370, -8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
y22 = fli_disk.project(np.array([9.630, -8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
fli_in_plane_y = am.subtract(y12, y11)
fli_in_plane_x = am.subtract(y21, y22)
fli_origin = fli_disk.project(np.array([4.630, -8.217, 7.659]), direction=np.array([0., 0., -1.]), plot=False)

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([5.680 - off_set_long_le, -8.352 - off_set, 7.061 + off_set]), np.array([9.630 + off_set_long_le, -8.280 - off_set, 7.012 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([5.680 - off_set_long_te_root, -7.464 + off_set, 6.798 + off_set]), np.array([9.630 + off_set_long_te_tip, -7.680 + off_set, 7.125 + off_set]), num_lifting_line)

fli_blade_1_le_low_res = fli_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fli_blade_1_te_low_res = fli_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fli_blade_1_chord_surface = am.linspace(fli_blade_1_le_low_res, fli_blade_1_te_low_res, 2)
fli_blade_1_upper_surface_wireframe = fli_blade_1.project(fli_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
fli_blade_1_lower_surface_wireframe = fli_blade_1.project(fli_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
fli_blade_1_ll_mesh = am.linspace(fli_blade_1_upper_surface_wireframe, fli_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_le, -7.923 + off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, -7.978 + off_set, 7.055 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, -8.753 - off_set, 6.577 + off_set]), np.array([-0.370 - off_set_long_te_tip, -8.586 - off_set, 6.998 + off_set]), num_lifting_line)

fli_blade_2_le_low_res = fli_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fli_blade_2_te_low_res = fli_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fli_blade_2_chord_surface = am.linspace(fli_blade_2_le_low_res, fli_blade_2_te_low_res, 2)
fli_blade_2_upper_surface_wireframe = fli_blade_2.project(fli_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
fli_blade_2_lower_surface_wireframe = fli_blade_2.project(fli_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
fli_blade_2_ll_mesh = am.linspace(fli_blade_2_upper_surface_wireframe, fli_blade_2_lower_surface_wireframe, 1) 
# spatial_rep.plot_meshes([fli_blade_1_ll_mesh, fli_blade_2_ll_mesh])

# chord 
b2_le_high_res_numpy = np.linspace(np.array([3.580 + off_set_long_le, -7.923 + off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, -7.978 + off_set, 7.055 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, -8.753 - off_set, 6.577 + off_set]), np.array([-0.370 - off_set_long_te_tip, -8.586 - off_set, 6.998 + off_set]), num_radial)
fli_blade_2_le_high_res = fli_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fli_blade_2_te_high_res = fli_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fli_chord_length = am.norm(am.subtract(fli_blade_2_le_high_res, fli_blade_2_te_high_res), axes=(1, ))
fli_chord_length = am.subtract(fli_blade_2_le_high_res, fli_blade_2_te_high_res)

# twist
fli_le_proj_disk = fli_disk.project(fli_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fli_te_proj_disk = fli_disk.project(fli_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fli_v_dist_le = am.subtract(fli_blade_2_le_high_res, fli_le_proj_disk)
fli_v_dist_te = am.subtract(fli_blade_2_te_high_res, fli_te_proj_disk)
fli_tot_v_dist = am.subtract(fli_v_dist_le, fli_v_dist_te)
# endregion

# region front right inner (fri) rotor meshes
# disk
y11 = fri_disk.project(np.array([4.630, 13.081, 6.344]), direction=np.array([0., 0., -1.]), plot=False)
y12 = fri_disk.project(np.array([4.630, 3.179, 7.736]), direction=np.array([0., 0., -1.]), plot=False)
y21 = fri_disk.project(np.array([-0.370, 8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
y22 = fri_disk.project(np.array([9.630, 8.130, 7.040]), direction=np.array([0., 0., -1.]), plot=False)
fri_in_plane_y = am.subtract(y12, y11)
fri_in_plane_x = am.subtract(y21, y22)
fri_origin = fri_disk.project(np.array([4.630, 8.217, 7.659]), direction=np.array([0., 0., -1.]), plot=False)

# lifting line mesh
# blade 1
b1_le_low_res_numpy = np.linspace(np.array([5.680 - off_set_long_le, 8.672 + off_set, 7.061 + off_set]), np.array([9.630 + off_set_long_le, 8.600 + off_set, 7.012 + off_set]), num_lifting_line)
b1_te_low_res_numpy = np.linspace(np.array([5.680 - off_set_long_te_root, 7.784 - off_set, 6.979 + off_set]), np.array([9.630 + off_set_long_te_tip, 8.000 - off_set, 7.125 + off_set]), num_lifting_line)

fri_blade_1_le_low_res = fri_blade_1.project(b1_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fri_blade_1_te_low_res = fri_blade_1.project(b1_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fri_blade_1_chord_surface = am.linspace(fri_blade_1_le_low_res, fri_blade_1_te_low_res, 2)
fri_blade_1_upper_surface_wireframe = fri_blade_1.project(fri_blade_1_chord_surface.value + np.array([0., 0.,  1.]), direction=np.array([0., 0., -1.]), grid_search_n=25)
fri_blade_1_lower_surface_wireframe = fri_blade_1.project(fri_blade_1_chord_surface.value - np.array([0., 0.,  1.]), direction=np.array([0., 0.,  1.]), grid_search_n=25)
fri_blade_1_ll_mesh = am.linspace(fri_blade_1_upper_surface_wireframe, fri_blade_1_lower_surface_wireframe, 1) 


# blade 2
b2_le_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_le, 8.243 - off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, 8.298 - off_set, 7.055 + off_set]), num_lifting_line)
b2_te_low_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, 9.073 + off_set, 6.798 + off_set]), np.array([-0.370 - off_set_long_te_tip, 8.906 + off_set, 6.998 + off_set]), num_lifting_line)

fri_blade_2_le_low_res = fri_blade_2.project(b2_le_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fri_blade_2_te_low_res = fri_blade_2.project(b2_te_low_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fri_blade_2_chord_surface = am.linspace(fri_blade_2_le_low_res, fri_blade_2_te_low_res, 2)
fri_blade_2_upper_surface_wireframe = fri_blade_2.project(fri_blade_2_chord_surface.value + np.array([0., 0., 1.01]), direction=np.array([0., 0., -1.]), grid_search_n=25)
fri_blade_2_lower_surface_wireframe = fri_blade_2.project(fri_blade_2_chord_surface.value - np.array([0., 0., -1.02]), direction=np.array([0., 0., 1.]), grid_search_n=25)
fri_blade_2_ll_mesh = am.linspace(fri_blade_2_upper_surface_wireframe, fri_blade_2_lower_surface_wireframe, 1) 

# chord 
b2_le_high_res_numpy = np.linspace(np.array([3.580 + off_set_long_le, 8.243 - off_set, 7.121 + off_set]), np.array([-0.370 - off_set_long_le, 8.298 - off_set, 7.055 + off_set]), num_radial)
b2_te_high_res_numpy = np.linspace(np.array([3.580 + off_set_long_te_root, 9.073 + off_set, 6.798 + off_set]), np.array([-0.370 - off_set_long_te_tip, 8.906 + off_set, 6.998 + off_set]), num_radial)
fri_blade_2_le_high_res = fri_blade_2.project(b2_le_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fri_blade_2_te_high_res = fri_blade_2.project(b2_te_high_res_numpy, direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
# fri_chord_length = am.norm(am.subtract(fri_blade_2_le_high_res, fri_blade_2_te_high_res), axes=(1, ))
fri_chord_length = am.subtract(fri_blade_2_le_high_res, fri_blade_2_te_high_res)

# twist
fri_le_proj_disk = fri_disk.project(fri_blade_2_le_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)
fri_te_proj_disk = fri_disk.project(fri_blade_2_te_high_res.evaluate(), direction=np.array([0., 0., -1.]), grid_search_n=50, plot=False)

fri_v_dist_le = am.subtract(fri_blade_2_le_high_res, fri_le_proj_disk)
fri_v_dist_te = am.subtract(fri_blade_2_te_high_res, fri_te_proj_disk)
fri_tot_v_dist = am.subtract(fri_v_dist_le, fri_v_dist_te)
# endregion

# spatial_rep.plot_meshes([
#     wing_camber_surface, htail_camber_surface,
#     flo_blade_1_ll_mesh, flo_blade_2_ll_mesh,
#     fli_blade_1_ll_mesh, fli_blade_2_ll_mesh,
#     fri_blade_1_ll_mesh, fri_blade_2_ll_mesh,
#     fro_blade_1_ll_mesh, fro_blade_2_ll_mesh,
#     rlo_blade_1_ll_mesh, rlo_blade_2_ll_mesh,
#     rli_blade_1_ll_mesh, rli_blade_2_ll_mesh,
#     rri_blade_1_ll_mesh, rri_blade_2_ll_mesh,
#     rro_blade_1_ll_mesh, rro_blade_2_ll_mesh,
#     ]
# )
# endregion

# region actuations
configuration_names = [
    "minus_1g_configuration", 
    "plus_3g_configuration",
    "hover_1_configuration", 
    "cruise_configuration", 
    "climb_configuration",
    "quasi_steady_transition_1",    
    "quasi_steady_transition_2",    
    "quasi_steady_transition_3",    
    "quasi_steady_transition_4",    
    "quasi_steady_transition_5",    
    "quasi_steady_transition_6",    
    "quasi_steady_transition_7",    
    "quasi_steady_transition_8",    
    "quasi_steady_transition_9",    
    "quasi_steady_transition_10",    
]
system_configurations = lpc_rep.declare_configurations(names=configuration_names)

# region Projections
horizontal_stabilizer_quarter_chord_port = htail.project(np.array([28.5, -10., 8.]))
horizontal_stabilizer_quarter_chord_starboard = htail.project(np.array([28.5, 10., 8.]))
horizontal_stabilizer_actuation_axis = horizontal_stabilizer_quarter_chord_starboard - horizontal_stabilizer_quarter_chord_port

wing_quarter_chord_port = wing.project(np.array([28.5, -10., 8.]))
wing_quarter_chord_starboard = wing.project(np.array([28.5, 10., 8.]))
wing_actuation_axis = wing_quarter_chord_starboard - wing_quarter_chord_port
# endregion

wing_vlm_mesh_name = f"{wing.parameters['name']}_vlm_mesh"
htail_vlm_mesh_name =  f"{htail.parameters['name']}_vlm_mesh"

# region Cruise
cruise_configuration = system_configurations['cruise_configuration']
cruise_configuration.set_num_nodes(num_nodes=1)
cruise_configuration.add_output(f"{htail_vlm_mesh_name}_cruise", htail_camber_surface)
cruise_configuration.add_output(f"{wing_vlm_mesh_name}_cruise", wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='cruise_tail_actuation', value=0, units='radians')
cruise_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='cruise_wing_actuation', value=np.deg2rad(0) , units='radians')
cruise_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region Climb 
climb_configuration = system_configurations['climb_configuration']
climb_configuration.set_num_nodes(num_nodes=1)
climb_configuration.add_output(f"{htail_vlm_mesh_name}_climb", htail_camber_surface)
climb_configuration.add_output(f'{wing_vlm_mesh_name}_climb', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='climb_tail_actuation', value=0, units='radians')
climb_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='climb_wing_actuation', value=np.deg2rad(4) , units='radians')
climb_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region +3g 
plus_3g_configuration = system_configurations['plus_3g_configuration']
plus_3g_configuration.set_num_nodes(num_nodes=1)
plus_3g_configuration.add_output(f'{htail_vlm_mesh_name}_plus_3g', htail_camber_surface)
plus_3g_configuration.add_output(f'{wing_vlm_mesh_name}_plus_3g', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='plus_3g_tail_actuation', value=0, units='radians')
plus_3g_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='plus_3g_wing_actuation', value=np.deg2rad(0) , units='radians')
plus_3g_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region -1g 
minus_1g_configuration = system_configurations['minus_1g_configuration']
minus_1g_configuration.set_num_nodes(num_nodes=1)
minus_1g_configuration.add_output(f'{htail_vlm_mesh_name}_minus_1g', htail_camber_surface)
minus_1g_configuration.add_output(f'{wing_vlm_mesh_name}_minus_1g', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='minus_1g_tail_actuation', value=0, units='radians')
minus_1g_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='minus_1g_wing_actuation', value=np.deg2rad(0) , units='radians')
minus_1g_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region hover 1
hover_1_configuration = system_configurations['hover_1_configuration']
hover_1_configuration.set_num_nodes(num_nodes=1)

hover_1_configuration.add_output(f"hover_1_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{rlo_disk.parameters['name']}_origin", rlo_origin)

hover_1_configuration.add_output(f"hover_1_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{rli_disk.parameters['name']}_origin", rli_origin)

hover_1_configuration.add_output(f"hover_1_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{rri_disk.parameters['name']}_origin", rri_origin)

hover_1_configuration.add_output(f"hover_1_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{rro_disk.parameters['name']}_origin", rro_origin)

hover_1_configuration.add_output(f"hover_1_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{flo_disk.parameters['name']}_origin", flo_origin)

hover_1_configuration.add_output(f"hover_1_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{fli_disk.parameters['name']}_origin", fli_origin)

hover_1_configuration.add_output(f"hover_1_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{fri_disk.parameters['name']}_origin", fri_origin)

hover_1_configuration.add_output(f"hover_1_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
hover_1_configuration.add_output(f"hover_1_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
hover_1_configuration.add_output(f"hover_1_{fro_disk.parameters['name']}_origin", fro_origin)


# rlo
rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
rlo_disk_actuator_solver_1.set_rotation(name='hover_1_rlo_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
rlo_disk_actuator_solver_2.set_rotation(name='hover_1_rlo_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# rli
rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
rli_disk_actuator_solver_1.set_rotation(name='hover_1_rli_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=rli_disk_actuator_solver_1)

rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
rli_disk_actuator_solver_2.set_rotation(name='hover_1_rli_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# rri
rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
rri_disk_actuator_solver_1.set_rotation(name='hover_1_rri_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=rri_disk_actuator_solver_1)

rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
rri_disk_actuator_solver_2.set_rotation(name='hover_1_rri_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# rro
rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
rro_disk_actuator_solver_1.set_rotation(name='hover_1_rro_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=rro_disk_actuator_solver_1)

rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
rro_disk_actuator_solver_2.set_rotation(name='hover_1_rro_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# flo
flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
flo_disk_actuator_solver_1.set_rotation(name='hover_1_flo_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=flo_disk_actuator_solver_1)

flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
flo_disk_actuator_solver_2.set_rotation(name='hover_1_flo_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# fli
fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
fli_disk_actuator_solver_1.set_rotation(name='hover_1_fli_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=fli_disk_actuator_solver_1)

fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
fli_disk_actuator_solver_2.set_rotation(name='hover_1_fli_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# fri
fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
fri_disk_actuator_solver_1.set_rotation(name='hover_1_fri_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=fri_disk_actuator_solver_1)

fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
fri_disk_actuator_solver_2.set_rotation(name='hover_1_fri_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# fro
fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
fro_disk_actuator_solver_1.set_rotation(name='hover_1_fro_disk_actuation_1', value=0, units='radians')
hover_1_configuration.actuate(transformation=fro_disk_actuator_solver_1)

fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
fro_disk_actuator_solver_2.set_rotation(name='hover_1_fro_disk_actuation_2', value=0, units='radians')
hover_1_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# endregion 

# region quasi_steady_transition_1
qst_1_configuration = system_configurations['quasi_steady_transition_1']
qst_1_configuration.set_num_nodes(num_nodes=1)
qst_1_configuration.add_output(f'{htail_vlm_mesh_name}_qst_1', htail_camber_surface)
qst_1_configuration.add_output(f'{wing_vlm_mesh_name}_qst_1', wing_camber_surface)

qst_1_configuration.add_output(f"qst_1_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{rlo_disk.parameters['name']}_origin", rlo_origin)

qst_1_configuration.add_output(f"qst_1_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{rli_disk.parameters['name']}_origin", rli_origin)

qst_1_configuration.add_output(f"qst_1_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{rri_disk.parameters['name']}_origin", rri_origin)

qst_1_configuration.add_output(f"qst_1_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{rro_disk.parameters['name']}_origin", rro_origin)

qst_1_configuration.add_output(f"qst_1_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{flo_disk.parameters['name']}_origin", flo_origin)

qst_1_configuration.add_output(f"qst_1_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{fli_disk.parameters['name']}_origin", fli_origin)

qst_1_configuration.add_output(f"qst_1_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{fri_disk.parameters['name']}_origin", fri_origin)

qst_1_configuration.add_output(f"qst_1_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
qst_1_configuration.add_output(f"qst_1_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
qst_1_configuration.add_output(f"qst_1_{fro_disk.parameters['name']}_origin", fro_origin)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_1_tail_actuation', value=0, units='radians')
qst_1_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_1_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_1_configuration.actuate(transformation=wing_actuator_solver)

# rlo
rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
rlo_disk_actuator_solver_1.set_rotation(name='qst_1_rlo_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
rlo_disk_actuator_solver_2.set_rotation(name='qst_1_rlo_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# rli
rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
rli_disk_actuator_solver_1.set_rotation(name='qst_1_rli_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=rli_disk_actuator_solver_1)

rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
rli_disk_actuator_solver_2.set_rotation(name='qst_1_rli_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# rri
rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
rri_disk_actuator_solver_1.set_rotation(name='qst_1_rri_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=rri_disk_actuator_solver_1)

rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
rri_disk_actuator_solver_2.set_rotation(name='qst_1_rri_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# rro
rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
rro_disk_actuator_solver_1.set_rotation(name='qst_1_rro_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=rro_disk_actuator_solver_1)

rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
rro_disk_actuator_solver_2.set_rotation(name='qst_1_rro_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# flo
flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
flo_disk_actuator_solver_1.set_rotation(name='qst_1_flo_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=flo_disk_actuator_solver_1)

flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
flo_disk_actuator_solver_2.set_rotation(name='qst_1_flo_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# fli
fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
fli_disk_actuator_solver_1.set_rotation(name='qst_1_fli_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=fli_disk_actuator_solver_1)

fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
fli_disk_actuator_solver_2.set_rotation(name='qst_1_fli_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# fri
fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
fri_disk_actuator_solver_1.set_rotation(name='qst_1_fri_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=fri_disk_actuator_solver_1)

fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
fri_disk_actuator_solver_2.set_rotation(name='qst_1_fri_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# fro
fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
fro_disk_actuator_solver_1.set_rotation(name='qst_1_fro_disk_actuation_1', value=0, units='radians')
qst_1_configuration.actuate(transformation=fro_disk_actuator_solver_1)

fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
fro_disk_actuator_solver_2.set_rotation(name='qst_1_fro_disk_actuation_2', value=0, units='radians')
qst_1_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# endregion

# region quasi_steady_transition_2
qst_2_configuration = system_configurations['quasi_steady_transition_2']
qst_2_configuration.set_num_nodes(num_nodes=1)
qst_2_configuration.add_output(f'{htail_vlm_mesh_name}_qst_2', htail_camber_surface)
qst_2_configuration.add_output(f'{wing_vlm_mesh_name}_qst_2', wing_camber_surface)

qst_2_configuration.add_output(f"qst_2_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{rlo_disk.parameters['name']}_origin", rlo_origin)

qst_2_configuration.add_output(f"qst_2_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{rli_disk.parameters['name']}_origin", rli_origin)

qst_2_configuration.add_output(f"qst_2_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{rri_disk.parameters['name']}_origin", rri_origin)

qst_2_configuration.add_output(f"qst_2_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{rro_disk.parameters['name']}_origin", rro_origin)

qst_2_configuration.add_output(f"qst_2_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{flo_disk.parameters['name']}_origin", flo_origin)

qst_2_configuration.add_output(f"qst_2_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{fli_disk.parameters['name']}_origin", fli_origin)

qst_2_configuration.add_output(f"qst_2_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{fri_disk.parameters['name']}_origin", fri_origin)

qst_2_configuration.add_output(f"qst_2_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
qst_2_configuration.add_output(f"qst_2_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
qst_2_configuration.add_output(f"qst_2_{fro_disk.parameters['name']}_origin", fro_origin)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_2_tail_actuation', value=0, units='radians')
qst_2_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_2_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_2_configuration.actuate(transformation=wing_actuator_solver)

# rlo
rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
rlo_disk_actuator_solver_1.set_rotation(name='qst_2_rlo_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
rlo_disk_actuator_solver_2.set_rotation(name='qst_2_rlo_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# rli
rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
rli_disk_actuator_solver_1.set_rotation(name='qst_2_rli_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=rli_disk_actuator_solver_1)

rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
rli_disk_actuator_solver_2.set_rotation(name='qst_2_rli_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# rri
rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
rri_disk_actuator_solver_1.set_rotation(name='qst_2_rri_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=rri_disk_actuator_solver_1)

rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
rri_disk_actuator_solver_2.set_rotation(name='qst_2_rri_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# rro
rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
rro_disk_actuator_solver_1.set_rotation(name='qst_2_rro_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=rro_disk_actuator_solver_1)

rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
rro_disk_actuator_solver_2.set_rotation(name='qst_2_rro_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# flo
flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
flo_disk_actuator_solver_1.set_rotation(name='qst_2_flo_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=flo_disk_actuator_solver_1)

flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
flo_disk_actuator_solver_2.set_rotation(name='qst_2_flo_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# fli
fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
fli_disk_actuator_solver_1.set_rotation(name='qst_2_fli_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=fli_disk_actuator_solver_1)

fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
fli_disk_actuator_solver_2.set_rotation(name='qst_2_fli_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# fri
fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
fri_disk_actuator_solver_1.set_rotation(name='qst_2_fri_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=fri_disk_actuator_solver_1)

fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
fri_disk_actuator_solver_2.set_rotation(name='qst_2_fri_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# fro
fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
fro_disk_actuator_solver_1.set_rotation(name='qst_2_fro_disk_actuation_1', value=0, units='radians')
qst_2_configuration.actuate(transformation=fro_disk_actuator_solver_1)

fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
fro_disk_actuator_solver_2.set_rotation(name='qst_2_fro_disk_actuation_2', value=0, units='radians')
qst_2_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# endregion

# region quasi_steady_transition_3
qst_3_configuration = system_configurations['quasi_steady_transition_3']
qst_3_configuration.set_num_nodes(num_nodes=1)
qst_3_configuration.add_output(f'{htail_vlm_mesh_name}_qst_3', htail_camber_surface)
qst_3_configuration.add_output(f'{wing_vlm_mesh_name}_qst_3', wing_camber_surface)

qst_3_configuration.add_output(f"qst_3_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{rlo_disk.parameters['name']}_origin", rlo_origin)

qst_3_configuration.add_output(f"qst_3_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{rli_disk.parameters['name']}_origin", rli_origin)

qst_3_configuration.add_output(f"qst_3_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{rri_disk.parameters['name']}_origin", rri_origin)

qst_3_configuration.add_output(f"qst_3_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{rro_disk.parameters['name']}_origin", rro_origin)

qst_3_configuration.add_output(f"qst_3_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{flo_disk.parameters['name']}_origin", flo_origin)

qst_3_configuration.add_output(f"qst_3_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{fli_disk.parameters['name']}_origin", fli_origin)

qst_3_configuration.add_output(f"qst_3_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{fri_disk.parameters['name']}_origin", fri_origin)

qst_3_configuration.add_output(f"qst_3_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
qst_3_configuration.add_output(f"qst_3_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
qst_3_configuration.add_output(f"qst_3_{fro_disk.parameters['name']}_origin", fro_origin)


horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_3_tail_actuation', value=0, units='radians')
qst_3_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_3_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_3_configuration.actuate(transformation=wing_actuator_solver)

# rlo
rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
rlo_disk_actuator_solver_1.set_rotation(name='qst_3_rlo_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
rlo_disk_actuator_solver_2.set_rotation(name='qst_3_rlo_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# rli
rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
rli_disk_actuator_solver_1.set_rotation(name='qst_3_rli_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=rli_disk_actuator_solver_1)

rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
rli_disk_actuator_solver_2.set_rotation(name='qst_3_rli_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# rri
rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
rri_disk_actuator_solver_1.set_rotation(name='qst_3_rri_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=rri_disk_actuator_solver_1)

rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
rri_disk_actuator_solver_2.set_rotation(name='qst_3_rri_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# rro
rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
rro_disk_actuator_solver_1.set_rotation(name='qst_3_rro_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=rro_disk_actuator_solver_1)

rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
rro_disk_actuator_solver_2.set_rotation(name='qst_3_rro_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# flo
flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
flo_disk_actuator_solver_1.set_rotation(name='qst_3_flo_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=flo_disk_actuator_solver_1)

flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
flo_disk_actuator_solver_2.set_rotation(name='qst_3_flo_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# fli
fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
fli_disk_actuator_solver_1.set_rotation(name='qst_3_fli_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=fli_disk_actuator_solver_1)

fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
fli_disk_actuator_solver_2.set_rotation(name='qst_3_fli_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# fri
fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
fri_disk_actuator_solver_1.set_rotation(name='qst_3_fri_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=fri_disk_actuator_solver_1)

fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
fri_disk_actuator_solver_2.set_rotation(name='qst_3_fri_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# fro
fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
fro_disk_actuator_solver_1.set_rotation(name='qst_3_fro_disk_actuation_1', value=0, units='radians')
qst_3_configuration.actuate(transformation=fro_disk_actuator_solver_1)

fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
fro_disk_actuator_solver_2.set_rotation(name='qst_3_fro_disk_actuation_2', value=0, units='radians')
qst_3_configuration.actuate(transformation=fro_disk_actuator_solver_2)
# endregion

# region quasi_steady_transition_4
qst_4_configuration = system_configurations['quasi_steady_transition_4']
qst_4_configuration.set_num_nodes(num_nodes=1)
qst_4_configuration.add_output(f'{htail_vlm_mesh_name}_qst_4', htail_camber_surface)
qst_4_configuration.add_output(f'{wing_vlm_mesh_name}_qst_4', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_4_tail_actuation', value=0, units='radians')
qst_4_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_4_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_4_configuration.actuate(transformation=wing_actuator_solver)

qst_4_configuration.add_output(f"qst_4_{rlo_disk.parameters['name']}_in_plane_1", rlo_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{rlo_disk.parameters['name']}_in_plane_2", rlo_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{rlo_disk.parameters['name']}_origin", rlo_origin)

qst_4_configuration.add_output(f"qst_4_{rli_disk.parameters['name']}_in_plane_1", rli_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{rli_disk.parameters['name']}_in_plane_2", rli_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{rli_disk.parameters['name']}_origin", rli_origin)

qst_4_configuration.add_output(f"qst_4_{rri_disk.parameters['name']}_in_plane_1", rri_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{rri_disk.parameters['name']}_in_plane_2", rri_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{rri_disk.parameters['name']}_origin", rri_origin)

qst_4_configuration.add_output(f"qst_4_{rro_disk.parameters['name']}_in_plane_1", rro_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{rro_disk.parameters['name']}_in_plane_2", rro_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{rro_disk.parameters['name']}_origin", rro_origin)

qst_4_configuration.add_output(f"qst_4_{flo_disk.parameters['name']}_in_plane_1", flo_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{flo_disk.parameters['name']}_in_plane_2", flo_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{flo_disk.parameters['name']}_origin", flo_origin)

qst_4_configuration.add_output(f"qst_4_{fli_disk.parameters['name']}_in_plane_1", fli_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{fli_disk.parameters['name']}_in_plane_2", fli_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{fli_disk.parameters['name']}_origin", fli_origin)

qst_4_configuration.add_output(f"qst_4_{fri_disk.parameters['name']}_in_plane_1", fri_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{fri_disk.parameters['name']}_in_plane_2", fri_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{fri_disk.parameters['name']}_origin", fri_origin)

qst_4_configuration.add_output(f"qst_4_{fro_disk.parameters['name']}_in_plane_1", fro_in_plane_y)
qst_4_configuration.add_output(f"qst_4_{fro_disk.parameters['name']}_in_plane_2", fro_in_plane_x)
qst_4_configuration.add_output(f"qst_4_{fro_disk.parameters['name']}_origin", fro_origin)

# rlo
rlo_disk_actuator_solver_1 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_y)
rlo_disk_actuator_solver_1.set_rotation(name='qst_4_rlo_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=rlo_disk_actuator_solver_1)

rlo_disk_actuator_solver_2 = PrescribedRotation(component=rlo_disk, axis_origin=rlo_origin, axis_vector=rlo_in_plane_x)
rlo_disk_actuator_solver_2.set_rotation(name='qst_4_rlo_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=rlo_disk_actuator_solver_2)

# rli
rli_disk_actuator_solver_1 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_y)
rli_disk_actuator_solver_1.set_rotation(name='qst_4_rli_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=rli_disk_actuator_solver_1)

rli_disk_actuator_solver_2 = PrescribedRotation(component=rli_disk, axis_origin=rli_origin, axis_vector=rli_in_plane_x)
rli_disk_actuator_solver_2.set_rotation(name='qst_4_rli_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=rli_disk_actuator_solver_2)

# rri
rri_disk_actuator_solver_1 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_y)
rri_disk_actuator_solver_1.set_rotation(name='qst_4_rri_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=rri_disk_actuator_solver_1)

rri_disk_actuator_solver_2 = PrescribedRotation(component=rri_disk, axis_origin=rri_origin, axis_vector=rri_in_plane_x)
rri_disk_actuator_solver_2.set_rotation(name='qst_4_rri_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=rri_disk_actuator_solver_2)

# rro
rro_disk_actuator_solver_1 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_y)
rro_disk_actuator_solver_1.set_rotation(name='qst_4_rro_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=rro_disk_actuator_solver_1)

rro_disk_actuator_solver_2 = PrescribedRotation(component=rro_disk, axis_origin=rro_origin, axis_vector=rro_in_plane_x)
rro_disk_actuator_solver_2.set_rotation(name='qst_4_rro_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=rro_disk_actuator_solver_2)

# flo
flo_disk_actuator_solver_1 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_y)
flo_disk_actuator_solver_1.set_rotation(name='qst_4_flo_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=flo_disk_actuator_solver_1)

flo_disk_actuator_solver_2 = PrescribedRotation(component=flo_disk, axis_origin=flo_origin, axis_vector=flo_in_plane_x)
flo_disk_actuator_solver_2.set_rotation(name='qst_4_flo_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=flo_disk_actuator_solver_2)

# fli
fli_disk_actuator_solver_1 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_y)
fli_disk_actuator_solver_1.set_rotation(name='qst_4_fli_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=fli_disk_actuator_solver_1)

fli_disk_actuator_solver_2 = PrescribedRotation(component=fli_disk, axis_origin=fli_origin, axis_vector=fli_in_plane_x)
fli_disk_actuator_solver_2.set_rotation(name='qst_4_fli_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=fli_disk_actuator_solver_2)

# fri
fri_disk_actuator_solver_1 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_y)
fri_disk_actuator_solver_1.set_rotation(name='qst_4_fri_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=fri_disk_actuator_solver_1)

fri_disk_actuator_solver_2 = PrescribedRotation(component=fri_disk, axis_origin=fri_origin, axis_vector=fri_in_plane_x)
fri_disk_actuator_solver_2.set_rotation(name='qst_4_fri_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=fri_disk_actuator_solver_2)

# fro
fro_disk_actuator_solver_1 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_y)
fro_disk_actuator_solver_1.set_rotation(name='qst_4_fro_disk_actuation_1', value=0, units='radians')
qst_4_configuration.actuate(transformation=fro_disk_actuator_solver_1)

fro_disk_actuator_solver_2 = PrescribedRotation(component=fro_disk, axis_origin=fro_origin, axis_vector=fro_in_plane_x)
fro_disk_actuator_solver_2.set_rotation(name='qst_4_fro_disk_actuation_2', value=0, units='radians')
qst_4_configuration.actuate(transformation=fro_disk_actuator_solver_2)

# endregion

# region quasi_steady_transition_5
qst_5_configuration = system_configurations['quasi_steady_transition_5']
qst_5_configuration.set_num_nodes(num_nodes=1)
qst_5_configuration.add_output(f'{htail_vlm_mesh_name}_qst_5', htail_camber_surface)
qst_5_configuration.add_output(f'{wing_vlm_mesh_name}_qst_5', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_5_tail_actuation', value=0, units='radians')
qst_5_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_5_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_5_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region quasi_steady_transition_6
qst_6_configuration = system_configurations['quasi_steady_transition_6']
qst_6_configuration.set_num_nodes(num_nodes=1)
qst_6_configuration.add_output(f'{htail_vlm_mesh_name}_qst_6', htail_camber_surface)
qst_6_configuration.add_output(f'{wing_vlm_mesh_name}_qst_6', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_6_tail_actuation', value=0, units='radians')
qst_6_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_6_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_6_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region quasi_steady_transition_7
qst_7_configuration = system_configurations['quasi_steady_transition_7']
qst_7_configuration.set_num_nodes(num_nodes=1)
qst_7_configuration.add_output(f'{htail_vlm_mesh_name}_qst_7', htail_camber_surface)
qst_7_configuration.add_output(f'{wing_vlm_mesh_name}_qst_7', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_7_tail_actuation', value=0, units='radians')
qst_7_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_7_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_7_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region quasi_steady_transition_8
qst_8_configuration = system_configurations['quasi_steady_transition_8']
qst_8_configuration.set_num_nodes(num_nodes=1)
qst_8_configuration.add_output(f'{htail_vlm_mesh_name}_qst_8', htail_camber_surface)
qst_8_configuration.add_output(f'{wing_vlm_mesh_name}_qst_8', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_8_tail_actuation', value=0, units='radians')
qst_8_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_8_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_8_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region quasi_steady_transition_9
qst_9_configuration = system_configurations['quasi_steady_transition_9']
qst_9_configuration.set_num_nodes(num_nodes=1)
qst_9_configuration.add_output(f'{htail_vlm_mesh_name}_qst_9', htail_camber_surface)
qst_9_configuration.add_output(f'{wing_vlm_mesh_name}_qst_9', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_9_tail_actuation', value=0, units='radians')
qst_9_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_9_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_9_configuration.actuate(transformation=wing_actuator_solver)
# endregion

# region quasi_steady_transition_10
qst_10_configuration = system_configurations['quasi_steady_transition_10']
qst_10_configuration.set_num_nodes(num_nodes=1)
qst_10_configuration.add_output(f'{htail_vlm_mesh_name}_qst_10', htail_camber_surface)
qst_10_configuration.add_output(f'{wing_vlm_mesh_name}_qst_10', wing_camber_surface)

horizontal_stabilizer_actuator_solver = PrescribedRotation(component=htail, axis_origin=horizontal_stabilizer_quarter_chord_port, axis_vector=horizontal_stabilizer_actuation_axis)
horizontal_stabilizer_actuator_solver.set_rotation(name='qst_10_tail_actuation', value=0, units='radians')
qst_10_configuration.actuate(transformation=horizontal_stabilizer_actuator_solver)

wing_actuator_solver = PrescribedRotation(component=wing, axis_origin=wing_quarter_chord_port, axis_vector=wing_actuation_axis)
wing_actuator_solver.set_rotation(name='qst_10_wing_actuation', value=np.deg2rad(4) , units='radians')
qst_10_configuration.actuate(transformation=wing_actuator_solver)
# endregion
# endregion

lpc_param.add_geometry_parameterization(ffd_set)
lpc_param.setup()


lpc_rep.add_output(name=wing_vlm_mesh_name, quantity=wing_camber_surface)
# lpc_rep.add_output(name=f"{wing_vlm_mesh_name}_cruise", quantity=wing_camber_surface)

lpc_rep.add_output(name=htail_vlm_mesh_name, quantity=htail_camber_surface)
lpc_rep.add_output(htail_oml_mesh_name_ml, tail_oml_mesh_ml)

# lpc_rep.add_output(name=f"{htail_vlm_mesh_name}_cruise", quantity=htail_camber_surface)

lpc_rep.add_output(name=f"{wing.parameters['name']}_oml_mesh", quantity=wing_oml_mesh)
lpc_rep.add_output(wing_oml_mesh_name_ml, wing_oml_mesh_ml)

# lpc_rep.add_output(name=f"{htail.parameters['name']}_oml_mesh", quantity=htail_camber_surface)

lpc_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
lpc_rep.add_output(name='wing_beam_width', quantity=width)
lpc_rep.add_output(name='wing_beam_height', quantity=height)

lpc_rep.add_output(name=f"{pp_disk.parameters['name']}_in_plane_1", quantity=pp_disk_in_plane_y)
lpc_rep.add_output(name=f"{pp_disk.parameters['name']}_in_plane_2", quantity=pp_disk_in_plane_x)
lpc_rep.add_output(name=f"{pp_disk.parameters['name']}_origin", quantity=pp_disk_origin)
lpc_rep.add_output(name="pp_chord_length", quantity=pp_chord_length)
lpc_rep.add_output(name='pp_twist', quantity=pp_tot_v_dist)

lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_in_plane_1", quantity=rlo_in_plane_y)
lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_in_plane_2", quantity=rlo_in_plane_x)
lpc_rep.add_output(name=f"{rlo_disk.parameters['name']}_origin", quantity=rlo_origin)
lpc_rep.add_output(name="rlo_chord_length", quantity=rlo_chord_length)
lpc_rep.add_output(name='rlo_twist', quantity=rlo_tot_v_dist)

lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_in_plane_1", quantity=rli_in_plane_y)
lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_in_plane_2", quantity=rli_in_plane_x)
lpc_rep.add_output(name=f"{rli_disk.parameters['name']}_origin", quantity=rli_origin)
lpc_rep.add_output(name="rli_chord_length", quantity=rli_chord_length)
lpc_rep.add_output(name='rli_twist', quantity=rli_tot_v_dist)

lpc_rep.add_output(name=f"{rri_disk.parameters['name']}_in_plane_1", quantity=rri_in_plane_y)
lpc_rep.add_output(name=f"{rri_disk.parameters['name']}_in_plane_2", quantity=rri_in_plane_x)
lpc_rep.add_output(name=f"{rri_disk.parameters['name']}_origin", quantity=rri_origin)
lpc_rep.add_output(name="rri_chord_length", quantity=rri_chord_length)
lpc_rep.add_output(name='rri_twist', quantity=rri_tot_v_dist)

lpc_rep.add_output(name=f"{rro_disk.parameters['name']}_in_plane_1", quantity=rro_in_plane_y)
lpc_rep.add_output(name=f"{rro_disk.parameters['name']}_in_plane_2", quantity=rro_in_plane_x)
lpc_rep.add_output(name=f"{rro_disk.parameters['name']}_origin", quantity=rro_origin)
lpc_rep.add_output(name="rro_chord_length", quantity=rro_chord_length)
lpc_rep.add_output(name='rro_twist', quantity=rro_tot_v_dist)

lpc_rep.add_output(name=f"{flo_disk.parameters['name']}_in_plane_1", quantity=flo_in_plane_y)
lpc_rep.add_output(name=f"{flo_disk.parameters['name']}_in_plane_2", quantity=flo_in_plane_x)
lpc_rep.add_output(name=f"{flo_disk.parameters['name']}_origin", quantity=flo_origin)
lpc_rep.add_output(name="flo_chord_length", quantity=flo_chord_length)
lpc_rep.add_output(name='flo_twist', quantity=flo_tot_v_dist)

lpc_rep.add_output(name=f"{fli_disk.parameters['name']}_in_plane_1", quantity=fli_in_plane_y)
lpc_rep.add_output(name=f"{fli_disk.parameters['name']}_in_plane_2", quantity=fli_in_plane_x)
lpc_rep.add_output(name=f"{fli_disk.parameters['name']}_origin", quantity=fli_origin)
lpc_rep.add_output(name="fli_chord_length", quantity=fli_chord_length)
lpc_rep.add_output(name='fli_twist', quantity=fli_tot_v_dist)

lpc_rep.add_output(name=f"{fri_disk.parameters['name']}_in_plane_1", quantity=fri_in_plane_y)
lpc_rep.add_output(name=f"{fri_disk.parameters['name']}_in_plane_2", quantity=fri_in_plane_x)
lpc_rep.add_output(name=f"{fri_disk.parameters['name']}_origin", quantity=fri_origin)
lpc_rep.add_output(name="fri_chord_length", quantity=fri_chord_length)
lpc_rep.add_output(name='fri_twist', quantity=fri_tot_v_dist)

lpc_rep.add_output(name=f"{fro_disk.parameters['name']}_in_plane_1", quantity=fro_in_plane_y)
lpc_rep.add_output(name=f"{fro_disk.parameters['name']}_in_plane_2", quantity=fro_in_plane_x)
lpc_rep.add_output(name=f"{fro_disk.parameters['name']}_origin", quantity=fro_origin)
lpc_rep.add_output(name="fro_chord_length", quantity=fro_chord_length)
lpc_rep.add_output(name='fro_twist', quantity=fro_tot_v_dist)


