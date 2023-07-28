import csdl
import caddee.api as cd
from caddee import GEOMETRY_FILES_FOLDER
from python_csdl_backend import Simulator

# Geometry
import array_mapper as am
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component

import numpy as np

debug_geom_flag = False


caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

# region Geometry
file_name = 'tbw.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name= GEOMETRY_FILES_FOLDER/ file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)


# region Lifting Surfaces
# wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if debug_geom_flag:
    wing.plot()
sys_rep.add_component(wing)

# Horizontal tail
tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Htail']).keys())
htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
if debug_geom_flag:
    htail.plot()
sys_rep.add_component(htail)

# Strut
strut_primitive_names = list(spatial_rep.get_primitives(search_names=['Strut']).keys())
strut = cd.LiftingSurface(name='strut', spatial_representation=spatial_rep, primitive_names=strut_primitive_names)
if debug_geom_flag:
    strut.plot()
sys_rep.add_component(strut)

# Jury
jury_primitive_names = list(spatial_rep.get_primitives(search_names=['Jury']).keys())
jury = cd.LiftingSurface(name='jury', spatial_representation=spatial_rep, primitive_names=jury_primitive_names)
if debug_geom_flag:
    jury.plot()
sys_rep.add_component(jury)

# region FFD surfaces

# region Wing
surfaces = []
for i in np.arange(28, 38):
    surfaces.append(f'Wing, 0, {i}')
right_inner_wing_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
right_inner_wing = LiftingSurface(name='right_inner', spatial_representation=spatial_rep, primitive_names=right_inner_wing_primitive_names)
if debug_geom_flag:
    right_inner_wing.plot()

surfaces = []
for i in np.arange(38, 45):
    surfaces.append(f'Wing, 0, {i}')
right_outer_wing_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
right_outer_wing = LiftingSurface(name='right_outer', spatial_representation=spatial_rep, primitive_names=right_outer_wing_primitive_names)
if debug_geom_flag:
    right_outer_wing.plot()

surfaces = []
for i in np.arange(46, 54):
    surfaces.append(f'Wing, 1, {i}')
left_inner_wing_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
left_inner_wing = LiftingSurface(name='left_inner', spatial_representation=spatial_rep, primitive_names=left_inner_wing_primitive_names)
if debug_geom_flag:
    left_inner_wing.plot()

surfaces = []
for i in np.arange(54, 60):
    surfaces.append(f'Wing, 1, {i}')
left_outer_wing_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
left_outer_wing = LiftingSurface(name='left_outer', spatial_representation=spatial_rep, primitive_names=left_outer_wing_primitive_names)
if debug_geom_flag:
    left_outer_wing.plot()
# endregion

# region Jury
surfaces = []
surfaces.append(f'Jury, 0')
jury_left_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
jury_left = LiftingSurface(name='jury_left', spatial_representation=spatial_rep, primitive_names=jury_left_primitive_names)
if debug_geom_flag:
    jury_left.plot()

surfaces = []
surfaces.append(f'Jury, 1')
jury_right_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
jury_right = LiftingSurface(name='jury_right', spatial_representation=spatial_rep, primitive_names=jury_right_primitive_names)
if debug_geom_flag:
    jury_right.plot()
# endregion

# region Strut
surface = []
surface.append('Strut, 0')
strut_right_primitive_names = list(spatial_rep.get_primitives(search_names=surface).keys())
strut_right = LiftingSurface(name='strut_right', spatial_representation=spatial_rep,
                       primitive_names=strut_right_primitive_names)
if debug_geom_flag:
    strut.plot()

surface = []
surface.append('Strut, 1')
strut_left_primitive_names = list(spatial_rep.get_primitives(search_names=surface).keys())
strut_left = LiftingSurface(name='strut_left', spatial_representation=spatial_rep,
                       primitive_names=strut_left_primitive_names)
if debug_geom_flag:
    strut_left.plot()
# endregion

# endregion

# endregion

# endregion

# region FFD

# region FFD box for right outer wing
right_outer_wing_geometry_primitives = right_outer_wing.get_geometry_primitives()
right_outer_wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    right_outer_wing_geometry_primitives,
    num_control_points=(11, 2, 2), order=(4, 2, 2),
    xyz_to_uvw_indices=(1, 0, 2)
)
right_outer_wing_ffd_block = cd.SRBGFFDBlock(name='right_outer_wing_ffd_block',
                                  primitive=right_outer_wing_ffd_bspline_volume,
                                  embedded_entities=right_outer_wing_geometry_primitives)
right_outer_wing_ffd_block.add_translation_u(name='right_outer_wing_linear_span',
                            order=2, num_dof=2,
                            cost_factor=1.)
right_outer_wing_ffd_block.add_scale_v(name='right_outer_wing_linear_taper',
                            order=2, num_dof=2,
                            cost_factor=1.)
right_outer_wing_ffd_block.add_translation_v(name='right_outer_wing_linear_sweep',
                            order=2, num_dof=2,
                            cost_factor=1.)  #todo: Create an input; make DV; connect to this parameter
right_outer_wing_ffd_block.add_rotation_u(name='right_outer_wing_twist_distribution',
                               connection_name='right_outer_wing_twist', order=2, value=np.zeros(5),
                               num_dof=5)  #todo: Create an input; make DV; connect to this parameter
# endregion

# region FFD box for inner wing
right_inner_wing_geometry_primitives = right_inner_wing.get_geometry_primitives()
left_inner_wing_geometry_primitives = left_inner_wing.get_geometry_primitives()
inner_wing_geometry_primitives = {**left_inner_wing_geometry_primitives, **right_inner_wing_geometry_primitives}
inner_wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    list(inner_wing_geometry_primitives.values()),
    num_control_points=(11, 2, 2), order=(4, 2, 2),
    xyz_to_uvw_indices=(1, 0, 2)
)
inner_wing_ffd_block = cd.SRBGFFDBlock(name='inner_wing_ffd_block',
                                  primitive=inner_wing_ffd_bspline_volume,
                                  embedded_entities=inner_wing_geometry_primitives)
inner_wing_ffd_block.add_translation_u(name='inner_wing_linear_span',
                            order=2, num_dof=2,
                            cost_factor=1.)
inner_wing_ffd_block.add_scale_v(name='inner_wing_linear_taper',
                            order=2, num_dof=3,
                            cost_factor=1.)
inner_wing_ffd_block.add_translation_v(name='inner_wing_linear_sweep',
                            order=2, num_dof=3,
                            cost_factor=1.)  #todo: Create an input; make DV; connect to this parameter
inner_wing_ffd_block.add_rotation_u(name='inner_wing_twist_distribution',
                               connection_name='inner_wing_twist', order=2, value=np.zeros(5),
                               num_dof=5)  #todo: Create an input; make DV; connect to this parameter
# endregion

# region FFD box for left outer wing
left_outer_wing_geometry_primitives = left_outer_wing.get_geometry_primitives()
left_outer_wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    left_outer_wing_geometry_primitives,
    num_control_points=(11, 2, 2), order=(4, 2, 2),
    xyz_to_uvw_indices=(1, 0, 2)
)
left_outer_wing_ffd_block = cd.SRBGFFDBlock(name='left_outer_wing_ffd_block',
                                  primitive=left_outer_wing_ffd_bspline_volume,
                                  embedded_entities=left_outer_wing_geometry_primitives)
left_outer_wing_ffd_block.add_translation_u(name='left_outer_wing_linear_span',
                            order=2, num_dof=2,
                            cost_factor=1.)
left_outer_wing_ffd_block.add_scale_v(name='left_outer_wing_linear_taper',
                            order=2, num_dof=2,
                            cost_factor=1.)
left_outer_wing_ffd_block.add_translation_v(name='left_outer_wing_linear_sweep',
                            order=2, num_dof=2,
                            cost_factor=1.)  #todo: Create an input; make DV; connect to this parameter
left_outer_wing_ffd_block.add_rotation_u(name='left_outer_wing_twist_distribution',
                               connection_name='left_outer_wing_twist', order=2,
                                         value=np.zeros(5),
                               num_dof=5)  #todo: Create an input; make DV; connect to this parameter
# endregion

ffd_blocks = {right_outer_wing_ffd_block.name : right_outer_wing_ffd_block,
              inner_wing_ffd_block.name : inner_wing_ffd_block,
              left_outer_wing_ffd_block.name: left_outer_wing_ffd_block}
ffd_set = cd.SRBGFFDSet(name='ffd_set', ffd_blocks=ffd_blocks)

sys_param.add_geometry_parameterization(ffd_set)


# endregion

# region Inner optimization
point10 = np.array([47.231,    0.000, 6.937 + 0.1]) # * ft2m # Center Leading Edge
point11 = np.array([57.953,   0.000, 6.574 + 0.1]) # * ft2m # Center Trailing edge
center_leading_edge = right_inner_wing.project(point10)
center_trailing_edge = right_inner_wing.project(point11)
root_chord = am.norm(center_leading_edge - center_trailing_edge)

sys_param.add_input(name='wing_root_chord', quantity=root_chord, value=np.array([30.]))

sys_param.setup()
# endregion


system_representation_model = sys_rep.assemble_csdl()
system_parameterization_model = sys_param.assemble_csdl()

my_model = csdl.Model()
my_model.add(system_parameterization_model, 'system_parameterization')
my_model.add(system_representation_model, 'system_representation')

sim = Simulator(my_model, analytics=True, display_scripts=True)
sim.run()


# caddee_csdl_model = caddee.assemble_csdl()
#
# # Create and run simulator
# sim = Simulator(caddee_csdl_model, analytics=True)
# sim.run()