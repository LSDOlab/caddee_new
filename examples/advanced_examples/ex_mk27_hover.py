import caddee.api as cd
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am
from aframe.core.beam_module import EBBeam, LinearBeamMesh
from aframe.core.mass import Mass, MassMesh
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
from caddee.utils.aircraft_models.mk27.mk27_weight import MK27MassProperties
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from VAST.core.generate_mappings_m3l import VASTNodalForces
# from modopt.snopt_library import SNOPT
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from caddee import GEOMETRY_FILES_FOLDER
import m3l
import lsdo_geo as lg
import aframe.core.beam_module as ebbeam

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

file_name = 'AmazonPrime.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)
# spatial_rep.plot()


# Wing definintions - Mid, Upper, Lower Wings, then Top, Bottom of frame, finally vertical stabilizer
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Horizontal Wing']).keys())
MidWing = LiftingSurface(name='MidWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names) 

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Upper Wing']).keys())
TopWing = LiftingSurface(name='TopWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Wing']).keys())
BotWing = LiftingSurface(name='BotWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Top of Frame']).keys())
TopFrame = LiftingSurface(name='TopFrame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Frame']).keys())
BotFrame = LiftingSurface(name='BotFrame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Vertical Stabilizer']).keys())
VertStab = LiftingSurface(name='VertStab', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

# wing plots - check progress
# MidWing.plot()
# TopWing.plot()
# BotWing.plot()
# TopFrame.plot()
# BotFrame.plot()
# VertStab.plot()



# region Rotors
# Pusher prop
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props, 0', 'Middle Props, 1', 'Middle Props, 2', 'Middle Props, 3']).keys())
ppm_left = cd.Rotor(name='ppm_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppm_left)
# ppm_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props, 4', 'Middle Props, 5', 'Middle Props, 6', 'Middle Props, 7']).keys())
ppm_right = cd.Rotor(name='ppm_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppm_right)
# ppm_right.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props, 4', 'Upper Props, 5', 'Upper Props, 6', 'Upper Props, 7']).keys())
ppu_left = cd.Rotor(name='ppu_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppu_left)
# ppu_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props, 0', 'Upper Props, 1', 'Upper Props, 2', 'Upper Props, 3']).keys())
ppu_right = cd.Rotor(name='ppu_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppu_right)
# ppu_right.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props, 4', 'Lower Props, 5', 'Lower Props, 6', 'Lower Props, 7']).keys())
ppl_left = cd.Rotor(name='ppl_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppl_left)
# ppl_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props, 0', 'Lower Props, 1', 'Lower Props, 2', 'Lower Props, 3']).keys())
ppl_right = cd.Rotor(name='ppl_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppl_right)
# ppl_right.plot()

whole_geometry_component_primitive_names = list(spatial_rep.get_primitives().keys())
whole_geometry_component = cd.Component(name='whole_geometry', spatial_representation=spatial_rep, 
                                        primitive_names=whole_geometry_component_primitive_names)

# endregion
# Rotor plots - check progress
#ppm.plot()
#ppu.plot()
#ppl.plot()


# Adding components
sys_rep.add_component(MidWing)
sys_rep.add_component(TopWing)
sys_rep.add_component(BotWing)
sys_rep.add_component(TopFrame)
sys_rep.add_component(BotFrame)
sys_rep.add_component(VertStab)

do_plots=False

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
ppu_left_ffd_block.add_translation_u(name='ppu_left_translation_u', order=2, num_dof=2, cost_factor=1.5)
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
ppu_right_ffd_block.add_translation_u(name='ppu_right_translation_u', order=2, num_dof=2, cost_factor=1.5)

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
ppl_left_ffd_block.add_translation_u(name='ppl_left_translation_u', order=2, num_dof=2, cost_factor=1.5)


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
ppl_right_ffd_block.add_translation_u(name='ppl_right_translation_u', order=2, num_dof=2, cost_factor=1.5)

# endregion

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

configuration_names = ["hover_configuration", "cruise_configuration"]
system_configurations = sys_rep.declare_configurations(names=configuration_names)
hover_configuration = system_configurations['hover_configuration']
cruise_configuration = system_configurations['cruise_configuration']

actuation_axis_port = whole_geometry_component.project(np.array([3., -1., 0.]), plot=False)
actuation_axis_starboard = whole_geometry_component.project(np.array([3., 1., 0.]), plot=False)
horizontal_stabilizer_actuation_axis = actuation_axis_starboard - actuation_axis_port
from caddee.core.caddee_core.system_representation.prescribed_actuations import PrescribedRotation
hover_actuator_solver = PrescribedRotation(component=whole_geometry_component, axis_origin=actuation_axis_port,
                                                           axis_vector=horizontal_stabilizer_actuation_axis)
hover_actuation_profile = np.pi/3
hover_actuator_solver.set_rotation(name='hover_actuation', value=hover_actuation_profile, units='radians')
hover_configuration.actuate(transformation=hover_actuator_solver)

# TEMPORARY
system_representation_model = sys_rep.assemble_csdl()
system_parameterization_model = sys_param.assemble_csdl()

my_model = csdl.Model()
my_model.add(system_parameterization_model, 'system_parameterization')
my_model.add(system_representation_model, 'system_representation')

sim = Simulator(my_model, analytics=True, display_scripts=True)
sim.run()

hover_geo = sim['hover_configuration_geometry']
spatial_rep.update(hover_geo)
spatial_rep.plot()
# removed blade meshes, twist

# design scenario
design_scenario = cd.DesignScenario(name='mk27')

# region hover condition
hover_model = m3l.Model()
hover_condition = cd.CruiseCondition(name="hover_1")
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
hover_condition.set_module_input(name='altitude', val=0)
hover_condition.set_module_input(name='mach_number', val=0, dv_flag=True, lower=0, upper=0)
hover_condition.set_module_input(name='range', val=0)
hover_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(0))
hover_condition.set_module_input(name='flight_path_angle', val=0)
hover_condition.set_module_input(name='roll_angle', val=0)
hover_condition.set_module_input(name='yaw_angle', val=0)
hover_condition.set_module_input(name='wind_angle', val=0)
hover_condition.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

hover_ac_states = hover_condition.evaluate_ac_states()
hover_model.register_output(hover_ac_states)

mk27_wt = MK27MassProperties()
mass, cg, I = mk27_wt.evaluate()

# BEM prop forces and moments
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
ppm_left_bem_model.set_module_input('rpm', val=4000, dv_flag=True)
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
ppm_left_bem_forces, ppm_left_bem_moments, _, _, _, _, _, _ = ppm_left_bem_model.evaluate(ac_states=hover_ac_states)
hover_model.register_output(ppm_left_bem_forces)
hover_model.register_output(ppm_left_bem_moments)

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
ppm_right_bem_model.set_module_input('rpm', val=4000, dv_flag=True)
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
ppm_right_bem_forces, ppm_right_bem_moments, _, _, _, _, _, _ = ppm_right_bem_model.evaluate(ac_states=hover_ac_states)
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
ppu_left_bem_model.set_module_input('rpm', val=4000, dv_flag=True)
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
ppu_left_bem_forces, ppu_left_bem_moments, _, _, _, _, _, _ = ppu_left_bem_model.evaluate(ac_states=hover_ac_states)
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
ppu_right_bem_model.set_module_input('rpm', val=4000, dv_flag=True)
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
ppu_right_bem_forces, ppu_right_bem_moments, _, _, _, _, _, _ = ppu_right_bem_model.evaluate(ac_states=hover_ac_states)
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
ppl_left_bem_model.set_module_input('rpm', val=4000, dv_flag=True)
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
ppl_left_bem_forces, ppl_left_bem_moments, _, _, _, _, _, _ = ppl_left_bem_model.evaluate(ac_states=hover_ac_states)
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
ppl_right_bem_model.set_module_input('rpm', val=4000, dv_flag=True)
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
ppl_right_bem_forces, ppl_right_bem_moments, _, _, _, _, _, _ = ppl_right_bem_model.evaluate(ac_states=hover_ac_states)
# cruise_model.register_output(ppl_right_bem_forces)
# cruise_model.register_output(ppl_right_bem_moments)
# endregion

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(wing_mass, battery_mass, mass_m4, wing_cg, cg_battery, cg_m4, wing_inertia_tensor, I_battery, I_m4)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

hover_model.register_output(total_mass)
hover_model.register_output(total_cg)
hover_model.register_output(total_inertia)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=hover_ac_states)
hover_model.register_output(inertial_forces)
hover_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    inertial_forces, inertial_moments,
    ppm_left_bem_forces, ppm_left_bem_moments,
    ppm_right_bem_forces, ppm_right_bem_moments,
    ppu_left_bem_forces, ppu_left_bem_moments,
    ppu_right_bem_forces, ppu_right_bem_moments,
    ppl_left_bem_forces, ppl_left_bem_moments,
    ppl_right_bem_forces, ppl_right_bem_moments,
)
hover_model.register_output(total_forces)
hover_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass,
    total_cg_vector=total_cg,
    total_inertia_tensor=total_inertia,
    total_forces=total_forces,
    total_moments=total_moments,
    ac_states=hover_ac_states
)

hover_model.register_output(trim_residual)

# Add cruise m3l model to cruise condition
hover_condition.add_m3l_model('hover_model', hover_model)

# Add design condition to design scenario
design_scenario.add_design_condition(hover_condition)
# endregion
system_model.add_design_scenario(design_scenario=design_scenario)

caddee_csdl_model = caddee.assemble_csdl()

caddee_csdl_model.add_objective('system_model.mk27.hover_1.hover_1.euler_eom_gen_ref_pt.trim_residual')

# caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()


# sim.compute_total_derivatives()
# sim.check_totals()


prob = CSDLProblem(problem_name='mk27', simulator=sim)
optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
optimizer.solve()
optimizer.print_results()

print("I'm done.")
print('trim: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
print('forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
print('moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
print('pitch: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle'])
print('rpm: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm'])