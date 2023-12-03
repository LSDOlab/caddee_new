'''Example 2 : Description of example 2'''
import caddee.api as cd
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am
from aframe.core.beam_module import EBBeam, LinearBeamMesh
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

file_name = 'lift_plus_cruise_final.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

# wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing_1']).keys())
wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
# wing.plot()

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

# Adding components
sys_rep.add_component(wing)
sys_rep.add_component(htail)
sys_rep.add_component(pp_disk)
sys_rep.add_component(pp_blade_1)
sys_rep.add_component(pp_blade_2)
sys_rep.add_component(pp_blade_3)
sys_rep.add_component(pp_blade_4)


# Wing FFD
wing_geometry_primitives = wing.get_geometry_primitives()
wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
wing_ffd_block = cd.SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
wing_ffd_block.add_scale_v(name='wing_linear_taper', order=2, num_dof=3, value=np.array([0., 0., 0.]), cost_factor=1.)
wing_ffd_block.add_rotation_u(name='wing_twist_distribution', connection_name='wing_incidence', order=1, num_dof=1, value=np.array([np.deg2rad(0)]))

# Tail FFD
htail_geometry_primitives = htail.get_geometry_primitives()
htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(htail_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block', primitive=htail_ffd_bspline_volume, embedded_entities=htail_geometry_primitives)
htail_ffd_block.add_scale_v(name='htail_linear_taper', order=2, num_dof=3, value=np.array([0., 0., 0.]), cost_factor=1.)
htail_ffd_block.add_rotation_u(name='htail_twist_distribution', connection_name='h_tail_act', order=1, num_dof=1, value=np.array([np.deg2rad(1.75)]))
# NOTE: line above is performaing actuation- change when actuations are ready

ffd_set = cd.SRBGFFDSet(
    name='ffd_set', 
    ffd_blocks={
        htail_ffd_block.name : htail_ffd_block,
        wing_ffd_block.name : wing_ffd_block
}
)

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

# 
sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()

# wing mesh
num_wing_vlm = 25
num_chordwise_vlm = 2
point00 = np.array([12.356, 25.250, 7.618 + 0.1]) # * ft2m # Right tip leading edge
point01 = np.array([13.400, 25.250, 7.617 + 0.1]) # * ft2m # Right tip trailing edge
point10 = np.array([8.892,    0.000, 8.633 + 0.1]) # * ft2m # Center Leading Edge
point11 = np.array([14.332,   0.000, 8.439 + 0.1]) # * ft2m # Center Trailing edge
point20 = np.array([12.356, -25.250, 7.618 + 0.1]) # * ft2m # Left tip leading edge
point21 = np.array([13.400, -25.250, 7.617 + 0.1]) # * ft2m # Left tip trailing edge

do_plots=False

leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_vlm/2+1))), axis=0)
trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_vlm/2+1))), axis=0)

leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)

chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
# spatial_rep.plot_meshes([chord_surface])
wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
wing_vlm_mesh_name = 'wing_vlm_mesh'
sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
wing_oml_mesh_name = 'wing_oml_mesh'
sys_rep.add_output(wing_oml_mesh_name, oml_mesh)
# spatial_rep.plot_meshes([wing_camber_surface])

# region tail mesh
plot_tail_mesh = False
num_spanwise_vlm = 15
num_chordwise_vlm = 2
leading_edge = htail.project(np.linspace(np.array([27., -6.75, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)  # returns MappedArray
trailing_edge = htail.project(np.linspace(np.array([31.5, -6.75, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)   # returns MappedArray
tail_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
htail_upper_surface_wireframe = htail.project(tail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
htail_lower_surface_wireframe = htail.project(tail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)
htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
htail_vlm_mesh_name = 'htail_vlm_mesh'
sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
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
sys_rep.add_output(f"{pp_disk.parameters['name']}_in_plane_1", pp_disk_in_plane_y)
sys_rep.add_output(f"{pp_disk.parameters['name']}_in_plane_2", pp_disk_in_plane_x)
sys_rep.add_output(f"{pp_disk.parameters['name']}_origin", pp_disk_origin)


# chord 
num_radial = 25
# num_lifting_line = 10
off_set = 1
off_set_long_le = 0.1
off_set_long_te_root = 0.35
off_set_long_te_tip = 0.25

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

sys_rep.add_output(name="pp_chord_length", quantity=pp_chord_length)
sys_rep.add_output(name='pp_twist', quantity=pp_tot_v_dist)
# endregion



# wing beam mesh
num_wing_beam = 11

leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_beam/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_beam/2+1))), axis=0)
trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_beam/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_beam/2+1))), axis=0)

leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)
wing_beam = am.linear_combination(leading_edge,trailing_edge,1,start_weights=np.ones((num_wing_beam,))*0.75,stop_weights=np.ones((num_wing_beam,))*0.25)
width = am.norm((leading_edge - trailing_edge)*0.5)
# width = am.subtract(leading_edge, trailing_edge)

if do_plots:
    spatial_rep.plot_meshes([wing_beam])

wing_beam = wing_beam.reshape((11,3))#*0.304

offset = np.array([0,0,0.5])
top = wing.project(wing_beam.value+offset, direction=np.array([0., 0., -1.]), plot=do_plots)
bot = wing.project(wing_beam.value-offset, direction=np.array([0., 0., 1.]), plot=do_plots)
height = am.norm((top - bot)*1)
# print('HEIGHT IN FEET', height.evaluate())
# height = am.subtract(top, bot)
# 
sys_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
sys_rep.add_output(name='wing_beam_width', quantity=width)
sys_rep.add_output(name='wing_beam_height', quantity=height)

# pass the beam meshes to aframe:
beam_mesh = LinearBeamMesh(
meshes = dict(
wing_beam = wing_beam,
wing_beam_width = width,
wing_beam_height = height,
)
)

# m3l sizing model
# sizing_model = m3l.Model()

# Battery sizing
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

simple_battery_sizing.set_module_input('battery_mass', val=800)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.8, 0, 0.5]), dv_flag=False, lower=np.array([3.5, -1e-4, 0.5 - 1e-4]), upper=np.array([4, +1e-4, 0.5 + 1e-4]), scaler=1e-1)
simple_battery_sizing.set_module_input('battery_energy_density', val=400)

battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()

# M4 regressions
m4_regression = cd.M4RegressionsM3L(exclude_wing=True)

mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# region cruise condition
cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.2, dv_flag=False, lower=0.17, upper=0.19)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='wind_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(ac_states)

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
    cl0=[0.4, 0]
)

# aero forces and moments
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states)
cruise_model.register_output(vlm_force)
cruise_model.register_output(vlm_moment)

vlm_force_mapping_model = VASTNodalForces(
    surface_names=[
        wing_vlm_mesh_name,
        htail_vlm_mesh_name,
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    initial_meshes=[
        wing_camber_surface,
        htail_camber_surface]
)

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[oml_mesh, oml_mesh])
wing_forces = oml_forces[0]
htail_forces = oml_forces[1]

# BEM prop forces and moments
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
pusher_bem_mesh = BEMMesh(
    meshes=dict(
    pp_disk_in_plane_1=pp_disk_in_plane_y,
    pp_disk_in_plane_2=pp_disk_in_plane_x,
    pp_disk_origin=pp_disk_origin,
    ),
    airfoil='NACA_4412',
    num_blades=4,
    num_radial=25,
    mesh_units='ft',
    use_airfoil_ml=False,

)

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=2000, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=ac_states)

# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
beams['wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'cs': 'box','nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1,1,1,1,1,1]}

# create the beam model:
beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints, mesh_units='ft')
beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
cruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(3,1), control_points_shape=((35,1)))
dummy_function_space = lg.BSplineSetSpace(name='dummy_space', spaces={'dummy_b_spline_space': dummy_b_spline_space})

cruise_wing_displacement_coefficients = m3l.Variable(name='cruise_wing_displacement_coefficients', shape=(35,3))
cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', space=dummy_function_space, coefficients=cruise_wing_displacement_coefficients)


beam_force_map_model = ebbeam.EBBeamForces(component=wing, beam_mesh=beam_mesh, beams=beams)
cruise_structural_wing_mesh_forces = beam_force_map_model.evaluate(nodal_forces=wing_forces,
                                                                   nodal_forces_mesh=oml_mesh)

beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
    forces=cruise_structural_wing_mesh_forces)

cruise_model.register_output(cruise_structural_wing_mesh_displacements)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(wing_mass, battery_mass, mass_m4, wing_cg, cg_battery, cg_m4, wing_inertia_tensor, I_battery, I_m4)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

cruise_model.register_output(total_mass)
cruise_model.register_output(total_cg)
cruise_model.register_output(total_inertia)

# inertial forces and moments
inertial_loads_model = cd.InertialLoads(load_factor=-1)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMoments()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states
)

cruise_model.register_output(trim_residual)

# Add cruise m3l model to cruise condition
cruise_condition.add_m3l_model('cruise_model', cruise_model)

# Add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)
# endregion
system_model.add_design_scenario(design_scenario=design_scenario)

caddee_csdl_model = caddee.assemble_csdl()

h_tail_act = caddee_csdl_model.create_input('h_tail_act', val=np.deg2rad(0))
caddee_csdl_model.add_design_variable('h_tail_act', 
                                lower=np.deg2rad(-25),
                                upper=np.deg2rad(25),
                                scaler=1,
                            )

wing_incidence = caddee_csdl_model.create_input('wing_incidence', val=np.deg2rad(4))
# caddee_csdl_model.add_design_variable('wing_incidence', 
#                                 lower=np.deg2rad(-5),
#                                 upper=np.deg2rad(6),
#                                 scaler=1,
#                             )

caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.new_stress',upper=500E6/1,scaler=1E-8)
caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual', equals=0.)
caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.total_constant_mass_properties.total_mass', scaler=1e-3)

# caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.vm_stress'])
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.wing_beam_forces'])

# 

# sim.compute_total_derivatives()
# sim.check_totals()


prob = CSDLProblem(problem_name='lpc', simulator=sim)
optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
optimizer.solve()
optimizer.print_results()

print("I'm done.")


# import caddee.api as cd
# from python_csdl_backend import Simulator
# import numpy as np
# import array_mapper as am
# from aframe.core.beam_module import EBBeam, LinearBeamMesh
# from VAST.core.vast_solver import VASTFluidSover
# from VAST.core.fluid_problem import FluidProblem
# from VAST.core.generate_mappings_m3l import VASTNodalForces
# # from modopt.snopt_library import SNOPT
# from modopt.scipy_library import SLSQP
# from modopt.csdl_library import CSDLProblem
# import csdl
# from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
# from caddee import GEOMETRY_FILES_FOLDER
# import m3l
# import lsdo_geo as lg
# import aframe.core.beam_module as ebbeam

# caddee = cd.CADDEE()
# caddee.system_model = system_model = cd.SystemModel()
# caddee.system_representation = sys_rep = cd.SystemRepresentation()
# caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

# file_name = 'lift_plus_cruise_final.stp'

# spatial_rep = sys_rep.spatial_representation
# spatial_rep.import_file(file_name=file_name)
# spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

# # wing
# wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing_1']).keys())
# wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
# # wing.plot()

# # Horizontal tail
# tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
# htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)

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

# # Adding components
# sys_rep.add_component(wing)
# sys_rep.add_component(htail)
# sys_rep.add_component(pp_disk)
# sys_rep.add_component(pp_blade_1)
# sys_rep.add_component(pp_blade_2)
# sys_rep.add_component(pp_blade_3)
# sys_rep.add_component(pp_blade_4)

# # Tail FFD
# htail_geometry_primitives = htail.get_geometry_primitives()
# htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(htail_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
# htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block', primitive=htail_ffd_bspline_volume, embedded_entities=htail_geometry_primitives)
# htail_ffd_block.add_scale_v(name='htail_linear_taper', order=2, num_dof=3, value=np.array([0., 0., 0.]), cost_factor=1.)
# htail_ffd_block.add_rotation_u(name='htail_twist_distribution', connection_name='h_tail_act', order=1, num_dof=1, value=np.array([np.deg2rad(1.75)]))
# # NOTE: line above is performaing actuation- change when actuations are ready

# ffd_set = cd.SRBGFFDSet(
#     name='ffd_set', 
#     ffd_blocks={htail_ffd_block.name : htail_ffd_block}
# )


# sys_param.add_geometry_parameterization(ffd_set)
# sys_param.setup()

# # wing mesh
# num_wing_vlm = 11
# num_chordwise_vlm = 5
# point00 = np.array([12.356, 25.250, 7.618 + 0.1]) # * ft2m # Right tip leading edge
# point01 = np.array([13.400, 25.250, 7.617 + 0.1]) # * ft2m # Right tip trailing edge
# point10 = np.array([8.892,    0.000, 8.633 + 0.1]) # * ft2m # Center Leading Edge
# point11 = np.array([14.332,   0.000, 8.439 + 0.1]) # * ft2m # Center Trailing edge
# point20 = np.array([12.356, -25.250, 7.618 + 0.1]) # * ft2m # Left tip leading edge
# point21 = np.array([13.400, -25.250, 7.617 + 0.1]) # * ft2m # Left tip trailing edge

# do_plots=False

# leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_vlm/2+1))), axis=0)
# trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_vlm/2+1))), axis=0)

# leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
# trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)

# chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
# # spatial_rep.plot_meshes([chord_surface])
# wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
# wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
# wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
# wing_vlm_mesh_name = 'wing_vlm_mesh'
# sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
# oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
# wing_oml_mesh_name = 'wing_oml_mesh'
# sys_rep.add_output(wing_oml_mesh_name, oml_mesh)
# # spatial_rep.plot_meshes([wing_camber_surface])

# # region tail mesh
# plot_tail_mesh = False
# num_spanwise_vlm = 10
# num_chordwise_vlm = 2
# leading_edge = htail.project(np.linspace(np.array([27., -6.75, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)  # returns MappedArray
# trailing_edge = htail.project(np.linspace(np.array([31.5, -6.75, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)   # returns MappedArray
# tail_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
# htail_upper_surface_wireframe = htail.project(tail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
# htail_lower_surface_wireframe = htail.project(tail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)
# htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
# htail_vlm_mesh_name = 'htail_vlm_mesh'
# sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
# # endregion

# # region pusher prop (pp) meshes
# # disk
# y11 = pp_disk.project(np.array([31.94, 0.00, 3.29]), direction=np.array([-1., 0., 0.]), plot=False)
# y12 = pp_disk.project(np.array([31.94, 0.00, 12.29]), direction=np.array([-1., 0., 0.]), plot=False)
# y21 = pp_disk.project(np.array([31.94, -4.50, 7.78]), direction=np.array([-1., 0., 0.]), plot=False)
# y22 = pp_disk.project(np.array([31.94, 4.45, 7.77]), direction=np.array([-1., 0., 0.]), plot=False)
# pp_disk_in_plane_y = am.subtract(y11, y12)
# pp_disk_in_plane_x = am.subtract(y21, y22)
# pp_disk_origin = pp_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))
# sys_rep.add_output(f"{pp_disk.parameters['name']}_in_plane_1", pp_disk_in_plane_y)
# sys_rep.add_output(f"{pp_disk.parameters['name']}_in_plane_2", pp_disk_in_plane_x)
# sys_rep.add_output(f"{pp_disk.parameters['name']}_origin", pp_disk_origin)


# # chord 
# num_radial = 25
# # num_lifting_line = 10
# off_set = 1
# off_set_long_le = 0.1
# off_set_long_te_root = 0.35
# off_set_long_te_tip = 0.25

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

# sys_rep.add_output(name="pp_chord_length", quantity=pp_chord_length)
# sys_rep.add_output(name='pp_twist', quantity=pp_tot_v_dist)
# # endregion



# # wing beam mesh
# num_wing_beam = 11
# leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=False, grid_search_n=50)
# trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=False, grid_search_n=50)
# wing_beam = am.linear_combination(leading_edge,trailing_edge,1,start_weights=np.ones((num_wing_beam,))*0.75,stop_weights=np.ones((num_wing_beam,))*0.25) #* 0.3048
# width = am.norm((leading_edge - trailing_edge)*0.4* 0.3048 )
# # width = am.subtract(leading_edge, trailing_edge)

# if True:
#     spatial_rep.plot_meshes([wing_beam])

# wing_beam = wing_beam.reshape((11,3)) 

# offset = np.array([0,0,0.5])
# top = wing.project(wing_beam.value+offset, direction=np.array([0., 0., -1.]), plot=True, grid_search_n=25)
# bot = wing.project(wing_beam.value-offset, direction=np.array([0., 0., 1.]), plot=True, grid_search_n=25)
# # height = am.norm((top - bot)*1* 0.3048+ 0.1)
# height = am.norm((top - bot)* 0.3048 - 0.05)
# # height = am.subtract(top, bot)
# # print(height.evaluate())
# # 

# sys_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
# sys_rep.add_output(name='wing_beam_width', quantity=width)
# sys_rep.add_output(name='wing_beam_height', quantity=height)

# # pass the beam meshes to aframe:
# beam_mesh = LinearBeamMesh(
# meshes = dict(
# wing_beam = wing_beam * 0.3048,
# wing_beam_width = width,
# wing_beam_height = height,
# )
# )

# # m3l sizing model
# # sizing_model = m3l.Model()

# # Battery sizing
# battery_component = cd.Component(name='battery')
# simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

# simple_battery_sizing.set_module_input('battery_mass', val=800)
# simple_battery_sizing.set_module_input('battery_position', val=np.array([3.5, 0, 0.5]))
# simple_battery_sizing.set_module_input('battery_energy_density', val=400)

# battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()

# # M4 regressions
# m4_regression = cd.M4RegressionsM3L()

# mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)

# # design scenario
# design_scenario = cd.DesignScenario(name='aircraft_trim')

# # region cruise condition
# cruise_model = m3l.Model()
# cruise_condition = cd.CruiseCondition(name="cruise_1")
# cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
# cruise_condition.set_module_input(name='altitude', val=1000)
# cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True, lower=0.17, upper=0.19)
# cruise_condition.set_module_input(name='range', val=40000)
# cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(10))
# cruise_condition.set_module_input(name='flight_path_angle', val=0)
# cruise_condition.set_module_input(name='roll_angle', val=0)
# cruise_condition.set_module_input(name='yaw_angle', val=0)
# cruise_condition.set_module_input(name='wind_angle', val=0)
# cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

# ac_states = cruise_condition.evaluate_ac_states()
# cruise_model.register_output(ac_states)

# vlm_model = VASTFluidSover(
#     surface_names=[
#         wing_vlm_mesh_name,
#         htail_vlm_mesh_name,
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
#     mesh_unit='ft',
#     cl0=[0.43, 0]
# )

# # aero forces and moments
# vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states)
# cruise_model.register_output(vlm_force)
# cruise_model.register_output(vlm_moment)

# vlm_force_mapping_model = VASTNodalForces(
#     surface_names=[
#         wing_vlm_mesh_name,
#         htail_vlm_mesh_name,
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     initial_meshes=[
#         wing_camber_surface,
#         htail_camber_surface]
# )

# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[oml_mesh, oml_mesh])
# wing_forces = oml_forces[0]
# htail_forces = oml_forces[1]

# # BEM prop forces and moments
# from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
# pusher_bem_mesh = BEMMesh(
#     meshes=dict(
#     pp_disk_in_plane_1=pp_disk_in_plane_y,
#     pp_disk_in_plane_2=pp_disk_in_plane_x,
#     pp_disk_origin=pp_disk_origin,
#     ),
#     airfoil='NACA_4412',
#     num_blades=4,
#     num_radial=25,
# )

# bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
# bem_forces, bem_moments = bem_model.evaluate(ac_states=ac_states)

# # create the aframe dictionaries:
# joints, bounds, beams = {}, {}, {}
# beams['wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'cs': 'box','nodes': list(range(num_wing_beam))}
# bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1,1,1,1,1,1]}

# # create the beam model:
# beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

# cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
# cruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
# cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
# cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

# dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(3,1), control_points_shape=((35,1)))
# dummy_function_space = lg.BSplineSetSpace(name='dummy_space', spaces={'dummy_b_spline_space': dummy_b_spline_space})

# cruise_wing_displacement_coefficients = m3l.Variable(name='cruise_wing_displacement_coefficients', shape=(35,3))
# cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', space=dummy_function_space, coefficients=cruise_wing_displacement_coefficients)


# beam_force_map_model = ebbeam.EBBeamForces(component=wing, beam_mesh=beam_mesh, beams=beams)
# cruise_structural_wing_mesh_forces = beam_force_map_model.evaluate(nodal_forces=wing_forces,
#                                                                    nodal_forces_mesh=oml_mesh)

# beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.0001, upper=0.04, scaler=1E3)
# beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.0001, upper=0.04, scaler=1E3)

# cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
#     forces=cruise_structural_wing_mesh_forces)

# cruise_model.register_output(cruise_structural_wing_mesh_displacements)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(wing_mass, battery_mass, wing_cg, cg_battery,  wing_inertia_tensor, I_battery)
# # total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

# cruise_model.register_output(total_mass)
# cruise_model.register_output(total_cg)
# cruise_model.register_output(total_inertia)

# # inertial forces and moments
# inertial_loads_model = cd.InertialLoads(load_factor=1.)
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
# cruise_model.register_output(inertial_forces)
# cruise_model.register_output(inertial_moments)

# # total forces and moments 
# total_forces_moments_model = cd.TotalForcesMoments()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
# cruise_model.register_output(total_forces)
# cruise_model.register_output(total_moments)

# # pass total forces/moments + mass properties into EoM model
# eom_m3l_model = cd.EoMEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=ac_states
# )

# cruise_model.register_output(trim_residual)

# # Add cruise m3l model to cruise condition
# cruise_condition.add_m3l_model('cruise_model', cruise_model)

# # Add design condition to design scenario
# design_scenario.add_design_condition(cruise_condition)
# # endregion
# system_model.add_design_scenario(design_scenario=design_scenario)

# caddee_csdl_model = caddee.assemble_csdl()

# h_tail_act = caddee_csdl_model.create_input('h_tail_act', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('h_tail_act', 
#                                 lower=np.deg2rad(-10),
#                                 upper=np.deg2rad(10),
#                                 scaler=1,
#                             )

# # caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.max_stress',upper=500E6/1,scaler=1E-8)
# # caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.new_stress', upper=500E6, scaler=1E-8)
# # caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual', equals=0.)
# # caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.total_constant_mass_properties.total_mass', scaler=1e-3)

# # caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# # create and run simulator
# sim = Simulator(caddee_csdl_model, analytics=True)
# sim.run()
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.vm_stress'])
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.wing_beam_forces'])

# 
# # sim.compute_total_derivatives()
# sim.check_totals()
# # 

# prob = CSDLProblem(problem_name='lpc', simulator=sim)
# optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
# optimizer.solve()
# optimizer.print_results()

# print("I'm done.")