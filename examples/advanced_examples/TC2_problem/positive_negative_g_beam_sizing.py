import caddee.api as cd
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am
from aframe.core.beam_module import EBBeam, LinearBeamMesh
from aframe.core.mass import Mass, MassMesh
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

# region import relevant geometries and parameterizations
from examples.advanced_examples.TC2_problem.ex_tc2_geometry_setup import lpc_rep, lpc_param, \
    wing_camber_surface, htail_camber_surface, wing_vlm_mesh_name, htail_vlm_mesh_name, wing_oml_mesh,\
    wing_beam, width, height, num_wing_beam, wing, wing_upper_surface_wireframe, wing_lower_surface_wireframe,\
    pp_disk_in_plane_x, pp_disk_in_plane_y, pp_disk_origin, pp_disk
    

caddee.system_representation = lpc_rep
caddee.system_parameterization = lpc_param
# endregion

caddee.system_model = system_model = cd.SystemModel()
system_m3l_model = m3l.Model()

# region sizing
# battery
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)
simple_battery_sizing.set_module_input('battery_mass', val=800)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.8, 0, 0.5]), dv_flag=False, lower=np.array([3.5, -1e-4, 0.5 - 1e-4]), upper=np.array([4, +1e-4, 0.5 + 1e-4]), scaler=1e-1)
simple_battery_sizing.set_module_input('battery_energy_density', val=400)
battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()

# M4 regressions
m4_regression = cd.M4RegressionsM3L(exclude_wing=True)
mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)

# beam sizing
# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
beams['wing_beam'] = {'E': 70E9, 'G': 70E9/(2*(1 + 0.33)), 'rho': 2700,'cs': 'box', 'nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1, 1, 1, 1, 1, 1]}

beam_mass_mesh = MassMesh(
    meshes = dict(
        wing_beam = wing_beam,
        wing_beam_width = width,
        wing_beam_height = height,
    )
)
beam_mass = Mass(component=wing, mesh=beam_mass_mesh, beams=beams, mesh_units='ft')
mass_model_wing_mass = beam_mass.evaluate()

# total constant mass 
constant_mps = cd.TotalConstantMassM3L()
total_constant_mass = constant_mps.evaluate(mass_model_wing_mass, battery_mass, mass_m4)

system_m3l_model.register_output(total_constant_mass)
# endregion


# region design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# region back of the envolope calculations
# L = n * W
# q = 0.5 * rho * V_inf^2
# Cl_max = L / (q * S) = n * W / (q * S)
# --> n = Cl_max * q * S / W
# --> q = n * W / (Cl_max * S)
# --> V_inf = sqrt(2 * n * W / (Cl_max * S * rho))
# For L+C
#   - Cl_max ~ 1.7
#   - Cl_min ~ -0.7
#   - W ~ 36300 N
#   - S ~ 19.5 m^2
#
# for n = +3
#     V_inf ~ 77 m/s; M ~ 0.23
# for n = -1
#     V_inf ~ 69 m/s; M ~ 0.21

# Results when minimizing structural wing mass
# for n = +3
#   - wing mass: 226.1 kg 
#   - pitch angle: 10.9 deg
#   - elevator: -14.5 deg
# for n = -1
#   - wing mass: 80.0 kg 
#   - pitch angle: -17.8 deg
#   - elevator: 13.4 deg
# endregion

# region +3g sizing condition
plus_3g_condition = cd.CruiseCondition(name="plus_3g_sizing")
plus_3g_condition.atmosphere_model = cd.SimpleAtmosphereModel()
plus_3g_condition.set_module_input(name='altitude', val=1000)
plus_3g_condition.set_module_input(name='mach_number', val=0.21, dv_flag=False, lower=0.17, upper=0.19)
plus_3g_condition.set_module_input(name='range', val=1)
plus_3g_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
plus_3g_condition.set_module_input(name='flight_path_angle', val=0)
plus_3g_condition.set_module_input(name='roll_angle', val=0)
plus_3g_condition.set_module_input(name='yaw_angle', val=0)
plus_3g_condition.set_module_input(name='wind_angle', val=0)
plus_3g_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = plus_3g_condition.evaluate_ac_states()
system_m3l_model.register_output(ac_states)

vlm_model = VASTFluidSover(
    surface_names=[
        f'{wing_vlm_mesh_name}_plus_3g',
       f'{htail_vlm_mesh_name}_plus_3g',
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
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=plus_3g_condition)
system_m3l_model.register_output(vlm_force, plus_3g_condition)
system_m3l_model.register_output(vlm_moment, plus_3g_condition)

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

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh], design_condition=plus_3g_condition)
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
bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=ac_states, design_condition=plus_3g_condition)

# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
beams['wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'cs': 'box','nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1,1,1,1,1,1]}

# create the beam model:
beam_mesh = LinearBeamMesh(
    meshes = dict(
        wing_beam = wing_beam,
        wing_beam_width = width,
        wing_beam_height = height,
    )
)
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
                                                                   nodal_forces_mesh=wing_oml_mesh,
                                                                   design_condition=plus_3g_condition)

beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
    forces=cruise_structural_wing_mesh_forces,
    design_condition=plus_3g_condition)

system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, plus_3g_condition)

# Total mass properties
total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=plus_3g_condition)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

system_m3l_model.register_output(total_mass, plus_3g_condition)
system_m3l_model.register_output(total_cg, plus_3g_condition)
system_m3l_model.register_output(total_inertia, plus_3g_condition)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L(load_factor=3)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=plus_3g_condition)
system_m3l_model.register_output(inertial_forces, plus_3g_condition)
system_m3l_model.register_output(inertial_moments, plus_3g_condition)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=plus_3g_condition)
system_m3l_model.register_output(total_forces, plus_3g_condition)
system_m3l_model.register_output(total_moments, plus_3g_condition)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states,
    design_condition=plus_3g_condition,
)

system_m3l_model.register_output(trim_residual, plus_3g_condition)

# Add cruise m3l model to cruise condition
# plus_3g_condition.add_m3l_model('plus_3g_sizing_model', plus_3g_model)

# Add design condition to design scenario
design_scenario.add_design_condition(plus_3g_condition)
# endregion

# region -1g condition
# minus_1g_model = m3l.Model()
minus_1g_condition = cd.CruiseCondition(name="minus_1g_sizing")
minus_1g_condition.atmosphere_model = cd.SimpleAtmosphereModel()
minus_1g_condition.set_module_input(name='altitude', val=1000)
minus_1g_condition.set_module_input(name='mach_number', val=0.21)
minus_1g_condition.set_module_input(name='range', val=1)
minus_1g_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
minus_1g_condition.set_module_input(name='flight_path_angle', val=0)
minus_1g_condition.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

ac_states = minus_1g_condition.evaluate_ac_states()
system_m3l_model.register_output(ac_states)

vlm_model = VASTFluidSover(
    surface_names=[
        f'{wing_vlm_mesh_name}_minus_1g',
        f'{htail_vlm_mesh_name}_minus_1g',
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
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=minus_1g_condition)
system_m3l_model.register_output(vlm_force, minus_1g_condition)
system_m3l_model.register_output(vlm_moment, minus_1g_condition)

vlm_force_mapping_model = VASTNodalForces(
    surface_names=[
        f'{wing_vlm_mesh_name}',
        f'{htail_vlm_mesh_name}',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    initial_meshes=[
        wing_camber_surface,
        htail_camber_surface]
)

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh], design_condition=minus_1g_condition)
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
bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=ac_states, design_condition=minus_1g_condition)

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
                                                                   nodal_forces_mesh=wing_oml_mesh,
                                                                   design_condition=minus_1g_condition)

beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
    forces=cruise_structural_wing_mesh_forces,
    design_condition=plus_3g_condition)

system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, minus_1g_condition)

# Total mass properties
total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=minus_1g_condition)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)
system_m3l_model.register_output(total_mass, minus_1g_condition)
system_m3l_model.register_output(total_cg, minus_1g_condition)
system_m3l_model.register_output(total_inertia, minus_1g_condition)


# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L(load_factor=-1)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=minus_1g_condition)
system_m3l_model.register_output(inertial_forces, minus_1g_condition)
system_m3l_model.register_output(inertial_moments, minus_1g_condition)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=minus_1g_condition)
system_m3l_model.register_output(total_forces, minus_1g_condition)
system_m3l_model.register_output(total_moments, minus_1g_condition)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states,
    design_condition=minus_1g_condition,
)

system_m3l_model.register_output(trim_residual, minus_1g_condition)

# Add cruise m3l model to cruise condition
# minus_1g_condition.add_m3l_model('minus_1g_sizing_model', minus_1g_model)

# Add design condition to design scenario
design_scenario.add_design_condition(minus_1g_condition)
# endregion

system_model.add_design_scenario(design_scenario=design_scenario)
system_model.add_m3l_model('system_m3l_model', system_m3l_model)
# endregion

caddee_csdl_model = caddee.assemble_csdl()

h_tail_act_plus_3g = caddee_csdl_model.create_input('plus_3g_tail_actuation', val=np.deg2rad(-10))
caddee_csdl_model.add_design_variable('plus_3g_tail_actuation', 
                                lower=np.deg2rad(-25),
                                upper=np.deg2rad(25),
                                scaler=1,
                            )
wing_act_plus_3g = caddee_csdl_model.create_input('plus_3g_wing_actuation', val=np.deg2rad(4))

h_tail_act_minus_1g = caddee_csdl_model.create_input('minus_1g_tail_actuation', val=np.deg2rad(10))
caddee_csdl_model.add_design_variable('minus_1g_tail_actuation', 
                                lower=np.deg2rad(-25),
                                upper=np.deg2rad(25),
                                scaler=1,
                            )
wing_act_minis_1g = caddee_csdl_model.create_input('minus_1g_wing_actuation', val=np.deg2rad(4))

# h_tail_act = caddee_csdl_model.create_input('h_tail_act', val=np.deg2rad(0))
# caddee_csdl_model.add_design_variable('h_tail_act', 
#                                 lower=np.deg2rad(-25),
#                                 upper=np.deg2rad(25),
#                                 scaler=1,
#                             )

# wing_incidence = caddee_csdl_model.create_input('wing_incidence', val=np.deg2rad(4))
# # caddee_csdl_model.add_design_variable('wing_incidence', 
# #                                 lower=np.deg2rad(-5),
# #                                 upper=np.deg2rad(6),
# #                                 scaler=1,
# #                             )

# caddee_csdl_model.add_constraint('system_model.aircraft_trim.plus_3g_sizing.plus_3g_sizing.wing_eb_beam_model.Aframe.new_stress',upper=276E6/1.5,scaler=1E-8)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_euler_eom_gen_ref_pt.trim_residual', equals=0.)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.minus_1g_sizing_euler_eom_gen_ref_pt.trim_residual', equals=0.)
caddee_csdl_model.add_objective('system_model.system_m3l_model.total_constant_mass_properties.total_constant_mass', scaler=1e-3)

# caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.vm_stress'])
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.wing_beam_forces'])

sim.check_totals()


exit()

# sim.compute_total_derivatives()


prob = CSDLProblem(problem_name='lpc', simulator=sim)
optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
optimizer.solve()
optimizer.print_results()

print("I'm done.")