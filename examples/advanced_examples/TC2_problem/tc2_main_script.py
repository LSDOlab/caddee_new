import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee import IMPORTS_FILES_FOLDER
import array_mapper as am
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.generate_mappings_m3l import VASTNodalForces
from VAST.core.vlm_llt.viscous_correction import ViscousCorrectionModel
from VAST.core.fluid_problem import FluidProblem
from aframe.core.beam_module import EBBeam, LinearBeamMesh
import aframe.core.beam_module as ebbeam
from aframe.core.mass import Mass, MassMesh
from modopt.scipy_library import SLSQP
from modopt.snopt_library import SNOPT
from modopt.csdl_library import CSDLProblem
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
from lsdo_rotor.core.pitt_peters.pitt_peters_m3l import PittPeters, PittPetersMesh
from lsdo_airfoil.core.pressure_profile import PressureProfile, NodalPressureProfile
from lsdo_acoustics import Acoustics
from lsdo_acoustics.core.m3l_models import Lowson, KS, SKM, GL, TotalAircraftNoise


caddee = cd.CADDEE()

# Import representation and the geometry from another file for brevity
from examples.advanced_examples.TC2_problem.ex_tc2_geometry_setup import lpc_rep, lpc_param, wing_camber_surface, htail_camber_surface, \
    wing_vlm_mesh_name, htail_vlm_mesh_name, wing_oml_mesh, wing_upper_surface_ml, htail_upper_surface_ml, \
    pp_disk_in_plane_x, pp_disk_in_plane_y, pp_disk_origin, pp_disk, \
    wing_beam, width, height, num_wing_beam, wing, wing_upper_surface_wireframe, wing_lower_surface_wireframe,\
    rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk 

# set system representation and parameterization
caddee.system_representation = lpc_rep
caddee.system_parameterization = lpc_param

# system model
caddee.system_model = system_model = cd.SystemModel()

system_m3l_model = m3l.Model()

# region sizing (transition only)
# Battery sizing
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

simple_battery_sizing.set_module_input('battery_mass', val=800, dv_flag=False, lower=600, scaler=1e-3)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.6, 0, 0.5]))
simple_battery_sizing.set_module_input('battery_energy_density', val=400)

battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()
system_m3l_model.register_output(battery_mass)
system_m3l_model.register_output(cg_battery)
system_m3l_model.register_output(I_battery)


# M4 regressions
m4_regression = cd.M4RegressionsM3L(exclude_wing=False)

mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)
system_m3l_model.register_output(mass_m4)
system_m3l_model.register_output(cg_m4)
system_m3l_model.register_output(I_m4)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

system_m3l_model.register_output(total_mass)
system_m3l_model.register_output(total_cg)
system_m3l_model.register_output(total_inertia)
# endregion

# region sizing transition plus beam
# # battery
# battery_component = cd.Component(name='battery')
# simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)
# simple_battery_sizing.set_module_input('battery_mass', val=800)
# simple_battery_sizing.set_module_input('battery_position', val=np.array([3.2, 0, 0.5]), dv_flag=False, lower=np.array([3.0, -1e-4, 0.5 - 1e-4]), upper=np.array([4, +1e-4, 0.5 + 1e-4]), scaler=1e-1)
# simple_battery_sizing.set_module_input('battery_energy_density', val=400)
# battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()

# # M4 regressions
# m4_regression = cd.M4RegressionsM3L(exclude_wing=True)
# mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)

# # beam sizing
# # create the aframe dictionaries:
# joints, bounds, beams = {}, {}, {}
# youngs_modulus = 72.4E9
# poisson_ratio = 0.33
# shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
# material_density = 2780

# beams['wing_beam'] = {'E': youngs_modulus, 'G': shear_modulus, 'rho': material_density, 'cs': 'box', 'nodes': list(range(num_wing_beam))}
# bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1, 1, 1, 1, 1, 1]}

# beam_mass_mesh = MassMesh(
#     meshes = dict(
#         wing_beam = wing_beam,
#         wing_beam_width = width,
#         wing_beam_height = height,
#     )
# )
# beam_mass = Mass(component=wing, mesh=beam_mass_mesh, beams=beams, mesh_units='ft')
# beam_mass.set_module_input('wing_beam_tcap', val=0.01, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# beam_mass.set_module_input('wing_beam_tweb', val=0.01, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# mass_model_wing_mass = beam_mass.evaluate()

# # total constant mass 
# constant_mps = cd.TotalConstantMassM3L()
# total_constant_mass = constant_mps.evaluate(mass_model_wing_mass, battery_mass, mass_m4)

# system_m3l_model.register_output(total_constant_mass)
# endregion


# region BEM meshes
pusher_bem_mesh = BEMMesh(
    airfoil='NACA_4412', 
    num_blades=4,
    num_radial=25,
    num_tangential=1,
    mesh_units='ft',
    use_airfoil_ml=False,
)
bem_mesh_lift = BEMMesh(
    num_blades=2,
    num_radial=25,
    num_tangential=25,
    airfoil='NACA_4412',
    use_airfoil_ml=False,
    mesh_units='ft',
)

pitt_peters_mesh_lift = PittPetersMesh(
    num_blades=2,
    num_radial=25,
    num_tangential=25,
    airfoil='NACA_4412',
    use_airfoil_ml=False,
    mesh_units='ft',
)
# endregion

design_scenario = cd.DesignScenario(name='quasi_steady_transition')

# # region +3g sizing condition
# plus_3g_condition = cd.CruiseCondition(name="plus_3g_sizing")
# plus_3g_condition.atmosphere_model = cd.SimpleAtmosphereModel()
# plus_3g_condition.set_module_input(name='altitude', val=1000)
# plus_3g_condition.set_module_input(name='mach_number', val=0.23, dv_flag=False, lower=0.17, upper=0.19)
# plus_3g_condition.set_module_input(name='range', val=1)
# plus_3g_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
# plus_3g_condition.set_module_input(name='flight_path_angle', val=0)
# plus_3g_condition.set_module_input(name='roll_angle', val=0)
# plus_3g_condition.set_module_input(name='yaw_angle', val=0)
# plus_3g_condition.set_module_input(name='wind_angle', val=0)
# plus_3g_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

# ac_states = plus_3g_condition.evaluate_ac_states()
# system_m3l_model.register_output(ac_states)

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_plus_3g',
#         f'{htail_vlm_mesh_name}_plus_3g',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# # aero forces and moments
# vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=plus_3g_condition)
# system_m3l_model.register_output(vlm_force, plus_3g_condition)
# system_m3l_model.register_output(vlm_moment, plus_3g_condition)

# vlm_force_mapping_model = VASTNodalForces(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_plus_3g',
#         f'{htail_vlm_mesh_name}_plus_3g',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     initial_meshes=[
#         wing_camber_surface,
#         htail_camber_surface]
# )

# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh], design_condition=plus_3g_condition)
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
#     mesh_units='ft',
#     use_airfoil_ml=False,

# )

# bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# bem_model.set_module_input('rpm', val=2000, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=ac_states, design_condition=plus_3g_condition)

# # create the beam model:
# beam_mesh = LinearBeamMesh(
#     meshes = dict(
#         wing_beam = wing_beam,
#         wing_beam_width = width,
#         wing_beam_height = height,
#     )
# )
# beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints, mesh_units='ft')
# # beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# # beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

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
#                                                                    nodal_forces_mesh=wing_oml_mesh,
#                                                                    design_condition=plus_3g_condition)

# beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# # beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
# # beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

# cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
#     forces=cruise_structural_wing_mesh_forces,
#     design_condition=plus_3g_condition)

# system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, plus_3g_condition)

# # Total mass properties
# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=plus_3g_condition)
# # total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

# system_m3l_model.register_output(total_mass, plus_3g_condition)
# system_m3l_model.register_output(total_cg, plus_3g_condition)
# system_m3l_model.register_output(total_inertia, plus_3g_condition)

# # inertial forces and moments
# inertial_loads_model = cd.InertialLoadsM3L(load_factor=3)
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=plus_3g_condition)
# system_m3l_model.register_output(inertial_forces, plus_3g_condition)
# system_m3l_model.register_output(inertial_moments, plus_3g_condition)

# # total forces and moments 
# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=plus_3g_condition)
# system_m3l_model.register_output(total_forces, plus_3g_condition)
# system_m3l_model.register_output(total_moments, plus_3g_condition)

# # pass total forces/moments + mass properties into EoM model
# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=ac_states,
#     design_condition=plus_3g_condition,
# )

# system_m3l_model.register_output(trim_residual, plus_3g_condition)

# # Add cruise m3l model to cruise condition
# # plus_3g_condition.add_m3l_model('plus_3g_sizing_model', plus_3g_model)
# # endregion

# # region -1g condition
# # minus_1g_model = m3l.Model()
# minus_1g_condition = cd.CruiseCondition(name="minus_1g_sizing")
# minus_1g_condition.atmosphere_model = cd.SimpleAtmosphereModel()
# minus_1g_condition.set_module_input(name='altitude', val=1000)
# minus_1g_condition.set_module_input(name='mach_number', val=0.23)
# minus_1g_condition.set_module_input(name='range', val=1)
# minus_1g_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-25), upper=np.deg2rad(20))
# minus_1g_condition.set_module_input(name='flight_path_angle', val=0)
# minus_1g_condition.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# ac_states = minus_1g_condition.evaluate_ac_states()
# system_m3l_model.register_output(ac_states)

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_minus_1g',
#         f'{htail_vlm_mesh_name}_minus_1g',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# # aero forces and moments
# vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=minus_1g_condition)
# system_m3l_model.register_output(vlm_force, minus_1g_condition)
# system_m3l_model.register_output(vlm_moment, minus_1g_condition)

# vlm_force_mapping_model = VASTNodalForces(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_minus_1g',
#         f'{htail_vlm_mesh_name}_minus_1g',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     initial_meshes=[
#         wing_camber_surface,
#         htail_camber_surface]
# )

# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh], design_condition=minus_1g_condition)
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
#     mesh_units='ft',
#     use_airfoil_ml=False,

# )

# bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# bem_model.set_module_input('rpm', val=2000, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=ac_states, design_condition=minus_1g_condition)

# # create the beam model:
# beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints, mesh_units='ft')
# # beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# # beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

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
#                                                                    nodal_forces_mesh=wing_oml_mesh,
#                                                                    design_condition=minus_1g_condition)

# beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# # beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
# # beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

# cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
#     forces=cruise_structural_wing_mesh_forces,
#     design_condition=minus_1g_condition)

# system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, minus_1g_condition)

# # Total mass properties
# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=minus_1g_condition)
# # total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)
# system_m3l_model.register_output(total_mass, minus_1g_condition)
# system_m3l_model.register_output(total_cg, minus_1g_condition)
# system_m3l_model.register_output(total_inertia, minus_1g_condition)


# # inertial forces and moments
# inertial_loads_model = cd.InertialLoadsM3L(load_factor=-1)
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=minus_1g_condition)
# system_m3l_model.register_output(inertial_forces, minus_1g_condition)
# system_m3l_model.register_output(inertial_moments, minus_1g_condition)

# # total forces and moments 
# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=minus_1g_condition)
# system_m3l_model.register_output(total_forces, minus_1g_condition)
# system_m3l_model.register_output(total_moments, minus_1g_condition)

# # pass total forces/moments + mass properties into EoM model
# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=ac_states,
#     design_condition=minus_1g_condition,
# )

# system_m3l_model.register_output(trim_residual, minus_1g_condition)

# # Add cruise m3l model to cruise condition
# # minus_1g_condition.add_m3l_model('minus_1g_sizing_model', minus_1g_model)
# # endregion


# # region hover oei flo
# hover_1_oei_flo = cd.HoverCondition(name='hover_1_oei_flo')
# hover_1_oei_flo.atmosphere_model = cd.SimpleAtmosphereModel()
# hover_1_oei_flo.set_module_input('altitude', val=300)
# hover_1_oei_flo.set_module_input(name='hover_time', val=90)
# hover_1_oei_flo.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# hover_1_ac_states = hover_1_oei_flo.evaluate_ac_states()

# rlo_bem_model = BEM(disk_prefix='hover_1_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# rli_bem_model = BEM(disk_prefix='hover_1_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# rri_bem_model = BEM(disk_prefix='hover_1_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# rro_bem_model = BEM(disk_prefix='hover_1_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# fli_bem_model = BEM(disk_prefix='hover_1_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# fri_bem_model = BEM(disk_prefix='hover_1_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# fro_bem_model = BEM(disk_prefix='hover_1_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1_oei_flo)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=hover_1_oei_flo)

# system_m3l_model.register_output(total_mass, hover_1_oei_flo)
# system_m3l_model.register_output(total_cg, hover_1_oei_flo)
# system_m3l_model.register_output(total_inertia, hover_1_oei_flo)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=hover_1_ac_states, 
#     design_condition=hover_1_oei_flo
# )
# system_m3l_model.register_output(inertial_forces, hover_1_oei_flo)
# system_m3l_model.register_output(inertial_moments, hover_1_oei_flo)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     design_condition=hover_1_oei_flo,
# )
# system_m3l_model.register_output(total_forces,hover_1_oei_flo)
# system_m3l_model.register_output(total_moments, hover_1_oei_flo)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=hover_1_ac_states,
#     design_condition=hover_1_oei_flo,
# )
# system_m3l_model.register_output(trim_residual, hover_1_oei_flo)
# # endregion

# # region qst 1 oei flo
# qst_1_oei_flo = cd.CruiseCondition(name='qst_1_oei_flo')
# qst_1_oei_flo.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_1_oei_flo.set_module_input('pitch_angle', val=0-0.0134037)
# qst_1_oei_flo.set_module_input('mach_number', val=0.00029412)
# qst_1_oei_flo.set_module_input('altitude', val=300)
# qst_1_oei_flo.set_module_input(name='range', val=20)
# qst_1_oei_flo.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_1_ac_states = qst_1_oei_flo.evaluate_ac_states()

# rlo_bem_model = BEM(disk_prefix='qst_1_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)

# rli_bem_model = BEM(disk_prefix='qst_1_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)

# rri_bem_model = BEM(disk_prefix='qst_1_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)

# rro_bem_model = BEM(disk_prefix='qst_1_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)


# fli_bem_model = BEM(disk_prefix='qst_1_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)

# fri_bem_model = BEM(disk_prefix='qst_1_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)

# fro_bem_model = BEM(disk_prefix='qst_1_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1_oei_flo)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_1_oei_flo)

# system_m3l_model.register_output(total_mass, qst_1_oei_flo)
# system_m3l_model.register_output(total_cg, qst_1_oei_flo)
# system_m3l_model.register_output(total_inertia, qst_1_oei_flo)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_1_ac_states, 
#     design_condition=qst_1_oei_flo
# )
# system_m3l_model.register_output(inertial_forces, qst_1_oei_flo)
# system_m3l_model.register_output(inertial_moments, qst_1_oei_flo)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     # pp_bem_forces,
#     # pp_bem_moments,
#     # vlm_forces,
#     # vlm_moments,
#     design_condition=qst_1_oei_flo,
# )
# system_m3l_model.register_output(total_forces,qst_1_oei_flo)
# system_m3l_model.register_output(total_moments, qst_1_oei_flo)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_1_ac_states,
#     design_condition=qst_1_oei_flo,
# )
# system_m3l_model.register_output(trim_residual, qst_1_oei_flo)
# # endregion


# region on design 

# region hover 1
hover_1 = cd.HoverCondition(name='hover_1')
hover_1.atmosphere_model = cd.SimpleAtmosphereModel()
hover_1.set_module_input('altitude', val=300)
hover_1.set_module_input(name='hover_time', val=90)
hover_1.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

hover_1_ac_states = hover_1.evaluate_ac_states()

rlo_bem_model = BEM(disk_prefix='hover_1_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments, rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct = rlo_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rli_bem_model = BEM(disk_prefix='hover_1_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments, rli_dT ,rli_dQ ,rli_dD, rli_Ct = rli_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rri_bem_model = BEM(disk_prefix='hover_1_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments, rri_dT ,rri_dQ ,rri_dD, rri_Ct = rri_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rro_bem_model = BEM(disk_prefix='hover_1_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments, rro_dT ,rro_dQ ,rro_dD, rro_Ct = rro_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

flo_bem_model = BEM(disk_prefix='hover_1_flo_disk', blade_prefix='flo', component=flo_disk, mesh=bem_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments, flo_dT ,flo_dQ ,flo_dD, flo_Ct = flo_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

fli_bem_model = BEM(disk_prefix='hover_1_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments, fli_dT ,fli_dQ ,fli_dD, fli_Ct = fli_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

fri_bem_model = BEM(disk_prefix='hover_1_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments, fri_dT ,fri_dQ ,fri_dD, fri_Ct = fri_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

fro_bem_model = BEM(disk_prefix='hover_1_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments, fro_dT ,fro_dQ ,fro_dD, fro_Ct = fro_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)


# acoustics
hover_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 300.])
)

hover_acoustics.add_observer(
    name='obs1',
    obs_position=np.array([0., 0., 0.]),
    time_vector=np.array([0. ]),
)

rlo_ks_model = KS(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rlo_disk', blade_prefix='rlo')
rlo_hover_tonal_SPL, rlo_hover_tonal_SPL_A_weighted = rlo_ks_model.evaluate_tonal_noise(rlo_dT, rlo_dD, hover_1_ac_states)
system_m3l_model.register_output(rlo_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rlo_hover_tonal_SPL_A_weighted, hover_1)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rlo_disk', blade_prefix='rlo')
rlo_hover_broadband_SPL, rlo_hover_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(hover_1_ac_states, rlo_Ct)
system_m3l_model.register_output(rlo_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rlo_hover_broadband_SPL_A_weighted, hover_1)


rli_ks_model = KS(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rli_disk', blade_prefix='rli')
rli_hover_tonal_SPL, rli_hover_tonal_SPL_A_weighted = rli_ks_model.evaluate_tonal_noise(rli_dT, rli_dD, hover_1_ac_states)
system_m3l_model.register_output(rli_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rli_hover_tonal_SPL_A_weighted, hover_1)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rli_disk', blade_prefix='rli')
rli_hover_broadband_SPL, rli_hover_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(hover_1_ac_states, rli_Ct)
system_m3l_model.register_output(rli_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rli_hover_broadband_SPL_A_weighted, hover_1)


rri_ks_model = KS(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rri_disk', blade_prefix='rri')
rri_hover_tonal_SPL, rri_hover_tonal_SPL_A_weighted = rri_ks_model.evaluate_tonal_noise(rri_dT, rri_dD, hover_1_ac_states)
system_m3l_model.register_output(rri_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rri_hover_tonal_SPL_A_weighted, hover_1)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rri_disk', blade_prefix='rri')
rri_hover_broadband_SPL, rri_hover_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(hover_1_ac_states, rri_Ct)
system_m3l_model.register_output(rri_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rri_hover_broadband_SPL_A_weighted, hover_1)


rro_ks_model = KS(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rro_disk', blade_prefix='rro')
rro_hover_tonal_SPL, rro_hover_tonal_SPL_A_weighted = rro_ks_model.evaluate_tonal_noise(rro_dT, rro_dD, hover_1_ac_states)
system_m3l_model.register_output(rro_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rro_hover_tonal_SPL_A_weighted, hover_1)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rro_disk', blade_prefix='rro')
rro_hover_broadband_SPL, rro_hover_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(hover_1_ac_states, rro_Ct)
system_m3l_model.register_output(rro_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rro_hover_broadband_SPL_A_weighted, hover_1)


flo_ks_model = KS(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_flo_disk', blade_prefix='flo')
flo_hover_tonal_SPL, flo_hover_tonal_SPL_A_weighted = flo_ks_model.evaluate_tonal_noise(flo_dT, flo_dD, hover_1_ac_states)
system_m3l_model.register_output(flo_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(flo_hover_tonal_SPL_A_weighted, hover_1)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_flo_disk', blade_prefix='flo')
flo_hover_broadband_SPL, flo_hover_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(hover_1_ac_states, flo_Ct)
system_m3l_model.register_output(flo_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(flo_hover_broadband_SPL_A_weighted, hover_1)


fli_ks_model = KS(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fli_disk', blade_prefix='fli')
fli_hover_tonal_SPL, fli_hover_tonal_SPL_A_weighted = fli_ks_model.evaluate_tonal_noise(fli_dT, fli_dD, hover_1_ac_states)
system_m3l_model.register_output(fli_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(fli_hover_tonal_SPL_A_weighted, hover_1)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fli_disk', blade_prefix='fli')
fli_hover_broadband_SPL, fli_hover_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(hover_1_ac_states, fli_Ct)
system_m3l_model.register_output(fli_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(fli_hover_broadband_SPL_A_weighted, hover_1)


fri_ks_model = KS(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fri_disk', blade_prefix='fri')
fri_hover_tonal_SPL, fri_hover_tonal_SPL_A_weighted = fri_ks_model.evaluate_tonal_noise(fri_dT, fri_dD, hover_1_ac_states)
system_m3l_model.register_output(fri_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(fri_hover_tonal_SPL_A_weighted, hover_1)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fri_disk', blade_prefix='fri')
fri_hover_broadband_SPL, fri_hover_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(hover_1_ac_states, fri_Ct)
system_m3l_model.register_output(fri_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(fri_hover_broadband_SPL_A_weighted, hover_1)


fro_ks_model = KS(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fro_disk', blade_prefix='fro')
fro_hover_tonal_SPL, fro_hover_tonal_SPL_A_weighted = fro_ks_model.evaluate_tonal_noise(fro_dT, fro_dD, hover_1_ac_states)
system_m3l_model.register_output(fro_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(fro_hover_tonal_SPL_A_weighted, hover_1)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fro_disk', blade_prefix='fro')
fro_hover_broadband_SPL, fro_hover_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(hover_1_ac_states, fro_Ct)
system_m3l_model.register_output(fro_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(fro_hover_broadband_SPL_A_weighted, hover_1)


total_noise_model_hover = TotalAircraftNoise(
    acoustics_data=hover_acoustics,
    component_list=[rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
)
noise_components = [
    rlo_hover_tonal_SPL, rlo_hover_broadband_SPL,
    rli_hover_tonal_SPL, rli_hover_broadband_SPL,
    rri_hover_tonal_SPL, rri_hover_broadband_SPL,
    rro_hover_tonal_SPL, rro_hover_broadband_SPL,
    flo_hover_tonal_SPL, flo_hover_broadband_SPL,
    fli_hover_tonal_SPL, fli_hover_broadband_SPL,
    fri_hover_tonal_SPL, fri_hover_broadband_SPL,
    fro_hover_tonal_SPL, fro_hover_broadband_SPL,

]
A_weighted_noise_components = [
    rlo_hover_tonal_SPL_A_weighted, rlo_hover_broadband_SPL_A_weighted,
    rli_hover_tonal_SPL_A_weighted, rli_hover_broadband_SPL_A_weighted,
    rri_hover_tonal_SPL_A_weighted, rri_hover_broadband_SPL_A_weighted,
    rro_hover_tonal_SPL_A_weighted, rro_hover_broadband_SPL_A_weighted,
    flo_hover_tonal_SPL_A_weighted, flo_hover_broadband_SPL_A_weighted,
    fli_hover_tonal_SPL_A_weighted, fli_hover_broadband_SPL_A_weighted,
    fri_hover_tonal_SPL_A_weighted, fri_hover_broadband_SPL_A_weighted,
    fro_hover_tonal_SPL_A_weighted, fro_hover_broadband_SPL_A_weighted,

]

hover_total_SPL, hover_total_SPL_A_weighted = total_noise_model_hover.evaluate(noise_components, A_weighted_noise_components)
system_m3l_model.register_output(hover_total_SPL)
system_m3l_model.register_output(hover_total_SPL_A_weighted)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=hover_1)

system_m3l_model.register_output(total_mass, hover_1)
system_m3l_model.register_output(total_cg, hover_1)
system_m3l_model.register_output(total_inertia, hover_1)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=hover_1_ac_states, 
    design_condition=hover_1
)
system_m3l_model.register_output(inertial_forces, hover_1)
system_m3l_model.register_output(inertial_moments, hover_1)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    rlo_bem_forces, 
    rlo_bem_moments, 
    rli_bem_forces, 
    rli_bem_moments,
    rri_bem_forces, 
    rri_bem_moments, 
    rro_bem_forces, 
    rro_bem_moments,  
    flo_bem_forces, 
    flo_bem_moments, 
    fli_bem_forces, 
    fli_bem_moments,
    fri_bem_forces, 
    fri_bem_moments, 
    fro_bem_forces, 
    fro_bem_moments,  
    inertial_forces, 
    inertial_moments,
    design_condition=hover_1,
)
system_m3l_model.register_output(total_forces,hover_1)
system_m3l_model.register_output(total_moments, hover_1)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=hover_1_ac_states,
    design_condition=hover_1,
)
system_m3l_model.register_output(trim_residual, hover_1)
# endregion

# # region qst 1
# qst_1 = cd.CruiseCondition(name='qst_1')
# qst_1.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_1.set_module_input('pitch_angle', val=0-0.0134037)
# qst_1.set_module_input('mach_number', val=0.00029412)
# qst_1.set_module_input('altitude', val=300)
# qst_1.set_module_input(name='range', val=20)
# qst_1.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_1_ac_states = qst_1.evaluate_ac_states()


# # vlm_model = VASTFluidSover(
# #     surface_names=[
# #         f"{wing_vlm_mesh_name}_qst_1",
# #         f"{htail_vlm_mesh_name}_qst_1",
# #     ],
# #     surface_shapes=[
# #         (1, ) + wing_camber_surface.evaluate().shape[1:],
# #         (1, ) + htail_camber_surface.evaluate().shape[1:],
# #     ],
# #     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
# #     mesh_unit='ft',
# #     cl0=[0.43, 0]
# # )

# # forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)
# # system_m3l_model.register_output(vlm_forces, design_condition=qst_1)
# # system_m3l_model.register_output(vlm_moments, design_condition=qst_1)

# # pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# # pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=0, upper=2000, scaler=1e-3)
# # pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# rlo_bem_model = BEM(disk_prefix='qst_1_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# rli_bem_model = BEM(disk_prefix='qst_1_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# rri_bem_model = BEM(disk_prefix='qst_1_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# rro_bem_model = BEM(disk_prefix='qst_1_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# flo_bem_model = BEM(disk_prefix='qst_1_flo_disk', blade_prefix='flo', component=flo_disk, mesh=bem_mesh_lift)
# flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# fli_bem_model = BEM(disk_prefix='qst_1_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# fri_bem_model = BEM(disk_prefix='qst_1_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# fro_bem_model = BEM(disk_prefix='qst_1_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_1)

# system_m3l_model.register_output(total_mass, qst_1)
# system_m3l_model.register_output(total_cg, qst_1)
# system_m3l_model.register_output(total_inertia, qst_1)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_1_ac_states, 
#     design_condition=qst_1
# )
# system_m3l_model.register_output(inertial_forces, qst_1)
# system_m3l_model.register_output(inertial_moments, qst_1)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     flo_bem_forces, 
#     flo_bem_moments, 
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     # pp_bem_forces,
#     # pp_bem_moments,
#     # vlm_forces,
#     # vlm_moments,
#     design_condition=qst_1,
# )
# system_m3l_model.register_output(total_forces,qst_1)
# system_m3l_model.register_output(total_moments, qst_1)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_1_ac_states,
#     design_condition=qst_1,
# )
# system_m3l_model.register_output(trim_residual, qst_1)
# # endregion

# # region qst 2
# qst_2 = cd.CruiseCondition(name='qst_2')
# qst_2.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_2.set_module_input('pitch_angle', val=-0.04973228)
# qst_2.set_module_input('mach_number', val=0.06489461)
# qst_2.set_module_input('altitude', val=300)
# qst_2.set_module_input(name='range', val=20)
# qst_2.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


# qst_2_ac_states = qst_2.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_2",
#         f"{htail_vlm_mesh_name}_qst_2",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_2)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_2)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=50, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# rlo_bem_model = PittPeters(disk_prefix='qst_2_rlo_disk',  blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# rli_bem_model = PittPeters(disk_prefix='qst_2_rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# rri_bem_model = PittPeters(disk_prefix='qst_2_rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# rro_bem_model = PittPeters(disk_prefix='qst_2_rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# flo_bem_model = PittPeters(disk_prefix='qst_2_flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
# flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# fli_bem_model = PittPeters(disk_prefix='qst_2_fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# fri_bem_model = PittPeters(disk_prefix='qst_2_fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# fro_bem_model = PittPeters(disk_prefix='qst_2_fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_2)

# system_m3l_model.register_output(total_mass, qst_2)
# system_m3l_model.register_output(total_cg, qst_2)
# system_m3l_model.register_output(total_inertia, qst_2)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_2_ac_states, 
#     design_condition=qst_2
# )
# system_m3l_model.register_output(inertial_forces, qst_2)
# system_m3l_model.register_output(inertial_moments, qst_2)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     flo_bem_forces, 
#     flo_bem_moments, 
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_2,
# )
# system_m3l_model.register_output(total_forces,qst_2)
# system_m3l_model.register_output(total_moments, qst_2)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_2_ac_states,
#     design_condition=qst_2,
# )
# system_m3l_model.register_output(trim_residual, qst_2)
# # endregion

# # region qst 3
# qst_3 = cd.CruiseCondition(name='qst_3')
# qst_3.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_3.set_module_input('pitch_angle', val=0.16195989)
# qst_3.set_module_input('mach_number', val=0.11471427)
# qst_3.set_module_input('altitude', val=300)
# qst_3.set_module_input(name='range', val=20)
# qst_3.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_3_ac_states = qst_3.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_3",
#         f"{htail_vlm_mesh_name}_qst_3",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_3)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_3)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=300, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# rlo_bem_model = PittPeters(disk_prefix='qst_3_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# rli_bem_model = PittPeters(disk_prefix='qst_3_rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# rri_bem_model = PittPeters(disk_prefix='qst_3_rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# rro_bem_model = PittPeters(disk_prefix='qst_3_rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# flo_bem_model = PittPeters(disk_prefix='qst_3_flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
# flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# fli_bem_model = PittPeters(disk_prefix='qst_3_fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# fri_bem_model = PittPeters(disk_prefix='qst_3_fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# fro_bem_model = PittPeters(disk_prefix='qst_3_fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_3)

# system_m3l_model.register_output(total_mass, qst_3)
# system_m3l_model.register_output(total_cg, qst_3)
# system_m3l_model.register_output(total_inertia, qst_3)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_3_ac_states, 
#     design_condition=qst_3
# )
# system_m3l_model.register_output(inertial_forces, qst_3)
# system_m3l_model.register_output(inertial_moments, qst_3)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     flo_bem_forces, 
#     flo_bem_moments, 
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_3,
# )
# system_m3l_model.register_output(total_forces,qst_3)
# system_m3l_model.register_output(total_moments, qst_3)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_3_ac_states,
#     design_condition=qst_3,
# )
# system_m3l_model.register_output(trim_residual, qst_3)
# # endregion

# # region qst 4
# qst_4 = cd.CruiseCondition(name='qst_4')
# qst_4.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_4.set_module_input('pitch_angle', val=0.10779469, dv_flag=True, lower=0.10779469 - np.deg2rad(2))
# qst_4.set_module_input('mach_number', val=0.13740796)
# qst_4.set_module_input('altitude', val=300)
# qst_4.set_module_input(name='range', val=20)
# qst_4.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


# qst_4_ac_states = qst_4.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_4",
#         f"{htail_vlm_mesh_name}_qst_4",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_4)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_4)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# rlo_bem_model = PittPeters(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# rli_bem_model = PittPeters(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# rri_bem_model = PittPeters(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# rro_bem_model = PittPeters(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# flo_bem_model = PittPeters(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
# flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# fli_bem_model = PittPeters(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# fri_bem_model = PittPeters(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# fro_bem_model = PittPeters(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_4)

# system_m3l_model.register_output(total_mass, qst_4)
# system_m3l_model.register_output(total_cg, qst_4)
# system_m3l_model.register_output(total_inertia, qst_4)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_4_ac_states, 
#     design_condition=qst_4
# )
# system_m3l_model.register_output(inertial_forces, qst_4)
# system_m3l_model.register_output(inertial_moments, qst_4)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     flo_bem_forces, 
#     flo_bem_moments, 
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_4,
# )
# system_m3l_model.register_output(total_forces,qst_4)
# system_m3l_model.register_output(total_moments, qst_4)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_4_ac_states,
#     design_condition=qst_4,
# )
# system_m3l_model.register_output(trim_residual, qst_4)
# # endregion

# # region qst 5
# qst_5 = cd.CruiseCondition(name='qst_5')
# qst_5.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_5.set_module_input('pitch_angle', val=0.08224058, dv_flag=True, lower=0.08224058 - np.deg2rad(2.5))
# qst_5.set_module_input('mach_number', val=0.14708026)
# qst_5.set_module_input('altitude', val=300)
# qst_5.set_module_input(name='range', val=20)
# qst_5.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


# qst_5_ac_states = qst_5.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_5",
#         f"{htail_vlm_mesh_name}_qst_5",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_5)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_5)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# rlo_bem_model = PittPeters(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
# rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# rli_bem_model = PittPeters(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
# rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# rri_bem_model = PittPeters(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
# rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# rro_bem_model = PittPeters(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
# rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# flo_bem_model = PittPeters(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
# flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# fli_bem_model = PittPeters(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
# fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# fri_bem_model = PittPeters(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
# fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# fro_bem_model = PittPeters(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
# fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
# fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_5)

# system_m3l_model.register_output(total_mass, qst_5)
# system_m3l_model.register_output(total_cg, qst_5)
# system_m3l_model.register_output(total_inertia, qst_5)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_5_ac_states, 
#     design_condition=qst_5
# )
# system_m3l_model.register_output(inertial_forces, qst_5)
# system_m3l_model.register_output(inertial_moments, qst_5)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     rlo_bem_forces, 
#     rlo_bem_moments, 
#     rli_bem_forces, 
#     rli_bem_moments,
#     rri_bem_forces, 
#     rri_bem_moments, 
#     rro_bem_forces, 
#     rro_bem_moments,  
#     flo_bem_forces, 
#     flo_bem_moments, 
#     fli_bem_forces, 
#     fli_bem_moments,
#     fri_bem_forces, 
#     fri_bem_moments, 
#     fro_bem_forces, 
#     fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_5,
# )
# system_m3l_model.register_output(total_forces,qst_5)
# system_m3l_model.register_output(total_moments, qst_5)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_5_ac_states,
#     design_condition=qst_5,
# )
# system_m3l_model.register_output(trim_residual, qst_5)
# # endregion

# # region qst 6
# qst_6 = cd.CruiseCondition(name='qst_6')
# qst_6.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_6.set_module_input('pitch_angle', val=0.06704556, dv_flag=True, lower=0.06704556 - np.deg2rad(1.5))
# qst_6.set_module_input('mach_number', val=0.15408429)
# qst_6.set_module_input('altitude', val=300)
# qst_6.set_module_input(name='range', val=20)
# qst_6.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_6_ac_states = qst_6.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_6",
#         f"{htail_vlm_mesh_name}_qst_6",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_6)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_6)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # rlo_bem_model = PittPeters(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
# # rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # rli_bem_model = PittPeters(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
# # rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # rri_bem_model = PittPeters(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
# # rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # rro_bem_model = PittPeters(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
# # rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # flo_bem_model = PittPeters(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
# # flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # fli_bem_model = PittPeters(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
# # fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # fri_bem_model = PittPeters(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
# # fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# # fro_bem_model = PittPeters(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
# # fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=10, upper=4000, scaler=1e-3)
# # fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_6)

# system_m3l_model.register_output(total_mass, qst_6)
# system_m3l_model.register_output(total_cg, qst_6)
# system_m3l_model.register_output(total_inertia, qst_6)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_6_ac_states, 
#     design_condition=qst_6
# )
# system_m3l_model.register_output(inertial_forces, qst_6)
# system_m3l_model.register_output(inertial_moments, qst_6)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     # rlo_bem_forces, 
#     # rlo_bem_moments, 
#     # rli_bem_forces, 
#     # rli_bem_moments,
#     # rri_bem_forces, 
#     # rri_bem_moments, 
#     # rro_bem_forces, 
#     # rro_bem_moments,  
#     # flo_bem_forces, 
#     # flo_bem_moments, 
#     # fli_bem_forces, 
#     # fli_bem_moments,
#     # fri_bem_forces, 
#     # fri_bem_moments, 
#     # fro_bem_forces, 
#     # fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_6,
# )
# system_m3l_model.register_output(total_forces, qst_6)
# system_m3l_model.register_output(total_moments, qst_6)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_6_ac_states,
#     design_condition=qst_6,
# )
# system_m3l_model.register_output(trim_residual, qst_6)
# # endregion

# # region qst 7
# qst_7 = cd.CruiseCondition(name='qst_7')
# qst_7.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_7.set_module_input('pitch_angle', val=0.05598293, dv_flag=True, lower=0.05598293-np.deg2rad(2))
# qst_7.set_module_input('mach_number', val=0.15983874)
# qst_7.set_module_input('altitude', val=300)
# qst_7.set_module_input(name='range', val=20)
# qst_7.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_7_ac_states = qst_7.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_7",
#         f"{htail_vlm_mesh_name}_qst_7",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_7)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_7)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # rlo_bem_model = BEM(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=rlo_bem_mesh)
# # rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # rli_bem_model = BEM(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=rli_bem_mesh)
# # rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # rri_bem_model = BEM(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=rri_bem_mesh)
# # rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # rro_bem_model = BEM(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=rro_bem_mesh)
# # rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # flo_bem_model = BEM(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=flo_bem_mesh)
# # flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # fli_bem_model = BEM(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=fli_bem_mesh)
# # fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # fri_bem_model = BEM(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=fri_bem_mesh)
# # fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# # fro_bem_model = BEM(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=fro_bem_mesh)
# # fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_7)

# system_m3l_model.register_output(total_mass, qst_7)
# system_m3l_model.register_output(total_cg, qst_7)
# system_m3l_model.register_output(total_inertia, qst_7)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_7_ac_states, 
#     design_condition=qst_7
# )
# system_m3l_model.register_output(inertial_forces, qst_7)
# system_m3l_model.register_output(inertial_moments, qst_7)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     # rlo_bem_forces, 
#     # rlo_bem_moments, 
#     # rli_bem_forces, 
#     # rli_bem_moments,
#     # rri_bem_forces, 
#     # rri_bem_moments, 
#     # rro_bem_forces, 
#     # rro_bem_moments,  
#     # flo_bem_forces, 
#     # flo_bem_moments, 
#     # fli_bem_forces, 
#     # fli_bem_moments,
#     # fri_bem_forces, 
#     # fri_bem_moments, 
#     # fro_bem_forces, 
#     # fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_7,
# )
# system_m3l_model.register_output(total_forces, qst_7)
# system_m3l_model.register_output(total_moments, qst_7)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_7_ac_states,
#     design_condition=qst_7,
# )
# system_m3l_model.register_output(trim_residual, qst_7)
# # endregion

# # region qst 8
# qst_8 = cd.CruiseCondition(name='qst_8')
# qst_8.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_8.set_module_input('pitch_angle', val=0.04712265, dv_flag=True, lower=0.04712265-np.deg2rad(3))
# qst_8.set_module_input('mach_number', val=0.16485417)
# qst_8.set_module_input('altitude', val=300)
# qst_8.set_module_input(name='range', val=20)
# qst_8.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_8_ac_states = qst_8.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_8",
#         f"{htail_vlm_mesh_name}_qst_8",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_8)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_8)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=3000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # rlo_bem_model = BEM(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=rlo_bem_mesh)
# # rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # rli_bem_model = BEM(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=rli_bem_mesh)
# # rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # rri_bem_model = BEM(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=rri_bem_mesh)
# # rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # rro_bem_model = BEM(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=rro_bem_mesh)
# # rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # flo_bem_model = BEM(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=flo_bem_mesh)
# # flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # fli_bem_model = BEM(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=fli_bem_mesh)
# # fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # fri_bem_model = BEM(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=fri_bem_mesh)
# # fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# # fro_bem_model = BEM(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=fro_bem_mesh)
# # fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_8)

# system_m3l_model.register_output(total_mass, qst_8)
# system_m3l_model.register_output(total_cg, qst_8)
# system_m3l_model.register_output(total_inertia, qst_8)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_8_ac_states, 
#     design_condition=qst_8
# )
# system_m3l_model.register_output(inertial_forces, qst_8)
# system_m3l_model.register_output(inertial_moments, qst_8)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     # rlo_bem_forces, 
#     # rlo_bem_moments, 
#     # rli_bem_forces, 
#     # rli_bem_moments,
#     # rri_bem_forces, 
#     # rri_bem_moments, 
#     # rro_bem_forces, 
#     # rro_bem_moments,  
#     # flo_bem_forces, 
#     # flo_bem_moments, 
#     # fli_bem_forces, 
#     # fli_bem_moments,
#     # fri_bem_forces, 
#     # fri_bem_moments, 
#     # fro_bem_forces, 
#     # fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_8,
# )
# system_m3l_model.register_output(total_forces, qst_8)
# system_m3l_model.register_output(total_moments, qst_8)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_8_ac_states,
#     design_condition=qst_8,
# )
# system_m3l_model.register_output(trim_residual, qst_8)
# # endregion

# # region qst 9
# qst_9 = cd.CruiseCondition(name='qst_9')
# qst_9.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_9.set_module_input('pitch_angle', val=0.03981101, dv_flag=True, lower=0.03981101-np.deg2rad(4))
# qst_9.set_module_input('mach_number', val=0.16937793)
# qst_9.set_module_input('altitude', val=300)
# qst_9.set_module_input(name='range', val=20)
# qst_9.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_9_ac_states = qst_9.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_9",
#         f"{htail_vlm_mesh_name}_qst_9",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_9)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_9)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # rlo_bem_model = BEM(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=rlo_bem_mesh)
# # rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # rli_bem_model = BEM(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=rli_bem_mesh)
# # rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # rri_bem_model = BEM(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=rri_bem_mesh)
# # rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # rro_bem_model = BEM(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=rro_bem_mesh)
# # rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # flo_bem_model = BEM(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=flo_bem_mesh)
# # flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # fli_bem_model = BEM(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=fli_bem_mesh)
# # fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # fri_bem_model = BEM(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=fri_bem_mesh)
# # fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# # fro_bem_model = BEM(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=fro_bem_mesh)
# # fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
# # fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_9)

# system_m3l_model.register_output(total_mass, qst_9)
# system_m3l_model.register_output(total_cg, qst_9)
# system_m3l_model.register_output(total_inertia, qst_9)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_9_ac_states, 
#     design_condition=qst_9
# )
# system_m3l_model.register_output(inertial_forces, qst_9)
# system_m3l_model.register_output(inertial_moments, qst_9)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     # rlo_bem_forces, 
#     # rlo_bem_moments, 
#     # rli_bem_forces, 
#     # rli_bem_moments,
#     # rri_bem_forces, 
#     # rri_bem_moments, 
#     # rro_bem_forces, 
#     # rro_bem_moments,  
#     # flo_bem_forces, 
#     # flo_bem_moments, 
#     # fli_bem_forces, 
#     # fli_bem_moments,
#     # fri_bem_forces, 
#     # fri_bem_moments, 
#     # fro_bem_forces, 
#     # fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_9,
# )
# system_m3l_model.register_output(total_forces, qst_9)
# system_m3l_model.register_output(total_moments, qst_9)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_9_ac_states,
#     design_condition=qst_9,
# )
# system_m3l_model.register_output(trim_residual, qst_9)
# # endregion

# # region qst 10
# qst_10 = cd.CruiseCondition(name='qst_10')
# qst_10.atmosphere_model = cd.SimpleAtmosphereModel()
# qst_10.set_module_input('pitch_angle', val=0.03369678, dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(5))
# qst_10.set_module_input('mach_number', val=0.17354959)
# qst_10.set_module_input('altitude', val=300)
# qst_10.set_module_input(name='range', val=20)
# qst_10.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# qst_10_ac_states = qst_10.evaluate_ac_states()

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_10",
#         f"{htail_vlm_mesh_name}_qst_10",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.25, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_10)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_10)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # rlo_bem_model = PittPeters(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
# # rlo_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # rli_bem_model = PittPeters(disk_prefix='rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
# # rli_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # rri_bem_model = PittPeters(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
# # rri_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # rro_bem_model = PittPeters(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
# # rro_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # flo_bem_model = PittPeters(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
# # flo_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # fli_bem_model = PittPeters(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
# # fli_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # fri_bem_model = PittPeters(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
# # fri_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# # fro_bem_model = PittPeters(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
# # fro_bem_model.set_module_input('rpm', val=200, dv_flag=True, lower=0.5, upper=4000, scaler=4e-3)
# # fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=qst_10)

# system_m3l_model.register_output(total_mass, qst_10)
# system_m3l_model.register_output(total_cg, qst_10)
# system_m3l_model.register_output(total_inertia, qst_10)

# inertial_loads_model = cd.InertialLoadsM3L()
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(
#     total_cg_vector=total_cg, 
#     totoal_mass=total_mass, 
#     ac_states=qst_10_ac_states, 
#     design_condition=qst_10
# )
# system_m3l_model.register_output(inertial_forces, qst_10)
# system_m3l_model.register_output(inertial_moments, qst_10)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(
#     # rlo_bem_forces, 
#     # rlo_bem_moments, 
#     # rli_bem_forces, 
#     # rli_bem_moments,
#     # rri_bem_forces, 
#     # rri_bem_moments, 
#     # rro_bem_forces, 
#     # rro_bem_moments,  
#     # flo_bem_forces, 
#     # flo_bem_moments, 
#     # fli_bem_forces, 
#     # fli_bem_moments,
#     # fri_bem_forces, 
#     # fri_bem_moments, 
#     # fro_bem_forces, 
#     # fro_bem_moments,  
#     inertial_forces, 
#     inertial_moments,
#     pp_bem_forces,
#     pp_bem_moments,
#     vlm_forces,
#     vlm_moments,
#     design_condition=qst_10,
# )
# system_m3l_model.register_output(total_forces, qst_10)
# system_m3l_model.register_output(total_moments, qst_10)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=qst_10_ac_states,
#     design_condition=qst_10,
# )
# system_m3l_model.register_output(trim_residual, qst_10)
# # endregion

# # region climb 1
# climb_1 = cd.ClimbCondition(name='climb_1')
# climb_1.atmosphere_model = cd.SimpleAtmosphereModel()
# climb_1.set_module_input(name='altitude', val=1000)
# climb_1.set_module_input(name='mach_number', val=0.17)
# climb_1.set_module_input(name='initial_altitude', val=300)
# climb_1.set_module_input(name='final_altitude', val=1000)
# climb_1.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(10))
# climb_1.set_module_input(name='flight_path_angle', val=0, dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(10))
# climb_1.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

# ac_states = climb_1.evaluate_ac_states()
# system_m3l_model.register_output(ac_states)

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_climb',
#         f'{htail_vlm_mesh_name}_climb',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
#     mesh_unit='ft',
#     cl0=[0.25, 0.],
#     ML=True,
# )

# # aero forces and moments
# cl_distribution, re_spans, vlm_panel_forces, panel_areas, evaluation_pt, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, ML=True, design_condition=climb_1)
# # vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=climb_1)
# system_m3l_model.register_output(vlm_force)
# system_m3l_model.register_output(vlm_moment)
# system_m3l_model.register_output(cl_distribution)
# system_m3l_model.register_output(re_spans)

# ml_pressures = PressureProfile(
#     airfoil_name='NASA_langley_ga_1',
#     use_inverse_cl_map=True,
# )

# cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans) #, mach_number, reynolds_number)
# system_m3l_model.register_output(cp_upper, design_condition=climb_1)
# system_m3l_model.register_output(cp_lower, design_condition=climb_1)

# viscous_drag_correction = ViscousCorrectionModel(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_climb',
#         f'{htail_vlm_mesh_name}_climb',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
# )
# moment_point = None
# vlm_F, vlm_M = viscous_drag_correction.evaluate(ac_states=ac_states, forces=vlm_panel_forces, cd_v=Cd, panel_area=panel_areas, moment_pt=moment_point, evaluation_pt=evaluation_pt, design_condition=climb_1)
# system_m3l_model.register_output(vlm_F, design_condition=climb_1)
# system_m3l_model.register_output(vlm_M, design_condition=climb_1)

# ml_pressures_oml_map = NodalPressureProfile(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_climb',
#         f'{htail_vlm_mesh_name}_climb',
#     ],
#     surface_shapes=[
#         wing_upper_surface_ml.value.shape,
#         htail_upper_surface_ml.value.shape,
#     ]
# )

# cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[])
# wing_oml_pressure_upper = cp_upper_oml[0]
# htail_oml_pressure_upper = cp_upper_oml[1]
# wing_oml_pressure_lower = cp_lower_oml[0]
# htail_oml_pressure_lower = cp_lower_oml[1]

# system_m3l_model.register_output(wing_oml_pressure_upper, design_condition=climb_1)
# system_m3l_model.register_output(htail_oml_pressure_upper, design_condition=climb_1)
# system_m3l_model.register_output(wing_oml_pressure_lower, design_condition=climb_1)
# system_m3l_model.register_output(htail_oml_pressure_lower, design_condition=climb_1)

# vlm_force_mapping_model = VASTNodalForces(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_climb',
#         f'{htail_vlm_mesh_name}_climb',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     initial_meshes=[
#         wing_camber_surface,
#         htail_camber_surface]
# )

# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh])
# wing_forces = oml_forces[0]
# htail_forces = oml_forces[1]

# bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
# bem_forces, bem_moments,_ ,_ ,_ = bem_model.evaluate(ac_states=ac_states, design_condition=climb_1)

# system_m3l_model.register_output(bem_forces, design_condition=climb_1)
# system_m3l_model.register_output(bem_moments, design_condition=climb_1)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=climb_1)

# system_m3l_model.register_output(total_mass, climb_1)
# system_m3l_model.register_output(total_cg, climb_1)
# system_m3l_model.register_output(total_inertia, climb_1)

# inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=climb_1)
# system_m3l_model.register_output(inertial_forces, climb_1)
# system_m3l_model.register_output(inertial_moments, climb_1)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_F, vlm_M, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=climb_1)
# # total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
# system_m3l_model.register_output(total_forces, climb_1)
# system_m3l_model.register_output(total_moments, climb_1)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=ac_states,
#     design_condition=climb_1,
# )

# system_m3l_model.register_output(trim_residual, climb_1)
# # endregion

# # region cruise condition
# cruise_condition = cd.CruiseCondition(name="cruise")
# cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
# cruise_condition.set_module_input(name='altitude', val=1000)
# cruise_condition.set_module_input(name='mach_number', val=0.173, dv_flag=False, lower=0.17, upper=0.19)
# cruise_condition.set_module_input(name='range', val=40000)
# cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(5))
# cruise_condition.set_module_input(name='flight_path_angle', val=0)
# cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

# ac_states = cruise_condition.evaluate_ac_states()
# system_m3l_model.register_output(ac_states)

# vlm_model = VASTFluidSover(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_cruise',
#         f'{htail_vlm_mesh_name}_cruise',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
#     mesh_unit='ft',
#     cl0=[0.25, 0.],
#     ML=True,
# )

# # aero forces and moments
# cl_distribution, re_spans, vlm_panel_forces, panel_areas, evaluation_pt, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, ML=True, design_condition=cruise_condition)
# # vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)
# system_m3l_model.register_output(vlm_force)
# system_m3l_model.register_output(vlm_moment)
# system_m3l_model.register_output(cl_distribution)
# system_m3l_model.register_output(re_spans)

# ml_pressures = PressureProfile(
#     airfoil_name='NASA_langley_ga_1',
#     use_inverse_cl_map=True,
# )

# cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans) #, mach_number, reynolds_number)
# system_m3l_model.register_output(cp_upper, design_condition=cruise_condition)
# system_m3l_model.register_output(cp_lower, design_condition=cruise_condition)

# viscous_drag_correction = ViscousCorrectionModel(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_cruise',
#         f'{htail_vlm_mesh_name}_cruise',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
# )
# moment_point = None
# vlm_F, vlm_M = viscous_drag_correction.evaluate(ac_states=ac_states, forces=vlm_panel_forces, cd_v=Cd, panel_area=panel_areas, moment_pt=moment_point, evaluation_pt=evaluation_pt, design_condition=cruise_condition)
# system_m3l_model.register_output(vlm_F, design_condition=cruise_condition)
# system_m3l_model.register_output(vlm_M, design_condition=cruise_condition)

# ml_pressures_oml_map = NodalPressureProfile(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_cruise',
#         f'{htail_vlm_mesh_name}_cruise',
#     ],
#     surface_shapes=[
#         wing_upper_surface_ml.value.shape,
#         htail_upper_surface_ml.value.shape,
#     ]
# )

# cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[])
# wing_oml_pressure_upper = cp_upper_oml[0]
# htail_oml_pressure_upper = cp_upper_oml[1]
# wing_oml_pressure_lower = cp_lower_oml[0]
# htail_oml_pressure_lower = cp_lower_oml[1]

# system_m3l_model.register_output(wing_oml_pressure_upper, design_condition=cruise_condition)
# system_m3l_model.register_output(htail_oml_pressure_upper, design_condition=cruise_condition)
# system_m3l_model.register_output(wing_oml_pressure_lower, design_condition=cruise_condition)
# system_m3l_model.register_output(htail_oml_pressure_lower, design_condition=cruise_condition)

# vlm_force_mapping_model = VASTNodalForces(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_cruise',
#         f'{htail_vlm_mesh_name}_cruise',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     initial_meshes=[
#         wing_camber_surface,
#         htail_camber_surface]
# )

# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh])
# wing_forces = oml_forces[0]
# htail_forces = oml_forces[1]

# bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
# bem_forces, bem_moments,_ ,_ ,_ = bem_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)

# system_m3l_model.register_output(bem_forces, design_condition=cruise_condition)
# system_m3l_model.register_output(bem_moments, design_condition=cruise_condition)

# # total_mass_properties = cd.TotalMassPropertiesM3L()
# # total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=cruise_condition)

# system_m3l_model.register_output(total_mass, cruise_condition)
# system_m3l_model.register_output(total_cg, cruise_condition)
# system_m3l_model.register_output(total_inertia, cruise_condition)

# inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
# inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=cruise_condition)
# system_m3l_model.register_output(inertial_forces, cruise_condition)
# system_m3l_model.register_output(inertial_moments, cruise_condition)

# total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_F, vlm_M, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=cruise_condition)
# # total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
# system_m3l_model.register_output(total_forces, cruise_condition)
# system_m3l_model.register_output(total_moments, cruise_condition)

# eom_m3l_model = cd.EoMM3LEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=ac_states,
#     design_condition=cruise_condition,
# )

# system_m3l_model.register_output(trim_residual, cruise_condition)
# # endregion

# endregion


# wing sizing conditions
# design_scenario.add_design_condition(plus_3g_condition)
# design_scenario.add_design_condition(minus_1g_condition)

# Off design (OEI )

# On design 
design_scenario.add_design_condition(hover_1)
# design_scenario.add_design_condition(qst_1)
# design_scenario.add_design_condition(qst_2)
# design_scenario.add_design_condition(qst_3)
# design_scenario.add_design_condition(qst_4)
# design_scenario.add_design_condition(qst_5)
# design_scenario.add_design_condition(qst_6)
# design_scenario.add_design_condition(qst_7)
# design_scenario.add_design_condition(qst_8)
# design_scenario.add_design_condition(qst_9)
# design_scenario.add_design_condition(qst_10)
# design_scenario.add_design_condition(climb_1)
# design_scenario.add_design_condition(cruise_condition)

system_model.add_m3l_model('system_m3l_model', system_m3l_model)

caddee_csdl_model = caddee.assemble_csdl()


# region connections
# caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tweb', 'system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.wing_beam_tweb')
# caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tcap', 'system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.wing_beam_tcap')
# caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tweb', 'system_model.system_m3l_model.minus_1g_sizing_wing_eb_beam_model.Aframe.wing_beam_tweb')
# caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tcap', 'system_model.system_m3l_model.minus_1g_sizing_wing_eb_beam_model.Aframe.wing_beam_tcap')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rlo_disk_bem_model.rpm', 'system_model.system_m3l_model.rlo_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rlo_disk_bem_model.rpm', 'system_model.system_m3l_model.rlo_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rli_disk_bem_model.rpm', 'system_model.system_m3l_model.rli_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rli_disk_bem_model.rpm', 'system_model.system_m3l_model.rli_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rri_disk_bem_model.rpm', 'system_model.system_m3l_model.rri_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rri_disk_bem_model.rpm', 'system_model.system_m3l_model.rri_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rro_disk_bem_model.rpm', 'system_model.system_m3l_model.rro_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rro_disk_bem_model.rpm', 'system_model.system_m3l_model.rro_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_flo_disk_bem_model.rpm', 'system_model.system_m3l_model.flo_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_flo_disk_bem_model.rpm', 'system_model.system_m3l_model.flo_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fli_disk_bem_model.rpm', 'system_model.system_m3l_model.fli_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fli_disk_bem_model.rpm', 'system_model.system_m3l_model.fli_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fri_disk_bem_model.rpm', 'system_model.system_m3l_model.fri_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fri_disk_bem_model.rpm', 'system_model.system_m3l_model.fri_disk_KS_tonal_model.rpm')

caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fro_disk_bem_model.rpm', 'system_model.system_m3l_model.fro_disk_GL_broadband_model.rpm')
caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fro_disk_bem_model.rpm', 'system_model.system_m3l_model.fro_disk_KS_tonal_model.rpm')
# endregion






tilt_1 = np.deg2rad(0)
tilt_2 = np.deg2rad(0)

# region actuations

# h_tail_act_plus_3g = caddee_csdl_model.create_input('plus_3g_tail_actuation', val=np.deg2rad(0))
# caddee_csdl_model.add_design_variable('plus_3g_tail_actuation', 
#                                 lower=np.deg2rad(-25),
#                                 upper=np.deg2rad(25),
#                                 scaler=1,
#                             )
# wing_act_plus_3g = caddee_csdl_model.create_input('plus_3g_wing_actuation', val=np.deg2rad(3.2))

# h_tail_act_minus_1g = caddee_csdl_model.create_input('minus_1g_tail_actuation', val=np.deg2rad(0))
# caddee_csdl_model.add_design_variable('minus_1g_tail_actuation', 
#                                 lower=np.deg2rad(-25),
#                                 upper=np.deg2rad(25),
#                                 scaler=1,
#                             )
# wing_act_minis_1g = caddee_csdl_model.create_input('minus_1g_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_1_rlo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_rlo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_rli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_rli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_rri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_rri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_rro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_rro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_flo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_flo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_fli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_fli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_fri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_fri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_1_fro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_1_fro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_1_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_1_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))



# caddee_csdl_model.create_input('qst_2_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_2_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_2_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_2_rlo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_rlo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_rli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_rli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_rri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_rri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_rro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_rro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_flo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_flo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_fli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_fli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_fri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_fri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_2_fro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_2_fro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_2_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_2_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


# caddee_csdl_model.create_input('qst_3_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_3_tail_actuation', lower=np.deg2rad(-30), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_3_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_3_rlo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_rlo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_rli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_rli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_rri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_rri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_rro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_rro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_flo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_flo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_fli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_fli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_fri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_fri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_3_fro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_3_fro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_3_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_3_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


# caddee_csdl_model.create_input('qst_4_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_4_tail_actuation', lower=np.deg2rad(-25), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_4_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_4_rlo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_rlo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_rli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_rli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_rri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_rri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_rro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_rro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_flo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_flo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_fli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_fli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_fri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_fri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('qst_4_fro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('qst_4_fro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('qst_4_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('qst_4_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


# caddee_csdl_model.create_input('qst_5_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_5_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_5_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_6_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_6_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_6_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_7_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_7_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_7_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_8_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_8_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_8_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_9_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_9_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_9_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.create_input('qst_10_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('qst_10_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('qst_10_wing_actuation', val=np.deg2rad(3.2))


# caddee_csdl_model.create_input('hover_1_rlo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_rlo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_rli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_rli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_rri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_rri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_rro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_rro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_flo_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_flo_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_fli_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_fli_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_fri_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_fri_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('hover_1_fro_disk_actuation_1', val=tilt_1)
# caddee_csdl_model.create_input('hover_1_fro_disk_actuation_2', val=tilt_2)
# caddee_csdl_model.add_design_variable('hover_1_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('hover_1_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# caddee_csdl_model.create_input('climb_tail_actuation', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('climb_tail_actuation', lower=np.deg2rad(-115), upper=np.deg2rad(15))
# caddee_csdl_model.create_input('climb_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('cruise_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('cruise_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
caddee_csdl_model.create_input('cruise_wing_actuation', val=np.deg2rad(3.2))
# endregion

# region geometric constraints/dvs
# rlo_radius = caddee_csdl_model.create_input('rlo_radius', val=2.5)
# caddee_csdl_model.connect('rlo_radius', 'rlo_in_plane_r1')
# caddee_csdl_model.connect('rlo_radius', 'rlo_in_plane_r2')
# caddee_csdl_model.connect('rlo_radius', 'rlo_in_plane_r3')
# caddee_csdl_model.connect('rlo_radius', 'rlo_in_plane_r4')
# endregion

# caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.minus_1g_sizing_euler_eom_gen_ref_pt.trim_residual', equals=0)

# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_1_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_2_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_4_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_5_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_6_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_7_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_8_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_9_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_10_euler_eom_gen_ref_pt.trim_residual', equals=0)

# caddee_csdl_model.add_constraint('system_model.system_m3l_model.hover_1_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.climb_1_euler_eom_gen_ref_pt.trim_residual', equals=0)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.cruise_euler_eom_gen_ref_pt.trim_residual', equals=0)

# caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.new_stress',upper=427E6/1.,scaler=1E-8)


# caddee_csdl_model.add_objective('system_model.system_m3l_model.total_constant_mass_properties.total_constant_mass', scaler=1e-3)

trim_1 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.hover_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_2 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.climb_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_3 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.cruise_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_1 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_2 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_2_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_3 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_4 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_4_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_5 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_5_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_6 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_6_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_7 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_7_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_8 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_8_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_9 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_9_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# trim_10 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_10_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# # caddee_csdl_model.add_objective('system_model.system_m3l_model.total_constant_mass_properties.total_mass', scaler=1e-3)

combined_trim = caddee_csdl_model.register_output('combined_trim', trim_1 * 1)
# combined_trim = caddee_csdl_model.register_output('combined_trim', trim_1 *1 + trim_2*1 + trim_3*1 + trim_4*1 + trim_5 * 1 + trim_6 * 1 + trim_7 * 1 + trim_8 * 1 + trim_9 * 1 + trim_10*1)
caddee_csdl_model.add_objective('combined_trim')


sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

# print('\n')
# sim.check_totals(of='system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', wrt='system_model.system_m3l_model.qst_3_pp_disk_bem_model.rpm')
# sim.check_totals()

cruise_geometry = sim['design_geometry']    
updated_primitives_names = list(lpc_rep.spatial_representation.primitives.keys()).copy()
# cruise_geometry = sim['design_geometry']
lpc_rep.spatial_representation.update(cruise_geometry, updated_primitives_names)
lpc_rep.spatial_representation.plot()
exit()
# print(sim['system_model.system_m3l_model.qst_1_rlo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rro_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_flo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fro_disk_bem_model.thrust_vector'])


# prob = CSDLProblem(problem_name='lpc', simulator=sim)
# optimizer = SLSQP(prob, maxiter=1000, ftol=1E-7)
# optimizer.solve()
# optimizer.print_results()

prob = CSDLProblem(problem_name='TC_2_problem', simulator=sim)

optimizer = SNOPT(
    prob, 
    Major_iterations=1000, 
    Major_optimality=1e-5, 
    Major_feasibility=1e-5,
    append2file=True,
)

optimizer.solve()
# opt.print_results()

for dv_name, dv_dict in sim.dvs.items():
    print(dv_name, dv_dict['index_lower'], dv_dict['index_upper'])

print('\n')
print('\n')

for c_name, c_dict in sim.cvs.items():
    print(c_name, c_dict['index_lower'], c_dict['index_upper'])
# print(sim['system_model.system_m3l_model.qst_1_rlo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rro_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_flo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fro_disk_bem_model.thrust_vector'])

print('\n')

# print(sim['system_model.system_m3l_model.qst_2_rlo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_rli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_rri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_rro_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_flo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_fli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_fri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_2_fro_disk_bem_model.thrust_vector'])

# prob = CSDLProblem(problem_name='lpc', simulator=sim)
# optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
# optimizer.solve()
# optimizer.print_results()