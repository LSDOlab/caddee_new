'''Example 1 : TC 2 analysis and optimization script

This advanced example demonstrates how to build the analysis script for the complex NASA ULI technical challenge 2 (TC2) problem,
which include physics-based models for aerodynamics, structures, acoustics, motors and other disciplines. 
'''


# Module imports
import numpy as np
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
from lsdo_rotor import BEMParameters, evaluate_multiple_BEM_models, BEM
from VAST import FluidProblem, VASTFluidSover, VASTNodalForces
from lsdo_acoustics import Acoustics, evaluate_multiple_acoustic_models
from lsdo_motor import evaluate_multiple_motor_sizing_models, evaluate_multiple_motor_analysis_models, MotorAnalysis, MotorSizing
from aframe import BeamMassModel, EBBeam, EBBeamForces
from caddee.utils.helper_functions.geometry_helpers import  make_vlm_camber_mesh

from ex_tc2_geometry_setup_updated import (wing_meshes, tail_meshes, box_beam_mesh, pp_mesh, rlo_mesh, v_tail_meshes, pp_mesh,
                                           rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh, num_wing_beam,
                                           box_beam_mesh, drag_comp_list, S_ref, wing_AR, system_model)


caddee = cd.CADDEE()


# Set parameters for BEM analysis
num_radial = 30
num_tangential = 30

bem_hover_rotor_parameters = BEMParameters(
    num_blades=2,
    num_radial=num_radial,
    num_tangential=num_tangential,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
    mesh_units='ft',
)

bem_pusher_rotor_parameters = BEMParameters(
    num_blades=4,
    num_radial=num_radial,
    num_tangential=1,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
    mesh_units='ft',
)

rotor_mesh_list = [rlo_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh]
origin_list = [mesh.thrust_origin for mesh in rotor_mesh_list]
num_rotors = len(rotor_mesh_list)

# region Sizing 
# Motors
motor_diameters = []
motor_lengths = []
for i in range(num_rotors):
    motor_diameters.append(system_model.create_input(f'motor_diameter_{i}', val=0.17, dv_flag=False, upper=0.15, lower=0.05, scaler=2))
    motor_lengths.append(system_model.create_input(f'motor_length_{i}', val=0.1, dv_flag=False, upper=0.12, lower=0.05, scaler=2))


motor_mass_properties = evaluate_multiple_motor_sizing_models(
    motor_diameter_list=motor_diameters,
    motor_length_list=motor_lengths,
    motor_origin_list=origin_list,
    name_prefix='motor_sizing',
    m3l_model=system_model,
)

# Beam sizing 
# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
youngs_modulus = 72.4E9 # 46E9 # 
poisson_ratio = 0.33
shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
material_density = 2780 # 1320  # 

beams['wing_beam'] = {'E': youngs_modulus, 'G': shear_modulus, 'rho': material_density, 'cs': 'box', 'nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 10,'fdim': [1, 1, 1, 1, 1, 1]}

wing_beam_tcap = system_model.create_input(name='wing_beam_tcap' ,val=0.005 * np.ones((num_wing_beam, )))
wing_beam_tweb = system_model.create_input(name='wing_beam_tweb' ,val=0.005 * np.ones((num_wing_beam, )))

beam_mass_model = BeamMassModel(
    beams=beams,
    name='wing_beam_mass_model',
)
wing_beam_mass = beam_mass_model.evaluate(beam_nodes=box_beam_mesh.beam_nodes,
                                        width=box_beam_mesh.width, height=box_beam_mesh.height, 
                                        t_cap=wing_beam_tcap, t_web=wing_beam_tweb)

system_model.register_output(wing_beam_mass)

# Battery sizing
battery_mass = system_model.create_input(name='battery_mass', val=700, shape=(1, ))
energy_density = system_model.create_input(name='battery_energy_density', val=400, shape=(1, ))
battery_position = system_model.create_input(name='battery_position', val=np.array([3.2, 0., 3.538]))

battery_model = cd.SimpleBatterySizing(
    name='simple_battery_model',
)
battery_mass_properties = battery_model.evaluate(battery_mass=battery_mass, battery_position=battery_position, battery_energy_density=energy_density)
system_model.register_output(battery_mass_properties)

# M4 regression
m4_regression = cd.M4Regressions(
    name='m4_regression',
    exclude_wing=True,
)
m4_mass_properties = m4_regression.evaluate(battery_mass=battery_mass)
system_model.register_output(m4_mass_properties)
# endregion

# region +3g sizing
sizing_3g_condition = cd.CruiseCondition(name='plus_3g_sizing')
h_3g = system_model.create_input('altitude_3g', val=1000)
M_3g = system_model.create_input('mach_3g', val=0.22)
r_3g = system_model.create_input('range_3g', val=10000)
theta_3g = system_model.create_input('pitch_angle_3g', val=np.deg2rad(0), dv_flag=True, lower=0, upper=np.deg2rad(2))

# ac sates + atmos
ac_states_3g, atmos_3g = sizing_3g_condition.evaluate(mach_number=M_3g, pitch_angle=theta_3g, cruise_range=r_3g, altitude=h_3g)
system_model.register_output(ac_states_3g)
system_model.register_output(atmos_3g)

# BEM solver
plus_3g_pusher_rpm = system_model.create_input('plus_3g_pusher_rpm', val=1500, shape=(1, ), dv_flag=True, lower=1000, upper=2500, scaler=1e-3)
plus_3g_bem_model = BEM(
    name='plus_3g_bem',
    num_nodes=1,
    BEM_parameters=bem_pusher_rotor_parameters,
)
plus_3g_bem_outputs = plus_3g_bem_model.evaluate(ac_states=ac_states_3g, rpm=plus_3g_pusher_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
                                                 thrust_origin=pp_mesh.thrust_origin, atmosphere=atmos_3g, blade_chord=pp_mesh.chord_profile, blade_twist=pp_mesh.twist_profile)

system_model.register_output(plus_3g_bem_outputs)

# VAST solver
vlm_model = VASTFluidSover(
    name='cruise_vlm_model',
    surface_names=[
        'wing_mesh_plus_3g',
        'tail_mesh_plus_3g',
        'vtail_mesh_plus_3g'
    ],
    surface_shapes=[
        (1, ) + wing_meshes.vlm_mesh.shape[1:],
        (1, ) + tail_meshes.vlm_mesh.shape[1:],
        (1, ) + v_tail_meshes.vlm_mesh.shape[1:],
    
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0., 0., 0.]
    # cl0=[0.]
)

# Evaluate VLM outputs and register them as outputs
vlm_outputs = vlm_model.evaluate(
    atmosphere=atmos_3g,
    ac_states=ac_states_3g,
    meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh, v_tail_meshes.vlm_mesh],
    # meshes=[wing_meshes.vlm_mesh],
    wing_AR=wing_AR,
)
system_model.register_output(vlm_outputs)

# Nodal forces
vlm_force_mapping_model = VASTNodalForces(
    name='vast_3g_nodal_forces',
    surface_names=[
        f'wing_mesh_plus_3g',
        # f'tail_mesh_plus_3g',
    ],
    surface_shapes=[
        (1, ) + wing_meshes.vlm_mesh.shape[1:],
        # (1, ) + tail_meshes.vlm_mesh.shape[1:],
    ],
    initial_meshes=[
        wing_meshes.vlm_mesh,
        # tail_meshes.vlm_mesh
    ]
)

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_outputs.panel_forces, nodal_force_meshes=[wing_meshes.oml_mesh])
# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_outputs.panel_forces, nodal_force_meshes=[wing_meshes.oml_mesh, tail_meshes.oml_mesh])
wing_oml_forces = oml_forces[0]
# tail_oml_forces = oml_forces[1]




system_model.register_output(wing_oml_forces)
# system_model.register_output(tail_oml_forces)

beam_force_map_model = EBBeamForces(
    name='eb_beam_force_map_3g',
    beams=beams,
    exclude_middle=True,
)

structural_wing_mesh_forces_3g = beam_force_map_model.evaluate(
    beam_mesh=box_beam_mesh.beam_nodes,
    nodal_forces=wing_oml_forces,
    nodal_forces_mesh=wing_meshes.oml_mesh
)

beam_displacement_model = EBBeam(
    name='eb_beam_3g',
    beams=beams,
    bounds=bounds,
    joints=joints,
    mesh_units='ft',
)

eb_beam_outputs = beam_displacement_model.evaluate(beam_mesh=box_beam_mesh, t_cap=wing_beam_tcap, t_web=wing_beam_tweb, forces=structural_wing_mesh_forces_3g)
system_model.register_output(eb_beam_outputs)
# NOTE:
# We are creating a new MassProperties object for the wing beam where
# we combine the cg and inertia tensor that the beam analysis model
# outputs with the beam mass, which is based on the skin, and spar
# thicknesses. This is because the +3g condition will size the top
# skin while the -1g condition will size the bottom skin (if buckling
# is considered).
wing_beam_mass_props = cd.MassProperties(
    mass=wing_beam_mass,
    cg_vector=eb_beam_outputs.cg_vector,
    inertia_tensor=eb_beam_outputs.inertia_tensor,
)


drag_build_up_model = cd.DragBuildUpModel(
    name='sizing_drag_build_up',
    num_nodes=1,
    units='ft',
)

drag_build_up_outputs = drag_build_up_model.evaluate(atmos=atmos_3g, ac_states=ac_states_3g, drag_comp_list=drag_comp_list, s_ref=S_ref)
system_model.register_output(drag_build_up_outputs)

trim_variables = sizing_3g_condition.assemble_trim_residual(
    mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
    aero_propulsive_outputs=[vlm_outputs, plus_3g_bem_outputs, drag_build_up_outputs],
    ac_states=ac_states_3g,
)
system_model.register_output(trim_variables)
system_model.add_objective(trim_variables.accelerations)

# endregion


# # region Hover condition
# hover_condition = cd.HoverCondition(
#     name='hover_condition_1',
# )

# # Create inputs for hover condition
# hover_1_time = system_model.create_input('hover_1_time', val=90)
# hover_1_altitude = system_model.create_input('hover_1_altitude', val=100)

# # Evaluate aircraft states and atmospheric properties and register them as outputs
# hover_1_ac_states, hover_1_atmosphere = hover_condition.evaluate(hover_time=hover_1_time, altitude=hover_1_altitude)
# system_model.register_output(hover_1_ac_states)
# system_model.register_output(hover_1_atmosphere)

# hover_1_rpms = []
# for i in range(num_rotors):
#     hover_1_rpms.append(system_model.create_input(f'hover_1_rpm_{i}', val=1200, dv_flag=True, lower=800, upper=2000, scaler=1e-3))

# hover_bem_output_list = evaluate_multiple_BEM_models(
#     num_instances=8,
#     name_prefix='hover_1_bem',
#     bem_parameters=bem_hover_rotor_parameters,
#     bem_mesh_list=rotor_mesh_list,
#     rpm_list=hover_1_rpms,
#     ac_states=hover_1_ac_states,
#     atmoshpere=hover_1_atmosphere,
#     num_nodes=1,
#     m3l_model=system_model,
# )

# hover_acoustics_data = Acoustics(
#     aircraft_position=np.array([0., 0., 100])
# )

# hover_acoustics_data.add_observer(
#     name='hover_observer',
#     obs_position=np.array([0., 0., 0.]),
#     time_vector=np.array([0.,]),

# )

# hover_total_noise, hover_total_noise_a_weighted = evaluate_multiple_acoustic_models(
#     rotor_outputs=hover_bem_output_list,
#     acoustics_data=hover_acoustics_data,
#     ac_states=hover_1_ac_states,
#     tonal_noise_model='Lowson',
#     broadband_noise_model='GL',
#     altitude=hover_1_altitude,
#     rotor_parameters=bem_hover_rotor_parameters,
#     rotor_meshes=rotor_mesh_list,
#     rpm_list=hover_1_rpms,
#     model_name_prefix='hover_noise',
#     num_nodes=1,
#     m3l_model=system_model,
# )

# hover_motor_outputs = evaluate_multiple_motor_analysis_models(
#     rotor_outputs_list=hover_bem_output_list,
#     motor_sizing_list=motor_mass_properties,
#     rotor_rpm_list=hover_1_rpms,
#     motor_diameter_list=motor_diameters,
#     name_prefix='hover_motor_analysis',
#     flux_weakening=False,
#     m3l_model=system_model,
# )

# # endregion

caddee_csdl_model = system_model.assemble_csdl()

sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

# sim.check_totals()
# exit()

# prob = CSDLProblem(problem_name='TC2_new_caddee_test', simulator=sim)
# optimizer = SLSQP(prob, maxiter=100, ftol=1E-5)
# optimizer.solve()
# optimizer.print_results()

cd.print_caddee_outputs(system_model, sim)
print(sim['plus_3g_sizing_total_mass_properties_model.total_cg_vector'])
