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

from ex_tc2_geometry_setup_updated import (wing_meshes, tail_meshes, box_beam_mesh, pp_mesh, rlo_mesh, 
                                           rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh, num_wing_beam,
                                           box_beam_mesh)


caddee = cd.CADDEE()
system_model = m3l.Model()

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

rotor_mesh_list = [rlo_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh]
origin_list = [mesh.thrust_origin for mesh in rotor_mesh_list]
num_rotors = len(rotor_mesh_list)

# region Sizing 
# Motors
motor_diameters = []
motor_lengths = []
for i in range(num_rotors):
    motor_diameters.append(system_model.create_input(f'motor_diameter_{i}', val=0.17, dv_flag=True, upper=0.15, lower=0.05, scaler=2))
    motor_lengths.append(system_model.create_input(f'motor_length_{i}', val=0.1, dv_flag=True, upper=0.12, lower=0.05, scaler=2))


motor_sizing_outputs = evaluate_multiple_motor_sizing_models(
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
# endregion

# region +3g sizing
sizing_3g_condition = cd.CruiseCondition(name='plus_3g_sizing')
h_3g = system_model.create_input('altitude_3g', val=10000)
M_3g = system_model.create_input('mach_3g', val=0.2)
r_3g = system_model.create_input('range_3g', val=10000)
theta_3g = system_model.create_input('pitch_angle_3g', val=np.deg2rad(0), dv_flag=True, lower=0, upper=np.deg2rad(20))

# ac sates + atmos
ac_states_3g, atmos_3g = sizing_3g_condition.evaluate(mach_number=M_3g, pitch_angle=theta_3g, cruise_range=r_3g, altitude=h_3g)
system_model.register_output(ac_states_3g)
system_model.register_output(atmos_3g)

# VAST solver
vlm_model = VASTFluidSover(
    name='cruise_vlm_model',
    surface_names=[
        'wing_mesh_plus_3g',
        'tail_mesh_plus_3g',
    ],
    surface_shapes=[
        (1, ) + wing_meshes.vlm_mesh.shape[1:],
        (1, ) + tail_meshes.vlm_mesh.shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

# Evaluate VLM outputs and register them as outputs
vlm_outputs = vlm_model.evaluate(
    atmosphere=atmos_3g,
    ac_states=ac_states_3g,
    meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
)


system_model.register_output(vlm_outputs)

# Nodal forces
vlm_force_mapping_model = VASTNodalForces(
    name='vast_3g_nodal_forces',
    surface_names=[
        f'wing_mesh_plus_3g',
        f'tail_mesh_plus_3g',
    ],
    surface_shapes=[
        (1, ) + wing_meshes.vlm_mesh.shape[1:],
        (1, ) + tail_meshes.vlm_mesh.shape[1:],
    ],
    initial_meshes=[
        wing_meshes.vlm_mesh,
        tail_meshes.vlm_mesh
    ]
)

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_outputs.panel_forces, nodal_force_meshes=[wing_meshes.oml_mesh, tail_meshes.oml_mesh])
wing_oml_forces = oml_forces[0]
tail_oml_forces = oml_forces[1]

system_model.register_output(wing_oml_forces)
system_model.register_output(tail_oml_forces)

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


# endregion


# region Hover condition
hover_condition = cd.HoverCondition(
    name='hover_condition_1',
)

# Create inputs for hover condition
hover_1_time = system_model.create_input('hover_1_time', val=90)
hover_1_altitude = system_model.create_input('hover_1_altitude', val=100)

# Evaluate aircraft states and atmospheric properties and register them as outputs
hover_1_ac_states, hover_1_atmosphere = hover_condition.evaluate(hover_time=hover_1_time, altitude=hover_1_altitude)
system_model.register_output(hover_1_ac_states)
system_model.register_output(hover_1_atmosphere)

hover_1_rpms = []
for i in range(num_rotors):
    hover_1_rpms.append(system_model.create_input(f'hover_1_rpm_{i}', val=1200, dv_flag=True, lower=800, upper=2000, scaler=1e-3))

hover_bem_output_list = evaluate_multiple_BEM_models(
    num_instances=8,
    name_prefix='hover_1_bem',
    bem_parameters=bem_hover_rotor_parameters,
    bem_mesh_list=rotor_mesh_list,
    rpm_list=hover_1_rpms,
    ac_states=hover_1_ac_states,
    atmoshpere=hover_1_atmosphere,
    num_nodes=1,
    m3l_model=system_model,
)

hover_acoustics_data = Acoustics(
    aircraft_position=np.array([0., 0., 100])
)

hover_acoustics_data.add_observer(
    name='hover_observer',
    obs_position=np.array([0., 0., 0.]),
    time_vector=np.array([0.,]),

)

hover_total_noise, hover_total_noise_a_weighted = evaluate_multiple_acoustic_models(
    rotor_outputs=hover_bem_output_list,
    acoustics_data=hover_acoustics_data,
    ac_states=hover_1_ac_states,
    tonal_noise_model='Lowson',
    broadband_noise_model='GL',
    altitude=hover_1_altitude,
    rotor_parameters=bem_hover_rotor_parameters,
    rotor_meshes=rotor_mesh_list,
    rpm_list=hover_1_rpms,
    model_name_prefix='hover_noise',
    num_nodes=1,
    m3l_model=system_model,
)

hover_motor_outputs = evaluate_multiple_motor_analysis_models(
    rotor_outputs_list=hover_bem_output_list,
    motor_sizing_list=motor_sizing_outputs,
    rotor_rpm_list=hover_1_rpms,
    motor_diameter_list=motor_diameters,
    name_prefix='hover_motor_analysis',
    flux_weakening=False,
    m3l_model=system_model,
)

# endregion

caddee_csdl_model = system_model.assemble_csdl()

sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

# cd.print_caddee_outputs(system_model, sim)
print(sim['motor_sizing_2.mass'])
print(sim['hover_motor_analysis_1.turns_per_phase'])
print(sim['hover_motor_analysis_0.implicit_em_torque_model.T_lower_lim'])
print(sim['hover_1_bem_2.Q'])
print(sim['wing_beam_mass_model.mass'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_system.adapter_comp.psiw'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_system.solve_gamma_b_group.prepossing_before_Solve.RHS_group.KinematicVelocityComp.tail_mesh_plus_3g_coll_vel'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_system.solve_gamma_b_group.prepossing_before_Solve.RHS_group.KinematicVelocityComp.tail_mesh_plus_3g_rot_ref'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.density'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.panel_area_tail_mesh_plus_3g'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.panel_area_wing_mesh_plus_3g'])
print(sim['hover_1_bem_2.thrust_origin'])
# print(sim['vast_3g_nodal_forces.wing_mesh_plus_3g_oml_forces'])
print(sim['eb_beam_3g.Aframe.comp_model.wing_beam_displacement'])
print(sim['cruise_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.F'])
print(sim['eb_beam_3g.Aframe.comp_model.cg_vector'])

