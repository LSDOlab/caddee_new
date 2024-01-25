'''Example 1 : TC 1 analysis and optimization script

This advanced example demonstrates how to build the analysis script for the complex NASA ULI technical 
challenge 1 (TC1) problem, which modls three steady design conditions: Hover, Climb and Cruise. We use 
a combination of low-fidelity pyhscisc-based models for the aerodynamics, semi-empirical, and regression
models for the analysis. 

The central geometry along with meshes required for the analysis are imported from 'ex_lpc_geom.py'.
'''

# Module imports
import numpy as np 
import caddee.api as cd
from lsdo_rotor import BEMParameters, evaluate_multiple_BEM_models, BEM
from VAST import FluidProblem, VASTFluidSover
from lsdo_motor import evaluate_multiple_motor_sizing_models, evaluate_multiple_motor_analysis_models, MotorAnalysis, MotorSizing
from lsdo_acoustics import Acoustics, evaluate_multiple_acoustic_models
from python_csdl_backend import Simulator

# Imports from geometry setup file
from ex_lpc_geom import (system_model, lift_rotor_mesh_list, all_rotor_origin_list, 
                         fuselage_length, h_tail_area, v_tail_area, S_ref, wing_AR, 
                         wing_meshes, tail_meshes, pp_mesh, drag_comp_list)


# Setting flags for dvs
motor_sizing_dv = False
battery_sizing_dv = False
motor_analysis = False
acoustic_analysis = False

# Defining BEM parameters
bem_hover_rotor_parameters = BEMParameters(
    num_blades=2,
    num_radial=30,
    num_tangential=30,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)

bem_pusher_rotor_parameters = BEMParameters(
    num_blades=4,
    num_radial=30,
    num_tangential=1,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)

lift_rotation_direction_list = ['cw', 'ccw', 'cw', 'ccw', 'cw', 'ccw', 'cw', 'ccw']

# region sizing
# motors
motor_diameters = []
motor_lengths = []
for i in range(9):
    motor_diameters.append(system_model.create_input(f'motor_diameter_{i}', val=0.17, dv_flag=motor_sizing_dv, upper=0.25, lower=0.05, scaler=2))
    motor_lengths.append(system_model.create_input(f'motor_length_{i}', val=0.1, dv_flag=motor_sizing_dv, upper=0.15, lower=0.05, scaler=2))

motor_mass_properties = evaluate_multiple_motor_sizing_models(
    motor_diameter_list=motor_diameters,
    motor_length_list=motor_lengths,
    motor_origin_list=all_rotor_origin_list,
    name_prefix='motor_sizing',
    m3l_model=system_model,
)
# system_model.register_output(motor_mass_properties)

# battery
battery_mass = system_model.create_input(name='battery_mass', val=800, shape=(1, ), dv_flag=battery_sizing_dv, lower=500, upper=1100, scaler=3e-3)
energy_density = system_model.create_input(name='battery_energy_density', val=400, shape=(1, ))
battery_position = system_model.create_input(name='battery_position', val=np.array([3., 0., 2.]))

battery_model = cd.SimpleBatterySizing(name='simple_battery_model')
battery_mass_properties = battery_model.evaluate(battery_mass=battery_mass, battery_position=battery_position, battery_energy_density=energy_density)
system_model.register_output(battery_mass_properties)

# structural weight estimation
m4_regression = cd.M4Regressions(
    name='m4_regression',
    exclude_wing=False,
)
m4_mass_properties = m4_regression.evaluate(battery_mass=battery_mass, fuselage_length=fuselage_length, 
                                            tail_area=h_tail_area, fin_area=v_tail_area, wing_area=S_ref, wing_AR=wing_AR)
system_model.register_output(m4_mass_properties)

# Total mass properties
total_mass_props_model = cd.TotalMassPropertiesM3L(
    name="total_mass_properties_model"
)
total_mass_props = total_mass_props_model.evaluate(component_mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties])
system_model.register_output(total_mass_props)
system_model.add_objective(total_mass_props.mass, scaler=1e-3)
# endregion

# region hover condition
hover_condition = cd.HoverCondition(
    name='hover_condition',
)

# Create inputs for hover condition
hover_1_time = system_model.create_input('hover_1_time', val=90)
hover_1_altitude = system_model.create_input('hover_1_altitude', val=300)

# Evaluate aircraft states and atmospheric properties and register them as outputs
hover_1_ac_states, hover_1_atmosphere = hover_condition.evaluate(hover_time=hover_1_time, altitude=hover_1_altitude)
system_model.register_output(hover_1_ac_states)
system_model.register_output(hover_1_atmosphere)

hover_1_rpms = []
for i in range(8):
    hover_1_rpms.append(system_model.create_input(f'hover_1_rpm_{i}', val=1000, dv_flag=True, lower=300, upper=2200, scaler=1e-3))

hover_bem_output_list = evaluate_multiple_BEM_models(
    name_prefix='hover_1_bem',
    bem_parameters=bem_hover_rotor_parameters,
    bem_mesh_list=lift_rotor_mesh_list,
    rpm_list=hover_1_rpms,
    ac_states=hover_1_ac_states,
    atmoshpere=hover_1_atmosphere,
    num_nodes=1,
    m3l_model=system_model,
    rotation_direction_list=lift_rotation_direction_list,
    chord_cp=True,
    twist_cp=True,
)

if acoustic_analysis:
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
        atmos=hover_1_atmosphere,
        tonal_noise_model='Lowson',
        broadband_noise_model='GL',
        altitude=hover_1_altitude,
        rotor_parameters=bem_hover_rotor_parameters,
        rotor_meshes=lift_rotor_mesh_list,
        rpm_list=hover_1_rpms,
        model_name_prefix='hover_noise',
        num_nodes=1,
        m3l_model=system_model,
    )
    system_model.add_constraint(hover_total_noise_a_weighted, upper=70, scaler=1e-2)

if motor_analysis:
    hover_motor_outputs = evaluate_multiple_motor_analysis_models(
        rotor_outputs_list=hover_bem_output_list,
        motor_sizing_list=motor_mass_properties[0:-1],
        rotor_rpm_list=hover_1_rpms,
        motor_diameter_list=motor_diameters,
        name_prefix='hover_motor_analysis',
        flux_weakening=False,
        m3l_model=system_model,
    )

    hover_energy_model = cd.EnergyModelM3L(name='energy_hover')
    hover_energy = hover_energy_model.evaluate(
        hover_motor_outputs,
        ac_states=hover_1_ac_states,
    )
    system_model.register_output(hover_energy)


hover_trim_variables = hover_condition.assemble_trim_residual(
    mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
    aero_propulsive_outputs=hover_bem_output_list,
    ac_states=hover_1_ac_states,
)
system_model.register_output(hover_trim_variables)
system_model.add_constraint(hover_trim_variables.accelerations, equals=0)

# endregion

# region climb condition
climb_condition = cd.ClimbCondition(
    name='steady_climb',
    num_nodes=1,
)

climb_M = system_model.create_input('climb_mach', val=0.195)
climb_hi = system_model.create_input('climb_initial_altitude', val=300)
climb_hf = system_model.create_input('climb_final_altitude', val=1000)
climb_flight_path_angle = system_model.create_input(name='climb_flight_path_angle', val=np.deg2rad(4.588))
climb_pitch = system_model.create_input('climb_pitch', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(10), scaler=10)

climb_ac_states, climb_atmos = climb_condition.evaluate(mach_number=climb_M, pitch_angle=climb_pitch, flight_path_angle=climb_flight_path_angle,
                                                        initial_altitude=climb_hi, final_altitude=climb_hf)

system_model.register_output(climb_ac_states)
system_model.register_output(climb_atmos)

climb_bem = BEM(
    name='climb_bem',
    num_nodes=1, 
    BEM_parameters=bem_pusher_rotor_parameters,
    rotation_direction='ignore',
)
climb_rpm = system_model.create_input('climb_rpm', val=2000, dv_flag=True, lower=600, upper=2500, scaler=1e-3)
climb_bem_outputs = climb_bem.evaluate(ac_states=climb_ac_states, rpm=climb_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
                                                thrust_origin=pp_mesh.thrust_origin, atmosphere=climb_atmos, blade_chord_cp=pp_mesh.chord_cps, blade_twist_cp=pp_mesh.twist_cps, 
                                                cg_vec=m4_mass_properties.cg_vector, reference_point=m4_mass_properties.cg_vector)
system_model.register_output(climb_bem_outputs) 

# VAST solver
vlm_model = VASTFluidSover(
    name='climb_vlm_model',
    surface_names=[
        'climb_wing_mesh',
        'climb_tail_mesh',
    ],
    surface_shapes=[
        (1, ) + wing_meshes.vlm_mesh.shape[1:],
        (1, ) + tail_meshes.vlm_mesh.shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0., 0.]
)
elevator = system_model.create_input('climb_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
# Evaluate VLM outputs and register them as outputs
vlm_outputs = vlm_model.evaluate(
    atmosphere=climb_atmos,
    ac_states=climb_ac_states,
    meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
    deflections=[None, elevator],
    wing_AR=wing_AR,
    eval_pt=m4_mass_properties.cg_vector,
)
system_model.register_output(vlm_outputs)

drag_build_up_model = cd.DragBuildUpModel(
    name='climb_drag_build_up',
    num_nodes=1,
    units='ft',
)

drag_build_up_outputs = drag_build_up_model.evaluate(atmos=climb_atmos, ac_states=climb_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
system_model.register_output(drag_build_up_outputs)

if motor_analysis:
    climb_motor_outputs = evaluate_multiple_motor_analysis_models(
        rotor_outputs_list=[climb_bem_outputs],
        motor_sizing_list=[motor_mass_properties[-1]],
        rotor_rpm_list=[climb_rpm],
        motor_diameter_list=[motor_diameters[-1]],
        name_prefix='climb_motor_analysis',
        flux_weakening=False,
        m3l_model=system_model,
        torque_delta=0.2,
    )

    climb_energy_model = cd.EnergyModelM3L(name='energy_climb')
    climb_energy = climb_energy_model.evaluate(
        climb_motor_outputs,
        ac_states=climb_ac_states,
    )
    system_model.register_output(climb_energy)

climb_trim_variables = climb_condition.assemble_trim_residual(
    mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
    aero_propulsive_outputs=[vlm_outputs, climb_bem_outputs, drag_build_up_outputs],
    ac_states=climb_ac_states,
    load_factor=1.,
    ref_pt=m4_mass_properties.cg_vector,
)
system_model.register_output(climb_trim_variables)
system_model.add_constraint(climb_trim_variables.accelerations, equals=0, scaler=5.)
# endregion 

# region cruise
cruise_condition = cd.CruiseCondition(
    name='steady_cruise',
    num_nodes=1,
    stability_flag=False,
)

cruise_M = system_model.create_input('cruise_mach', val=0.195)
cruise_h = system_model.create_input('cruise_altitude', val=1000)
cruise_range = system_model.create_input('cruise_range', val=60000)
cruise_pitch = system_model.create_input('cruise_pitch', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(5), scaler=10)

cruise_ac_states, cruise_atmos = cruise_condition.evaluate(mach_number=cruise_M, pitch_angle=cruise_pitch, altitude=cruise_h, cruise_range=cruise_range)

system_model.register_output(cruise_ac_states)
system_model.register_output(cruise_atmos)

cruise_bem = BEM(
    name='cruise_bem',
    num_nodes=1, 
    BEM_parameters=bem_pusher_rotor_parameters,
    rotation_direction='ignore',
)
cruise_rpm = system_model.create_input('cruise_rpm', val=2000, dv_flag=True, lower=600, upper=2500, scaler=1e-3)
cruise_bem_outputs = cruise_bem.evaluate(ac_states=cruise_ac_states, rpm=cruise_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
                                                thrust_origin=pp_mesh.thrust_origin, atmosphere=cruise_atmos, blade_chord_cp=pp_mesh.chord_cps, blade_twist_cp=pp_mesh.twist_cps, 
                                                cg_vec=m4_mass_properties.cg_vector, reference_point=m4_mass_properties.cg_vector)
system_model.register_output(cruise_bem_outputs) 

# VAST solver
vlm_model = VASTFluidSover(
    name='cruise_vlm_model',
    surface_names=[
        'cruise_wing_mesh',
        'cruise_tail_mesh',
        # 'cruise_vtail_mesh',
        # 'cruise_fuselage_mesh',
    ],
    surface_shapes=[
        (1, ) + wing_meshes.vlm_mesh.shape[1:],
        (1, ) + tail_meshes.vlm_mesh.shape[1:],
        # (1, ) + vtail_meshes.vlm_mesh.shape[1:],
        # (1, ) + fuesleage_mesh.shape,
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0., 0., 0., 0.]
)
elevator = system_model.create_input('cruise_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
# Evaluate VLM outputs and register them as outputs
vlm_outputs = vlm_model.evaluate(
    atmosphere=cruise_atmos,
    ac_states=cruise_ac_states,
    meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh], #, vtail_meshes.vlm_mesh, fuesleage_mesh],
    deflections=[None, elevator], #, None, None],
    wing_AR=wing_AR,
    eval_pt=m4_mass_properties.cg_vector,
)
system_model.register_output(vlm_outputs)

drag_build_up_model = cd.DragBuildUpModel(
    name='cruise_drag_build_up',
    num_nodes=1,
    units='ft',
)

drag_build_up_outputs = drag_build_up_model.evaluate(atmos=cruise_atmos, ac_states=cruise_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
system_model.register_output(drag_build_up_outputs)

if motor_analysis:
    cruise_motor_outputs = evaluate_multiple_motor_analysis_models(
        rotor_outputs_list=[cruise_bem_outputs],
        motor_sizing_list=[motor_mass_properties[-1]],
        rotor_rpm_list=[cruise_rpm],
        motor_diameter_list=[motor_diameters[-1]],
        name_prefix='cruise_motor_analysis',
        flux_weakening=False,
        m3l_model=system_model,
    )

    cruise_energy_model = cd.EnergyModelM3L(name='energy_cruise')
    cruise_energy = cruise_energy_model.evaluate(
        cruise_motor_outputs,
        ac_states=cruise_ac_states,
    )
    system_model.register_output(cruise_energy)

cruise_trim_variables = cruise_condition.assemble_trim_residual(
    mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
    aero_propulsive_outputs=[vlm_outputs, cruise_bem_outputs, drag_build_up_outputs],
    ac_states=cruise_ac_states,
    load_factor=1.,
    ref_pt=m4_mass_properties.cg_vector,
)
system_model.register_output(cruise_trim_variables)
system_model.add_constraint(cruise_trim_variables.accelerations, equals=0, scaler=5.)

csdl_model = system_model.assemble_csdl()

sim = Simulator(csdl_model)
sim.run()

cd.print_caddee_outputs(system_model, sim)