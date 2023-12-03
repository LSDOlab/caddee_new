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
from modopt.snopt_library import SNOPT
from modopt.csdl_library import CSDLProblem
from lsdo_rotor import BEMParameters, evaluate_multiple_BEM_models, BEM, PittPeters, PittPetersParameters, evaluate_multiple_pitt_peters_models
from VAST import FluidProblem, VASTFluidSover, VASTNodalForces
from lsdo_acoustics import Acoustics, evaluate_multiple_acoustic_models
from lsdo_motor import evaluate_multiple_motor_sizing_models, evaluate_multiple_motor_analysis_models, MotorAnalysis, MotorSizing
from aframe import BeamMassModel, EBBeam, EBBeamForces
from caddee.utils.helper_functions.geometry_helpers import  make_vlm_camber_mesh


from ex_tc2_geometry_setup_updated import (wing_meshes, tail_meshes, box_beam_mesh, pp_mesh, rlo_mesh, 
                                           v_tail_meshes, pp_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh, num_wing_beam,
                                           box_beam_mesh, drag_comp_list, S_ref, wing_AR, system_model, geometry, rlo_disk, rli_disk, fuesleage_mesh, geometry)


caddee = cd.CADDEE()


sizing = True
acoustics = False
motor = False

# Set parameters for BEM analysis
num_radial = 30
num_tangential = 30

bem_hover_rotor_parameters = BEMParameters(
    num_blades=2,
    num_radial=num_radial,
    num_tangential=num_tangential,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)

pitt_peters_parameters = PittPetersParameters(
    num_blades=2,
    num_radial=num_radial,
    num_tangential=num_tangential,
    airfoil='NACA_4412',
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
rotation_direction_list = ['cw', 'ccw', 'cw', 'ccw', 'cw', 'ccw', 'cw', 'ccw']
num_rotors = len(rotor_mesh_list)

# region Sizing 
# Motors
motor_diameters = []
motor_lengths = []
for i in range(num_rotors):
    motor_diameters.append(system_model.create_input(f'motor_diameter_{i}', val=0.17, dv_flag=motor, upper=0.15, lower=0.05, scaler=2))
    motor_lengths.append(system_model.create_input(f'motor_length_{i}', val=0.1, dv_flag=motor, upper=0.12, lower=0.05, scaler=2))


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
youngs_modulus = 46E9 #72.4E9 #  
poisson_ratio = 0.33
shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
material_density = 1320  # 2780 # 

beams['wing_beam'] = {'E': youngs_modulus, 'G': shear_modulus, 'rho': material_density, 'cs': 'box', 'nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 10,'fdim': [1, 1, 1, 1, 1, 1]}

wing_beam_t_top = system_model.create_input(name='wing_beam_ttop' ,val=0.005 * np.ones((num_wing_beam, )), dv_flag=sizing, lower=0.00127, upper=0.1, scaler=10)
wing_beam_t_bot = system_model.create_input(name='wing_beam_tbot' ,val=0.005 * np.ones((num_wing_beam, )), dv_flag=sizing, lower=0.00127, upper=0.1, scaler=10)
wing_beam_tweb = system_model.create_input(name='wing_beam_tweb' ,val=0.005 * np.ones((num_wing_beam, )), dv_flag=sizing, lower=0.00127, upper=0.1, scaler=10)

beam_mass_model = BeamMassModel(
    beams=beams,
    name='wing_beam_mass_model',
)
wing_beam_mass_props = beam_mass_model.evaluate(beam_nodes=box_beam_mesh.beam_nodes,
                                        width=box_beam_mesh.width, height=box_beam_mesh.height, 
                                        t_top=wing_beam_t_top, t_bot=wing_beam_t_bot ,t_web=wing_beam_tweb)

system_model.register_output(wing_beam_mass_props)


# print(len(mass_properties))

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

total_mass_props_model = cd.TotalMassPropertiesM3L(
    name=f"total_mass_properties_model"
)
total_mass_props = total_mass_props_model.evaluate(component_mass_properties=[motor_mass_properties, wing_beam_mass_props, battery_mass_properties, m4_mass_properties])
system_model.register_output(total_mass_props)
if sizing:
    system_model.add_objective(total_mass_props.mass, scaler=1e-3)
# endregion

if sizing:
    # region +3g sizing
    sizing_3g_condition = cd.CruiseCondition(
        name='plus_3g_sizing',
        stability_flag=False,
    )

    h_3g = system_model.create_input('altitude_3g', val=1000)
    M_3g = system_model.create_input('mach_3g', val=0.24, dv_flag=False, lower=0.18, upper=0.26)
    r_3g = system_model.create_input('range_3g', val=10000)
    theta_3g = system_model.create_input('pitch_angle_3g', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=1e1)

    # ac sates + atmos
    ac_states_3g, atmos_3g = sizing_3g_condition.evaluate(mach_number=M_3g, pitch_angle=theta_3g, cruise_range=r_3g, altitude=h_3g)
    system_model.register_output(ac_states_3g)
    system_model.register_output(atmos_3g)

    # BEM solver
    plus_3g_pusher_rpm = system_model.create_input('plus_3g_pusher_rpm', val=2000, shape=(1, ), dv_flag=True, lower=1500, upper=3000, scaler=1e-3)
    plus_3g_bem_model = BEM(
        name='plus_3g_bem',
        num_nodes=1,
        BEM_parameters=bem_pusher_rotor_parameters,
        rotation_direction='ignore',
    )
    plus_3g_bem_outputs = plus_3g_bem_model.evaluate(ac_states=ac_states_3g, rpm=plus_3g_pusher_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
                                                    thrust_origin=pp_mesh.thrust_origin, atmosphere=atmos_3g, blade_chord=pp_mesh.chord_profile, blade_twist=pp_mesh.twist_profile, 
                                                    cg_vec=m4_mass_properties.cg_vector, reference_point=m4_mass_properties.cg_vector)

    system_model.register_output(plus_3g_bem_outputs)

    # VAST solver
    vlm_model = VASTFluidSover(
        name='plus_3g_vlm_model',
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
        cl0=[0., 0.]
    )

    plus_3g_elevator = system_model.create_input('plus_3g_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))


    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=atmos_3g,
        ac_states=ac_states_3g,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, plus_3g_elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
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
    # # system_model.register_output(tail_oml_forces)

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

    plus_3g_eb_beam_outputs = beam_displacement_model.evaluate(beam_mesh=box_beam_mesh, t_top=wing_beam_t_top, t_bot=wing_beam_t_bot, t_web=wing_beam_tweb, forces=structural_wing_mesh_forces_3g)
    system_model.add_constraint(plus_3g_eb_beam_outputs.bot_buckling, upper=1)
    system_model.add_constraint(plus_3g_eb_beam_outputs.top_buckling, upper=1)
    system_model.add_constraint(plus_3g_eb_beam_outputs.displacements, lower=-0.5, upper=0.5)
    system_model.add_constraint(plus_3g_eb_beam_outputs.stresses, upper=600E6/1.5, scaler=0.5e-8)
    system_model.register_output(plus_3g_eb_beam_outputs)
    # # NOTE:
    # # We are creating a new MassProperties object for the wing beam where
    # # we combine the cg and inertia tensor that the beam analysis model
    # # outputs with the beam mass, which is based on the skin, and spar
    # # thicknesses. This is because the +3g condition will size the top
    # # skin while the -1g condition will size the bottom skin (if buckling
    # # is considered).
    wing_beam_mass_props = cd.MassProperties(
        mass=plus_3g_eb_beam_outputs.mass,
        cg_vector=plus_3g_eb_beam_outputs.cg_vector,
        inertia_tensor=plus_3g_eb_beam_outputs.inertia_tensor,
    )


    drag_build_up_model = cd.DragBuildUpModel(
        name='sizing_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=atmos_3g, ac_states=ac_states_3g, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    plus_3g_trim_variables = sizing_3g_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        # mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
        aero_propulsive_outputs=[vlm_outputs, plus_3g_bem_outputs, drag_build_up_outputs],
        ac_states=ac_states_3g,
        load_factor=3.,
        ref_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(plus_3g_trim_variables)
    system_model.add_constraint(plus_3g_trim_variables.accelerations, equals=0.)
    # endregion

    # region -1g sizing
    sizing_minus_1g_condition = cd.CruiseCondition(
        name='minus_1g_sizing',
        stability_flag=False,
    )

    h_minus_1g = system_model.create_input('altitude_minus_1g', val=1000)
    M_minus_1g = system_model.create_input('mach_minus_1g', val=0.24, dv_flag=False, lower=0.18, upper=0.26)
    r_minus_1g = system_model.create_input('range_minus_1g', val=10000)
    theta_minus_1g = system_model.create_input('pitch_angle_minus_1g', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=1e1)

    # ac sates + atmos
    ac_states_minus_1g, atmos_minus_1g = sizing_minus_1g_condition.evaluate(mach_number=M_minus_1g, pitch_angle=theta_minus_1g, cruise_range=r_minus_1g, altitude=h_minus_1g)
    system_model.register_output(ac_states_minus_1g)
    system_model.register_output(atmos_minus_1g)

    # BEM solver
    minus_1g_pusher_rpm = system_model.create_input('minus_1g_pusher_rpm', val=2000, shape=(1, ), dv_flag=True, lower=1500, upper=3000, scaler=1e-3)
    minus_1g_bem_model = BEM(
        name='minus_1g_bem',
        num_nodes=1,
        BEM_parameters=bem_pusher_rotor_parameters,
        rotation_direction='ignore',
    )
    minus_1g_bem_outputs = minus_1g_bem_model.evaluate(ac_states=ac_states_minus_1g, rpm=minus_1g_pusher_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
                                                    thrust_origin=pp_mesh.thrust_origin, atmosphere=atmos_minus_1g, blade_chord=pp_mesh.chord_profile, blade_twist=pp_mesh.twist_profile, 
                                                    cg_vec=m4_mass_properties.cg_vector, reference_point=m4_mass_properties.cg_vector)

    system_model.register_output(minus_1g_bem_outputs)

    # VAST solver
    vlm_model = VASTFluidSover(
        name='minus_1g_vlm_model',
        surface_names=[
            'wing_mesh_minus_1g',
            'tail_mesh_minus_1g',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )

    minus_1g_elevator = system_model.create_input('minus_1g_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=atmos_minus_1g,
        ac_states=ac_states_minus_1g,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, minus_1g_elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    # Nodal forces
    vlm_force_mapping_model = VASTNodalForces(
        name='vast_minus_1g_nodal_forces',
        surface_names=[
            f'wing_mesh_minus_1g',
            f'tail_mesh_minus_1g',
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

    # oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_outputs.panel_forces, nodal_force_meshes=[wing_meshes.oml_mesh])
    oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_outputs.panel_forces, nodal_force_meshes=[wing_meshes.oml_mesh, tail_meshes.oml_mesh])
    wing_oml_forces = oml_forces[0]
    tail_oml_forces = oml_forces[1]


    system_model.register_output(wing_oml_forces)
    # # system_model.register_output(tail_oml_forces)

    beam_force_map_model = EBBeamForces(
        name='eb_beam_force_map_minus_1g',
        beams=beams,
        exclude_middle=True,
    )

    structural_wing_mesh_forces_minus_1g = beam_force_map_model.evaluate(
        beam_mesh=box_beam_mesh.beam_nodes,
        nodal_forces=wing_oml_forces,
        nodal_forces_mesh=wing_meshes.oml_mesh
    )

    beam_displacement_model = EBBeam(
        name='eb_beam_minus_1g',
        beams=beams,
        bounds=bounds,
        joints=joints,
        mesh_units='ft',
    )

    minus_1g_eb_beam_outputs = beam_displacement_model.evaluate(beam_mesh=box_beam_mesh, t_top=wing_beam_t_top, t_bot=wing_beam_t_bot, t_web=wing_beam_tweb, forces=structural_wing_mesh_forces_minus_1g)
    system_model.add_constraint(minus_1g_eb_beam_outputs.bot_buckling, upper=1)
    system_model.add_constraint(minus_1g_eb_beam_outputs.top_buckling, upper=1)
    system_model.add_constraint(minus_1g_eb_beam_outputs.displacements, lower=-0.5, upper=0.5)
    system_model.add_constraint(minus_1g_eb_beam_outputs.stresses, upper=600E6/1.5, scaler=0.5e-8)
    system_model.register_output(minus_1g_eb_beam_outputs)
    # # NOTE:
    # # We are creating a new MassProperties object for the wing beam where
    # # we combine the cg and inertia tensor that the beam analysis model
    # # outputs with the beam mass, which is based on the skin, and spar
    # # thicknesses. This is because the +3g condition will size the top
    # # skin while the -1g condition will size the bottom skin (if buckling
    # # is considered).
    wing_beam_mass_props = cd.MassProperties(
        mass=minus_1g_eb_beam_outputs.mass,
        cg_vector=minus_1g_eb_beam_outputs.cg_vector,
        inertia_tensor=minus_1g_eb_beam_outputs.inertia_tensor,
    )


    drag_build_up_model = cd.DragBuildUpModel(
        name='minus_1g_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=atmos_minus_1g, ac_states=ac_states_minus_1g, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    minus_1g_trim_variables = sizing_minus_1g_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        # mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
        # aero_propulsive_outputs=[vlm_outputs, minus_1g_bem_outputs],
        aero_propulsive_outputs=[vlm_outputs, minus_1g_bem_outputs, drag_build_up_outputs],
        ac_states=ac_states_minus_1g,
        load_factor=-1.,
        ref_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(minus_1g_trim_variables)
    system_model.add_constraint(minus_1g_trim_variables.accelerations, equals=0.)
    # endregion


# # region cruise
# cruise_condition = cd.CruiseCondition(
#     name='steady_cruise',
#     num_nodes=1,
#     stability_flag=False,
# )

# cruise_M = system_model.create_input('cruise_mach', val=0.195)
# cruise_h = system_model.create_input('cruise_altitude', val=1000)
# cruise_range = system_model.create_input('cruise_range', val=40000)
# cruise_pitch = system_model.create_input('cruise_pitch', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(15), scaler=10)

# cruise_ac_states, cruise_atmos = cruise_condition.evaluate(mach_number=cruise_M, pitch_angle=cruise_pitch, altitude=cruise_h, cruise_range=cruise_range)

# system_model.register_output(cruise_ac_states)
# system_model.register_output(cruise_atmos)

# cruise_bem = BEM(
#     name='cruise_bem',
#     num_nodes=1, 
#     BEM_parameters=bem_pusher_rotor_parameters,
#     rotation_direction='ignore',
# )
# cruise_rpm = system_model.create_input('cruise_rpm', val=1200, dv_flag=True, lower=600, upper=2500, scaler=1e-3)
# cruise_bem_outputs = cruise_bem.evaluate(ac_states=cruise_ac_states, rpm=cruise_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
#                                                  thrust_origin=pp_mesh.thrust_origin, atmosphere=cruise_atmos, blade_chord=pp_mesh.chord_profile, blade_twist=pp_mesh.twist_profile, 
#                                                 cg_vec=m4_mass_properties.cg_vector, reference_point=m4_mass_properties.cg_vector)
# system_model.register_output(cruise_bem_outputs) 

# # VAST solver
# vlm_model = VASTFluidSover(
#     name='cruise_vlm_model',
#     surface_names=[
#         'cruise_wing_mesh',
#         'cruise_tail_mesh',
#         'cruise_vtail_mesh',
#         # 'cruise_fuselage_mesh',
#     ],
#     surface_shapes=[
#         (1, ) + wing_meshes.vlm_mesh.shape[1:],
#         (1, ) + tail_meshes.vlm_mesh.shape[1:],
#         (1, ) + v_tail_meshes.vlm_mesh.shape[1:],
#         # (1, ) + fuesleage_mesh.shape,
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0., 0., 0.]
# )
# elevator = system_model.create_input('cruise_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
# # Evaluate VLM outputs and register them as outputs
# vlm_outputs = vlm_model.evaluate(
#     atmosphere=cruise_atmos,
#     ac_states=cruise_ac_states,
#     meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh, v_tail_meshes.vlm_mesh], #, fuesleage_mesh],
#     deflections=[None, elevator, None],
#     wing_AR=wing_AR,
#     eval_pt=m4_mass_properties.cg_vector,
# )
# system_model.register_output(vlm_outputs)

# drag_build_up_model = cd.DragBuildUpModel(
#     name='cruise_drag_build_up',
#     num_nodes=1,
#     units='ft',
# )

# drag_build_up_outputs = drag_build_up_model.evaluate(atmos=cruise_atmos, ac_states=cruise_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
# system_model.register_output(drag_build_up_outputs)

# cruise_trim_variables = cruise_condition.assemble_trim_residual(
#     mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
#     aero_propulsive_outputs=[vlm_outputs, cruise_bem_outputs, drag_build_up_outputs],
#     ac_states=cruise_ac_states,
#     load_factor=3.,
#     ref_pt=m4_mass_properties.cg_vector,
# )
# system_model.register_output(cruise_trim_variables)
# # system_model.add_constraint(cruise_trim_variables.accelerations, equals=0)
# # endregion


# region Hover condition
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
#     hover_1_rpms.append(system_model.create_input(f'hover_1_rpm_{i}', val=1200, dv_flag=True, lower=600, upper=2200, scaler=1e-3))

# hover_bem_output_list = evaluate_multiple_BEM_models(
#     name_prefix='hover_1_bem',
#     bem_parameters=bem_hover_rotor_parameters,
#     bem_mesh_list=rotor_mesh_list,
#     rpm_list=hover_1_rpms,
#     ac_states=hover_1_ac_states,
#     atmoshpere=hover_1_atmosphere,
#     num_nodes=1,
#     m3l_model=system_model,
#     rotation_direction_list=rotation_direction_list,
# )

# if acoustics:
#     hover_acoustics_data = Acoustics(
#         aircraft_position=np.array([0., 0., 100])
#     )

#     hover_acoustics_data.add_observer(
#         name='hover_observer',
#         obs_position=np.array([0., 0., 0.]),
#         time_vector=np.array([0.,]),

#     )

#     hover_total_noise, hover_total_noise_a_weighted = evaluate_multiple_acoustic_models(
#         rotor_outputs=hover_bem_output_list,
#         acoustics_data=hover_acoustics_data,
#         ac_states=hover_1_ac_states,
#         tonal_noise_model='Lowson',
#         broadband_noise_model='GL',
#         altitude=hover_1_altitude,
#         rotor_parameters=bem_hover_rotor_parameters,
#         rotor_meshes=rotor_mesh_list,
#         rpm_list=hover_1_rpms,
#         model_name_prefix='hover_noise',
#         num_nodes=1,
#         m3l_model=system_model,
#     )
#     system_model.add_constraint(hover_total_noise_a_weighted, upper=70, scaler=1e-2)

# if motor:
#     hover_motor_outputs = evaluate_multiple_motor_analysis_models(
#         rotor_outputs_list=hover_bem_output_list,
#         motor_sizing_list=motor_mass_properties,
#         rotor_rpm_list=hover_1_rpms,
#         motor_diameter_list=motor_diameters,
#         name_prefix='hover_motor_analysis',
#         flux_weakening=False,
#         m3l_model=system_model,
#     )


# hover_trim_variables = hover_condition.assemble_trim_residual(
#     mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
#     aero_propulsive_outputs=hover_bem_output_list,
#     ac_states=hover_1_ac_states,
# )
# system_model.register_output(hover_trim_variables)
# system_model.add_constraint(hover_trim_variables.accelerations, equals=0)

# endregion


if sizing is False:
# if True:
    objective = plus_3g_trim_variables.accelerations  +  minus_1g_trim_variables.accelerations #+ cruise_trim_variables.accelerations
    # objective =  cruise_trim_variables.accelerations + cruise_2_trim_variables.accelerations  # + minus_1g_trim_variables.accelerations 
    # objective = cruise_trim_variables.accelerations 
    system_model.register_output(objective)
    system_model.add_objective(objective)


# region qst1
# qst1 = cd.CruiseCondition(
#     name='qst_1',
#     num_nodes=1,
#     stability_flag=False
# )

# qst1_mach = system_model.create_input('qst_1_mach', val=0.05)
# qst1_altitude = system_model.create_input('qst_1_altitude', val=1000)
# qst1_pitch_angle = system_model.create_input('qst_1_pitch_angle', val=0.)
# qst1_range = system_model.create_input('qst_1_range', val=500)

# qst_1_ac_states, qst_1_atmos = qst1.evaluate(mach_number=qst1_mach, pitch_angle=qst1_pitch_angle, altitude=qst1_altitude, cruise_range=qst1_range)
# system_model.register_output(qst_1_ac_states)
# system_model.register_output(qst_1_atmos)

# qst_1_rpms = []
# for i in range(num_rotors):
#     qst_1_rpms.append(system_model.create_input(f'qst_1_rpm_{i}', val=1200, dv_flag=True, lower=800, upper=2000, scaler=1e-3))

# qst1_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
#     num_instances=8,
#     name_prefix='qst_1_pitt_peters',
#     pitt_peters_parameters=pitt_peters_parameters,
#     pitt_peters_mesh_list=rotor_mesh_list,
#     rpm_list=qst_1_rpms,
#     ac_states=qst_1_ac_states,
#     atmoshpere=qst_1_atmos,
#     m3l_model=system_model,
# )

# # VAST solver
# vlm_model = VASTFluidSover(
#     name='qst1_vlm_model',
#     surface_names=[
#         'wing_mesh_qst_1',
#         'tail_mesh_qst_1',
#         'vtail_mesh_qst_1',
#         'fuselage_mesh_qst_1',
#     ],
#     surface_shapes=[
#         (1, ) + wing_meshes.vlm_mesh.shape[1:],
#         (1, ) + tail_meshes.vlm_mesh.shape[1:],
#         (1, ) + v_tail_meshes.vlm_mesh.shape[1:],
#         (1, ) + fuesleage_mesh.shape,
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0., 0., 0., 0.]
# )

# # Evaluate VLM outputs and register them as outputs
# vlm_outputs = vlm_model.evaluate(
#     atmosphere=qst_1_atmos,
#     ac_states=qst_1_ac_states,
#     meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh, v_tail_meshes.vlm_mesh, fuesleage_mesh],
#     wing_AR=wing_AR,
#     eval_pt=m4_mass_properties.cg_vector,
# )
# system_model.register_output(vlm_outputs)

# qst_1_trim_variables = qst1.assemble_trim_residual(
#     mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
#     aero_propulsive_outputs=[vlm_outputs, qst1_pitt_peters_outputs],
#     ac_states=qst_1_ac_states,
# )

# system_model.register_output(qst_1_trim_variables)
# system_model.add_objective(qst_1_trim_variables.accelerations)

# endregion


caddee_csdl_model = system_model.assemble_csdl()
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()




# print(sim['cruise_drag_build_up.comp_0_s_wet'])
# print(sim['cruise_drag_build_up.comp_1_s_wet'])
# print(sim['cruise_drag_build_up.comp_2_s_wet'])
# print(sim['cruise_drag_build_up.comp_3_s_wet'])
# print(sim['cruise_drag_build_up.comp_4_s_wet'])
# print(sim['cruise_drag_build_up.comp_5_s_wet'])
# print(sim['cruise_drag_build_up.comp_6_s_wet'])
# print(sim['cruise_drag_build_up.sref_test'])
# print(sim['cruise_2_vlm_model.cruise_2_vlm_model.VLMSolverModel.VLM_outputs.LiftDrag.wing_AR'])
# print(sim['cruise_2_vlm_model.wing_AR_exp'])
# print(sim['5495_division_5494_operation.5496'])
# print(sim['5441_reshape_operation.5441_reshaped'])
# print(np.mean(sim['LPC_final_custom_blades_3_coefficients']))
# print(sim['87_reshaped_rotated_by_85_about_58_at_point_55_operation.rotated_points_flattened'])
# exit()
# sim.check_totals(f'{objective.operation.name}.{objective.name}', 'cruise_tail_act_angle')
# sim.check_totals(f'{objective.operation.name}.{objective.name}', 'plus_3g_tail_act_angle')
# sim.check_totals(f'{objective.operation.name}.{objective.name}', 'minus_1g_tail_act_angle')
# 
prob = CSDLProblem(problem_name='TC2_new_caddee_test', simulator=sim)

# optimizer = SNOPT(
#         prob, 
#         Major_iterations=50, 
#         Major_optimality=1e-5, 
#         Major_feasibility=1e-4,
#         append2file=True,
#         Iteration_limit=500000,
#         Major_step_limit= 1.5,
#         Linesearch_tolerance=0.6,
#     )

optimizer = SLSQP(prob, maxiter=125, ftol=1E-7)
optimizer.solve()
cd.print_caddee_outputs(system_model, sim, compact_print=False)


exit()
# Create a function space for the quantity of interest

ux = sim['qst_1_pitt_peters_1._dT'].reshape((num_radial, num_tangential))

mesh = rli_mesh.disk_mesh_physical.reshape((num_radial, num_tangential, 3))

x = mesh.value[:, :, 0]
y = mesh.value[:, :, 1]
z = mesh.value[:, :, 2]

import matplotlib.pyplot as plt

# Create a scatter plot with color-coded points based on temperature
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(x, y, z, c=ux, cmap='viridis')
cbar = plt.colorbar(sc)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

print(sim['qst_1_pitt_peters_0.T'])
print(sim['qst_1_pitt_peters_0.F'])

plt.show()
# 
ux = sim['qst_1_pitt_peters_1._dT']


disk_ux_space = geometry.space.create_sub_space(sub_space_name='wing_lift_space', b_spline_names=rli_disk.b_spline_names)

print(disk_ux_space.spaces)


for space_name, space in disk_ux_space.spaces.items():
    space.order = (3, 3)

# Fit a b-spline based on the VLM data by solving a least-squares problem to find the control points (i.e., coefficients)
pressure_coefficients = disk_ux_space.fit_b_spline_set(fitting_points=ux.reshape((-1, 1)), fitting_parametric_coordinates=rli_mesh.disk_mesh_parametric, regularization_parameter=1e-2)

# Create a function from the function space
wing_pressure_function = disk_ux_space.create_function(name='left_wing_pressure_function', 
                                                                    coefficients=pressure_coefficients, num_physical_dimensions=1)

rli_disk.plot(color=wing_pressure_function)
# sim.check_totals()
# 




# cd.print_caddee_outputs(system_model, sim)
# print(sim['plus_3g_sizing_total_mass_properties_model.total_cg_vector'])

# print(sim['cruise_tail_act_angle'])
# print(sim['plus_3g_sizing.theta']*180/np.pi)
# print(sim['plus_3g_pusher_rpm'])
