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
import time 
import pickle


from ex_tc2_geometry_setup_updated import (wing_meshes, tail_meshes, box_beam_mesh, rlo_mesh, h_tail_area, v_tail_area, fuselage_length,
                                           vtail_meshes, pp_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh, num_wing_beam,
                                           box_beam_mesh, drag_comp_list, S_ref, wing_AR, system_model, geometry, rlo_disk, rli_disk, fuesleage_mesh, geometry)

# from ex_tc2_geometry_setup_updated import (wing_meshes, tail_meshes, box_beam_mesh, pp_mesh, rlo_mesh, 
#                                            pp_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh, num_wing_beam,
#                                            box_beam_mesh, drag_comp_list, S_ref, wing_AR, system_model, geometry, rlo_disk, rli_disk, fuesleage_mesh, geometry)

t2 = time.time()
t3 = time.time()
caddee = cd.CADDEE()

sizing = False
oei = False
hover = True
quasi_steady_transition = False
cruise = True
climb = True
descent = True

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

bem_hover_rotor_parameters_oei = BEMParameters(
    num_blades=2,
    num_radial=num_radial,
    num_tangential=1,
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
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)

bem_pusher_rotor_parameters_acoustics = BEMParameters(
    num_blades=4,
    num_radial=num_radial,
    num_tangential=num_tangential,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)

rotor_mesh_list = [rlo_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh]
rotor_mesh_list_oei_flo = [rlo_mesh, rli_mesh, rri_mesh, rro_mesh, fli_mesh, fri_mesh, fro_mesh]
rotor_mesh_list_oei_fli = [rlo_mesh, rli_mesh, rri_mesh, rro_mesh, flo_mesh, fri_mesh, fro_mesh]
rotor_mesh_list_oei_rlo = [rli_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh]
rotor_mesh_list_oei_rli = [rlo_mesh, rri_mesh, rro_mesh, flo_mesh, fli_mesh, fri_mesh, fro_mesh]

origin_list = [mesh.thrust_origin for mesh in rotor_mesh_list]
rotation_direction_list = ['cw', 'ccw', 'cw', 'ccw', 'cw', 'ccw', 'cw', 'ccw']
num_rotors = len(rotor_mesh_list)

origin_list_plus_pusher = origin_list + [pp_mesh.thrust_origin]

# region Sizing 
# Motors
motor_diameters = []
motor_lengths = []
for i in range(num_rotors + 1):
    motor_diameters.append(system_model.create_input(f'motor_diameter_{i}', val=0.17, dv_flag=motor, upper=0.25, lower=0.05, scaler=2))
    motor_lengths.append(system_model.create_input(f'motor_length_{i}', val=0.1, dv_flag=motor, upper=0.15, lower=0.05, scaler=2))


motor_mass_properties = evaluate_multiple_motor_sizing_models(
    motor_diameter_list=motor_diameters,
    motor_length_list=motor_lengths,
    motor_origin_list=origin_list_plus_pusher,
    name_prefix='motor_sizing',
    m3l_model=system_model,
)

# Beam sizing 
# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
youngs_modulus = 72.4E9 #  46E9 #
poisson_ratio = 0.33
shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
material_density = 2780 #  1320  # 

beams['wing_beam'] = {'E': youngs_modulus, 'G': shear_modulus, 'rho': material_density, 'cs': 'box', 'nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 10,'fdim': [1, 1, 1, 1, 1, 1]}
# bounds['wing_root'] = {'beam': 'wing_beam','node': 2, 'fdim': [1, 1, 1, 1, 1, 1]}

wing_beam_t_top = system_model.create_input(name='wing_beam_ttop' ,val=0.005 * np.ones((num_wing_beam, )), dv_flag=sizing, lower=0.0008, upper=0.02, scaler=10) # lower=0.00127
wing_beam_t_bot = system_model.create_input(name='wing_beam_tbot' ,val=0.005 * np.ones((num_wing_beam, )), dv_flag=sizing, lower=0.0008, upper=0.02, scaler=10)
wing_beam_tweb = system_model.create_input(name='wing_beam_tweb' ,val=0.005 * np.ones((num_wing_beam, )), dv_flag=sizing, lower=0.000508, upper=0.02, scaler=10)

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
battery_mass = system_model.create_input(name='battery_mass', val=800, shape=(1, ), dv_flag=motor, lower=500, upper=1100, scaler=3e-3)
energy_density = system_model.create_input(name='battery_energy_density', val=400, shape=(1, ))
battery_position = system_model.create_input(name='battery_position', val=np.array([3., 0., 2.]))

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
m4_mass_properties = m4_regression.evaluate(battery_mass=battery_mass, fuselage_length=fuselage_length, 
                                            tail_area=h_tail_area, fin_area=v_tail_area, wing_area=S_ref, wing_AR=wing_AR)
system_model.register_output(m4_mass_properties)

total_mass_props_model = cd.TotalMassPropertiesM3L(
    name=f"total_mass_properties_model"
)
total_mass_props = total_mass_props_model.evaluate(component_mass_properties=[motor_mass_properties, wing_beam_mass_props, battery_mass_properties, m4_mass_properties])
system_model.register_output(total_mass_props)
if sizing: pass


system_model.add_objective(total_mass_props.mass, scaler=1e-3)
# endregion

if sizing:
    # region +3g sizing
    sizing_3g_condition = cd.CruiseCondition(
        name='plus_3g_sizing',
        stability_flag=False,
    )

    h_3g = system_model.create_input('altitude_3g', val=1000)
    M_3g = system_model.create_input('mach_3g', val=0.25, dv_flag=False, lower=0.18, upper=0.26)
    r_3g = system_model.create_input('range_3g', val=10000)
    theta_3g = system_model.create_input('pitch_angle_3g', val=np.deg2rad(10), dv_flag=True, lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=1e1)

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
                                                    thrust_origin=pp_mesh.thrust_origin, atmosphere=atmos_3g, blade_chord_cp=pp_mesh.chord_cps, blade_twist_cp=pp_mesh.twist_cps, 
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
    # system_model.add_constraint(plus_3g_eb_beam_outputs.bot_buckling, upper=1)
    system_model.add_constraint(plus_3g_eb_beam_outputs.top_buckling, upper=1.3)
    system_model.add_constraint(plus_3g_eb_beam_outputs.displacements, lower=-0.5, upper=0.5)
    system_model.add_constraint(plus_3g_eb_beam_outputs.stresses, upper=350E6/1., scaler=0.5e-8)
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

    if False:
        plus_3_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=[plus_3g_bem_outputs],
            motor_sizing_list=[motor_mass_properties[-1]],
            rotor_rpm_list=[plus_3g_pusher_rpm],
            motor_diameter_list=[motor_diameters[-1]],
            name_prefix='plus_3g_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
            torque_delta=0.1,
        )

    plus_3g_trim_variables = sizing_3g_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        # mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties],
        aero_propulsive_outputs=[vlm_outputs, plus_3g_bem_outputs, drag_build_up_outputs],
        ac_states=ac_states_3g,
        load_factor=3.,
        ref_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(plus_3g_trim_variables)
    system_model.add_constraint(plus_3g_trim_variables.accelerations, equals=0., scaler=8)
    # endregion

    # region -1g sizing
    sizing_minus_1g_condition = cd.CruiseCondition(
        name='minus_1g_sizing',
        stability_flag=False,
    )

    h_minus_1g = system_model.create_input('altitude_minus_1g', val=1000)
    M_minus_1g = system_model.create_input('mach_minus_1g', val=0.25, dv_flag=False, lower=0.18, upper=0.26)
    r_minus_1g = system_model.create_input('range_minus_1g', val=10000)
    theta_minus_1g = system_model.create_input('pitch_angle_minus_1g', val=np.deg2rad(-10), dv_flag=True, lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=1e1)

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
                                                    thrust_origin=pp_mesh.thrust_origin, atmosphere=atmos_minus_1g, blade_chord_cp=pp_mesh.chord_cps, blade_twist_cp=pp_mesh.twist_cps, 
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
    system_model.add_constraint(minus_1g_eb_beam_outputs.bot_buckling, upper=1.3)
    # system_model.add_constraint(minus_1g_eb_beam_outputs.top_buckling, upper=1.)
    system_model.add_constraint(minus_1g_eb_beam_outputs.displacements, lower=-0.5, upper=0.5)
    system_model.add_constraint(minus_1g_eb_beam_outputs.stresses, upper=350E6/1., scaler=0.5e-8)
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
    system_model.add_constraint(minus_1g_trim_variables.accelerations, equals=0., scaler=8)
    # endregion

if oei:
    # region oei flo condition
    oei_flo_condition = cd.HoverCondition(
        name='oei_flo_condition_1',
    )

    # Create inputs for hover condition
    oei_flo_1_time = system_model.create_input('oei_flo_1_time', val=90)
    oei_flo_1_altitude = system_model.create_input('oei_flo_1_altitude', val=100)

    # Evaluate aircraft states and atmospheric properties and register them as outputs
    oei_flo_1_ac_states, oei_flo_1_atmosphere = oei_flo_condition.evaluate(hover_time=oei_flo_1_time, altitude=oei_flo_1_altitude)
    system_model.register_output(oei_flo_1_ac_states)
    system_model.register_output(oei_flo_1_atmosphere)

    oei_flo_1_rpms = []
    for i in range(num_rotors-1):
        oei_flo_1_rpms.append(system_model.create_input(f'oei_flo_1_rpm_{i}', val=1000, dv_flag=True, lower=500, upper=2500, scaler=1e-3))

    oei_flo_bem_output_list = evaluate_multiple_BEM_models(
        name_prefix='oei_flo_1_bem',
        bem_parameters=bem_hover_rotor_parameters_oei,
        bem_mesh_list=rotor_mesh_list_oei_flo,
        rpm_list=oei_flo_1_rpms,
        ac_states=oei_flo_1_ac_states,
        atmoshpere=oei_flo_1_atmosphere,
        num_nodes=1,
        m3l_model=system_model,
        rotation_direction_list=rotation_direction_list[:4] + rotation_direction_list[4 + 1:],
        chord_cp=True,
        twist_cp=True,
    )

    if motor:
        oei_flo_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=oei_flo_bem_output_list,
            motor_sizing_list=motor_mass_properties[:4] + motor_mass_properties[4 + 1:-1],
            rotor_rpm_list=oei_flo_1_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='oei_flo_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
            torque_delta=0.1,
        )


    oei_flo_trim_variables = oei_flo_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=oei_flo_bem_output_list,
        ac_states=oei_flo_1_ac_states,
    )
    system_model.register_output(oei_flo_trim_variables)
    system_model.add_constraint(oei_flo_trim_variables.accelerations, equals=0, scaler=5)

    # endregion

    # region oei fli condition
    oei_fli_condition = cd.HoverCondition(
        name='oei_fli_condition_1',
    )

    # Create inputs for hover condition
    oei_fli_1_time = system_model.create_input('oei_fli_1_time', val=90)
    oei_fli_1_altitude = system_model.create_input('oei_fli_1_altitude', val=100)

    # Evaluate aircraft states and atmospheric properties and register them as outputs
    oei_fli_1_ac_states, oei_fli_1_atmosphere = oei_fli_condition.evaluate(hover_time=oei_fli_1_time, altitude=oei_fli_1_altitude)
    system_model.register_output(oei_fli_1_ac_states)
    system_model.register_output(oei_fli_1_atmosphere)

    oei_fli_1_rpms = []
    for i in range(num_rotors-1):
        oei_fli_1_rpms.append(system_model.create_input(f'oei_fli_1_rpm_{i}', val=1000, dv_flag=True, lower=500, upper=2500, scaler=1e-3))

    oei_fli_bem_output_list = evaluate_multiple_BEM_models(
        name_prefix='oei_fli_1_bem',
        bem_parameters=bem_hover_rotor_parameters_oei,
        bem_mesh_list=rotor_mesh_list_oei_fli,
        rpm_list=oei_fli_1_rpms,
        ac_states=oei_fli_1_ac_states,
        atmoshpere=oei_fli_1_atmosphere,
        num_nodes=1,
        m3l_model=system_model,
        rotation_direction_list=rotation_direction_list[:5] + rotation_direction_list[5 + 1:],
        chord_cp=True,
        twist_cp=True,
    )

    if motor:
        oei_fli_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=oei_fli_bem_output_list,
            motor_sizing_list=motor_mass_properties[:5] + motor_mass_properties[5 + 1:-1],
            rotor_rpm_list=oei_fli_1_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='oei_fli_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
            torque_delta=0.1,
        )


    oei_fli_trim_variables = oei_fli_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=oei_fli_bem_output_list,
        ac_states=oei_fli_1_ac_states,
    )
    system_model.register_output(oei_fli_trim_variables)
    system_model.add_constraint(oei_fli_trim_variables.accelerations, equals=0, scaler=5)

    # endregion

    # region oei rlo condition
    oei_rlo_condition = cd.HoverCondition(
        name='oei_rlo_condition_1',
    )

    # Create inputs for hover condition
    oei_rlo_1_time = system_model.create_input('oei_rlo_1_time', val=90)
    oei_rlo_1_altitude = system_model.create_input('oei_rlo_1_altitude', val=100)

    # Evaluate aircraft states and atmospheric properties and register them as outputs
    oei_rlo_1_ac_states, oei_rlo_1_atmosphere = oei_rlo_condition.evaluate(hover_time=oei_rlo_1_time, altitude=oei_rlo_1_altitude)
    system_model.register_output(oei_rlo_1_ac_states)
    system_model.register_output(oei_rlo_1_atmosphere)

    oei_rlo_1_rpms = []
    for i in range(num_rotors-1):
        oei_rlo_1_rpms.append(system_model.create_input(f'oei_rlo_1_rpm_{i}', val=1000, dv_flag=True, lower=500, upper=2500, scaler=1e-3))

    oei_rlo_bem_output_list = evaluate_multiple_BEM_models(
        name_prefix='oei_rlo_1_bem',
        bem_parameters=bem_hover_rotor_parameters_oei,
        bem_mesh_list=rotor_mesh_list_oei_rlo,
        rpm_list=oei_rlo_1_rpms,
        ac_states=oei_rlo_1_ac_states,
        atmoshpere=oei_rlo_1_atmosphere,
        num_nodes=1,
        m3l_model=system_model,
        rotation_direction_list= rotation_direction_list[:0] + rotation_direction_list[0 + 1:],
        chord_cp=True,
        twist_cp=True,
    )

    if motor:
        oei_rlo_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=oei_rlo_bem_output_list,
            motor_sizing_list=motor_mass_properties[:0] + motor_mass_properties[0 + 1:-1],
            rotor_rpm_list=oei_rlo_1_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='oei_rlo_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
            torque_delta=0.1,
        )


    oei_rlo_trim_variables = oei_rlo_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=oei_rlo_bem_output_list,
        ac_states=oei_rlo_1_ac_states,
    )
    system_model.register_output(oei_rlo_trim_variables)
    system_model.add_constraint(oei_rlo_trim_variables.accelerations, equals=0, scaler=5)

    # endregion

    # region oei rli condition
    oei_rli_condition = cd.HoverCondition(
        name='oei_rli_condition_1',
    )

    # Create inputs for hover condition
    oei_rli_1_time = system_model.create_input('oei_rli_1_time', val=90)
    oei_rli_1_altitude = system_model.create_input('oei_rli_1_altitude', val=100)

    # Evaluate aircraft states and atmospheric properties and register them as outputs
    oei_rli_1_ac_states, oei_rli_1_atmosphere = oei_rli_condition.evaluate(hover_time=oei_rli_1_time, altitude=oei_rli_1_altitude)
    system_model.register_output(oei_rli_1_ac_states)
    system_model.register_output(oei_rli_1_atmosphere)

    oei_rli_1_rpms = []
    for i in range(num_rotors-1):
        oei_rli_1_rpms.append(system_model.create_input(f'oei_rli_1_rpm_{i}', val=1000, dv_flag=True, lower=500, upper=2500, scaler=1e-3))

    oei_rli_bem_output_list = evaluate_multiple_BEM_models(
        name_prefix='oei_rli_1_bem',
        bem_parameters=bem_hover_rotor_parameters_oei,
        bem_mesh_list=rotor_mesh_list_oei_rli,
        rpm_list=oei_rli_1_rpms,
        ac_states=oei_rli_1_ac_states,
        atmoshpere=oei_rli_1_atmosphere,
        num_nodes=1,
        m3l_model=system_model,
        rotation_direction_list=rotation_direction_list[:1] + rotation_direction_list[1 + 1:],
        chord_cp=True,
        twist_cp=True,
    )

    if motor:
        oei_rli_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=oei_rli_bem_output_list,
            motor_sizing_list=motor_mass_properties[:1] + motor_mass_properties[1 + 1:-1],
            rotor_rpm_list=oei_rli_1_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='oei_rli_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
            torque_delta=0.1,
        )


    oei_rli_trim_variables = oei_rli_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=oei_rli_bem_output_list,
        ac_states=oei_rli_1_ac_states,
    )
    system_model.register_output(oei_rli_trim_variables)
    system_model.add_constraint(oei_rli_trim_variables.accelerations, equals=0, scaler=5)

    # endregion


if hover:
    # region Hover condition
    hover_condition = cd.HoverCondition(
        name='hover_condition_1',
    )

    # Create inputs for hover condition
    hover_1_time = system_model.create_input('hover_1_time', val=90)
    hover_1_altitude = system_model.create_input('hover_1_altitude', val=300)

    # Evaluate aircraft states and atmospheric properties and register them as outputs
    hover_1_ac_states, hover_1_atmosphere = hover_condition.evaluate(hover_time=hover_1_time, altitude=hover_1_altitude)
    system_model.register_output(hover_1_ac_states)
    system_model.register_output(hover_1_atmosphere)

    hover_1_rpms = []
    for i in range(num_rotors):
        hover_1_rpms.append(system_model.create_input(f'hover_1_rpm_{i}', val=1000, dv_flag=True, lower=300, upper=2200, scaler=1e-3))

    hover_bem_output_list = evaluate_multiple_BEM_models(
        name_prefix='hover_1_bem',
        bem_parameters=bem_hover_rotor_parameters,
        bem_mesh_list=rotor_mesh_list,
        rpm_list=hover_1_rpms,
        ac_states=hover_1_ac_states,
        atmoshpere=hover_1_atmosphere,
        num_nodes=1,
        m3l_model=system_model,
        rotation_direction_list=rotation_direction_list,
        chord_cp=True,
        twist_cp=True,
    )

    if acoustics:
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
            rotor_meshes=rotor_mesh_list,
            rpm_list=hover_1_rpms,
            model_name_prefix='hover_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        system_model.add_constraint(hover_total_noise_a_weighted, upper=70, scaler=1e-2)

    if motor:
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
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=hover_bem_output_list,
        ac_states=hover_1_ac_states,
    )
    system_model.register_output(hover_trim_variables)
    system_model.add_constraint(hover_trim_variables.accelerations, equals=0)

    # endregion


if quasi_steady_transition:
    # region qst_1
    qst_1 = cd.CruiseCondition(
        name='qst_1',
        num_nodes=1,
        stability_flag=False
    )

    qst_1_mach = system_model.create_input('qst_1_mach', val=0.00029412)
    qst_1_altitude = system_model.create_input('qst_1_altitude', val=300)
    qst_1_pitch_angle = system_model.create_input('qst_1_pitch_angle', val=-0.0134037, dv_flag=True, lower=-0.0134037-np.deg2rad(5), upper=-0.0134037+np.deg2rad(5), scaler=5)
    qst_1_range = system_model.create_input('qst_1_range', val=0.72)

    qst_1_ac_states, qst_1_atmos = qst_1.evaluate(mach_number=qst_1_mach, pitch_angle=qst_1_pitch_angle, altitude=qst_1_altitude, cruise_range=qst_1_range)
    system_model.register_output(qst_1_ac_states)
    system_model.register_output(qst_1_atmos)

    qst_1_pp_rpm = system_model.create_input('qst_1_pp_rpm', val=1200, dv_flag=True, lower=500, upper=2000, scaler=3e-3)

    qst_1_rpms = []
    for i in range(num_rotors):
        qst_1_rpms.append(system_model.create_input(f'qst_1_rpm_{i}', val=1200, dv_flag=True, lower=200, upper=2000, scaler=1e-3))

    qst_1_rpms.append(qst_1_pp_rpm)

    qst_1_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_1_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_1_rpms,
        ac_states=qst_1_ac_states,
        atmoshpere=qst_1_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,

    )

    if acoustics:
        qst_1_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_1_acoustics_data.add_observer(
            name='qst_1_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_1_total_noise, qst_1_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_1_pitt_peters_outputs,
            acoustics_data=qst_1_acoustics_data,
            ac_states=qst_1_ac_states,
            atmos=qst_1_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_1_altitude,
            rotor_parameters=[pitt_peters_parameters]*9, # + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_1_rpms,
            model_name_prefix='qst_1_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        system_model.add_constraint(qst_1_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_1_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_1_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_1_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_1_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
            torque_delta=0.1,
        )

        qst_1_energy_model = cd.EnergyModelM3L(name='energy_qst_1')
        qst_1_energy = qst_1_energy_model.evaluate(
            qst_1_motor_outputs,
            ac_states=qst_1_ac_states,
        )
        system_model.register_output(qst_1_energy)


    qst_1_trim_variables = qst_1.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_1_pitt_peters_outputs],
        ac_states=qst_1_ac_states,
    )

    system_model.register_output(qst_1_trim_variables)
    system_model.add_constraint(qst_1_trim_variables.du_dt, equals=3.05090108)
    system_model.add_constraint(qst_1_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_1_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_1_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_1_trim_variables.dq_dt, equals=0)

    # endregion

    # region qst_2
    qst_2 = cd.CruiseCondition(
        name='qst_2',
        num_nodes=1,
        stability_flag=False
    )

    qst_2_mach = system_model.create_input('qst_2_mach', val=0.06489461)
    qst_2_altitude = system_model.create_input('qst_2_altitude', val=300)
    qst_2_pitch_angle = system_model.create_input('qst_2_pitch_angle', val=-0.04973228, dv_flag=True, lower=-0.04973228-np.deg2rad(5), upper=-0.04973228+np.deg2rad(5), scaler=5)
    qst_2_range = system_model.create_input('qst_2_range', val=158)

    qst_2_ac_states, qst_2_atmos = qst_2.evaluate(mach_number=qst_2_mach, pitch_angle=qst_2_pitch_angle, altitude=qst_2_altitude, cruise_range=qst_2_range)
    system_model.register_output(qst_2_ac_states)
    system_model.register_output(qst_2_atmos)

    qst_2_pp_rpm = system_model.create_input('qst_2_pp_rpm', val=1200, dv_flag=True, lower=600, upper=2000, scaler=3e-3)

    qst_2_rpms = []
    for i in range(num_rotors):
        qst_2_rpms.append(system_model.create_input(f'qst_2_rpm_{i}', val=1200, dv_flag=True, lower=400, upper=2000, scaler=1e-3))

    qst_2_rpms.append(qst_2_pp_rpm)

    qst_2_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_2_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_2_rpms,
        ac_states=qst_2_ac_states,
        atmoshpere=qst_2_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_2_vlm_model',
        surface_names=[
            'wing_mesh_qst_2',
            'tail_mesh_qst_2',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )

    elevator = system_model.create_input('qst_2_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_2_atmos,
        ac_states=qst_2_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_2_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_2_atmos, ac_states=qst_2_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_2_trim_variables = qst_2.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_2_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_2_ac_states,
    )

    system_model.register_output(qst_2_trim_variables)
    system_model.add_constraint(qst_2_trim_variables.du_dt, equals=1.84555602)
    system_model.add_constraint(qst_2_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_2_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_2_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_2_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_2_trim_variables.dr_dt, equals=0)

    if acoustics:
        qst_2_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_2_acoustics_data.add_observer(
            name='qst_2_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_2_total_noise, qst_2_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_2_pitt_peters_outputs,
            acoustics_data=qst_2_acoustics_data,
            ac_states=qst_2_ac_states,
            atmos=qst_2_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_2_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_2_rpms,
            model_name_prefix='qst_2_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_2_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_2_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_2_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_2_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_2_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_2_energy_model = cd.EnergyModelM3L(name='energy_qst_2')
        qst_2_energy = qst_2_energy_model.evaluate(
            qst_2_motor_outputs,
            ac_states=qst_2_ac_states,
        )
        system_model.register_output(qst_2_energy)

    # endregion

    # region qst_3
    qst_3 = cd.CruiseCondition(
        name='qst_3',
        num_nodes=1,
        stability_flag=False
    )

    qst_3_mach = system_model.create_input('qst_3_mach', val=0.11471427)
    qst_3_altitude = system_model.create_input('qst_3_altitude', val=300)
    qst_3_pitch_angle = system_model.create_input('qst_3_pitch_angle', val=0.16195989, dv_flag=True, lower=0.16195989-np.deg2rad(8), upper=0.16195989+np.deg2rad(8))
    qst_3_range = system_model.create_input('qst_3_range', val=280)

    qst_3_ac_states, qst_3_atmos = qst_3.evaluate(mach_number=qst_3_mach, pitch_angle=qst_3_pitch_angle, altitude=qst_3_altitude, cruise_range=qst_3_range)
    system_model.register_output(qst_3_ac_states)
    system_model.register_output(qst_3_atmos)

    qst_3_pp_rpm = system_model.create_input('qst_3_pp_rpm', val=400, dv_flag=True, lower=100, upper=2500, scaler=3e-3)

    qst_3_rpms = []
    for i in range(num_rotors):
        qst_3_rpms.append(system_model.create_input(f'qst_3_rpm_{i}', val=1200, dv_flag=True, lower=50, upper=2500, scaler=1e-3))

    qst_3_rpms.append(qst_3_pp_rpm)

    qst_3_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_3_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_3_rpms,
        ac_states=qst_3_ac_states,
        atmoshpere=qst_3_atmos,
        m3l_model=system_model,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        rotation_direction_list=['ignore'] * 9,
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_3_vlm_model',
        surface_names=[
            'wing_mesh_qst_3',
            'tail_mesh_qst_3',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )

    elevator = system_model.create_input('qst_3_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_3_atmos,
        ac_states=qst_3_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_3_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_3_atmos, ac_states=qst_3_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_3_trim_variables = qst_3.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_3_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_3_ac_states,
    )

    system_model.register_output(qst_3_trim_variables)
    system_model.add_constraint(qst_3_trim_variables.du_dt, equals=0.67632681)
    system_model.add_constraint(qst_3_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_3_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_3_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_3_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_3_trim_variables.dr_dt, equals=0)

    if acoustics:
        qst_3_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_3_acoustics_data.add_observer(
            name='qst_3_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_3_total_noise, qst_3_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_3_pitt_peters_outputs,
            acoustics_data=qst_3_acoustics_data,
            atmos=qst_3_atmos,
            ac_states=qst_3_ac_states,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_3_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_3_rpms,
            model_name_prefix='qst_3_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_3_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_3_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_3_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_3_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_3_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_3_energy_model = cd.EnergyModelM3L(name='energy_qst_3')
        qst_3_energy = qst_3_energy_model.evaluate(
            qst_3_motor_outputs,
            ac_states=qst_3_ac_states,
        )
        system_model.register_output(qst_3_energy)

    # endregion

    # region qst_4
    qst_4 = cd.CruiseCondition(
        name='qst_4',
        num_nodes=1,
        stability_flag=False
    )

    qst_4_mach = system_model.create_input('qst_4_mach', val=0.13740796)
    qst_4_altitude = system_model.create_input('qst_4_altitude', val=300)
    qst_4_pitch_angle = system_model.create_input('qst_4_pitch_angle', val=0.10779469, dv_flag=True, lower=0.10779469-np.deg2rad(7), upper=0.10779469+np.deg2rad(7))
    qst_4_range = system_model.create_input('qst_4_range', val=336)

    qst_4_ac_states, qst_4_atmos = qst_4.evaluate(mach_number=qst_4_mach, pitch_angle=qst_4_pitch_angle, altitude=qst_4_altitude, cruise_range=qst_4_range)
    system_model.register_output(qst_4_ac_states)
    system_model.register_output(qst_4_atmos)

    qst_4_pp_rpm = system_model.create_input('qst_4_pp_rpm', val=1000, dv_flag=True, lower=200, upper=2000, scaler=3e-3)

    qst_4_rpms = []
    for i in range(num_rotors):
        qst_4_rpms.append(system_model.create_input(f'qst_4_rpm_{i}', val=1200, dv_flag=True, lower=100, upper=2000, scaler=1e-3))

    qst_4_rpms.append(qst_4_pp_rpm)

    qst_4_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_4_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_4_rpms,
        ac_states=qst_4_ac_states,
        atmoshpere=qst_4_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_4_vlm_model',
        surface_names=[
            'wing_mesh_qst_4',
            'tail_mesh_qst_4',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )

    elevator = system_model.create_input('qst_4_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_4_atmos,
        ac_states=qst_4_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_4_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_4_atmos, ac_states=qst_4_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_4_trim_variables = qst_4.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_4_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_4_ac_states,
    )

    system_model.register_output(qst_4_trim_variables)
    system_model.add_constraint(qst_4_trim_variables.du_dt, equals=0.39583939)
    system_model.add_constraint(qst_4_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_4_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_4_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_4_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_4_trim_variables.dr_dt, equals=0)

    if acoustics:
        qst_4_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_4_acoustics_data.add_observer(
            name='qst_4_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_4_total_noise, qst_4_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_4_pitt_peters_outputs,
            acoustics_data=qst_4_acoustics_data,
            ac_states=qst_4_ac_states,
            atmos=qst_4_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_4_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_4_rpms,
            model_name_prefix='qst_4_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_4_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_4_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_4_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_4_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_4_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_4_energy_model = cd.EnergyModelM3L(name='energy_qst_4')
        qst_4_energy = qst_4_energy_model.evaluate(
            qst_4_motor_outputs,
            ac_states=qst_4_ac_states,
        )
        system_model.register_output(qst_4_energy)

    # endregion

    # region qst_5
    qst_5 = cd.CruiseCondition(
        name='qst_5',
        num_nodes=1,
        stability_flag=False
    )

    qst_5_mach = system_model.create_input('qst_5_mach', val=0.14708026)
    qst_5_altitude = system_model.create_input('qst_5_altitude', val=300)
    # qst_5_pitch_angle = system_model.create_input('qst_5_pitch_angle', val=0.08224058, dv_flag=True, lower=0.08224058-np.deg2rad(7), upper=0.08224058+np.deg2rad(7))
    qst_5_pitch_angle = system_model.create_input('qst_5_pitch_angle', val=0.04, dv_flag=True, lower=0.04-np.deg2rad(7), upper=0.04+np.deg2rad(7))
    qst_5_range = system_model.create_input('qst_5_range', val=360)

    qst_5_ac_states, qst_5_atmos = qst_5.evaluate(mach_number=qst_5_mach, pitch_angle=qst_5_pitch_angle, altitude=qst_5_altitude, cruise_range=qst_5_range)
    system_model.register_output(qst_5_ac_states)
    system_model.register_output(qst_5_atmos)

    qst_5_pp_rpm = system_model.create_input('qst_5_pp_rpm', val=1000, dv_flag=True, lower=100, upper=2000, scaler=3e-3)

    qst_5_rpms = []
    for i in range(num_rotors):
        qst_5_rpms.append(system_model.create_input(f'qst_5_rpm_{i}', val=500, dv_flag=True, lower=100, upper=1200, scaler=1e-3))

    qst_5_rpms.append(qst_5_pp_rpm)

    qst_5_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_5_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_5_rpms,
        ac_states=qst_5_ac_states,
        atmoshpere=qst_5_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_5_vlm_model',
        surface_names=[
            'wing_mesh_qst_5',
            'tail_mesh_qst_5',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    
    elevator = system_model.create_input('qst_5_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_5_atmos,
        ac_states=qst_5_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_5_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_5_atmos, ac_states=qst_5_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_5_trim_variables = qst_5.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_5_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_5_ac_states,
    )

    system_model.register_output(qst_5_trim_variables)
    system_model.add_constraint(qst_5_trim_variables.du_dt, equals=0.30159843)
    system_model.add_constraint(qst_5_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_5_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_5_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_5_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_5_trim_variables.dr_dt, equals=0)

    if acoustics:
        qst_5_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_5_acoustics_data.add_observer(
            name='qst_5_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_5_total_noise, qst_5_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_5_pitt_peters_outputs,
            acoustics_data=qst_5_acoustics_data,
            ac_states=qst_5_ac_states,
            atmos=qst_5_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_5_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_5_rpms,
            model_name_prefix='qst_5_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_5_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_5_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_5_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_5_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_5_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_5_energy_model = cd.EnergyModelM3L(name='energy_qst_5')
        qst_5_energy = qst_5_energy_model.evaluate(
            qst_5_motor_outputs,
            ac_states=qst_5_ac_states,
        )
        system_model.register_output(qst_5_energy)

    # endregion

    # region qst_6
    qst_6 = cd.CruiseCondition(
        name='qst_6',
        num_nodes=1,
        stability_flag=False
    )

    qst_6_mach = system_model.create_input('qst_6_mach', val=0.15408429)
    qst_6_altitude = system_model.create_input('qst_6_altitude', val=300)
    qst_6_pitch_angle = system_model.create_input('qst_6_pitch_angle', val=0.06704556, dv_flag=True, lower=0.06704556 - np.deg2rad(7), upper=0.06704556+np.deg2rad(7),  scaler=1)
    qst_6_range = system_model.create_input('qst_6_range', val=377.)

    qst_6_ac_states, qst_6_atmos = qst_6.evaluate(mach_number=qst_6_mach, pitch_angle=qst_6_pitch_angle, altitude=qst_6_altitude, cruise_range=qst_6_range)
    system_model.register_output(qst_6_ac_states)
    system_model.register_output(qst_6_atmos)

    qst_6_pp_rpm = system_model.create_input('qst_6_pp_rpm', val=1000, dv_flag=True, lower=50, upper=2000, scaler=3e-3)

    qst_6_rpms = []
    for i in range(num_rotors):
        qst_6_rpms.append(system_model.create_input(f'qst_6_rpm_{i}', val=200, dv_flag=True, lower=100, upper=1500, scaler=1e-3))

    qst_6_rpms.append(qst_6_pp_rpm)

    qst_6_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_6_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_6_rpms,
        ac_states=qst_6_ac_states,
        atmoshpere=qst_6_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )
    
    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_6_vlm_model',
        surface_names=[
            'wing_mesh_qst_6',
            'tail_mesh_qst_6',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    
    elevator = system_model.create_input('qst_6_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_6_atmos,
        ac_states=qst_6_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_6_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_6_atmos, ac_states=qst_6_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_6_trim_variables = qst_6.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_6_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_6_ac_states,
    )

    system_model.register_output(qst_6_trim_variables)
    system_model.add_constraint(qst_6_trim_variables.du_dt, equals=0.25379256)
    system_model.add_constraint(qst_6_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_6_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_6_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_6_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_6_trim_variables.dr_dt, equals=0)

    if acoustics: # if False: # 
        qst_6_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_6_acoustics_data.add_observer(
            name='qst_6_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_6_total_noise, qst_6_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_6_pitt_peters_outputs,
            acoustics_data=qst_6_acoustics_data,
            ac_states=qst_6_ac_states,
            atmos=qst_6_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_6_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_6_rpms,
            model_name_prefix='qst_6_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_6_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_6_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_6_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_6_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_6_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_6_energy_model = cd.EnergyModelM3L(name='energy_qst_6')
        qst_6_energy = qst_6_energy_model.evaluate(
            qst_6_motor_outputs,
            ac_states=qst_6_ac_states,
        )
        system_model.register_output(qst_6_energy)

    # endregion

    # region qst_7
    qst_7 = cd.CruiseCondition(
        name='qst_7',
        num_nodes=1,
        stability_flag=False
    )

    qst_7_mach = system_model.create_input('qst_7_mach', val=0.15983874)
    qst_7_altitude = system_model.create_input('qst_7_altitude', val=300)
    qst_7_pitch_angle = system_model.create_input('qst_7_pitch_angle', val=0.05598293, dv_flag=True, lower=0.05598293-np.deg2rad(7), upper=np.deg2rad(7), scaler=1)
    qst_7_range = system_model.create_input('qst_7_range', val=391.)

    qst_7_ac_states, qst_7_atmos = qst_7.evaluate(mach_number=qst_7_mach, pitch_angle=qst_7_pitch_angle, altitude=qst_7_altitude, cruise_range=qst_7_range)
    system_model.register_output(qst_7_ac_states)
    system_model.register_output(qst_7_atmos)

    qst_7_pp_rpm = system_model.create_input('qst_7_pp_rpm', val=1000, dv_flag=True, lower=800, upper=2500, scaler=3e-3)
    qst_7_rpms = []
    for i in range(num_rotors):
        qst_7_rpms.append(system_model.create_input(f'qst_7_rpm_{i}', val=500, dv_flag=True, lower=100, upper=1500, scaler=1e-3))

    qst_7_rpms.append(qst_7_pp_rpm)

    qst_7_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_7_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_7_rpms,
        ac_states=qst_7_ac_states,
        atmoshpere=qst_7_atmos,
        m3l_model=system_model,
        rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_7_vlm_model',
        surface_names=[
            'wing_mesh_qst_7',
            'tail_mesh_qst_7',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    
    elevator = system_model.create_input('qst_7_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_7_atmos,
        ac_states=qst_7_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_7_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_7_atmos, ac_states=qst_7_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_7_trim_variables = qst_7.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_7_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_7_ac_states,
    )

    system_model.register_output(qst_7_trim_variables)
    system_model.add_constraint(qst_7_trim_variables.du_dt, equals=0.22345727)
    system_model.add_constraint(qst_7_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_7_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_7_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_7_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_7_trim_variables.dr_dt, equals=0)

    if acoustics: # if False: # 
        qst_7_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_7_acoustics_data.add_observer(
            name='qst_7_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_7_total_noise, qst_7_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_7_pitt_peters_outputs,
            acoustics_data=qst_7_acoustics_data,
            ac_states=qst_7_ac_states,
            atmos=qst_7_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_7_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_7_rpms,
            model_name_prefix='qst_7_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_7_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_7_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_7_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_7_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_7_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_7_energy_model = cd.EnergyModelM3L(name='energy_qst_7')
        qst_7_energy = qst_7_energy_model.evaluate(
            qst_7_motor_outputs,
            ac_states=qst_7_ac_states,
        )
        system_model.register_output(qst_7_energy)


   
    # endregion

    # region qst_8
    qst_8 = cd.CruiseCondition(
        name='qst_8',
        num_nodes=1,
        stability_flag=False
    )

    qst_8_mach = system_model.create_input('qst_8_mach', val=0.16485417)
    qst_8_altitude = system_model.create_input('qst_8_altitude', val=300)
    qst_8_pitch_angle = system_model.create_input('qst_8_pitch_angle', val=0.04712265, dv_flag=True, lower=0.04712265-np.deg2rad(7), upper=np.deg2rad(7), scaler=1)
    qst_8_range = system_model.create_input('qst_8_range', val=403.)

    qst_8_ac_states, qst_8_atmos = qst_8.evaluate(mach_number=qst_8_mach, pitch_angle=qst_8_pitch_angle, altitude=qst_8_altitude, cruise_range=qst_8_range)
    system_model.register_output(qst_8_ac_states)
    system_model.register_output(qst_8_atmos)

    qst_8_pp_rpm = system_model.create_input('qst_8_pp_rpm', val=1000, dv_flag=True, lower=500, upper=2500, scaler=3e-3)

    qst_8_rpms = []
    for i in range(num_rotors):
        qst_8_rpms.append(system_model.create_input(f'qst_8_rpm_{i}', val=500, dv_flag=True, lower=100, upper=1500, scaler=1e-3))

    qst_8_rpms.append(qst_8_pp_rpm)

    qst_8_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_8_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_8_rpms,
        ac_states=qst_8_ac_states,
        atmoshpere=qst_8_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_8_vlm_model',
        surface_names=[
            'wing_mesh_qst_8',
            'tail_mesh_qst_8',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    
    elevator = system_model.create_input('qst_8_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_8_atmos,
        ac_states=qst_8_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_8_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_8_atmos, ac_states=qst_8_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_8_trim_variables = qst_8.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_8_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_8_ac_states,
    )

    system_model.register_output(qst_8_trim_variables)
    system_model.add_constraint(qst_8_trim_variables.du_dt, equals=0.20269499)
    system_model.add_constraint(qst_8_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_8_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_8_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_8_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_8_trim_variables.dr_dt, equals=0)

    if False: # if acoustics
        qst_8_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_8_acoustics_data.add_observer(
            name='qst_8_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_8_total_noise, qst_8_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_8_pitt_peters_outputs,
            acoustics_data=qst_8_acoustics_data,
            ac_states=qst_8_ac_states,
            atmos=qst_8_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_8_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_8_rpms,
            model_name_prefix='qst_8_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_8_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_8_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_8_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_8_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_8_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_8_energy_model = cd.EnergyModelM3L(name='energy_qst_8')
        qst_8_energy = qst_8_energy_model.evaluate(
            qst_8_motor_outputs,
            ac_states=qst_8_ac_states,
        )
        system_model.register_output(qst_8_energy)
        
    # endregion

    # region qst_9
    qst_9 = cd.CruiseCondition(
        name='qst_9',
        num_nodes=1,
        stability_flag=False
    )

    qst_9_mach = system_model.create_input('qst_9_mach', val=0.16937793)
    qst_9_altitude = system_model.create_input('qst_9_altitude', val=300)
    qst_9_pitch_angle = system_model.create_input('qst_9_pitch_angle', val=0.03981101, dv_flag=True, lower=0.03981101-np.deg2rad(7), upper=np.deg2rad(7), scaler=1)
    qst_9_range = system_model.create_input('qst_9_range', val=414.)

    qst_9_ac_states, qst_9_atmos = qst_9.evaluate(mach_number=qst_9_mach, pitch_angle=qst_9_pitch_angle, altitude=qst_9_altitude, cruise_range=qst_9_range)
    system_model.register_output(qst_9_ac_states)
    system_model.register_output(qst_9_atmos)

    qst_9_pp_rpm = system_model.create_input('qst_9_pp_rpm', val=1000, dv_flag=True, lower=500, upper=2500, scaler=3e-3)

    qst_9_rpms = []
    for i in range(num_rotors):
        qst_9_rpms.append(system_model.create_input(f'qst_9_rpm_{i}', val=500, dv_flag=True, lower=100, upper=1500, scaler=1e-3))

    qst_9_rpms.append(qst_9_pp_rpm)

    qst_9_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_9_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_9_rpms,
        ac_states=qst_9_ac_states,
        atmoshpere=qst_9_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_9_vlm_model',
        surface_names=[
            'wing_mesh_qst_9',
            'tail_mesh_qst_9',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    
    elevator = system_model.create_input('qst_9_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_9_atmos,
        ac_states=qst_9_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_9_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_9_atmos, ac_states=qst_9_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_9_trim_variables = qst_9.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_9_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_9_ac_states,
    )

    system_model.register_output(qst_9_trim_variables)
    system_model.add_constraint(qst_9_trim_variables.du_dt, equals=0.18808881)
    system_model.add_constraint(qst_9_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_9_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_9_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_9_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_9_trim_variables.dr_dt, equals=0)

    if False: # if acoustics
        qst_9_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_9_acoustics_data.add_observer(
            name='qst_9_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_9_total_noise, qst_9_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_9_pitt_peters_outputs,
            acoustics_data=qst_9_acoustics_data,
            atmos=qst_9_atmos,
            ac_states=qst_9_ac_states,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_9_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_9_rpms,
            model_name_prefix='qst_9_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_9_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_9_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_9_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_9_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_9_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_9_energy_model = cd.EnergyModelM3L(name='energy_qst_9')
        qst_9_energy = qst_9_energy_model.evaluate(
            qst_9_motor_outputs,
            ac_states=qst_9_ac_states,
        )
        system_model.register_output(qst_9_energy)

    # endregion

    # region qst_10
    qst_10 = cd.CruiseCondition(
        name='qst_10',
        num_nodes=1,
        stability_flag=False
    )

    qst_10_mach = system_model.create_input('qst_10_mach', val=0.17354959)
    qst_10_altitude = system_model.create_input('qst_10_altitude', val=300)
    qst_10_pitch_angle = system_model.create_input('qst_10_pitch_angle', val=0.03369678, dv_flag=True, lower=np.deg2rad(-7), upper=np.deg2rad(7), scaler=1)
    qst_10_range = system_model.create_input('qst_10_range', val=424.)

    qst_10_ac_states, qst_10_atmos = qst_10.evaluate(mach_number=qst_10_mach, pitch_angle=qst_10_pitch_angle, altitude=qst_10_altitude, cruise_range=qst_10_range)
    system_model.register_output(qst_10_ac_states)
    system_model.register_output(qst_10_atmos)

    qst_10_pp_rpm = system_model.create_input('qst_10_pp_rpm', val=1000, dv_flag=True, lower=500, upper=2500, scaler=3e-3)

    qst_10_rpms = []
    for i in range(num_rotors):
        qst_10_rpms.append(system_model.create_input(f'qst_10_rpm_{i}', val=500, dv_flag=True, lower=100, upper=1500, scaler=1e-3))

    qst_10_rpms.append(qst_10_pp_rpm)

    qst_10_pitt_peters_outputs = evaluate_multiple_pitt_peters_models(
        name_prefix='qst_10_rotor',
        pitt_peters_parameters=[pitt_peters_parameters]* 9, # + [bem_pusher_rotor_parameters_acoustics],
        pitt_peters_mesh_list=rotor_mesh_list + [pp_mesh],
        rpm_list=qst_10_rpms,
        ac_states=qst_10_ac_states,
        atmoshpere=qst_10_atmos,
        m3l_model=system_model,
        rotation_direction_list=['ignore'] * 9,
        # rotation_direction_list=rotation_direction_list + ['ignore'],
        chord_cp=True,
        twist_cp=True,
    )

    # VAST solver
    vlm_model = VASTFluidSover(
        name='qst_10_vlm_model',
        surface_names=[
            'wing_mesh_qst_10',
            'tail_mesh_qst_10',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    
    elevator = system_model.create_input('qst_10_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))

    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=qst_10_atmos,
        ac_states=qst_10_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='qst_10_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=qst_10_atmos, ac_states=qst_10_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    qst_10_trim_variables = qst_10.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, m4_mass_properties, wing_beam_mass_props],
        aero_propulsive_outputs=[qst_10_pitt_peters_outputs, vlm_outputs, drag_build_up_outputs],
        ac_states=qst_10_ac_states,
    )

    system_model.register_output(qst_10_trim_variables)
    system_model.add_constraint(qst_10_trim_variables.du_dt, equals=0.17860702)
    system_model.add_constraint(qst_10_trim_variables.dv_dt, equals=0)
    system_model.add_constraint(qst_10_trim_variables.dw_dt, equals=0)
    system_model.add_constraint(qst_10_trim_variables.dp_dt, equals=0)
    system_model.add_constraint(qst_10_trim_variables.dq_dt, equals=0)
    system_model.add_constraint(qst_10_trim_variables.dr_dt, equals=0)

    if False: #  if acoustics: # 
        qst_10_acoustics_data = Acoustics(
            aircraft_position=np.array([0., 0., 100])
        )

        qst_10_acoustics_data.add_observer(
            name='qst_10_observer',
            obs_position=np.array([0., 0., 0.]),
            time_vector=np.array([0.,]),

        )

        qst_10_total_noise, qst_10_total_noise_a_weighted = evaluate_multiple_acoustic_models(
            rotor_outputs=qst_10_pitt_peters_outputs,
            acoustics_data=qst_10_acoustics_data,
            ac_states=qst_10_ac_states,
            atmos=qst_10_atmos,
            tonal_noise_model='Lowson',
            broadband_noise_model='GL',
            altitude=qst_10_altitude,
            rotor_parameters=[pitt_peters_parameters]*8 + [bem_pusher_rotor_parameters_acoustics],
            rotor_meshes=rotor_mesh_list + [pp_mesh],
            rpm_list=qst_10_rpms,
            model_name_prefix='qst_10_noise',
            num_nodes=1,
            m3l_model=system_model,
        )
        # system_model.add_constraint(qst_10_total_noise_a_weighted, upper=70, scaler=1e-2)

    # if motor:
    if False:
        qst_10_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=qst_10_pitt_peters_outputs,
            motor_sizing_list=motor_mass_properties,
            rotor_rpm_list=qst_10_rpms,
            motor_diameter_list=motor_diameters,
            name_prefix='qst_10_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        qst_10_energy_model = cd.EnergyModelM3L(name='energy_qst_10')
        qst_10_energy = qst_10_energy_model.evaluate(
            qst_10_motor_outputs,
            ac_states=qst_10_ac_states,
        )
        system_model.register_output(qst_10_energy)


    

    # endregion


if climb:
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

    if motor:
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
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=[vlm_outputs, climb_bem_outputs, drag_build_up_outputs],
        ac_states=climb_ac_states,
        load_factor=1.,
        ref_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(climb_trim_variables)
    system_model.add_constraint(climb_trim_variables.accelerations, equals=0, scaler=5.)

if cruise:
    # region cruise
    cruise_condition = cd.CruiseCondition(
        name='steady_cruise',
        num_nodes=1,
        stability_flag=True,
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
            'cruise_vtail_mesh',
            'cruise_fuselage_mesh',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
            (1, ) + vtail_meshes.vlm_mesh.shape[1:],
            (1, ) + fuesleage_mesh.shape,
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
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh, vtail_meshes.vlm_mesh, fuesleage_mesh],
        deflections=[None, elevator, None, None],
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

    if motor:
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
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=[vlm_outputs, cruise_bem_outputs, drag_build_up_outputs],
        ac_states=cruise_ac_states,
        load_factor=1.,
        ref_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(cruise_trim_variables)
    system_model.add_constraint(cruise_trim_variables.accelerations, equals=0, scaler=5.)
    # system_model.register_output(cruise_trim_variables.longitudinal_stability.phugoid_time_to_double)
    # system_model.register_output(cruise_trim_variables.longitudinal_stability.short_period_damping_ratio)
    # system_model.add_constraint(cruise_trim_variables.longitudinal_stability.phugoid_time_to_double, lower=55.)
    # system_model.add_constraint(cruise_trim_variables.longitudinal_stability.short_period_damping_ratio, lower=0.15)
    # endregion

if descent:
    descent_condition = cd.ClimbCondition(
        name='steady_descent',
        num_nodes=1,
    )

    descent_M = system_model.create_input('descent_mach', val=0.195)
    descent_hi = system_model.create_input('descent_initial_altitude', val=1000)
    descent_hf = system_model.create_input('descent_final_altitude', val=300)
    descent_flight_path_angle = system_model.create_input(name='descent_flight_path_angle', val=np.deg2rad(-4))
    descent_pitch = system_model.create_input('descent_pitch', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(5), scaler=10)

    descent_ac_states, descent_atmos = descent_condition.evaluate(mach_number=descent_M, pitch_angle=descent_pitch, flight_path_angle=descent_flight_path_angle,
                                                            initial_altitude=descent_hi, final_altitude=descent_hf)

    system_model.register_output(descent_ac_states)
    system_model.register_output(descent_atmos)

    descent_bem = BEM(
        name='descent_bem',
        num_nodes=1, 
        BEM_parameters=bem_pusher_rotor_parameters,
        rotation_direction='ignore',
    )
    descent_rpm = system_model.create_input('descent_rpm', val=2000, dv_flag=True, lower=600, upper=2500, scaler=1e-3)
    descent_bem_outputs = descent_bem.evaluate(ac_states=descent_ac_states, rpm=descent_rpm, rotor_radius=pp_mesh.radius, thrust_vector=pp_mesh.thrust_vector,
                                                    thrust_origin=pp_mesh.thrust_origin, atmosphere=descent_atmos, blade_chord_cp=pp_mesh.chord_cps, blade_twist_cp=pp_mesh.twist_cps, 
                                                    cg_vec=m4_mass_properties.cg_vector, reference_point=m4_mass_properties.cg_vector)
    system_model.register_output(descent_bem_outputs) 

    # VAST solver
    vlm_model = VASTFluidSover(
        name='descent_vlm_model',
        surface_names=[
            'descent_wing_mesh',
            'descent_tail_mesh',
        ],
        surface_shapes=[
            (1, ) + wing_meshes.vlm_mesh.shape[1:],
            (1, ) + tail_meshes.vlm_mesh.shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
        mesh_unit='ft',
        cl0=[0., 0.]
    )
    elevator = system_model.create_input('descent_elevator', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20))
    # Evaluate VLM outputs and register them as outputs
    vlm_outputs = vlm_model.evaluate(
        atmosphere=descent_atmos,
        ac_states=descent_ac_states,
        meshes=[wing_meshes.vlm_mesh, tail_meshes.vlm_mesh],
        deflections=[None, elevator],
        wing_AR=wing_AR,
        eval_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(vlm_outputs)

    drag_build_up_model = cd.DragBuildUpModel(
        name='descent_drag_build_up',
        num_nodes=1,
        units='ft',
    )

    drag_build_up_outputs = drag_build_up_model.evaluate(atmos=descent_atmos, ac_states=descent_ac_states, drag_comp_list=drag_comp_list, s_ref=S_ref)
    system_model.register_output(drag_build_up_outputs)

    if motor:
        descent_motor_outputs = evaluate_multiple_motor_analysis_models(
            rotor_outputs_list=[descent_bem_outputs],
            motor_sizing_list=[motor_mass_properties[-1]],
            rotor_rpm_list=[descent_rpm],
            motor_diameter_list=[motor_diameters[-1]],
            name_prefix='descent_motor_analysis',
            flux_weakening=False,
            m3l_model=system_model,
        )

        descent_energy_model = cd.EnergyModelM3L(name='energy_descent')
        descent_energy = descent_energy_model.evaluate(
            descent_motor_outputs,
            ac_states=descent_ac_states,
        )
        system_model.register_output(descent_energy)

    descent_trim_variables = descent_condition.assemble_trim_residual(
        mass_properties=[motor_mass_properties, battery_mass_properties, wing_beam_mass_props, m4_mass_properties],
        aero_propulsive_outputs=[vlm_outputs, descent_bem_outputs, drag_build_up_outputs],
        ac_states=descent_ac_states,
        load_factor=1.,
        ref_pt=m4_mass_properties.cg_vector,
    )
    system_model.register_output(descent_trim_variables)
    system_model.add_constraint(descent_trim_variables.accelerations, equals=0, scaler=5)

# Total energy + SoC
if motor:
    total_energy_model = cd.TotalEnergyModelM3L(name='total_energy_model')
    total_energy = total_energy_model.evaluate(
        hover_energy, 
        climb_energy, 
        cruise_energy, 
        descent_energy,
        # qst_1_energy,
        # qst_2_energy,
        # qst_3_energy,
        # qst_4_energy,
        # qst_5_energy,
        # qst_6_energy,
        # qst_7_energy,
        # qst_8_energy,
        # qst_9_energy,
        # qst_10_energy,
    )
    system_model.register_output(total_energy)

    soc_model = cd.SOCModelM3L(name='SoC_model', battery_energy_density=400) # Note, either turn this parameter into csdl variable or connect it from battery sizing 
    final_SoC = soc_model.evaluate(battery_mass=battery_mass, total_energy_consumption=total_energy, mission_multiplier=2.7)
    system_model.register_output(final_SoC)
    system_model.add_constraint(final_SoC, equals=0.2)

 
if False:
    total_spl =   hover_total_noise_a_weighted + qst_1_total_noise_a_weighted + qst_2_total_noise_a_weighted+ qst_3_total_noise_a_weighted + qst_4_total_noise_a_weighted \
                + qst_5_total_noise_a_weighted #+ qst_6_total_noise_a_weighted + qst_7_total_noise_a_weighted
    system_model.register_output(total_spl)
    system_model.add_objective(total_spl, scaler=1e-3)

caddee_csdl_model = system_model.assemble_csdl()

# Enforce beam symmetry
if sizing:
    wing_beam_ttop = caddee_csdl_model.declare_variable('wing_beam_ttop', shape=(num_wing_beam, ))
    wing_beam_tbot = caddee_csdl_model.declare_variable('wing_beam_tbot', shape=(num_wing_beam, ))
    wing_beam_tweb = caddee_csdl_model.declare_variable('wing_beam_tweb', shape=(num_wing_beam, ))

    n = int((num_wing_beam - 1) / 2)

    left_ttop = wing_beam_ttop[0:n]
    left_tbot = wing_beam_tbot[0:n]
    left_tweb = wing_beam_tweb[0:n]

    right_ttop = caddee_csdl_model.create_output('right_ttop', shape=(n, ))
    right_tbot = caddee_csdl_model.create_output('right_tbot', shape=(n, ))
    right_tweb = caddee_csdl_model.create_output('right_tweb', shape=(n, ))
    indices = np.flip(np.arange(n+1, num_wing_beam, 1, dtype=int))
    for i in range(n):
        right_ttop[i] = wing_beam_ttop[int(indices[i])]
        right_tbot[i] = wing_beam_tbot[int(indices[i])]
        right_tweb[i] = wing_beam_tweb[int(indices[i])]


    caddee_csdl_model.register_output('ttop_symmetry', right_ttop-left_ttop)
    caddee_csdl_model.register_output('tbot_symmetry', right_tbot-left_tbot)
    caddee_csdl_model.register_output('tweb_symmetry', right_tweb-left_tweb)

    caddee_csdl_model.add_constraint('ttop_symmetry', equals=0, scaler=1e-2)
    caddee_csdl_model.add_constraint('tbot_symmetry', equals=0, scaler=1e-2)
    caddee_csdl_model.add_constraint('tweb_symmetry', equals=0, scaler=1e-2)

import csdl
# rep = csdl.GraphRepresentation(caddee_csdl_model)
# sim = Simulator(rep, analytics=True)

sim = Simulator(caddee_csdl_model,name = 'save_outputs', analytics=True)


import pickle
with open('NEW_CADDEE_sizing_dvs_with_buckling_2.pickle', 'rb') as handle:
    dvs = pickle.load(handle)
    sim['wing_beam_ttop'] = dvs['wing_beam_ttop']
    sim['wing_beam_tbot'] = dvs['wing_beam_tbot']
    sim['wing_beam_tweb'] = dvs['wing_beam_tweb']
    # for key, val in dvs.items():
    #     sim[key] = val  

# with open('NEW_CADDEE_ACOUSTICS_OBJ_DVS.pickle', 'rb') as handle:
#     dvs = pickle.load(handle)
#     for key, val in dvs.items():
#         sim[key] = val  

# if hover and (not climb and not cruise and not descent):
if hover and climb and cruise and descent:
    with open('NEW_CADDEE_steady_dcs_trim_only_dvs_new.pickle', 'rb') as handle:
            dvs = pickle.load(handle)
            for key, val in dvs.items():
                    sim[key] = val

if oei:
    with open('NEW_CADDEE_ALL_OEI_trim_only_dvs.pickle', 'rb') as handle:
            dvs = pickle.load(handle)
            for key, val in dvs.items():
                sim[key] = val

if quasi_steady_transition: 
    with open('NEW_CADDEE_QST_ALL_trim_only_dvs.pickle', 'rb') as handle:
            dvs = pickle.load(handle)
            for key, val in dvs.items():
                sim[key] = val

sim.run()
cd.print_caddee_outputs(system_model, sim, compact_print=True)
# sim.check_totals(step=1e-3)
exit()

prob = CSDLProblem(problem_name='TC2_new_caddee_FULL_MDO', simulator=sim)

optimizer = SNOPT(
        prob, 
        Major_iterations=100, 
        Major_optimality=1e-4, 
        Major_feasibility=1e-4,
        append2file=True,
        Iteration_limit=500000,
        Major_step_limit= 1.5,
        Linesearch_tolerance=0.6,
)

# optimizer = SLSQP(prob, maxiter=100, ftol=1E-5)
optimizer.solve()
cd.print_caddee_outputs(system_model, sim, compact_print=True)
optimizer.print_results()

dv_dictionary = {}
for dv_name, dv_dict in sim.dvs.items():
    print(dv_name, dv_dict['index_lower'], dv_dict['index_upper'])
    print(sim[dv_name])
    dv_dictionary[dv_name] = sim[dv_name]

# exit()

if sizing:
    print(sim['ttop_symmetry'])
    print(sim['tbot_symmetry'])
    print(sim['tweb_symmetry'])

import pickle
dv_dictionary = {}
for dv_name, dv_dict in sim.dvs.items():
    print(dv_name, dv_dict['index_lower'], dv_dict['index_upper'])
    print(sim[dv_name])
    dv_dictionary[dv_name] = sim[dv_name]

with open('NEW_CADDEE_DVS_FULL_MDO.pickle', 'wb') as handle:
    pickle.dump(dv_dictionary, handle, protocol=pickle.HIGHEST_PROTOCOL)

c_dictionary = {}
for c_name, c_dict in sim.cvs.items():
    print(c_name, c_dict['index_lower'], c_dict['index_upper'])
    print(sim[c_name])
    c_dict[c_name] = sim[c_name]
with open('NEW_CADDEE_CONSTRAINTS_FULL_MDO.pickle', 'wb') as handle:
    pickle.dump(c_dictionary, handle, protocol=pickle.HIGHEST_PROTOCOL)

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
