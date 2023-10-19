'''Example 2 : Description of example 2'''
import caddee.api as cd
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am
from aframe.core.beam_module import EBBeam, LinearBeamMesh
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from VAST.core.generate_mappings_m3l import VASTNodalForces
from lsdo_airfoil.core.pressure_profile import PressureProfile, NodalPressureProfile
from VAST.core.vlm_llt.viscous_correction import ViscousCorrectionModel

# from modopt.snopt_library import SNOPT
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from caddee import GEOMETRY_FILES_FOLDER
import m3l
import lsdo_geo as lg
import aframe.core.beam_module as ebbeam


from examples.advanced_examples.TC2_problem.ex_tc2_geometry_setup import lpc_rep, lpc_param, \
    wing_camber_surface, htail_camber_surface, wing_vlm_mesh_name, htail_vlm_mesh_name, wing_oml_mesh, wing_upper_surface_ml, htail_upper_surface_ml, \
    wing_beam, width, height, num_wing_beam, wing, wing_upper_surface_wireframe, wing_lower_surface_wireframe,\
    pp_disk_in_plane_x, pp_disk_in_plane_y, pp_disk_origin, pp_disk

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = lpc_rep
caddee.system_parameterization = lpc_param


# m3l sizing model
# sizing_model = m3l.Model()

# Battery sizing
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

simple_battery_sizing.set_module_input('battery_mass', val=800)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.5, 0, 0.5]))
simple_battery_sizing.set_module_input('battery_energy_density', val=400)

battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()

# M4 regressions
m4_regression = cd.M4RegressionsM3L()

mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# region cruise condition
cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True, lower=0.17, upper=0.19)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(5))
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(ac_states)

vlm_model = VASTFluidSover(
    surface_names=[
        f'{wing_vlm_mesh_name}_cruise',
        f'{htail_vlm_mesh_name}_cruise',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.25, 0.],
    ML=True,
)

# aero forces and moments
cl_distribution, re_spans, vlm_panel_forces, panel_areas, evaluation_pt, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, ML=True)
# vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)
cruise_model.register_output(vlm_force)
cruise_model.register_output(vlm_moment)
cruise_model.register_output(cl_distribution)
cruise_model.register_output(re_spans)

ml_pressures = PressureProfile(
    airfoil_name='NASA_langley_ga_1',
    use_inverse_cl_map=True,
)

cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans) #, mach_number, reynolds_number)
cruise_model.register_output(cp_upper)
cruise_model.register_output(cp_lower)

viscous_drag_correction = ViscousCorrectionModel(
    surface_names=[
        f'{wing_vlm_mesh_name}_cruise',
        f'{htail_vlm_mesh_name}_cruise',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
)
moment_point = None
vlm_F, vlm_M = viscous_drag_correction.evaluate(ac_states=ac_states, forces=vlm_panel_forces, cd_v=Cd, panel_area=panel_areas, moment_pt=moment_point, evaluation_pt=evaluation_pt, design_condition=cruise_condition)
cruise_model.register_output(vlm_F)
cruise_model.register_output(vlm_M)

ml_pressures_oml_map = NodalPressureProfile(
    surface_names=[
        f'{wing_vlm_mesh_name}_cruise',
        f'{htail_vlm_mesh_name}_cruise',
    ],
    surface_shapes=[
        wing_upper_surface_ml.value.shape,
        htail_upper_surface_ml.value.shape,
    ]
)

cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[])
wing_oml_pressure_upper = cp_upper_oml[0]
htail_oml_pressure_upper = cp_upper_oml[1]
wing_oml_pressure_lower = cp_lower_oml[0]
htail_oml_pressure_lower = cp_lower_oml[1]

cruise_model.register_output(wing_oml_pressure_upper)
cruise_model.register_output(htail_oml_pressure_upper)
cruise_model.register_output(wing_oml_pressure_lower)
cruise_model.register_output(htail_oml_pressure_lower)

vlm_force_mapping_model = VASTNodalForces(
    surface_names=[
        f'{wing_vlm_mesh_name}_cruise',
        f'{htail_vlm_mesh_name}_cruise',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    initial_meshes=[
        wing_camber_surface,
        htail_camber_surface]
)

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh])
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
    use_airfoil_ml=False,
    mesh_units='ft'
)

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
bem_forces, bem_moments,_ ,_ ,_, _ = bem_model.evaluate(ac_states=ac_states)

# create the aframe dictionaries:
# joints, bounds, beams = {}, {}, {}
# beams['wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'cs': 'box','nodes': list(range(num_wing_beam))}
# bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1,1,1,1,1,1]}

# # create the beam model:
# beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# #beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# #beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

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

total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(wing_mass, battery_mass, mass_m4, wing_cg, cg_battery, cg_m4, wing_inertia_tensor, I_battery, I_m4)
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

cruise_model.register_output(total_mass)
cruise_model.register_output(total_cg)
cruise_model.register_output(total_inertia)

# inertial forces and moments
inertial_loads_model = cd.InertialLoads(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMoments()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_F, vlm_M, bem_forces, bem_moments, inertial_forces, inertial_moments)
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
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
                                lower=np.deg2rad(-10),
                                upper=np.deg2rad(10),
                                scaler=1,
                            )
wing_act_plus_3g = caddee_csdl_model.create_input('cruise_wing_actuation', val=np.deg2rad(3.2))

# caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.new_stress',upper=500E6/1,scaler=1E-8)
# caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual', equals=0.)
# caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.total_constant_mass_properties.total_mass', scaler=1e-3)

# caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True, display_scripts=True)
sim.run()
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.vm_stress'])
# print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.wing_beam_forces'])

exit()

# sim.compute_total_derivatives()
# sim.check_totals()


# prob = CSDLProblem(problem_name='lpc', simulator=sim)
# optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
# optimizer.solve()
# optimizer.print_results()

print("I'm done.")


# print("I'm done.")