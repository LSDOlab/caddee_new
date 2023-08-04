nthreads = 1
import os
os.environ["OPENBLAS_NUM_THREADS"] = str(nthreads)
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
from lsdo_motor import MotorSizing, MotorAnalysis
from caddee import PROJECTIONS_FOLDER
import pickle
import sys
sys.setrecursionlimit(100000)


caddee = cd.CADDEE()

# Import representation and the geometry from another file for brevity
from examples.advanced_examples.TC2_problem.ex_tc2_geometry_setup_updated import lpc_rep, lpc_param, wing_camber_surface, htail_camber_surface, \
    wing_vlm_mesh_name, htail_vlm_mesh_name, wing_oml_mesh,htail_oml_mesh,  wing_upper_surface_ml, htail_upper_surface_ml, \
    pp_disk_in_plane_x, pp_disk_in_plane_y, pp_disk_origin, pp_disk, wing,\
    wing_beam, width, height, num_wing_beam, wing, wing_upper_surface_wireframe, wing_lower_surface_wireframe,\
    rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk, \
    rro_disk_mesh, flo_disk_mesh, fro_disk_mesh, rri_disk_mesh, fli_disk_mesh, fri_disk_mesh, rlo_disk_mesh, rli_disk_mesh




wing_cp_index_functions = {}

def build_cp_index_function(wing, grid_num, design_condition, system_m3l_model, wing_cp_index_functions, wing_oml_pressure_upper, wing_oml_pressure_lower):


    vstack = m3l.VStack()
    wing_oml_pressure = vstack.evaluate(wing_oml_pressure_upper, wing_oml_pressure_lower)
    import pickle
    from caddee import PROJECTIONS_FOLDER
    with open(PROJECTIONS_FOLDER /  'wing_cp_projections.pcikle', 'rb') as f:
            nodes_parametric = pickle.load(f)
    counter = 0
    for item in nodes_parametric:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric[counter] = new_tuple
        counter += 1

    from m3l.utils.utils import index_functions
    surface_names = list(wing.get_primitives().keys())
    para_grid = []
    for name in surface_names:
        for u in np.linspace(0,1,grid_num):
            for v in np.linspace(0,1,grid_num):
                para_grid.append((name, np.array([u,v]).reshape((1,2))))
    # evaluated_parametric = lpc_rep.spatial_representation.evaluate_parametric(para_grid)
    tip_patches_names = ['Wing_1, 0, 83', 'Wing_1, 0, 84', 'Wing_1, 0, 87', 'Wing_1, 0, 88', 'Wing_1, 1, 103', 'Wing_1, 1, 104', 'Wing_1, 1, 107', 'Wing_1, 1, 108']
    
    mid_patch_names = ['Wing_1, 0, 79', 'Wing_1, 0, 80', 'Wing_1, 1, 99', 'Wing_1, 1, 100']
    coefficients = {}
    spaces = {}
    for entry in surface_names:
        if entry in tip_patches_names:
            bsplinespace = lg.BSplineSpace(name=f'cp_{design_condition.name}_spline_space_{entry}', order=(1, 4), control_points_shape=(1, 10))
            coefficients[entry] = m3l.Variable(f'{entry}_cp_coefficients', shape=(1, 10, 1))
            spaces[entry] = bsplinespace
        elif entry in mid_patch_names:
            bsplinespace = lg.BSplineSpace(name=f'cp_{design_condition.name}_spline_space_{entry}', order=(2, 4), control_points_shape=(2, 10))
            coefficients[entry] = m3l.Variable(f'{entry}_cp_coefficients', shape=(2, 10, 1))
            spaces[entry] = bsplinespace
        else: 
            bsplinespace = lg.BSplineSpace(name=f'cp_{design_condition.name}_spline_space_{entry}', order=(3, 4), control_points_shape=(3, 10))
            coefficients[entry] = m3l.Variable(f'{entry}_cp_coefficients', shape=(3, 10, 1))
            spaces[entry] = bsplinespace

    cp_cruise_index_space = m3l.IndexedFunctionSpace(name='wing_cp_space', spaces=spaces)
    cp_cruise_indexed_function = m3l.IndexedFunction(name='wing_cp_indexed_function', space=cp_cruise_index_space, coefficients=coefficients)

    cp_cruise_indexed_function.inverse_evaluate(nodes_parametric, wing_oml_pressure, regularization_coeff = 1e-2)
    system_m3l_model.register_output(cp_cruise_indexed_function.coefficients, design_condition=design_condition)
    wing_cp_index_functions[design_condition.parameters['name']] = cp_cruise_indexed_function

# with open(PROJECTIONS_FOLDER /  'wing_cp_projections.pcikle', 'rb') as f:
#             nodes_parametric_2 = pickle.load(f)
# valid_surfaces_ml = []
# for item in nodes_parametric_2:
#     if not item[0] in valid_surfaces_ml:
#         valid_surfaces_ml.append(item[0])


# set system representation and parameterization
caddee.system_representation = lpc_rep
caddee.system_parameterization = lpc_param

# system model
caddee.system_model = system_model = cd.SystemModel()

system_m3l_model = m3l.Model()

# region sizing (m4 + battery only)
# # Battery sizing
# battery_component = cd.Component(name='battery')
# simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

# simple_battery_sizing.set_module_input('battery_mass', val=800, dv_flag=False, lower=600, scaler=1e-3)
# simple_battery_sizing.set_module_input('battery_position', val=np.array([3.6, 0, 0.5]))
# simple_battery_sizing.set_module_input('battery_energy_density', val=400)

# battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()
# system_m3l_model.register_output(battery_mass)
# system_m3l_model.register_output(cg_battery)
# system_m3l_model.register_output(I_battery)


# # M4 regressions
# m4_regression = cd.M4RegressionsM3L(exclude_wing=False)

# mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)
# system_m3l_model.register_output(mass_m4)
# system_m3l_model.register_output(cg_m4)
# system_m3l_model.register_output(I_m4)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

# system_m3l_model.register_output(total_mass)
# system_m3l_model.register_output(total_cg)
# system_m3l_model.register_output(total_inertia)
# endregion

# region sizing transition plus beam
# battery
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)
simple_battery_sizing.set_module_input('battery_mass', val=900, dv_flag=True, lower=600, scaler=1e-3)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.1, 0, 0.5]), dv_flag=False, lower=np.array([3.0, -1e-4, 0.5 - 1e-4]), upper=np.array([4, +1e-4, 0.5 + 1e-4]), scaler=1e-1)
simple_battery_sizing.set_module_input('battery_energy_density', val=400)
battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()

# M4 regressions
m4_regression = cd.M4RegressionsM3L(exclude_wing=True)
mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)

# Motors (9) 
# pusher 
pusher_motor_sizing = MotorSizing(
    rotor_component=pp_disk, # This will provide the center of mass of the mass 
)
pusher_motor_sizing.set_module_input('motor_diameter', val=0.22, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
pusher_motor_sizing.set_module_input('motor_length', val=0.15, dv_flag=True, lower=0.05, upper=0.2, scaler=1)
m_mass_pp, m_cg_pp, m_I_pp, pp_motor_parameters = pusher_motor_sizing.evaluate()

# rlo
rlo_motor_sizing = MotorSizing(
    rotor_component=rlo_disk, # This will provide the center of mass of the mass 
)
rlo_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
rlo_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_rlo, m_cg_rlo, m_I_rlo, rlo_motor_parameters = rlo_motor_sizing.evaluate()

# rli
rli_motor_sizing = MotorSizing(
    rotor_component=rli_disk, # This will provide the center of mass of the mass 
)
rli_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
rli_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_rli, m_cg_rli, m_I_rli, rli_motor_parameters = rli_motor_sizing.evaluate()


# rri
rri_motor_sizing = MotorSizing(
    rotor_component=rri_disk, # This will provide the center of mass of the mass 
)
rri_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
rri_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_rri, m_cg_rri, m_I_rri, rri_motor_parameters = rri_motor_sizing.evaluate()


# rro
rro_motor_sizing = MotorSizing(
    rotor_component=rro_disk, # This will provide the center of mass of the mass 
)
rro_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
rro_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_rro, m_cg_rro, m_I_rro, rro_motor_parameters = rro_motor_sizing.evaluate()


# flo
flo_motor_sizing = MotorSizing(
    rotor_component=flo_disk, # This will provide the center of mass of the mass 
)
flo_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
flo_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_flo, m_cg_flo, m_I_flo, flo_motor_parameters = flo_motor_sizing.evaluate()


# fli
fli_motor_sizing = MotorSizing(
    rotor_component=fli_disk, # This will provide the center of mass of the mass 
)
fli_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
fli_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_fli, m_cg_fli, m_I_fli, fli_motor_parameters = fli_motor_sizing.evaluate()


# fri
fri_motor_sizing = MotorSizing(
    rotor_component=fri_disk, # This will provide the center of mass of the mass 
)
fri_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
fri_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_fri, m_cg_fri, m_I_fri, fri_motor_parameters = fri_motor_sizing.evaluate()


# fro
fro_motor_sizing = MotorSizing(
    rotor_component=fro_disk, # This will provide the center of mass of the mass 
)
fro_motor_sizing.set_module_input('motor_diameter', val=0.17, dv_flag=True, lower=0.08, upper=0.3, scaler=1)
fro_motor_sizing.set_module_input('motor_length', val=0.1, dv_flag=True, lower=0.05, upper=0.12, scaler=1)
m_mass_fro, m_cg_fro, m_I_fro, fro_motor_parameters = fro_motor_sizing.evaluate()

# beam sizing
# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
youngs_modulus = 46E9 # 72.4E9 #
poisson_ratio = 0.33
shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
material_density =  1320  # 2780 #

beams['wing_beam'] = {'E': youngs_modulus, 'G': shear_modulus, 'rho': material_density, 'cs': 'box', 'nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 10,'fdim': [1, 1, 1, 1, 1, 1]}

beam_mass_mesh = MassMesh(
    meshes = dict(
        wing_beam = wing_beam,
        wing_beam_width = width,
        wing_beam_height = height,
    )
)
beam_mass = Mass(component=wing, mesh=beam_mass_mesh, beams=beams, mesh_units='ft')
tweb_array = np.array([0.00722213, 0.00122947, 0.00060947, 0.0005723,  0.00056801 ,0.00056522,
 0.00056304, 0.00058129, 0.00064809, 0.00066848, 0.00066848, 0.00064809,
 0.00058129, 0.00056304, 0.00056522, 0.00056801, 0.0005723  ,0.00060947,
 0.00122947, 0.00722213])
tcap_array = np.array([0.01414826, 0.00188506, 0.00241721, 0.00360027, 0.00492075,0.00635837,
 0.00781769, 0.0092473,  0.01067087, 0.0119696 , 0.0119696 , 0.01067087,
 0.0092473,  0.00781769 ,0.00635837, 0.00492075, 0.00360027 ,0.00241721,
 0.00188506 ,0.01414826])
# wing_beam_tcap = np.array([0.00299052, 0.00161066, 0.00256325, 0.0049267,  0.00713053, 0.00387933, 0.00149764, 0.00141774, 0.00199601, 0.00299244])
# wing_beam_tweb = np.array([[0.00437991, 0.00353425, 0.0035855, 0.00373859, 0.00381257, 0.00342, 0.0032762,  0.003281, 0.00368699, 0.00437148]])
wing_beam_tcap = 0.005 * np.ones((20, ))
wing_beam_tweb = 0.005 * np.ones((20, ))
beam_mass.set_module_input('wing_beam_tcap', val=wing_beam_tcap, dv_flag=True, lower=0.0008, upper=0.02, scaler=1E2)
beam_mass.set_module_input('wing_beam_tweb', val=wing_beam_tweb, dv_flag=True, lower=0.000508, upper=0.02, scaler=1E2)
mass_model_wing_mass = beam_mass.evaluate()

# total constant mass 
constant_mps = cd.TotalConstantMassM3L()
total_constant_mass = constant_mps.evaluate(
    mass_model_wing_mass, 
    battery_mass, 
    mass_m4,
    m_mass_pp,
    m_mass_rlo,
    m_mass_rli,
    m_mass_rri,
    m_mass_rro,
    m_mass_flo,
    m_mass_fli,
    m_mass_fri,
    m_mass_fro,
)
system_m3l_model.register_output(total_constant_mass)
# endregion


# region BEM meshes
pusher_bem_mesh = BEMMesh(
    airfoil='NACA_4412', 
    num_blades=4,
    num_radial=25,
    num_tangential=25,
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

# region +3g sizing condition
plus_3g_condition = cd.CruiseCondition(name="plus_3g_sizing")
plus_3g_condition.atmosphere_model = cd.SimpleAtmosphereModel()
plus_3g_condition.set_module_input(name='altitude', val=1000)
plus_3g_condition.set_module_input(name='mach_number', val=0.24)
plus_3g_condition.set_module_input(name='range', val=1)
plus_3g_condition.set_module_input(name='pitch_angle', val=np.deg2rad(12.249534565223376), dv_flag=True, lower=np.deg2rad(-20), upper=np.deg2rad(20), scaler=1e1)
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
    cl0=[0.25, 0]
)

# aero forces and moments
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=plus_3g_condition)
system_m3l_model.register_output(vlm_force, plus_3g_condition)
system_m3l_model.register_output(vlm_moment, plus_3g_condition)

surfaces = list(wing.get_primitives())
oml_para_mesh = []
grid_num = 20
for name in surfaces:
    for u in np.linspace(0,1,grid_num):
        for v in np.linspace(0,1,grid_num):
            oml_para_mesh.append((name, np.array([u,v]).reshape((1,2))))

oml_geo_mesh = lpc_rep.spatial_representation.evaluate_parametric(oml_para_mesh)
# create the beam model:

vlm_force_mapping_model = VASTNodalForces(
    surface_names=[
        f'{wing_vlm_mesh_name}_plus_3g',
        f'{htail_vlm_mesh_name}_plus_3g',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    initial_meshes=[
        wing_camber_surface,
        htail_camber_surface]
)

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[oml_geo_mesh, htail_oml_mesh], design_condition=plus_3g_condition)
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
bem_model.set_module_input('rpm', val=1639.17444615, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
bem_forces, bem_moments, _, _, _, _,pp_Q, _ = bem_model.evaluate(ac_states=ac_states, design_condition=plus_3g_condition)

# Motor model
# pp_motor_model = cd.ConstantPowerDensityMotorM3L(component=pp_disk)
# pp_input_power = pp_motor_model.evaluate(pp_Q, design_condition=plus_3g_condition)
pp_motor_model = MotorAnalysis(component=pp_disk)
pp_input_power, pp_efficiency = pp_motor_model.evaluate(pp_Q, pp_motor_parameters, design_condition=plus_3g_condition)
system_m3l_model.register_output(pp_input_power, design_condition=plus_3g_condition)

# Energy Consumption
energy_model = cd.EnergyModelM3L()
plus_3g_energy = energy_model.evaluate(pp_input_power, ac_states=ac_states, design_condition=plus_3g_condition)
system_m3l_model.register_output(plus_3g_energy, plus_3g_condition)

beam_mesh = LinearBeamMesh(
    meshes = dict(
        wing_beam = wing_beam,
        wing_beam_width = width,
        wing_beam_height = height,
    )
)
beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints, mesh_units='ft')
# beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
cruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(3,1), control_points_shape=((35,1)))
dummy_function_space = lg.BSplineSetSpace(name='dummy_space', spaces={'dummy_b_spline_space': dummy_b_spline_space})

cruise_wing_displacement_coefficients = m3l.Variable(name='cruise_wing_displacement_coefficients', shape=(35,3))
cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', space=dummy_function_space, coefficients=cruise_wing_displacement_coefficients)

beam_force_map_model = ebbeam.EBBeamForces(component=wing, beam_mesh=beam_mesh, beams=beams, exclude_middle=True)
cruise_structural_wing_mesh_forces = beam_force_map_model.evaluate(nodal_forces=wing_forces,
                                                                   nodal_forces_mesh=oml_geo_mesh,
                                                                   design_condition=plus_3g_condition)

beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
# beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
    forces=cruise_structural_wing_mesh_forces,
    design_condition=plus_3g_condition)

system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, plus_3g_condition)

from m3l.utils.utils import index_functions
nodal_displacment = ebbeam.EBBeamNodalDisplacements(component=wing, beam_mesh=beam_mesh, beams=beams)
surface_names = list(wing.get_primitives().keys())
grid_num = 10
para_grid = []
for name in surface_names:
    for u in np.linspace(0,1,grid_num):
        for v in np.linspace(0,1,grid_num):
            para_grid.append((name, np.array([u,v]).reshape((1,2))))
# grid_thing = for surface in wing.get_primitives(): make grid
evaluated_parametric = lpc_rep.spatial_representation.evaluate_parametric(para_grid)
nodal_displacements = nodal_displacment.evaluate(cruise_structural_wing_mesh_displacements, evaluated_parametric, design_condition=plus_3g_condition)
blinespace = lg.BSplineSpace(name='displacements_spline_space', order=(3, 3), control_points_shape=(5, 5))
displacements_plus_3g = index_functions(surface_names, 'displacements_index', blinespace, 3)
displacements_plus_3g.inverse_evaluate(para_grid, nodal_displacements)
system_m3l_model.register_output(displacements_plus_3g.coefficients, design_condition=plus_3g_condition)

system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, plus_3g_condition)



# Total mass properties
total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=plus_3g_condition)
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
# endregion

# region -1g condition
# minus_1g_model = m3l.Model()
minus_1g_condition = cd.CruiseCondition(name="minus_1g_sizing")
minus_1g_condition.atmosphere_model = cd.SimpleAtmosphereModel()
minus_1g_condition.set_module_input(name='altitude', val=1000)
minus_1g_condition.set_module_input(name='mach_number', val=0.23)
minus_1g_condition.set_module_input(name='range', val=1)
minus_1g_condition.set_module_input(name='pitch_angle', val=np.deg2rad(-13.625869143501449), dv_flag=True, lower=np.deg2rad(-25), upper=np.deg2rad(20), scaler=1e1)
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
    cl0=[0.25, 0]
)

# aero forces and moments
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=minus_1g_condition)
system_m3l_model.register_output(vlm_force, minus_1g_condition)
system_m3l_model.register_output(vlm_moment, minus_1g_condition)

vlm_force_mapping_model = VASTNodalForces(
    surface_names=[
        f'{wing_vlm_mesh_name}_minus_1g',
        f'{htail_vlm_mesh_name}_minus_1g',
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
bem_model.set_module_input('rpm', val=1498.35656739, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
bem_forces, bem_moments, _, _, _, _,pp_Q, _ = bem_model.evaluate(ac_states=ac_states, design_condition=minus_1g_condition)

# Motor model
# pp_motor_model = cd.ConstantPowerDensityMotorM3L(component=pp_disk)
# pp_input_power = pp_motor_model.evaluate(pp_Q, design_condition=minus_1g_condition)
pp_motor_model = MotorAnalysis(component=pp_disk)
pp_input_power, pp_efficiency = pp_motor_model.evaluate(pp_Q, pp_motor_parameters, design_condition=minus_1g_condition)
system_m3l_model.register_output(pp_input_power, design_condition=minus_1g_condition)

# Energy Consumption
energy_model = cd.EnergyModelM3L()
minus_1g_energy = energy_model.evaluate(pp_input_power, ac_states=ac_states, design_condition=minus_1g_condition)
system_m3l_model.register_output(minus_1g_energy, minus_1g_condition)


# create the beam model:
beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints, mesh_units='ft')
# beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
# beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

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
# beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
# beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
    forces=cruise_structural_wing_mesh_forces,
    design_condition=minus_1g_condition)

system_m3l_model.register_output(cruise_structural_wing_mesh_displacements, minus_1g_condition)

from m3l.utils.utils import index_functions
nodal_displacment = ebbeam.EBBeamNodalDisplacements(component=wing, beam_mesh=beam_mesh, beams=beams)
surface_names = list(wing.get_primitives().keys())
grid_num = 10
para_grid = []
for name in surface_names:
    for u in np.linspace(0,1,grid_num):
        for v in np.linspace(0,1,grid_num):
            para_grid.append((name, np.array([u,v]).reshape((1,2))))
# grid_thing = for surface in wing.get_primitives(): make grid
evaluated_parametric = lpc_rep.spatial_representation.evaluate_parametric(para_grid)
nodal_displacements = nodal_displacment.evaluate(cruise_structural_wing_mesh_displacements, evaluated_parametric, design_condition=minus_1g_condition)
blinespace = lg.BSplineSpace(name='displacements_spline_space_m1g', order=(3, 3), control_points_shape=(5, 5))
displacements_minus_1g = index_functions(surface_names, 'displacements_index_m1g', blinespace, 3)
displacements_minus_1g.inverse_evaluate(para_grid, nodal_displacements)
system_m3l_model.register_output(displacements_minus_1g.coefficients, design_condition=minus_1g_condition)

# Total mass properties
total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=minus_1g_condition)
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
# endregion

# region hover oei flo
hover_1_oei_flo = cd.HoverCondition(name='hover_1_oei_flo')
hover_1_oei_flo.atmosphere_model = cd.SimpleAtmosphereModel()
hover_1_oei_flo.set_module_input('altitude', val=300)
hover_1_oei_flo.set_module_input(name='hover_time', val=90)
hover_1_oei_flo.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

hover_1_oei_flo_ac_states = hover_1_oei_flo.evaluate_ac_states()

rlo_bem_model = BEM(disk_prefix='hover_1_oei_flo_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1233.65794802, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments,_ ,_ ,_,_,rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

rli_bem_model = BEM(disk_prefix='hover_1_oei_flo_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1120.44195053, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments,_ ,_ ,_ ,_,rli_Q, rli_ux= rli_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

rri_bem_model = BEM(disk_prefix='hover_1_oei_flo_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
rri_bem_model.set_module_input('rpm', val=918.91319239, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments,_ ,_ ,_,_,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

rro_bem_model = BEM(disk_prefix='hover_1_oei_flo_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments,_ ,_ ,_,_,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

fli_bem_model = BEM(disk_prefix='hover_1_oei_flo_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments,_ ,_ ,_,_ ,fli_Q, fli_ux= fli_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

fri_bem_model = BEM(disk_prefix='hover_1_oei_flo_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments,_ ,_ ,_,_,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

fro_bem_model = BEM(disk_prefix='hover_1_oei_flo_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments,_ ,_ ,_,_,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo)

rotor_index_functions_hover_1_oei_flo = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=hover_1_oei_flo)
    rotor_index_functions_hover_1_oei_flo.append(rlo_ux_function)


# # Hover Motor Analysis
# hover_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# hover_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=hover_1_oei_flo)
hover_rlo_motor_model = MotorAnalysis(component=rlo_disk)
hover_rlo_input_power, hover_rlo_efficiency = hover_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_rlo_input_power, hover_1_oei_flo)

# hover_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# hover_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=hover_1_oei_flo)
hover_rli_motor_model = MotorAnalysis(component=rli_disk)
hover_rli_input_power, hover_rli_efficiency = hover_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_rli_input_power, hover_1_oei_flo)

# hover_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# hover_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=hover_1_oei_flo)
hover_rri_motor_model = MotorAnalysis(component=rri_disk)
hover_rri_input_power, hover_rri_efficiency = hover_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_rri_input_power, hover_1_oei_flo)

# hover_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# hover_rro_input_power = hover_rri_motor_model.evaluate(rro_Q, design_condition=hover_1_oei_flo)
hover_rro_motor_model = MotorAnalysis(component=rro_disk)
hover_rro_input_power, hover_rro_efficiency = hover_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_rro_input_power, hover_1_oei_flo)

# hover_fli_motor_model = cd.ConstantPowerDensityMotorM3L(component=fli_disk)
# hover_fli_input_power = hover_fli_motor_model.evaluate(fli_Q, design_condition=hover_1_oei_flo)
hover_fli_motor_model = MotorAnalysis(component=fli_disk)
hover_fli_input_power, hover_fli_efficiency = hover_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_fli_input_power, hover_1_oei_flo)

# hover_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# hover_fro_input_power = hover_fli_motor_model.evaluate(fro_Q, design_condition=hover_1_oei_flo)
hover_fro_motor_model = MotorAnalysis(component=fro_disk)
hover_fro_input_power, hover_fro_efficiency = hover_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_fro_input_power, hover_1_oei_flo)

# hover_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# hover_fri_input_power = hover_fli_motor_model.evaluate(fri_Q, design_condition=hover_1_oei_flo)
hover_fri_motor_model = MotorAnalysis(component=fri_disk)
hover_fri_input_power, hover_fri_efficiency = hover_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=hover_1_oei_flo)
system_m3l_model.register_output(hover_fri_input_power, hover_1_oei_flo)

# Hover Energy Consumption
hover_oei_fli_energy_model = cd.EnergyModelM3L()
hover_oei_fli_energy = hover_oei_fli_energy_model.evaluate(
    hover_rlo_input_power, hover_rli_input_power, hover_rri_input_power, hover_rro_input_power,
    hover_fli_input_power, hover_fri_input_power, hover_fro_input_power,
    ac_states=hover_1_oei_flo_ac_states, design_condition=hover_1_oei_flo
)

system_m3l_model.register_output(hover_oei_fli_energy, hover_1_oei_flo)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,    
    design_condition=hover_1_oei_flo
)

system_m3l_model.register_output(total_mass, hover_1_oei_flo)
system_m3l_model.register_output(total_cg, hover_1_oei_flo)
system_m3l_model.register_output(total_inertia, hover_1_oei_flo)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=hover_1_oei_flo_ac_states, 
    design_condition=hover_1_oei_flo
)
system_m3l_model.register_output(inertial_forces, hover_1_oei_flo)
system_m3l_model.register_output(inertial_moments, hover_1_oei_flo)

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
    fli_bem_forces, 
    fli_bem_moments,
    fri_bem_forces, 
    fri_bem_moments, 
    fro_bem_forces, 
    fro_bem_moments,  
    inertial_forces, 
    inertial_moments,
    design_condition=hover_1_oei_flo,
)
system_m3l_model.register_output(total_forces,hover_1_oei_flo)
system_m3l_model.register_output(total_moments, hover_1_oei_flo)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=hover_1_oei_flo_ac_states,
    design_condition=hover_1_oei_flo,
)
system_m3l_model.register_output(trim_residual, hover_1_oei_flo)
# endregion

# region hover oei fli
hover_1_oei_fli = cd.HoverCondition(name='hover_1_oei_fli')
hover_1_oei_fli.atmosphere_model = cd.SimpleAtmosphereModel()
hover_1_oei_fli.set_module_input('altitude', val=300)
hover_1_oei_fli.set_module_input(name='hover_time', val=90)
hover_1_oei_fli.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

hover_1_oei_fli_ac_states = hover_1_oei_fli.evaluate_ac_states()

rlo_bem_model = BEM(disk_prefix='hover_1_oei_fli_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments,_ ,_ ,_,_,rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

rli_bem_model = BEM(disk_prefix='hover_1_oei_fli_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments,_ ,_ ,_ ,_,rli_Q, rli_ux= rli_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

rri_bem_model = BEM(disk_prefix='hover_1_oei_fli_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments,_ ,_ ,_,_,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

rro_bem_model = BEM(disk_prefix='hover_1_oei_fli_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments,_ ,_ ,_,_,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

flo_bem_model = BEM(disk_prefix='hover_1_oei_fli_flo_disk', blade_prefix='flo', component=flo_disk, mesh=bem_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments,_ ,_ ,_,_,flo_Q, flo_ux = flo_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

fri_bem_model = BEM(disk_prefix='hover_1_oei_fli_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments,_ ,_ ,_,_ ,fri_Q, fri_ux= fri_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

fro_bem_model = BEM(disk_prefix='hover_1_oei_fli_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments,_ ,_ ,_,_,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli)

rotor_index_functions_hover_1_oei_fli = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': fli_ux, 'mesh': flo_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=hover_1_oei_fli)
    rotor_index_functions_hover_1_oei_fli.append(rlo_ux_function)

# Hover Motor Analysis
# hover_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# hover_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=hover_1_oei_fli)
hover_rlo_motor_model = MotorAnalysis(component=rlo_disk)
hover_rlo_input_power, hover_rlo_efficiency = hover_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_rlo_input_power, hover_1_oei_fli)

# hover_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# hover_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=hover_1_oei_fli)
hover_rli_motor_model = MotorAnalysis(component=rli_disk)
hover_rli_input_power, hover_rli_efficiency = hover_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_rli_input_power, hover_1_oei_fli)

# hover_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# hover_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=hover_1_oei_fli)
hover_rri_motor_model = MotorAnalysis(component=rri_disk)
hover_rri_input_power, hover_rri_efficiency = hover_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_rri_input_power, hover_1_oei_fli)

# hover_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# hover_rro_input_power = hover_rro_motor_model.evaluate(rro_Q, design_condition=hover_1_oei_fli)
hover_rro_motor_model = MotorAnalysis(component=rro_disk)
hover_rro_input_power, hover_rro_efficiency = hover_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_rro_input_power, hover_1_oei_fli)

# hover_flo_motor_model = cd.ConstantPowerDensityMotorM3L(component=flo_disk)
# hover_flo_input_power = hover_flo_motor_model.evaluate(flo_Q, design_condition=hover_1_oei_fli)
hover_flo_motor_model = MotorAnalysis(component=flo_disk)
hover_flo_input_power, hover_flo_efficiency = hover_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_flo_input_power, hover_1_oei_fli)

# hover_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# hover_fro_input_power = hover_fro_motor_model.evaluate(fro_Q, design_condition=hover_1_oei_fli)
hover_fro_motor_model = MotorAnalysis(component=fro_disk)
hover_fro_input_power, hover_fro_efficiency = hover_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_fro_input_power, hover_1_oei_fli)

# hover_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# hover_fri_input_power = hover_fri_motor_model.evaluate(fri_Q, design_condition=hover_1_oei_fli)
hover_fri_motor_model = MotorAnalysis(component=fri_disk)
hover_fri_input_power, hover_fri_efficiency = hover_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=hover_1_oei_fli)
system_m3l_model.register_output(hover_fri_input_power, hover_1_oei_fli)

# Hover Energy Consumption
hover_oei_fli_energy_model = cd.EnergyModelM3L()
hover_oei_fli_energy = hover_oei_fli_energy_model.evaluate(
    hover_rlo_input_power, hover_rli_input_power, hover_rri_input_power, hover_rro_input_power,
    hover_flo_input_power, hover_fri_input_power, hover_fro_input_power,
    ac_states=hover_1_oei_fli_ac_states, design_condition=hover_1_oei_fli
)

system_m3l_model.register_output(hover_oei_fli_energy, hover_1_oei_fli)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,    
    design_condition=hover_1_oei_fli
)

system_m3l_model.register_output(total_mass, hover_1_oei_fli)
system_m3l_model.register_output(total_cg, hover_1_oei_fli)
system_m3l_model.register_output(total_inertia, hover_1_oei_fli)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=hover_1_oei_fli_ac_states, 
    design_condition=hover_1_oei_fli
)
system_m3l_model.register_output(inertial_forces, hover_1_oei_fli)
system_m3l_model.register_output(inertial_moments, hover_1_oei_fli)

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
    fri_bem_forces, 
    fri_bem_moments, 
    fro_bem_forces, 
    fro_bem_moments,  
    inertial_forces, 
    inertial_moments,
    design_condition=hover_1_oei_fli,
)
system_m3l_model.register_output(total_forces,hover_1_oei_fli)
system_m3l_model.register_output(total_moments, hover_1_oei_fli)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=hover_1_oei_fli_ac_states,
    design_condition=hover_1_oei_fli,
)
system_m3l_model.register_output(trim_residual, hover_1_oei_fli)
# endregion


# region qst 1 oei flo
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
# endregion


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
rlo_bem_forces, rlo_bem_moments, rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct,rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rli_bem_model = BEM(disk_prefix='hover_1_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments, rli_dT ,rli_dQ ,rli_dD, rli_Ct,rli_Q, rli_ux = rli_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rri_bem_model = BEM(disk_prefix='hover_1_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments, rri_dT ,rri_dQ ,rri_dD, rri_Ct,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rro_bem_model = BEM(disk_prefix='hover_1_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments, rro_dT ,rro_dQ ,rro_dD, rro_Ct,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

flo_bem_model = BEM(disk_prefix='hover_1_flo_disk', blade_prefix='flo', component=flo_disk, mesh=bem_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments, flo_dT ,flo_dQ ,flo_dD, flo_Ct,flo_Q, flo_ux = flo_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

fli_bem_model = BEM(disk_prefix='hover_1_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments, fli_dT ,fli_dQ ,fli_dD, fli_Ct,fli_Q, fli_ux = fli_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

fri_bem_model = BEM(disk_prefix='hover_1_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments, fri_dT ,fri_dQ ,fri_dD, fri_Ct,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

fro_bem_model = BEM(disk_prefix='hover_1_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments, fro_dT ,fro_dQ ,fro_dD, fro_Ct,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=hover_1_ac_states, design_condition=hover_1)

rotor_index_functions_hover_1 = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': flo_ux, 'mesh': flo_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))
    print(nodes.shape)
    # exit()
    # # Project all points onto each target
    # for target_name in targets.keys():
    #     target = targets[target_name]
    #     target_projected_points = target.project(points=nodes, properties=['geometry', 'parametric_coordinates'])
    #             # properties are not passed in here because we NEED geometry
    #     projected_points_on_each_target.append(target_projected_points)
    #     target_names.append(target_name)
    # num_targets = len(target_names)
    # distances = np.zeros(tuple((num_targets,)) + (nodes.shape[0],))
    # for i in range(num_targets):
    #         distances[i,:] = np.linalg.norm(projected_points_on_each_target[i]['geometry'].value - nodes, axis=-1)
    #         # distances[i,:] = np.linalg.norm(projected_points_on_each_target[i]['geometry'].value - nodes.value, axis=-1)

    # closest_surfaces_indices = np.argmin(distances, axis=0) # Take argmin across surfaces
    # flattened_surface_indices = closest_surfaces_indices.flatten()
    # for i in range(nodes.shape[0]):
    #     target_index = flattened_surface_indices[i]
    #     receiving_target_name = target_names[target_index]
    #     receiving_target = targets[receiving_target_name]
    #     u_coord = projected_points_on_each_target[target_index]['parametric_coordinates'][0][i]
    #     v_coord = projected_points_on_each_target[target_index]['parametric_coordinates'][1][i]
    #     node_parametric_coordinates = np.array([u_coord, v_coord])
    #     nodes_parametric.append((receiving_target_name, node_parametric_coordinates))
    # with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'wb') as f:
    #         pickle.dump(nodes_parametric, f)
    # nodes_parametric_rlo = nodes_parametric
    # exit('EXIT')

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=hover_1)
    rotor_index_functions_hover_1.append(rlo_ux_function)

# # Hover Motor Analysis
# hover_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# hover_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=hover_1)
hover_rlo_motor_model = MotorAnalysis(component=rlo_disk)
hover_rlo_input_power, hover_rlo_efficiency = hover_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_rlo_input_power, hover_1)

# hover_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# hover_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=hover_1)
hover_rli_motor_model = MotorAnalysis(component=rli_disk)
hover_rli_input_power, hover_rli_efficiency = hover_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_rli_input_power, hover_1)

# hover_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# hover_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=hover_1)
hover_rri_motor_model = MotorAnalysis(component=rri_disk)
hover_rri_input_power, hover_rri_efficiency = hover_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_rri_input_power, hover_1)

# hover_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# hover_rro_input_power = hover_rro_motor_model.evaluate(rro_Q, design_condition=hover_1)
hover_rro_motor_model = MotorAnalysis(component=rro_disk)
hover_rro_input_power, hover_rro_efficiency = hover_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_rro_input_power, hover_1)

# hover_flo_motor_model = cd.ConstantPowerDensityMotorM3L(component=flo_disk)
# hover_flo_input_power = hover_flo_motor_model.evaluate(flo_Q, design_condition=hover_1)
hover_flo_motor_model = MotorAnalysis(component=flo_disk)
hover_flo_input_power, hover_flo_efficiency = hover_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_flo_input_power, hover_1)

# hover_fli_motor_model = cd.ConstantPowerDensityMotorM3L(component=fli_disk)
# hover_fli_input_power = hover_fli_motor_model.evaluate(fli_Q, design_condition=hover_1)
hover_fli_motor_model = MotorAnalysis(component=fli_disk)
hover_fli_input_power, hover_fli_efficiency = hover_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_fli_input_power, hover_1)

# hover_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# hover_fro_input_power = hover_fro_motor_model.evaluate(fro_Q, design_condition=hover_1)
hover_fro_motor_model = MotorAnalysis(component=fro_disk)
hover_fro_input_power, hover_fro_efficiency = hover_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_fro_input_power, hover_1)

# hover_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# hover_fri_input_power = hover_fri_motor_model.evaluate(fri_Q, design_condition=hover_1)
hover_fri_motor_model = MotorAnalysis(component=fri_disk)
hover_fri_input_power, hover_fri_efficiency = hover_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=hover_1)
system_m3l_model.register_output(hover_fri_input_power, hover_1)

# Hover Energy Consumption
hover_energy_model = cd.EnergyModelM3L()
hover_energy = hover_energy_model.evaluate(
    hover_rlo_input_power, hover_rli_input_power, hover_rri_input_power, hover_rro_input_power,
    hover_flo_input_power, hover_fli_input_power, hover_fri_input_power, hover_fro_input_power,
    ac_states=hover_1_ac_states, design_condition=hover_1
)

system_m3l_model.register_output(hover_energy, hover_1)


# acoustics
hover_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 90.])
)

hover_acoustics.add_observer(
    name='obs1',
    # obs_position=np.array([53.74, 0., 19.26]),
    obs_position=np.array([0, 0., 0.]),
    time_vector=np.array([0. ]),
)

rlo_ks_model = KS(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rlo_disk', blade_prefix='rlo')
rlo_hover_tonal_SPL, rlo_hover_tonal_SPL_A_weighted = rlo_ks_model.evaluate_tonal_noise(rlo_dT, rlo_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(rlo_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rlo_hover_tonal_SPL_A_weighted, hover_1)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rlo_disk', blade_prefix='rlo')
rlo_hover_broadband_SPL, rlo_hover_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(hover_1_ac_states, rlo_Ct, design_condition=hover_1)
system_m3l_model.register_output(rlo_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rlo_hover_broadband_SPL_A_weighted, hover_1)


rli_ks_model = KS(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rli_disk', blade_prefix='rli')
rli_hover_tonal_SPL, rli_hover_tonal_SPL_A_weighted = rli_ks_model.evaluate_tonal_noise(rli_dT, rli_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(rli_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rli_hover_tonal_SPL_A_weighted, hover_1)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rli_disk', blade_prefix='rli')
rli_hover_broadband_SPL, rli_hover_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(hover_1_ac_states, rli_Ct, design_condition=hover_1)
system_m3l_model.register_output(rli_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rli_hover_broadband_SPL_A_weighted, hover_1)


rri_ks_model = KS(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rri_disk', blade_prefix='rri')
rri_hover_tonal_SPL, rri_hover_tonal_SPL_A_weighted = rri_ks_model.evaluate_tonal_noise(rri_dT, rri_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(rri_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rri_hover_tonal_SPL_A_weighted, hover_1)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rri_disk', blade_prefix='rri')
rri_hover_broadband_SPL, rri_hover_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(hover_1_ac_states, rri_Ct, design_condition=hover_1)
system_m3l_model.register_output(rri_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rri_hover_broadband_SPL_A_weighted, hover_1)


rro_ks_model = KS(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rro_disk', blade_prefix='rro')
rro_hover_tonal_SPL, rro_hover_tonal_SPL_A_weighted = rro_ks_model.evaluate_tonal_noise(rro_dT, rro_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(rro_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(rro_hover_tonal_SPL_A_weighted, hover_1)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_rro_disk', blade_prefix='rro')
rro_hover_broadband_SPL, rro_hover_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(hover_1_ac_states, rro_Ct, design_condition=hover_1)
system_m3l_model.register_output(rro_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(rro_hover_broadband_SPL_A_weighted, hover_1)


flo_ks_model = KS(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_flo_disk', blade_prefix='flo')
flo_hover_tonal_SPL, flo_hover_tonal_SPL_A_weighted = flo_ks_model.evaluate_tonal_noise(flo_dT, flo_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(flo_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(flo_hover_tonal_SPL_A_weighted, hover_1)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_flo_disk', blade_prefix='flo')
flo_hover_broadband_SPL, flo_hover_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(hover_1_ac_states, flo_Ct, design_condition=hover_1)
system_m3l_model.register_output(flo_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(flo_hover_broadband_SPL_A_weighted, hover_1)


fli_ks_model = KS(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fli_disk', blade_prefix='fli')
fli_hover_tonal_SPL, fli_hover_tonal_SPL_A_weighted = fli_ks_model.evaluate_tonal_noise(fli_dT, fli_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(fli_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(fli_hover_tonal_SPL_A_weighted, hover_1)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fli_disk', blade_prefix='fli')
fli_hover_broadband_SPL, fli_hover_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(hover_1_ac_states, fli_Ct, design_condition=hover_1)
system_m3l_model.register_output(fli_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(fli_hover_broadband_SPL_A_weighted, hover_1)


fri_ks_model = KS(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fri_disk', blade_prefix='fri')
fri_hover_tonal_SPL, fri_hover_tonal_SPL_A_weighted = fri_ks_model.evaluate_tonal_noise(fri_dT, fri_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(fri_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(fri_hover_tonal_SPL_A_weighted, hover_1)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fri_disk', blade_prefix='fri')
fri_hover_broadband_SPL, fri_hover_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(hover_1_ac_states, fri_Ct, design_condition=hover_1)
system_m3l_model.register_output(fri_hover_broadband_SPL, hover_1)
system_m3l_model.register_output(fri_hover_broadband_SPL_A_weighted, hover_1)


fro_ks_model = KS(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fro_disk', blade_prefix='fro')
fro_hover_tonal_SPL, fro_hover_tonal_SPL_A_weighted = fro_ks_model.evaluate_tonal_noise(fro_dT, fro_dD, hover_1_ac_states, design_condition=hover_1)
system_m3l_model.register_output(fro_hover_tonal_SPL, hover_1)
system_m3l_model.register_output(fro_hover_tonal_SPL_A_weighted, hover_1)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=hover_acoustics, disk_prefix='hover_1_fro_disk', blade_prefix='fro')
fro_hover_broadband_SPL, fro_hover_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(hover_1_ac_states, fro_Ct, design_condition=hover_1)
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

hover_total_SPL, hover_total_SPL_A_weighted = total_noise_model_hover.evaluate(noise_components, A_weighted_noise_components, design_condition=hover_1)
system_m3l_model.register_output(hover_total_SPL, hover_1)
system_m3l_model.register_output(hover_total_SPL_A_weighted, hover_1)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=hover_1
)

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

# region qst 1
qst_1 = cd.CruiseCondition(name='qst_1')
qst_1.atmosphere_model = cd.SimpleAtmosphereModel()
qst_1.set_module_input('pitch_angle', val=-0.0134037)
qst_1.set_module_input('mach_number', val=0.00029412)
qst_1.set_module_input('altitude', val=300)
qst_1.set_module_input(name='range', val=0.72)
qst_1.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_1_ac_states = qst_1.evaluate_ac_states()


# vlm_model = VASTFluidSover(
#     surface_names=[
#         f"{wing_vlm_mesh_name}_qst_1",
#         f"{htail_vlm_mesh_name}_qst_1",
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
#     mesh_unit='ft',
#     cl0=[0.43, 0]
# )

# forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)
# system_m3l_model.register_output(vlm_forces, design_condition=qst_1)
# system_m3l_model.register_output(vlm_moments, design_condition=qst_1)

# pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
# pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=0, upper=2000, scaler=1e-3)
# pp_bem_forces, pp_bem_moments, _, _, _ = pp_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

rlo_bem_model = BEM(disk_prefix='qst_1_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=bem_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments, rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct,rlo_Q, rlo_ux   = rlo_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

rli_bem_model = BEM(disk_prefix='qst_1_rli_disk', blade_prefix='rli', component=rli_disk, mesh=bem_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments, rli_dT ,rli_dQ ,rli_dD, rli_Ct,rli_Q, rli_ux = rli_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

rri_bem_model = BEM(disk_prefix='qst_1_rri_disk', blade_prefix='rri', component=rri_disk, mesh=bem_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments, rri_dT ,rri_dQ ,rri_dD, rri_Ct,rri_Q, rri_ux= rri_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

rro_bem_model = BEM(disk_prefix='qst_1_rro_disk', blade_prefix='rro', component=rro_disk, mesh=bem_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments, rro_dT ,rro_dQ ,rro_dD, rro_Ct,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

flo_bem_model = BEM(disk_prefix='qst_1_flo_disk', blade_prefix='flo', component=flo_disk, mesh=bem_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments,flo_dT ,flo_dQ ,flo_dD, flo_Ct,flo_Q, flo_ux = flo_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

fli_bem_model = BEM(disk_prefix='qst_1_fli_disk', blade_prefix='fli', component=fli_disk, mesh=bem_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments,fli_dT ,fli_dQ ,fli_dD, fli_Ct,fli_Q, fli_ux = fli_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

fri_bem_model = BEM(disk_prefix='qst_1_fri_disk', blade_prefix='fri', component=fri_disk, mesh=bem_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments, fri_dT ,fri_dQ ,fri_dD, fri_Ct,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

fro_bem_model = BEM(disk_prefix='qst_1_fro_disk', blade_prefix='fro', component=fro_disk, mesh=bem_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments, fro_dT ,fro_dQ ,fro_dD, fro_Ct,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=qst_1_ac_states, design_condition=qst_1)

rotor_index_functions_qst_1 = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': flo_ux, 'mesh': flo_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=qst_1)
    rotor_index_functions_qst_1.append(rlo_ux_function)

# qst 1 Motor Analysis
# qst_1_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# qst_1_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=qst_1)
qst_1_rlo_motor_model = MotorAnalysis(component=rlo_disk)
qst_1_rlo_input_power, qst_1_rlo_efficiency = qst_1_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_rlo_input_power, qst_1)

# qst_1_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# qst_1_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=qst_1)
qst_1_rli_motor_model = MotorAnalysis(component=rli_disk)
qst_1_rli_input_power, qst_1_rli_efficiency = qst_1_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_rli_input_power, qst_1)

# qst_1_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# qst_1_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=qst_1)
qst_1_rri_motor_model = MotorAnalysis(component=rri_disk)
qst_1_rri_input_power, qst_1_rri_efficiency = qst_1_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_rri_input_power, qst_1)

# qst_1_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# qst_1_rro_input_power = hover_rro_motor_model.evaluate(rro_Q, design_condition=qst_1)
qst_1_rro_motor_model = MotorAnalysis(component=rro_disk)
qst_1_rro_input_power, qst_1_rro_efficiency = qst_1_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_rro_input_power, qst_1)

# qst_1_flo_motor_model = cd.ConstantPowerDensityMotorM3L(component=flo_disk)
# qst_1_flo_input_power = hover_flo_motor_model.evaluate(flo_Q, design_condition=qst_1)
qst_1_flo_motor_model = MotorAnalysis(component=flo_disk)
qst_1_flo_input_power, qst_1_flo_efficiency = qst_1_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_flo_input_power, qst_1)

# qst_1_fli_motor_model = cd.ConstantPowerDensityMotorM3L(component=fli_disk)
# qst_1_fli_input_power = hover_fli_motor_model.evaluate(fli_Q, design_condition=qst_1)
qst_1_fli_motor_model = MotorAnalysis(component=fli_disk)
qst_1_fli_input_power, qst_1_fli_efficiency = qst_1_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_fli_input_power, qst_1)

# qst_1_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# qst_1_fro_input_power = hover_fro_motor_model.evaluate(fro_Q, design_condition=qst_1)
qst_1_fro_motor_model = MotorAnalysis(component=fro_disk)
qst_1_fro_input_power, qst_1_fro_efficiency = qst_1_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_fro_input_power, qst_1)

# qst_1_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# qst_1_fri_input_power = hover_fri_motor_model.evaluate(fri_Q, design_condition=qst_1)
qst_1_fri_motor_model = MotorAnalysis(component=fri_disk)
qst_1_fri_input_power, qst_1_fri_efficiency = qst_1_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=qst_1)
system_m3l_model.register_output(qst_1_fri_input_power, qst_1)

# qst_1 Energy Consumption
qst_1_energy_model = cd.EnergyModelM3L()
qst_1_energy = qst_1_energy_model.evaluate(
    qst_1_rlo_input_power, qst_1_rli_input_power, qst_1_rri_input_power, qst_1_rro_input_power,
    qst_1_flo_input_power, qst_1_fli_input_power, qst_1_fri_input_power, qst_1_fro_input_power,
    ac_states=qst_1_ac_states, design_condition=qst_1
)

system_m3l_model.register_output(qst_1_energy, qst_1)

# acoustics
qst_1_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 90.])
)

qst_1_acoustics.add_observer(
    name='obs1',
    # obs_position=np.array([53.74, 0., 19.26]),
    obs_position=np.array([0, 0., 0]),
    time_vector=np.array([0. ]),
)

rlo_ks_model = KS(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rlo_disk', blade_prefix='rlo')
rlo_qst_1_tonal_SPL, rlo_qst_1_tonal_SPL_A_weighted = rlo_ks_model.evaluate_tonal_noise(rlo_dT, rlo_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(rlo_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(rlo_qst_1_tonal_SPL_A_weighted, qst_1)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rlo_disk', blade_prefix='rlo')
rlo_qst_1_broadband_SPL, rlo_qst_1_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(qst_1_ac_states, rlo_Ct, design_condition=qst_1)
system_m3l_model.register_output(rlo_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(rlo_qst_1_broadband_SPL_A_weighted, qst_1)


rli_ks_model = KS(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rli_disk', blade_prefix='rli')
rli_qst_1_tonal_SPL, rli_qst_1_tonal_SPL_A_weighted = rli_ks_model.evaluate_tonal_noise(rli_dT, rli_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(rli_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(rli_qst_1_tonal_SPL_A_weighted, qst_1)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rli_disk', blade_prefix='rli')
rli_qst_1_broadband_SPL, rli_qst_1_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(qst_1_ac_states, rli_Ct, design_condition=qst_1)
system_m3l_model.register_output(rli_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(rli_qst_1_broadband_SPL_A_weighted, qst_1)


rri_ks_model = KS(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rri_disk', blade_prefix='rri')
rri_qst_1_tonal_SPL, rri_qst_1_tonal_SPL_A_weighted = rri_ks_model.evaluate_tonal_noise(rri_dT, rri_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(rri_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(rri_qst_1_tonal_SPL_A_weighted, qst_1)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rri_disk', blade_prefix='rri')
rri_qst_1_broadband_SPL, rri_qst_1_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(qst_1_ac_states, rri_Ct, design_condition=qst_1)
system_m3l_model.register_output(rri_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(rri_qst_1_broadband_SPL_A_weighted, qst_1)


rro_ks_model = KS(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rro_disk', blade_prefix='rro')
rro_qst_1_tonal_SPL, rro_qst_1_tonal_SPL_A_weighted = rro_ks_model.evaluate_tonal_noise(rro_dT, rro_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(rro_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(rro_qst_1_tonal_SPL_A_weighted, qst_1)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_rro_disk', blade_prefix='rro')
rro_qst_1_broadband_SPL, rro_qst_1_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(qst_1_ac_states, rro_Ct, design_condition=qst_1)
system_m3l_model.register_output(rro_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(rro_qst_1_broadband_SPL_A_weighted, qst_1)


flo_ks_model = KS(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_flo_disk', blade_prefix='flo')
flo_qst_1_tonal_SPL, flo_qst_1_tonal_SPL_A_weighted = flo_ks_model.evaluate_tonal_noise(flo_dT, flo_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(flo_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(flo_qst_1_tonal_SPL_A_weighted, qst_1)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_flo_disk', blade_prefix='flo')
flo_qst_1_broadband_SPL, flo_qst_1_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(qst_1_ac_states, flo_Ct, design_condition=qst_1)
system_m3l_model.register_output(flo_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(flo_qst_1_broadband_SPL_A_weighted, qst_1)


fli_ks_model = KS(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_fli_disk', blade_prefix='fli')
fli_qst_1_tonal_SPL, fli_qst_1_tonal_SPL_A_weighted = fli_ks_model.evaluate_tonal_noise(fli_dT, fli_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(fli_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(fli_qst_1_tonal_SPL_A_weighted, qst_1)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_fli_disk', blade_prefix='fli')
fli_qst_1_broadband_SPL, fli_qst_1_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(qst_1_ac_states, fli_Ct, design_condition=qst_1)
system_m3l_model.register_output(fli_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(fli_qst_1_broadband_SPL_A_weighted, qst_1)


fri_ks_model = KS(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_fri_disk', blade_prefix='fri')
fri_qst_1_tonal_SPL, fri_qst_1_tonal_SPL_A_weighted = fri_ks_model.evaluate_tonal_noise(fri_dT, fri_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(fri_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(fri_qst_1_tonal_SPL_A_weighted, qst_1)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_fri_disk', blade_prefix='fri')
fri_qst_1_broadband_SPL, fri_qst_1_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(qst_1_ac_states, fri_Ct, design_condition=qst_1)
system_m3l_model.register_output(fri_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(fri_qst_1_broadband_SPL_A_weighted, qst_1)


fro_ks_model = KS(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_fro_disk', blade_prefix='fro')
fro_qst_1_tonal_SPL, fro_qst_1_tonal_SPL_A_weighted = fro_ks_model.evaluate_tonal_noise(fro_dT, fro_dD, qst_1_ac_states, design_condition=qst_1)
system_m3l_model.register_output(fro_qst_1_tonal_SPL, qst_1)
system_m3l_model.register_output(fro_qst_1_tonal_SPL_A_weighted, qst_1)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_1_acoustics, disk_prefix='qst_1_fro_disk', blade_prefix='fro')
fro_qst_1_broadband_SPL, fro_qst_1_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(qst_1_ac_states, fro_Ct, design_condition=qst_1)
system_m3l_model.register_output(fro_qst_1_broadband_SPL, qst_1)
system_m3l_model.register_output(fro_qst_1_broadband_SPL_A_weighted, qst_1)


total_noise_model_qst_1 = TotalAircraftNoise(
    acoustics_data=qst_1_acoustics,
    component_list=[rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
)
noise_components = [
    rlo_qst_1_tonal_SPL, rlo_qst_1_broadband_SPL,
    rli_qst_1_tonal_SPL, rli_qst_1_broadband_SPL,
    rri_qst_1_tonal_SPL, rri_qst_1_broadband_SPL,
    rro_qst_1_tonal_SPL, rro_qst_1_broadband_SPL,
    flo_qst_1_tonal_SPL, flo_qst_1_broadband_SPL,
    fli_qst_1_tonal_SPL, fli_qst_1_broadband_SPL,
    fri_qst_1_tonal_SPL, fri_qst_1_broadband_SPL,
    fro_qst_1_tonal_SPL, fro_qst_1_broadband_SPL,

]
A_weighted_noise_components = [
    rlo_qst_1_tonal_SPL_A_weighted, rlo_qst_1_broadband_SPL_A_weighted,
    rli_qst_1_tonal_SPL_A_weighted, rli_qst_1_broadband_SPL_A_weighted,
    rri_qst_1_tonal_SPL_A_weighted, rri_qst_1_broadband_SPL_A_weighted,
    rro_qst_1_tonal_SPL_A_weighted, rro_qst_1_broadband_SPL_A_weighted,
    flo_qst_1_tonal_SPL_A_weighted, flo_qst_1_broadband_SPL_A_weighted,
    fli_qst_1_tonal_SPL_A_weighted, fli_qst_1_broadband_SPL_A_weighted,
    fri_qst_1_tonal_SPL_A_weighted, fri_qst_1_broadband_SPL_A_weighted,
    fro_qst_1_tonal_SPL_A_weighted, fro_qst_1_broadband_SPL_A_weighted,

]

qst_1_total_SPL, qst_1_total_SPL_A_weighted = total_noise_model_qst_1.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_1)
system_m3l_model.register_output(qst_1_total_SPL, qst_1)
system_m3l_model.register_output(qst_1_total_SPL_A_weighted, qst_1)


total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_1,
)

system_m3l_model.register_output(total_mass, qst_1)
system_m3l_model.register_output(total_cg, qst_1)
system_m3l_model.register_output(total_inertia, qst_1)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_1_ac_states, 
    design_condition=qst_1
)
system_m3l_model.register_output(inertial_forces, qst_1)
system_m3l_model.register_output(inertial_moments, qst_1)

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
    # pp_bem_forces,
    # pp_bem_moments,
    # vlm_forces,
    # vlm_moments,
    design_condition=qst_1,
)
system_m3l_model.register_output(total_forces,qst_1)
system_m3l_model.register_output(total_moments, qst_1)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_1_ac_states,
    design_condition=qst_1,
)
system_m3l_model.register_output(trim_residual, qst_1)
# endregion

# region qst 2
qst_2 = cd.CruiseCondition(name='qst_2')
qst_2.atmosphere_model = cd.SimpleAtmosphereModel()
qst_2.set_module_input('pitch_angle', val=-0.04973228)
qst_2.set_module_input('mach_number', val=0.06489461)
qst_2.set_module_input('altitude', val=300)
qst_2.set_module_input(name='range', val=158)
qst_2.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


qst_2_ac_states = qst_2.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_2",
        f"{htail_vlm_mesh_name}_qst_2",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(vlm_forces, design_condition=qst_2)
system_m3l_model.register_output(vlm_moments, design_condition=qst_2)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=50, dv_flag=True, lower=50, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct,_, _ = pp_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

rlo_bem_model = PittPeters(disk_prefix='qst_2_rlo_disk',  blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments, rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct, rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

rli_bem_model = PittPeters(disk_prefix='qst_2_rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments, rli_dT ,rli_dQ ,rli_dD, rli_Ct, rli_Q, rli_ux= rli_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

rri_bem_model = PittPeters(disk_prefix='qst_2_rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments, rri_dT ,rri_dQ ,rri_dD, rri_Ct,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

rro_bem_model = PittPeters(disk_prefix='qst_2_rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments, rro_dT ,rro_dQ ,rro_dD, rro_Ct,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

flo_bem_model = PittPeters(disk_prefix='qst_2_flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments, flo_dT ,flo_dQ ,flo_dD, flo_Ct,flo_Q, flo_ux = flo_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

fli_bem_model = PittPeters(disk_prefix='qst_2_fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments, fli_dT ,fli_dQ ,fli_dD, fli_Ct ,fli_Q, fli_ux= fli_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

fri_bem_model = PittPeters(disk_prefix='qst_2_fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments, fri_dT ,fri_dQ ,fri_dD, fri_Ct,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

fro_bem_model = PittPeters(disk_prefix='qst_2_fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments, fro_dT ,fro_dQ ,fro_dD, fro_Ct, fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=qst_2_ac_states, design_condition=qst_2)

rotor_index_functions_qst_2 = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': flo_ux, 'mesh': flo_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=qst_2)
    rotor_index_functions_qst_2.append(rlo_ux_function)

# qst 2 Motor Analysis
# qst_2_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# qst_2_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=qst_2)
qst_2_rlo_motor_model = MotorAnalysis(component=rlo_disk)
qst_2_rlo_input_power, qst_2_rlo_efficiency = qst_2_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_rlo_input_power, qst_2)

# qst_2_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# qst_2_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=qst_2)
qst_2_rli_motor_model = MotorAnalysis(component=rli_disk)
qst_2_rli_input_power, qst_2_rli_efficiency = qst_2_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_rli_input_power, qst_2)

# qst_2_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# qst_2_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=qst_2)
qst_2_rri_motor_model = MotorAnalysis(component=rri_disk)
qst_2_rri_input_power, qst_2_rri_efficiency = qst_2_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_rri_input_power, qst_2)

# qst_2_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# qst_2_rro_input_power = hover_rro_motor_model.evaluate(rro_Q, design_condition=qst_2)
qst_2_rro_motor_model = MotorAnalysis(component=rro_disk)
qst_2_rro_input_power, qst_2_rro_efficiency = qst_2_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_rro_input_power, qst_2)

# qst_2_flo_motor_model = cd.ConstantPowerDensityMotorM3L(component=flo_disk)
# qst_2_flo_input_power = hover_flo_motor_model.evaluate(flo_Q, design_condition=qst_2)
qst_2_flo_motor_model = MotorAnalysis(component=flo_disk)
qst_2_flo_input_power, qst_2_flo_efficiency = qst_2_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_flo_input_power, qst_2)

# qst_2_fli_motor_model = cd.ConstantPowerDensityMotorM3L(component=fli_disk)
# qst_2_fli_input_power = hover_fli_motor_model.evaluate(fli_Q, design_condition=qst_2)
qst_2_fli_motor_model = MotorAnalysis(component=fli_disk)
qst_2_fli_input_power, qst_2_fli_efficiency = qst_2_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_fli_input_power, qst_2)

# qst_2_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# qst_2_fro_input_power = hover_fro_motor_model.evaluate(fro_Q, design_condition=qst_2)
qst_2_fro_motor_model = MotorAnalysis(component=fro_disk)
qst_2_fro_input_power, qst_2_fro_efficiency = qst_2_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_fro_input_power, qst_2)

# qst_2_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# qst_2_fri_input_power = hover_fri_motor_model.evaluate(fri_Q, design_condition=qst_2)
qst_2_fri_motor_model = MotorAnalysis(component=fri_disk)
qst_2_fri_input_power, qst_2_fri_efficiency = qst_2_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=qst_2)
system_m3l_model.register_output(qst_2_fri_input_power, qst_2)

# qst_2 Energy Consumption
qst_2_energy_model = cd.EnergyModelM3L()
qst_2_energy = qst_2_energy_model.evaluate(
    qst_2_rlo_input_power, qst_2_rli_input_power, qst_2_rri_input_power, qst_2_rro_input_power,
    qst_2_flo_input_power, qst_2_fli_input_power, qst_2_fri_input_power, qst_2_fro_input_power,
    ac_states=qst_2_ac_states, design_condition=qst_2
)

system_m3l_model.register_output(qst_2_energy, qst_2)

# acoustics
qst_2_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 90.])
)

qst_2_acoustics.add_observer(
    name='obs1',
    # obs_position=np.array([53.74, 0., 19.26]),
    obs_position=np.array([0, 0., 0.]),
    time_vector=np.array([0. ]),
)

pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_2_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_2_tonal_SPL, pp_qst_2_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(pp_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(pp_qst_2_tonal_SPL_A_weighted, qst_2)

pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_pp_disk', blade_prefix='pp')
pp_qst_2_broadband_SPL, pp_qst_2_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_2_ac_states, pp_Ct, design_condition=qst_2)
system_m3l_model.register_output(pp_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(pp_qst_2_broadband_SPL_A_weighted, qst_2)


rlo_lowson_model = Lowson(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rlo_disk', blade_prefix='rlo')
rlo_qst_2_tonal_SPL, rlo_qst_2_tonal_SPL_A_weighted = rlo_lowson_model.evaluate_tonal_noise(rlo_dT, rlo_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(rlo_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(rlo_qst_2_tonal_SPL_A_weighted, qst_2)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rlo_disk', blade_prefix='rlo')
rlo_qst_2_broadband_SPL, rlo_qst_2_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(qst_2_ac_states, rlo_Ct, design_condition=qst_2)
system_m3l_model.register_output(rlo_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(rlo_qst_2_broadband_SPL_A_weighted, qst_2)


rli_lowson_model = Lowson(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rli_disk', blade_prefix='rli')
rli_qst_2_tonal_SPL, rli_qst_2_tonal_SPL_A_weighted = rli_lowson_model.evaluate_tonal_noise(rli_dT, rli_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(rli_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(rli_qst_2_tonal_SPL_A_weighted, qst_2)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rli_disk', blade_prefix='rli')
rli_qst_2_broadband_SPL, rli_qst_2_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(qst_2_ac_states, rli_Ct, design_condition=qst_2)
system_m3l_model.register_output(rli_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(rli_qst_2_broadband_SPL_A_weighted, qst_2)


rri_lowson_model = Lowson(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rri_disk', blade_prefix='rri')
rri_qst_2_tonal_SPL, rri_qst_2_tonal_SPL_A_weighted = rri_lowson_model.evaluate_tonal_noise(rri_dT, rri_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(rri_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(rri_qst_2_tonal_SPL_A_weighted, qst_2)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rri_disk', blade_prefix='rri')
rri_qst_2_broadband_SPL, rri_qst_2_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(qst_2_ac_states, rri_Ct, design_condition=qst_2)
system_m3l_model.register_output(rri_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(rri_qst_2_broadband_SPL_A_weighted, qst_2)


rro_lowson_model = Lowson(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rro_disk', blade_prefix='rro')
rro_qst_2_tonal_SPL, rro_qst_2_tonal_SPL_A_weighted = rro_lowson_model.evaluate_tonal_noise(rro_dT, rro_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(rro_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(rro_qst_2_tonal_SPL_A_weighted, qst_2)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_rro_disk', blade_prefix='rro')
rro_qst_2_broadband_SPL, rro_qst_2_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(qst_2_ac_states, rro_Ct, design_condition=qst_2)
system_m3l_model.register_output(rro_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(rro_qst_2_broadband_SPL_A_weighted, qst_2)


flo_lowson_model = Lowson(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_flo_disk', blade_prefix='flo')
flo_qst_2_tonal_SPL, flo_qst_2_tonal_SPL_A_weighted = flo_lowson_model.evaluate_tonal_noise(flo_dT, flo_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(flo_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(flo_qst_2_tonal_SPL_A_weighted, qst_2)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_flo_disk', blade_prefix='flo')
flo_qst_2_broadband_SPL, flo_qst_2_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(qst_2_ac_states, flo_Ct, design_condition=qst_2)
system_m3l_model.register_output(flo_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(flo_qst_2_broadband_SPL_A_weighted, qst_2)


fli_lowson_model = Lowson(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_fli_disk', blade_prefix='fli')
fli_qst_2_tonal_SPL, fli_qst_2_tonal_SPL_A_weighted = fli_lowson_model.evaluate_tonal_noise(fli_dT, fli_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(fli_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(fli_qst_2_tonal_SPL_A_weighted, qst_2)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_fli_disk', blade_prefix='fli')
fli_qst_2_broadband_SPL, fli_qst_2_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(qst_2_ac_states, fli_Ct, design_condition=qst_2)
system_m3l_model.register_output(fli_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(fli_qst_2_broadband_SPL_A_weighted, qst_2)


fri_lowson_model = Lowson(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_fri_disk', blade_prefix='fri')
fri_qst_2_tonal_SPL, fri_qst_2_tonal_SPL_A_weighted = fri_lowson_model.evaluate_tonal_noise(fri_dT, fri_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(fri_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(fri_qst_2_tonal_SPL_A_weighted, qst_2)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_fri_disk', blade_prefix='fri')
fri_qst_2_broadband_SPL, fri_qst_2_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(qst_2_ac_states, fri_Ct, design_condition=qst_2)
system_m3l_model.register_output(fri_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(fri_qst_2_broadband_SPL_A_weighted, qst_2)


fro_lowson_model = Lowson(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_fro_disk', blade_prefix='fro')
fro_qst_2_tonal_SPL, fro_qst_2_tonal_SPL_A_weighted = fro_lowson_model.evaluate_tonal_noise(fro_dT, fro_dD, qst_2_ac_states, design_condition=qst_2)
system_m3l_model.register_output(fro_qst_2_tonal_SPL, qst_2)
system_m3l_model.register_output(fro_qst_2_tonal_SPL_A_weighted, qst_2)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_2_acoustics, disk_prefix='qst_2_fro_disk', blade_prefix='fro')
fro_qst_2_broadband_SPL, fro_qst_2_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(qst_2_ac_states, fro_Ct, design_condition=qst_2)
system_m3l_model.register_output(fro_qst_2_broadband_SPL, qst_2)
system_m3l_model.register_output(fro_qst_2_broadband_SPL_A_weighted, qst_2)


total_noise_model_qst_2 = TotalAircraftNoise(
    acoustics_data=qst_2_acoustics,
    component_list=[pp_disk, rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
    # component_list=[rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
)
noise_components = [
    pp_qst_2_tonal_SPL, #pp_qst_2_broadband_SPL,
    rlo_qst_2_tonal_SPL, rlo_qst_2_broadband_SPL,
    rli_qst_2_tonal_SPL, rli_qst_2_broadband_SPL,
    rri_qst_2_tonal_SPL, rri_qst_2_broadband_SPL,
    rro_qst_2_tonal_SPL, rro_qst_2_broadband_SPL,
    flo_qst_2_tonal_SPL, flo_qst_2_broadband_SPL,
    fli_qst_2_tonal_SPL, fli_qst_2_broadband_SPL,
    fri_qst_2_tonal_SPL, fri_qst_2_broadband_SPL,
    fro_qst_2_tonal_SPL, fro_qst_2_broadband_SPL,

]
A_weighted_noise_components = [
    pp_qst_2_tonal_SPL_A_weighted, #pp_qst_2_broadband_SPL_A_weighted,
    rlo_qst_2_tonal_SPL_A_weighted, rlo_qst_2_broadband_SPL_A_weighted,
    rli_qst_2_tonal_SPL_A_weighted, rli_qst_2_broadband_SPL_A_weighted,
    rri_qst_2_tonal_SPL_A_weighted, rri_qst_2_broadband_SPL_A_weighted,
    rro_qst_2_tonal_SPL_A_weighted, rro_qst_2_broadband_SPL_A_weighted,
    flo_qst_2_tonal_SPL_A_weighted, flo_qst_2_broadband_SPL_A_weighted,
    fli_qst_2_tonal_SPL_A_weighted, fli_qst_2_broadband_SPL_A_weighted,
    fri_qst_2_tonal_SPL_A_weighted, fri_qst_2_broadband_SPL_A_weighted,
    fro_qst_2_tonal_SPL_A_weighted, fro_qst_2_broadband_SPL_A_weighted,

]

qst_2_total_SPL, qst_2_total_SPL_A_weighted = total_noise_model_qst_2.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_2)
system_m3l_model.register_output(qst_2_total_SPL, qst_2)
system_m3l_model.register_output(qst_2_total_SPL_A_weighted, qst_2)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_2
)

system_m3l_model.register_output(total_mass, qst_2)
system_m3l_model.register_output(total_cg, qst_2)
system_m3l_model.register_output(total_inertia, qst_2)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_2_ac_states, 
    design_condition=qst_2
)
system_m3l_model.register_output(inertial_forces, qst_2)
system_m3l_model.register_output(inertial_moments, qst_2)

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
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_2,
)
system_m3l_model.register_output(total_forces,qst_2)
system_m3l_model.register_output(total_moments, qst_2)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_2_ac_states,
    design_condition=qst_2,
)
system_m3l_model.register_output(trim_residual, qst_2)
# endregion

# region qst 3
qst_3 = cd.CruiseCondition(name='qst_3')
qst_3.atmosphere_model = cd.SimpleAtmosphereModel()
qst_3.set_module_input('pitch_angle', val=0.16195989)
qst_3.set_module_input('mach_number', val=0.11471427)
qst_3.set_module_input('altitude', val=300)
qst_3.set_module_input(name='range', val=280)
qst_3.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_3_ac_states = qst_3.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_3",
        f"{htail_vlm_mesh_name}_qst_3",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(vlm_forces, design_condition=qst_3)
system_m3l_model.register_output(vlm_moments, design_condition=qst_3)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=700, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct,qst_3_pp_Q,_ = pp_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

rlo_bem_model = PittPeters(disk_prefix='qst_3_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments,rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct,rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

rli_bem_model = PittPeters(disk_prefix='qst_3_rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments,rli_dT ,rli_dQ ,rli_dD, rli_Ct, rli_Q, rli_ux = rli_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

rri_bem_model = PittPeters(disk_prefix='qst_3_rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments,rri_dT ,rri_dQ ,rri_dD, rri_Ct,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

rro_bem_model = PittPeters(disk_prefix='qst_3_rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments, rro_dT ,rro_dQ ,rro_dD, rro_Ct,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

flo_bem_model = PittPeters(disk_prefix='qst_3_flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments,flo_dT ,flo_dQ ,flo_dD, flo_Ct,flo_Q, flo_ux = flo_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

fli_bem_model = PittPeters(disk_prefix='qst_3_fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments,fli_dT ,fli_dQ ,fli_dD, fli_Ct,fli_Q, fli_ux = fli_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

fri_bem_model = PittPeters(disk_prefix='qst_3_fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments, fri_dT ,fri_dQ ,fri_dD, fri_Ct,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

fro_bem_model = PittPeters(disk_prefix='qst_3_fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=20, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments, fro_dT ,fro_dQ ,fro_dD, fro_Ct,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=qst_3_ac_states, design_condition=qst_3)

rotor_index_functions_qst_3 = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': flo_ux, 'mesh': flo_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=qst_2)
    rotor_index_functions_qst_3.append(rlo_ux_function)

# qst 3 Motor Analysis
qst_3_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_3_pp_input_power, qst_3_pp_efficiency = qst_3_pp_motor_model.evaluate(qst_3_pp_Q, pp_motor_parameters, design_condition=qst_3)
system_m3l_model.register_output(qst_3_pp_input_power, qst_3)

# qst_3_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# qst_3_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=qst_3)
# # qst_3_rlo_motor_model = MotorAnalysis(component=rlo_disk)
# # qst_3_rlo_input_power, qst_3_rlo_efficiency = qst_3_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_rlo_input_power, qst_3)

# qst_3_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# qst_3_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=qst_3)
# # qst_3_rli_motor_model = MotorAnalysis(component=rli_disk)
# # qst_3_rli_input_power, qst_3_rli_efficiency = qst_3_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_rli_input_power, qst_3)

# qst_3_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# qst_3_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=qst_3)
# # qst_3_rri_motor_model = MotorAnalysis(component=rri_disk)
# # qst_3_rri_input_power, qst_3_rri_efficiency = qst_3_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_rri_input_power, qst_3)

# qst_3_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# qst_3_rro_input_power = hover_rro_motor_model.evaluate(rro_Q, design_condition=qst_3)
# # qst_3_rro_motor_model = MotorAnalysis(component=rro_disk)
# # qst_3_rro_input_power, qst_3_rro_efficiency = qst_3_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_rro_input_power, qst_3)

# qst_3_flo_motor_model = cd.ConstantPowerDensityMotorM3L(component=flo_disk)
# qst_3_flo_input_power = hover_flo_motor_model.evaluate(flo_Q, design_condition=qst_3)
# # qst_3_flo_motor_model = MotorAnalysis(component=flo_disk)
# # qst_3_flo_input_power, qst_3_flo_efficiency = qst_3_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_flo_input_power, qst_3)

# qst_3_fli_motor_model = cd.ConstantPowerDensityMotorM3L(component=fli_disk)
# qst_3_fli_input_power = hover_fli_motor_model.evaluate(fli_Q, design_condition=qst_3)
# # qst_3_fli_motor_model = MotorAnalysis(component=fli_disk)
# # qst_3_fli_input_power, qst_3_fli_efficiency = qst_3_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_fli_input_power, qst_3)

# qst_3_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# qst_3_fro_input_power = hover_fro_motor_model.evaluate(fro_Q, design_condition=qst_3)
# # qst_3_fro_motor_model = MotorAnalysis(component=fro_disk)
# # qst_3_fro_input_power, qst_3_fro_efficiency = qst_3_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_fro_input_power, qst_3)

# qst_3_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# qst_3_fri_input_power = hover_fri_motor_model.evaluate(fri_Q, design_condition=qst_3)
# # qst_3_fri_motor_model = MotorAnalysis(component=fri_disk)
# # qst_3_fri_input_power, qst_3_fri_efficiency = qst_3_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=qst_3)
# system_m3l_model.register_output(qst_3_fri_input_power, qst_3)

# qst 3 Energy Consumption
qst_3_energy_model = cd.EnergyModelM3L()
qst_3_energy = qst_3_energy_model.evaluate(
    qst_3_pp_input_power,
    # qst_3_rlo_input_power, qst_3_rli_input_power, qst_3_rri_input_power, qst_3_rro_input_power,
    # qst_3_flo_input_power, qst_3_fli_input_power, qst_3_fri_input_power, qst_3_fro_input_power,
    ac_states=qst_3_ac_states, design_condition=qst_3
)

system_m3l_model.register_output(qst_3_energy, qst_3)

# acoustics
qst_3_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 90.])
)

qst_3_acoustics.add_observer(
    name='obs1',
    # obs_position=np.array([53.74, 0., 19.26]),
    obs_position=np.array([0, 0., 0.]),
    time_vector=np.array([0. ]),
)

pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_3_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_3_tonal_SPL, pp_qst_3_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(pp_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(pp_qst_3_tonal_SPL_A_weighted, qst_3)

pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_3_broadband_SPL, pp_qst_3_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_3_ac_states, pp_Ct, design_condition=qst_3)
system_m3l_model.register_output(pp_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(pp_qst_3_broadband_SPL_A_weighted, qst_3)

rlo_lowson_model = Lowson(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rlo_disk', blade_prefix='rlo')
rlo_qst_3_tonal_SPL, rlo_qst_3_tonal_SPL_A_weighted = rlo_lowson_model.evaluate_tonal_noise(rlo_dT, rlo_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(rlo_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(rlo_qst_3_tonal_SPL_A_weighted, qst_3)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rlo_disk', blade_prefix='rlo')
rlo_qst_3_broadband_SPL, rlo_qst_3_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(qst_3_ac_states, rlo_Ct, design_condition=qst_3)
system_m3l_model.register_output(rlo_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(rlo_qst_3_broadband_SPL_A_weighted, qst_3)


rli_lowson_model = Lowson(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rli_disk', blade_prefix='rli')
rli_qst_3_tonal_SPL, rli_qst_3_tonal_SPL_A_weighted = rli_lowson_model.evaluate_tonal_noise(rli_dT, rli_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(rli_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(rli_qst_3_tonal_SPL_A_weighted, qst_3)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rli_disk', blade_prefix='rli')
rli_qst_3_broadband_SPL, rli_qst_3_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(qst_3_ac_states, rli_Ct, design_condition=qst_3)
system_m3l_model.register_output(rli_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(rli_qst_3_broadband_SPL_A_weighted, qst_3)


rri_lowson_model = Lowson(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rri_disk', blade_prefix='rri')
rri_qst_3_tonal_SPL, rri_qst_3_tonal_SPL_A_weighted = rri_lowson_model.evaluate_tonal_noise(rri_dT, rri_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(rri_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(rri_qst_3_tonal_SPL_A_weighted, qst_3)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rri_disk', blade_prefix='rri')
rri_qst_3_broadband_SPL, rri_qst_3_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(qst_3_ac_states, rri_Ct, design_condition=qst_3)
system_m3l_model.register_output(rri_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(rri_qst_3_broadband_SPL_A_weighted, qst_3)


rro_lowson_model = Lowson(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rro_disk', blade_prefix='rro')
rro_qst_3_tonal_SPL, rro_qst_3_tonal_SPL_A_weighted = rro_lowson_model.evaluate_tonal_noise(rro_dT, rro_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(rro_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(rro_qst_3_tonal_SPL_A_weighted, qst_3)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_rro_disk', blade_prefix='rro')
rro_qst_3_broadband_SPL, rro_qst_3_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(qst_3_ac_states, rro_Ct, design_condition=qst_3)
system_m3l_model.register_output(rro_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(rro_qst_3_broadband_SPL_A_weighted, qst_3)


flo_lowson_model = Lowson(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_flo_disk', blade_prefix='flo')
flo_qst_3_tonal_SPL, flo_qst_3_tonal_SPL_A_weighted = flo_lowson_model.evaluate_tonal_noise(flo_dT, flo_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(flo_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(flo_qst_3_tonal_SPL_A_weighted, qst_3)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_flo_disk', blade_prefix='flo')
flo_qst_3_broadband_SPL, flo_qst_3_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(qst_3_ac_states, flo_Ct, design_condition=qst_3)
system_m3l_model.register_output(flo_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(flo_qst_3_broadband_SPL_A_weighted, qst_3)


fli_lowson_model = Lowson(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_fli_disk', blade_prefix='fli')
fli_qst_3_tonal_SPL, fli_qst_3_tonal_SPL_A_weighted = fli_lowson_model.evaluate_tonal_noise(fli_dT, fli_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(fli_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(fli_qst_3_tonal_SPL_A_weighted, qst_3)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_fli_disk', blade_prefix='fli')
fli_qst_3_broadband_SPL, fli_qst_3_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(qst_3_ac_states, fli_Ct, design_condition=qst_3)
system_m3l_model.register_output(fli_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(fli_qst_3_broadband_SPL_A_weighted, qst_3)


fri_lowson_model = Lowson(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_fri_disk', blade_prefix='fri')
fri_qst_3_tonal_SPL, fri_qst_3_tonal_SPL_A_weighted = fri_lowson_model.evaluate_tonal_noise(fri_dT, fri_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(fri_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(fri_qst_3_tonal_SPL_A_weighted, qst_3)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_fri_disk', blade_prefix='fri')
fri_qst_3_broadband_SPL, fri_qst_3_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(qst_3_ac_states, fri_Ct, design_condition=qst_3)
system_m3l_model.register_output(fri_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(fri_qst_3_broadband_SPL_A_weighted, qst_3)


fro_lowson_model = Lowson(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_fro_disk', blade_prefix='fro')
fro_qst_3_tonal_SPL, fro_qst_3_tonal_SPL_A_weighted = fro_lowson_model.evaluate_tonal_noise(fro_dT, fro_dD, qst_3_ac_states, design_condition=qst_3)
system_m3l_model.register_output(fro_qst_3_tonal_SPL, qst_3)
system_m3l_model.register_output(fro_qst_3_tonal_SPL_A_weighted, qst_3)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_3_acoustics, disk_prefix='qst_3_fro_disk', blade_prefix='fro')
fro_qst_3_broadband_SPL, fro_qst_3_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(qst_3_ac_states, fro_Ct, design_condition=qst_3)
system_m3l_model.register_output(fro_qst_3_broadband_SPL, qst_3)
system_m3l_model.register_output(fro_qst_3_broadband_SPL_A_weighted, qst_3)


total_noise_model_qst_3 = TotalAircraftNoise(
    acoustics_data=qst_3_acoustics,
    component_list=[pp_disk, rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
    # component_list=[rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
)
noise_components = [
    pp_qst_3_tonal_SPL, #pp_qst_3_broadband_SPL,
    rlo_qst_3_tonal_SPL, rlo_qst_3_broadband_SPL,
    rli_qst_3_tonal_SPL, rli_qst_3_broadband_SPL,
    rri_qst_3_tonal_SPL, rri_qst_3_broadband_SPL,
    rro_qst_3_tonal_SPL, rro_qst_3_broadband_SPL,
    flo_qst_3_tonal_SPL, flo_qst_3_broadband_SPL,
    fli_qst_3_tonal_SPL, fli_qst_3_broadband_SPL,
    fri_qst_3_tonal_SPL, fri_qst_3_broadband_SPL,
    fro_qst_3_tonal_SPL, fro_qst_3_broadband_SPL,

]
A_weighted_noise_components = [
    pp_qst_3_tonal_SPL_A_weighted, #pp_qst_3_broadband_SPL_A_weighted,
    rlo_qst_3_tonal_SPL_A_weighted, rlo_qst_3_broadband_SPL_A_weighted,
    rli_qst_3_tonal_SPL_A_weighted, rli_qst_3_broadband_SPL_A_weighted,
    rri_qst_3_tonal_SPL_A_weighted, rri_qst_3_broadband_SPL_A_weighted,
    rro_qst_3_tonal_SPL_A_weighted, rro_qst_3_broadband_SPL_A_weighted,
    flo_qst_3_tonal_SPL_A_weighted, flo_qst_3_broadband_SPL_A_weighted,
    fli_qst_3_tonal_SPL_A_weighted, fli_qst_3_broadband_SPL_A_weighted,
    fri_qst_3_tonal_SPL_A_weighted, fri_qst_3_broadband_SPL_A_weighted,
    fro_qst_3_tonal_SPL_A_weighted, fro_qst_3_broadband_SPL_A_weighted,

]

qst_3_total_SPL, qst_3_total_SPL_A_weighted = total_noise_model_qst_3.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_3)
system_m3l_model.register_output(qst_3_total_SPL, qst_3)
system_m3l_model.register_output(qst_3_total_SPL_A_weighted, qst_3)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_3
)

system_m3l_model.register_output(total_mass, qst_3)
system_m3l_model.register_output(total_cg, qst_3)
system_m3l_model.register_output(total_inertia, qst_3)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_3_ac_states, 
    design_condition=qst_3
)
system_m3l_model.register_output(inertial_forces, qst_3)
system_m3l_model.register_output(inertial_moments, qst_3)

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
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_3,
)
system_m3l_model.register_output(total_forces,qst_3)
system_m3l_model.register_output(total_moments, qst_3)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_3_ac_states,
    design_condition=qst_3,
)
system_m3l_model.register_output(trim_residual, qst_3)
# endregion

# region qst 4
qst_4 = cd.CruiseCondition(name='qst_4')
qst_4.atmosphere_model = cd.SimpleAtmosphereModel()
qst_4.set_module_input('pitch_angle', val=0.10779469, dv_flag=True, lower=0.10779469 - np.deg2rad(2), scaler=1e1)
qst_4.set_module_input('mach_number', val=0.13740796)
qst_4.set_module_input('altitude', val=300)
qst_4.set_module_input(name='range', val=336)
qst_4.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


qst_4_ac_states = qst_4.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_4",
        f"{htail_vlm_mesh_name}_qst_4",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(vlm_forces, design_condition=qst_4)
system_m3l_model.register_output(vlm_moments, design_condition=qst_4)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct, qst_4_pp_Q,_ = pp_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

rlo_bem_model = PittPeters(disk_prefix='qst_4_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments,rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct,rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

rli_bem_model = PittPeters(disk_prefix='qst_4_rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments, rli_dT ,rli_dQ ,rli_dD, rli_Ct,rli_Q, rli_ux = rli_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

rri_bem_model = PittPeters(disk_prefix='qst_4_rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments,rri_dT ,rri_dQ ,rri_dD, rri_Ct,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

rro_bem_model = PittPeters(disk_prefix='qst_4_rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments,rro_dT ,rro_dQ ,rro_dD, rro_Ct,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

flo_bem_model = PittPeters(disk_prefix='qst_4_flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments,flo_dT ,flo_dQ ,flo_dD, flo_Ct,flo_Q, flo_ux= flo_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

fli_bem_model = PittPeters(disk_prefix='qst_4_fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments,fli_dT ,fli_dQ ,fli_dD, fli_Ct,fli_Q, fli_ux = fli_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

fri_bem_model = PittPeters(disk_prefix='qst_4_fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments,fri_dT ,fri_dQ ,fri_dD, fri_Ct,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)

fro_bem_model = PittPeters(disk_prefix='qst_4_fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments,fro_dT ,fro_dQ ,fro_dD, fro_Ct,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=qst_4_ac_states, design_condition=qst_4)


rotor_index_functions_qst_4 = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': flo_ux, 'mesh': flo_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=qst_2)
    rotor_index_functions_qst_4.append(rlo_ux_function)


# # qst 4 Motor Analysis
qst_4_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_4_pp_input_power, qst_4_pp_efficiency = qst_4_pp_motor_model.evaluate(qst_4_pp_Q, pp_motor_parameters, design_condition=qst_4)
system_m3l_model.register_output(qst_4_pp_input_power, qst_4)

# qst_4_rlo_motor_model = cd.ConstantPowerDensityMotorM3L(component=rlo_disk)
# qst_4_rlo_input_power = hover_rlo_motor_model.evaluate(rlo_Q, design_condition=qst_4)
# # qst_4_rlo_motor_model = MotorAnalysis(component=rlo_disk)
# # qst_4_rlo_input_power, qst_4_rlo_efficiency = qst_4_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_rlo_input_power, qst_4)

# qst_4_rli_motor_model = cd.ConstantPowerDensityMotorM3L(component=rli_disk)
# qst_4_rli_input_power = hover_rli_motor_model.evaluate(rli_Q, design_condition=qst_4)
# # qst_4_rli_motor_model = MotorAnalysis(component=rli_disk)
# # qst_4_rli_input_power, qst_4_rli_efficiency = qst_4_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_rli_input_power, qst_4)

# qst_4_rri_motor_model = cd.ConstantPowerDensityMotorM3L(component=rri_disk)
# qst_4_rri_input_power = hover_rri_motor_model.evaluate(rri_Q, design_condition=qst_4)
# # qst_4_rri_motor_model = MotorAnalysis(component=rri_disk)
# # qst_4_rri_input_power, qst_4_rri_efficiency = qst_4_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_rri_input_power, qst_4)

# qst_4_rro_motor_model = cd.ConstantPowerDensityMotorM3L(component=rro_disk)
# qst_4_rro_input_power = hover_rro_motor_model.evaluate(rro_Q, design_condition=qst_4)
# # qst_4_rro_motor_model = MotorAnalysis(component=rro_disk)
# # qst_4_rro_input_power, qst_4_rro_efficiency = qst_4_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_rro_input_power, qst_4)

# qst_4_flo_motor_model = cd.ConstantPowerDensityMotorM3L(component=flo_disk)
# qst_4_flo_input_power = hover_flo_motor_model.evaluate(flo_Q, design_condition=qst_4)
# # qst_4_flo_motor_model = MotorAnalysis(component=flo_disk)
# # qst_4_flo_input_power, qst_4_flo_efficiency = qst_4_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_flo_input_power, qst_4)

# qst_4_fli_motor_model = cd.ConstantPowerDensityMotorM3L(component=fli_disk)
# qst_4_fli_input_power = hover_fli_motor_model.evaluate(fli_Q, design_condition=qst_4)
# # qst_4_fli_motor_model = MotorAnalysis(component=fli_disk)
# # qst_4_fli_input_power, qst_4_fli_efficiency = qst_4_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_fli_input_power, qst_4)

# qst_4_fro_motor_model = cd.ConstantPowerDensityMotorM3L(component=fro_disk)
# qst_4_fro_input_power = hover_fro_motor_model.evaluate(fro_Q, design_condition=qst_4)
# # qst_4_fro_motor_model = MotorAnalysis(component=fro_disk)
# # qst_4_fro_input_power, qst_4_fro_efficiency = qst_4_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_fro_input_power, qst_4)

# qst_4_fri_motor_model = cd.ConstantPowerDensityMotorM3L(component=fri_disk)
# qst_4_fri_input_power = hover_fri_motor_model.evaluate(fri_Q, design_condition=qst_4)
# # qst_4_fri_motor_model = MotorAnalysis(component=fri_disk)
# # qst_4_fri_input_power, qst_4_fri_efficiency = qst_4_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=qst_4)
# system_m3l_model.register_output(qst_4_fri_input_power, qst_4)

# qst 4 Energy Consumption
qst_4_energy_model = cd.EnergyModelM3L()
qst_4_energy = qst_4_energy_model.evaluate(
    qst_4_pp_input_power,
    # qst_4_rlo_input_power, qst_4_rli_input_power, qst_4_rri_input_power, qst_4_rro_input_power,
    # qst_4_flo_input_power, qst_4_fli_input_power, qst_4_fri_input_power, qst_4_fro_input_power,
    ac_states=qst_4_ac_states, design_condition=qst_4
)

system_m3l_model.register_output(qst_4_energy, qst_4)

# acoustics
qst_4_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 90.])
)

qst_4_acoustics.add_observer(
    name='obs1',
    obs_position=np.array([0, 0., 0.]),
    # obs_position=np.array([53.74, 0., 19.26]),
    time_vector=np.array([0. ]),
)

pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_4_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_4_tonal_SPL, pp_qst_4_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(pp_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(pp_qst_4_tonal_SPL_A_weighted, qst_4)

pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_4_broadband_SPL, pp_qst_4_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_4_ac_states, pp_Ct, design_condition=qst_4)
system_m3l_model.register_output(pp_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(pp_qst_4_broadband_SPL_A_weighted, qst_4)

rlo_lowson_model = Lowson(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rlo_disk', blade_prefix='rlo')
rlo_qst_4_tonal_SPL, rlo_qst_4_tonal_SPL_A_weighted = rlo_lowson_model.evaluate_tonal_noise(rlo_dT, rlo_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(rlo_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(rlo_qst_4_tonal_SPL_A_weighted, qst_4)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rlo_disk', blade_prefix='rlo')
rlo_qst_4_broadband_SPL, rlo_qst_4_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(qst_4_ac_states, rlo_Ct, design_condition=qst_4)
system_m3l_model.register_output(rlo_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(rlo_qst_4_broadband_SPL_A_weighted, qst_4)


rli_lowson_model = Lowson(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rli_disk', blade_prefix='rli')
rli_qst_4_tonal_SPL, rli_qst_4_tonal_SPL_A_weighted = rli_lowson_model.evaluate_tonal_noise(rli_dT, rli_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(rli_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(rli_qst_4_tonal_SPL_A_weighted, qst_4)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rli_disk', blade_prefix='rli')
rli_qst_4_broadband_SPL, rli_qst_4_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(qst_4_ac_states, rli_Ct, design_condition=qst_4)
system_m3l_model.register_output(rli_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(rli_qst_4_broadband_SPL_A_weighted, qst_4)


rri_lowson_model = Lowson(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rri_disk', blade_prefix='rri')
rri_qst_4_tonal_SPL, rri_qst_4_tonal_SPL_A_weighted = rri_lowson_model.evaluate_tonal_noise(rri_dT, rri_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(rri_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(rri_qst_4_tonal_SPL_A_weighted, qst_4)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rri_disk', blade_prefix='rri')
rri_qst_4_broadband_SPL, rri_qst_4_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(qst_4_ac_states, rri_Ct, design_condition=qst_4)
system_m3l_model.register_output(rri_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(rri_qst_4_broadband_SPL_A_weighted, qst_4)


rro_lowson_model = Lowson(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rro_disk', blade_prefix='rro')
rro_qst_4_tonal_SPL, rro_qst_4_tonal_SPL_A_weighted = rro_lowson_model.evaluate_tonal_noise(rro_dT, rro_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(rro_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(rro_qst_4_tonal_SPL_A_weighted, qst_4)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_rro_disk', blade_prefix='rro')
rro_qst_4_broadband_SPL, rro_qst_4_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(qst_4_ac_states, rro_Ct, design_condition=qst_4)
system_m3l_model.register_output(rro_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(rro_qst_4_broadband_SPL_A_weighted, qst_4)


flo_lowson_model = Lowson(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_flo_disk', blade_prefix='flo')
flo_qst_4_tonal_SPL, flo_qst_4_tonal_SPL_A_weighted = flo_lowson_model.evaluate_tonal_noise(flo_dT, flo_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(flo_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(flo_qst_4_tonal_SPL_A_weighted, qst_4)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_flo_disk', blade_prefix='flo')
flo_qst_4_broadband_SPL, flo_qst_4_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(qst_4_ac_states, flo_Ct, design_condition=qst_4)
system_m3l_model.register_output(flo_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(flo_qst_4_broadband_SPL_A_weighted, qst_4)


fli_lowson_model = Lowson(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_fli_disk', blade_prefix='fli')
fli_qst_4_tonal_SPL, fli_qst_4_tonal_SPL_A_weighted = fli_lowson_model.evaluate_tonal_noise(fli_dT, fli_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(fli_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(fli_qst_4_tonal_SPL_A_weighted, qst_4)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_fli_disk', blade_prefix='fli')
fli_qst_4_broadband_SPL, fli_qst_4_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(qst_4_ac_states, fli_Ct, design_condition=qst_4)
system_m3l_model.register_output(fli_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(fli_qst_4_broadband_SPL_A_weighted, qst_4)


fri_lowson_model = Lowson(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_fri_disk', blade_prefix='fri')
fri_qst_4_tonal_SPL, fri_qst_4_tonal_SPL_A_weighted = fri_lowson_model.evaluate_tonal_noise(fri_dT, fri_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(fri_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(fri_qst_4_tonal_SPL_A_weighted, qst_4)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_fri_disk', blade_prefix='fri')
fri_qst_4_broadband_SPL, fri_qst_4_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(qst_4_ac_states, fri_Ct, design_condition=qst_4)
system_m3l_model.register_output(fri_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(fri_qst_4_broadband_SPL_A_weighted, qst_4)


fro_lowson_model = Lowson(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_fro_disk', blade_prefix='fro')
fro_qst_4_tonal_SPL, fro_qst_4_tonal_SPL_A_weighted = fro_lowson_model.evaluate_tonal_noise(fro_dT, fro_dD, qst_4_ac_states, design_condition=qst_4)
system_m3l_model.register_output(fro_qst_4_tonal_SPL, qst_4)
system_m3l_model.register_output(fro_qst_4_tonal_SPL_A_weighted, qst_4)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_4_acoustics, disk_prefix='qst_4_fro_disk', blade_prefix='fro')
fro_qst_4_broadband_SPL, fro_qst_4_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(qst_4_ac_states, fro_Ct, design_condition=qst_4)
system_m3l_model.register_output(fro_qst_4_broadband_SPL, qst_4)
system_m3l_model.register_output(fro_qst_4_broadband_SPL_A_weighted, qst_4)


total_noise_model_qst_4 = TotalAircraftNoise(
    acoustics_data=qst_4_acoustics,
    component_list=[pp_disk, rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
    # component_list=[rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
)
noise_components = [
    pp_qst_4_tonal_SPL, #pp_qst_4_broadband_SPL,
    rlo_qst_4_tonal_SPL, rlo_qst_4_broadband_SPL,
    rli_qst_4_tonal_SPL, rli_qst_4_broadband_SPL,
    rri_qst_4_tonal_SPL, rri_qst_4_broadband_SPL,
    rro_qst_4_tonal_SPL, rro_qst_4_broadband_SPL,
    flo_qst_4_tonal_SPL, flo_qst_4_broadband_SPL,
    fli_qst_4_tonal_SPL, fli_qst_4_broadband_SPL,
    fri_qst_4_tonal_SPL, fri_qst_4_broadband_SPL,
    fro_qst_4_tonal_SPL, fro_qst_4_broadband_SPL,

]
A_weighted_noise_components = [
    pp_qst_4_tonal_SPL_A_weighted, #pp_qst_4_broadband_SPL_A_weighted,
    rlo_qst_4_tonal_SPL_A_weighted, rlo_qst_4_broadband_SPL_A_weighted,
    rli_qst_4_tonal_SPL_A_weighted, rli_qst_4_broadband_SPL_A_weighted,
    rri_qst_4_tonal_SPL_A_weighted, rri_qst_4_broadband_SPL_A_weighted,
    rro_qst_4_tonal_SPL_A_weighted, rro_qst_4_broadband_SPL_A_weighted,
    flo_qst_4_tonal_SPL_A_weighted, flo_qst_4_broadband_SPL_A_weighted,
    fli_qst_4_tonal_SPL_A_weighted, fli_qst_4_broadband_SPL_A_weighted,
    fri_qst_4_tonal_SPL_A_weighted, fri_qst_4_broadband_SPL_A_weighted,
    fro_qst_4_tonal_SPL_A_weighted, fro_qst_4_broadband_SPL_A_weighted,

]

qst_4_total_SPL, qst_4_total_SPL_A_weighted = total_noise_model_qst_4.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_4)
system_m3l_model.register_output(qst_4_total_SPL, qst_4)
system_m3l_model.register_output(qst_4_total_SPL_A_weighted, qst_4)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_4
)

system_m3l_model.register_output(total_mass, qst_4)
system_m3l_model.register_output(total_cg, qst_4)
system_m3l_model.register_output(total_inertia, qst_4)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_4_ac_states, 
    design_condition=qst_4
)
system_m3l_model.register_output(inertial_forces, qst_4)
system_m3l_model.register_output(inertial_moments, qst_4)

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
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_4,
)
system_m3l_model.register_output(total_forces, qst_4)
system_m3l_model.register_output(total_moments, qst_4)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_4_ac_states,
    design_condition=qst_4,
)
system_m3l_model.register_output(trim_residual, qst_4)
# endregion

# region qst 5
qst_5 = cd.CruiseCondition(name='qst_5')
qst_5.atmosphere_model = cd.SimpleAtmosphereModel()
qst_5.set_module_input('pitch_angle', val=0.08224058, dv_flag=True, lower=0.08224058 - np.deg2rad(2.5), scaler=1e1)
qst_5.set_module_input('mach_number', val=0.14708026)
qst_5.set_module_input('altitude', val=300)
qst_5.set_module_input(name='range', val=360)
qst_5.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


qst_5_ac_states = qst_5.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_5",
        f"{htail_vlm_mesh_name}_qst_5",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(vlm_forces, design_condition=qst_5)
system_m3l_model.register_output(vlm_moments, design_condition=qst_5)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct,qst_5_pp_Q,_ = pp_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

rlo_bem_model = PittPeters(disk_prefix='qst_5_rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=pitt_peters_mesh_lift)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments,rlo_dT ,rlo_dQ ,rlo_dD, rlo_Ct,rlo_Q, rlo_ux = rlo_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

rli_bem_model = PittPeters(disk_prefix='qst_5_rli_disk', blade_prefix='rli', component=rli_disk, mesh=pitt_peters_mesh_lift)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments,rli_dT ,rli_dQ ,rli_dD, rli_Ct,rli_Q, rli_ux = rli_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

rri_bem_model = PittPeters(disk_prefix='qst_5_rri_disk', blade_prefix='rri', component=rri_disk, mesh=pitt_peters_mesh_lift)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments,rri_dT ,rri_dQ ,rri_dD, rri_Ct,rri_Q, rri_ux = rri_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

rro_bem_model = PittPeters(disk_prefix='qst_5_rro_disk', blade_prefix='rro', component=rro_disk, mesh=pitt_peters_mesh_lift)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments,rro_dT ,rro_dQ ,rro_dD, rro_Ct,rro_Q, rro_ux = rro_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

flo_bem_model = PittPeters(disk_prefix='qst_5_flo_disk', blade_prefix='flo', component=flo_disk, mesh=pitt_peters_mesh_lift)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments,flo_dT ,flo_dQ ,flo_dD, flo_Ct,flo_Q, flo_ux = flo_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

fli_bem_model = PittPeters(disk_prefix='qst_5_fli_disk', blade_prefix='fli', component=fli_disk, mesh=pitt_peters_mesh_lift)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments,fli_dT ,fli_dQ ,fli_dD, fli_Ct,fli_Q, fli_ux = fli_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

fri_bem_model = PittPeters(disk_prefix='qst_5_fri_disk', blade_prefix='fri', component=fri_disk, mesh=pitt_peters_mesh_lift)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments,fri_dT ,fri_dQ ,fri_dD, fri_Ct,fri_Q, fri_ux = fri_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

fro_bem_model = PittPeters(disk_prefix='qst_5_fro_disk', blade_prefix='fro', component=fro_disk, mesh=pitt_peters_mesh_lift)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=5, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments, fro_dT ,fro_dQ ,fro_dD, fro_Ct,fro_Q, fro_ux = fro_bem_model.evaluate(ac_states=qst_5_ac_states, design_condition=qst_5)

rotor_index_functions_qst_5 = []

rotor_fitting_dict = {
     'rlo': {'disk': rlo_disk, 'ux': rlo_ux, 'mesh': rlo_disk_mesh} ,
     'rli': {'disk': rli_disk, 'ux': rli_ux, 'mesh': rli_disk_mesh} ,
     'rro': {'disk': rro_disk, 'ux': rro_ux, 'mesh': rro_disk_mesh} ,
     'flo': {'disk': flo_disk, 'ux': flo_ux, 'mesh': flo_disk_mesh} ,
     'fro': {'disk': fro_disk, 'ux': fro_ux, 'mesh': fro_disk_mesh} ,
     'rri': {'disk': rri_disk, 'ux': rri_ux, 'mesh': rri_disk_mesh} ,
     'fli': {'disk': fli_disk, 'ux': fli_ux, 'mesh': fli_disk_mesh} ,
     'fri': {'disk': fri_disk, 'ux': fri_ux, 'mesh': fri_disk_mesh} ,
}

for rotor_name, rotor_dict in rotor_fitting_dict.items():
    rotor_disk = rotor_dict['disk']
    rotor_ux = rotor_dict['ux']
    rotor_mesh = rotor_dict['mesh']

    pressure_spaces = {}
    coefficients = {}

    nodes_parametric = []
    targets = lpc_rep.spatial_representation.get_primitives(rotor_disk.get_primitives())
    num_targets = len(targets.keys())

    from m3l.utils.utils import index_functions
    order = 3
    shape = 5
    space_u = lg.BSplineSpace(name=f'{rotor_name}_ux_sapce',
                            order=(2, 2),
                            control_points_shape=(5, 5))
    rlo_ux_function = index_functions(list(rotor_disk.get_primitives().keys()), f'{rotor_name}_ux', space_u, 1)


    projected_points_on_each_target = []
    target_names = []
    nodes = rotor_mesh.value.reshape((25*25, 3))

    with open(PROJECTIONS_FOLDER /  f'{rotor_name}_disk_dt_projections.pickle', 'rb') as f:
            nodes_parametric_rlo = pickle.load(f)

    counter = 0
    for item in nodes_parametric_rlo:
        new_tuple = (item[0], item[1].reshape(1, 2)) 
        nodes_parametric_rlo[counter] = new_tuple
        counter += 1

    rlo_ux_function.inverse_evaluate(nodes_parametric_rlo, rotor_ux)
    system_m3l_model.register_output(rlo_ux_function.coefficients, design_condition=qst_5)
    rotor_index_functions_qst_5.append(rlo_ux_function)

# # qst 5 Motor Analysis
qst_5_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_5_pp_input_power, qst_5_pp_efficiency = qst_5_pp_motor_model.evaluate(qst_5_pp_Q, pp_motor_parameters, design_condition=qst_5)
system_m3l_model.register_output(qst_5_pp_input_power, qst_5)

# qst_5_rlo_motor_model = MotorAnalysis(component=rlo_disk)
# qst_5_rlo_input_power, qst_5_rlo_efficiency = qst_5_rlo_motor_model.evaluate(rlo_Q, rlo_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_rlo_input_power, qst_5)

# qst_5_rli_motor_model = MotorAnalysis(component=rli_disk)
# qst_5_rli_input_power, qst_5_rli_efficiency = qst_5_rli_motor_model.evaluate(rli_Q, rli_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_rli_input_power, qst_5)

# qst_5_rri_motor_model = MotorAnalysis(component=rri_disk)
# qst_5_rri_input_power, qst_5_rri_efficiency = qst_5_rri_motor_model.evaluate(rri_Q, rri_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_rri_input_power, qst_5)

# qst_5_rro_motor_model = MotorAnalysis(component=rro_disk)
# qst_5_rro_input_power, qst_5_rro_efficiency = qst_5_rro_motor_model.evaluate(rro_Q, rro_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_rro_input_power, qst_5)

# qst_5_flo_motor_model = MotorAnalysis(component=flo_disk)
# qst_5_flo_input_power, qst_5_flo_efficiency = qst_5_flo_motor_model.evaluate(flo_Q, flo_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_flo_input_power, qst_5)

# qst_5_fli_motor_model = MotorAnalysis(component=fli_disk)
# qst_5_fli_input_power, qst_5_fli_efficiency = qst_5_fli_motor_model.evaluate(fli_Q, fli_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_fli_input_power, qst_5)

# qst_5_fro_motor_model = MotorAnalysis(component=fro_disk)
# qst_5_fro_input_power, qst_5_fro_efficiency = qst_5_fro_motor_model.evaluate(fro_Q, fro_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_fro_input_power, qst_5)

# qst_5_fri_motor_model = MotorAnalysis(component=fri_disk)
# qst_5_fri_input_power, qst_5_fri_efficiency = qst_5_fri_motor_model.evaluate(fri_Q, fri_motor_parameters, design_condition=qst_5)
# system_m3l_model.register_output(qst_5_fri_input_power, qst_5)

# qst 5 Energy Consumption
qst_5_energy_model = cd.EnergyModelM3L()
qst_5_energy = qst_5_energy_model.evaluate(
    qst_5_pp_input_power,
    # qst_5_rlo_input_power, qst_5_rli_input_power, qst_5_rri_input_power, qst_5_rro_input_power,
    # qst_5_flo_input_power, qst_5_fli_input_power, qst_5_fri_input_power, qst_5_fro_input_power,
    ac_states=qst_5_ac_states, design_condition=qst_5
)

system_m3l_model.register_output(qst_5_energy, qst_5)

# acoustics
qst_5_acoustics = Acoustics(
    aircraft_position = np.array([0.,0., 90.])
)

qst_5_acoustics.add_observer(
    name='obs1',
    # obs_position=np.array([53.74, 0., 19.26]),
    obs_position=np.array([0, 0., 0]),
    time_vector=np.array([0. ]),
)

pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_5_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_5_tonal_SPL, pp_qst_5_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(pp_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(pp_qst_5_tonal_SPL_A_weighted, qst_5)

pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
pp_qst_5_broadband_SPL, pp_qst_5_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_5_ac_states, pp_Ct, design_condition=qst_5)
system_m3l_model.register_output(pp_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(pp_qst_5_broadband_SPL_A_weighted, qst_5)


rlo_lowson_model = Lowson(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rlo_disk', blade_prefix='rlo')
rlo_qst_5_tonal_SPL, rlo_qst_5_tonal_SPL_A_weighted = rlo_lowson_model.evaluate_tonal_noise(rlo_dT, rlo_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(rlo_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(rlo_qst_5_tonal_SPL_A_weighted, qst_5)

rlo_gl_model = GL(component=rlo_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rlo_disk', blade_prefix='rlo')
rlo_qst_5_broadband_SPL, rlo_qst_5_broadband_SPL_A_weighted = rlo_gl_model.evaluate_broadband_noise(qst_5_ac_states, rlo_Ct, design_condition=qst_5)
system_m3l_model.register_output(rlo_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(rlo_qst_5_broadband_SPL_A_weighted, qst_5)


rli_lowson_model = Lowson(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rli_disk', blade_prefix='rli')
rli_qst_5_tonal_SPL, rli_qst_5_tonal_SPL_A_weighted = rli_lowson_model.evaluate_tonal_noise(rli_dT, rli_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(rli_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(rli_qst_5_tonal_SPL_A_weighted, qst_5)

rli_gl_model = GL(component=rli_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rli_disk', blade_prefix='rli')
rli_qst_5_broadband_SPL, rli_qst_5_broadband_SPL_A_weighted = rli_gl_model.evaluate_broadband_noise(qst_5_ac_states, rli_Ct, design_condition=qst_5)
system_m3l_model.register_output(rli_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(rli_qst_5_broadband_SPL_A_weighted, qst_5)


rri_lowson_model = Lowson(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rri_disk', blade_prefix='rri')
rri_qst_5_tonal_SPL, rri_qst_5_tonal_SPL_A_weighted = rri_lowson_model.evaluate_tonal_noise(rri_dT, rri_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(rri_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(rri_qst_5_tonal_SPL_A_weighted, qst_5)

rri_gl_model = GL(component=rri_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rri_disk', blade_prefix='rri')
rri_qst_5_broadband_SPL, rri_qst_5_broadband_SPL_A_weighted = rri_gl_model.evaluate_broadband_noise(qst_5_ac_states, rri_Ct, design_condition=qst_5)
system_m3l_model.register_output(rri_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(rri_qst_5_broadband_SPL_A_weighted, qst_5)


rro_lowson_model = Lowson(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rro_disk', blade_prefix='rro')
rro_qst_5_tonal_SPL, rro_qst_5_tonal_SPL_A_weighted = rro_lowson_model.evaluate_tonal_noise(rro_dT, rro_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(rro_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(rro_qst_5_tonal_SPL_A_weighted, qst_5)

rro_gl_model = GL(component=rro_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_rro_disk', blade_prefix='rro')
rro_qst_5_broadband_SPL, rro_qst_5_broadband_SPL_A_weighted = rro_gl_model.evaluate_broadband_noise(qst_5_ac_states, rro_Ct, design_condition=qst_5)
system_m3l_model.register_output(rro_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(rro_qst_5_broadband_SPL_A_weighted, qst_5)


flo_lowson_model = Lowson(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_flo_disk', blade_prefix='flo')
flo_qst_5_tonal_SPL, flo_qst_5_tonal_SPL_A_weighted = flo_lowson_model.evaluate_tonal_noise(flo_dT, flo_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(flo_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(flo_qst_5_tonal_SPL_A_weighted, qst_5)

flo_gl_model = GL(component=flo_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_flo_disk', blade_prefix='flo')
flo_qst_5_broadband_SPL, flo_qst_5_broadband_SPL_A_weighted = flo_gl_model.evaluate_broadband_noise(qst_5_ac_states, flo_Ct, design_condition=qst_5)
system_m3l_model.register_output(flo_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(flo_qst_5_broadband_SPL_A_weighted, qst_5)


fli_lowson_model = Lowson(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_fli_disk', blade_prefix='fli')
fli_qst_5_tonal_SPL, fli_qst_5_tonal_SPL_A_weighted = fli_lowson_model.evaluate_tonal_noise(fli_dT, fli_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(fli_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(fli_qst_5_tonal_SPL_A_weighted, qst_5)

fli_gl_model = GL(component=fli_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_fli_disk', blade_prefix='fli')
fli_qst_5_broadband_SPL, fli_qst_5_broadband_SPL_A_weighted = fli_gl_model.evaluate_broadband_noise(qst_5_ac_states, fli_Ct, design_condition=qst_5)
system_m3l_model.register_output(fli_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(fli_qst_5_broadband_SPL_A_weighted, qst_5)


fri_lowson_model = Lowson(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_fri_disk', blade_prefix='fri')
fri_qst_5_tonal_SPL, fri_qst_5_tonal_SPL_A_weighted = fri_lowson_model.evaluate_tonal_noise(fri_dT, fri_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(fri_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(fri_qst_5_tonal_SPL_A_weighted, qst_5)

fri_gl_model = GL(component=fri_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_fri_disk', blade_prefix='fri')
fri_qst_5_broadband_SPL, fri_qst_5_broadband_SPL_A_weighted = fri_gl_model.evaluate_broadband_noise(qst_5_ac_states, fri_Ct, design_condition=qst_5)
system_m3l_model.register_output(fri_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(fri_qst_5_broadband_SPL_A_weighted, qst_5)


fro_lowson_model = Lowson(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_fro_disk', blade_prefix='fro')
fro_qst_5_tonal_SPL, fro_qst_5_tonal_SPL_A_weighted = fro_lowson_model.evaluate_tonal_noise(fro_dT, fro_dD, qst_5_ac_states, design_condition=qst_5)
system_m3l_model.register_output(fro_qst_5_tonal_SPL, qst_5)
system_m3l_model.register_output(fro_qst_5_tonal_SPL_A_weighted, qst_5)

fro_gl_model = GL(component=fro_disk, mesh=bem_mesh_lift, acoustics_data=qst_5_acoustics, disk_prefix='qst_5_fro_disk', blade_prefix='fro')
fro_qst_5_broadband_SPL, fro_qst_5_broadband_SPL_A_weighted = fro_gl_model.evaluate_broadband_noise(qst_5_ac_states, fro_Ct, design_condition=qst_5)
system_m3l_model.register_output(fro_qst_5_broadband_SPL, qst_5)
system_m3l_model.register_output(fro_qst_5_broadband_SPL_A_weighted, qst_5)


total_noise_model_qst_5 = TotalAircraftNoise(
    acoustics_data=qst_5_acoustics,
    # component_list=[rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
    component_list=[pp_disk, rlo_disk, rli_disk, rri_disk, rro_disk, flo_disk, fli_disk, fri_disk, fro_disk],
)
noise_components = [
    pp_qst_5_tonal_SPL, # pp_qst_5_broadband_SPL,
    rlo_qst_5_tonal_SPL, rlo_qst_5_broadband_SPL,
    rli_qst_5_tonal_SPL, rli_qst_5_broadband_SPL,
    rri_qst_5_tonal_SPL, rri_qst_5_broadband_SPL,
    rro_qst_5_tonal_SPL, rro_qst_5_broadband_SPL,
    flo_qst_5_tonal_SPL, flo_qst_5_broadband_SPL,
    fli_qst_5_tonal_SPL, fli_qst_5_broadband_SPL,
    fri_qst_5_tonal_SPL, fri_qst_5_broadband_SPL,
    fro_qst_5_tonal_SPL, fro_qst_5_broadband_SPL,

]
A_weighted_noise_components = [
    pp_qst_5_tonal_SPL_A_weighted, # pp_qst_5_broadband_SPL_A_weighted,
    rlo_qst_5_tonal_SPL_A_weighted, rlo_qst_5_broadband_SPL_A_weighted,
    rli_qst_5_tonal_SPL_A_weighted, rli_qst_5_broadband_SPL_A_weighted,
    rri_qst_5_tonal_SPL_A_weighted, rri_qst_5_broadband_SPL_A_weighted,
    rro_qst_5_tonal_SPL_A_weighted, rro_qst_5_broadband_SPL_A_weighted,
    flo_qst_5_tonal_SPL_A_weighted, flo_qst_5_broadband_SPL_A_weighted,
    fli_qst_5_tonal_SPL_A_weighted, fli_qst_5_broadband_SPL_A_weighted,
    fri_qst_5_tonal_SPL_A_weighted, fri_qst_5_broadband_SPL_A_weighted,
    fro_qst_5_tonal_SPL_A_weighted, fro_qst_5_broadband_SPL_A_weighted,

]

qst_5_total_SPL, qst_5_total_SPL_A_weighted = total_noise_model_qst_5.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_5)
system_m3l_model.register_output(qst_5_total_SPL, qst_5)
system_m3l_model.register_output(qst_5_total_SPL_A_weighted, qst_5)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_5
)

system_m3l_model.register_output(total_mass, qst_5)
system_m3l_model.register_output(total_cg, qst_5)
system_m3l_model.register_output(total_inertia, qst_5)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_5_ac_states, 
    design_condition=qst_5
)
system_m3l_model.register_output(inertial_forces, qst_5)
system_m3l_model.register_output(inertial_moments, qst_5)

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
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_5,
)
system_m3l_model.register_output(total_forces,qst_5)
system_m3l_model.register_output(total_moments, qst_5)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_5_ac_states,
    design_condition=qst_5,
)
system_m3l_model.register_output(trim_residual, qst_5)
# endregion

# region qst 6
qst_6 = cd.CruiseCondition(name='qst_6')
qst_6.atmosphere_model = cd.SimpleAtmosphereModel()
qst_6.set_module_input('pitch_angle', val=0.06704556, dv_flag=True, lower=0.06704556 - np.deg2rad(1.5), scaler=1e1)
qst_6.set_module_input('mach_number', val=0.15408429)
qst_6.set_module_input('altitude', val=300)
qst_6.set_module_input(name='range', val=377)
qst_6.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_6_ac_states = qst_6.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_6",
        f"{htail_vlm_mesh_name}_qst_6",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)
system_m3l_model.register_output(vlm_forces, design_condition=qst_6)
system_m3l_model.register_output(vlm_moments, design_condition=qst_6)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct, qst_6_pp_Q, _ = pp_bem_model.evaluate(ac_states=qst_6_ac_states, design_condition=qst_6)

 # qst 6 Motor Analysis
qst_6_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_6_pp_input_power, qst_6_pp_efficiency = qst_6_pp_motor_model.evaluate(qst_6_pp_Q, pp_motor_parameters, design_condition=qst_6)
system_m3l_model.register_output(qst_6_pp_input_power, qst_6)

# qst 4 Energy Consumption
qst_6_energy_model = cd.EnergyModelM3L()
qst_6_energy = qst_6_energy_model.evaluate(
    qst_6_pp_input_power,
    ac_states=qst_6_ac_states, design_condition=qst_6
)

system_m3l_model.register_output(qst_6_energy, qst_6)

# # acoustics
# qst_6_acoustics = Acoustics(
#     aircraft_position = np.array([0.,0., 73.])
# )

# qst_6_acoustics.add_observer(
#     name='obs1',
#     obs_position=np.array([0., 0., 0.]),
#     time_vector=np.array([0. ]),
# )

# pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_2_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_6_tonal_SPL, pp_qst_6_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_6_ac_states, design_condition=qst_6)
# system_m3l_model.register_output(pp_qst_6_tonal_SPL, qst_6)
# system_m3l_model.register_output(pp_qst_6_tonal_SPL_A_weighted, qst_6)

# pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_6_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_6_broadband_SPL, pp_qst_6_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_6_ac_states, pp_Ct, design_condition=qst_6)
# system_m3l_model.register_output(pp_qst_6_broadband_SPL, qst_6)
# system_m3l_model.register_output(pp_qst_6_broadband_SPL_A_weighted, qst_6)

# total_noise_model_qst_6 = TotalAircraftNoise(
#     acoustics_data=qst_6_acoustics,
#     component_list=[pp_disk],
# )
# noise_components = [
#     pp_qst_6_tonal_SPL, #pp_qst_6_broadband_SPL,

# ]
# A_weighted_noise_components = [
#     pp_qst_6_tonal_SPL_A_weighted, #pp_qst_6_broadband_SPL_A_weighted,
# ]

# qst_6_total_SPL, qst_6_total_SPL_A_weighted = total_noise_model_qst_6.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_6)
# system_m3l_model.register_output(qst_6_total_SPL, qst_6)
# system_m3l_model.register_output(qst_6_total_SPL_A_weighted, qst_6)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_6
)

system_m3l_model.register_output(total_mass, qst_6)
system_m3l_model.register_output(total_cg, qst_6)
system_m3l_model.register_output(total_inertia, qst_6)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_6_ac_states, 
    design_condition=qst_6
)
system_m3l_model.register_output(inertial_forces, qst_6)
system_m3l_model.register_output(inertial_moments, qst_6)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    inertial_forces, 
    inertial_moments,
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_6,
)
system_m3l_model.register_output(total_forces, qst_6)
system_m3l_model.register_output(total_moments, qst_6)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_6_ac_states,
    design_condition=qst_6,
)
system_m3l_model.register_output(trim_residual, qst_6)
# endregion

# region qst 7
qst_7 = cd.CruiseCondition(name='qst_7')
qst_7.atmosphere_model = cd.SimpleAtmosphereModel()
qst_7.set_module_input('pitch_angle', val=0.05598293, dv_flag=True, lower=0.05598293-np.deg2rad(2), upper=np.deg2rad(6), scaler=1e1)
qst_7.set_module_input('mach_number', val=0.15983874)
qst_7.set_module_input('altitude', val=300)
qst_7.set_module_input(name='range', val=391)
qst_7.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_7_ac_states = qst_7.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_7",
        f"{htail_vlm_mesh_name}_qst_7",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)
system_m3l_model.register_output(vlm_forces, design_condition=qst_7)
system_m3l_model.register_output(vlm_moments, design_condition=qst_7)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct, qst_7_pp_Q,_ = pp_bem_model.evaluate(ac_states=qst_7_ac_states, design_condition=qst_7)

# qst 7 Motor Analysis
qst_7_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_7_pp_input_power, qst_7_pp_efficiency = qst_7_pp_motor_model.evaluate(qst_7_pp_Q, pp_motor_parameters, design_condition=qst_7)
system_m3l_model.register_output(qst_7_pp_input_power, qst_7)

# qst 7 Energy Consumption
qst_7_energy_model = cd.EnergyModelM3L()
qst_7_energy = qst_7_energy_model.evaluate(
    qst_7_pp_input_power,
    ac_states=qst_7_ac_states, design_condition=qst_7
)

system_m3l_model.register_output(qst_7_energy, qst_7)

# # acoustics
# qst_7_acoustics = Acoustics(
#     aircraft_position = np.array([0.,0., 73.])
# )

# qst_7_acoustics.add_observer(
#     name='obs1',
#     obs_position=np.array([0., 0., 0.]),
#     time_vector=np.array([0. ]),
# )

# pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_2_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_7_tonal_SPL, pp_qst_7_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_7_ac_states, design_condition=qst_7)
# system_m3l_model.register_output(pp_qst_7_tonal_SPL, qst_7)
# system_m3l_model.register_output(pp_qst_7_tonal_SPL_A_weighted, qst_7)

# pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_7_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_7_broadband_SPL, pp_qst_7_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_7_ac_states, pp_Ct, design_condition=qst_7)
# system_m3l_model.register_output(pp_qst_7_broadband_SPL, qst_7)
# system_m3l_model.register_output(pp_qst_7_broadband_SPL_A_weighted, qst_7)

# total_noise_model_qst_7 = TotalAircraftNoise(
#     acoustics_data=qst_7_acoustics,
#     component_list=[pp_disk],
# )
# noise_components = [
#     pp_qst_7_tonal_SPL, #pp_qst_7_broadband_SPL,

# ]
# A_weighted_noise_components = [
#     pp_qst_7_tonal_SPL_A_weighted, #pp_qst_7_broadband_SPL_A_weighted,
# ]

# qst_7_total_SPL, qst_7_total_SPL_A_weighted = total_noise_model_qst_7.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_7)
# system_m3l_model.register_output(qst_7_total_SPL, qst_7)
# system_m3l_model.register_output(qst_7_total_SPL_A_weighted, qst_7)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_7
)

system_m3l_model.register_output(total_mass, qst_7)
system_m3l_model.register_output(total_cg, qst_7)
system_m3l_model.register_output(total_inertia, qst_7)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_7_ac_states, 
    design_condition=qst_7
)
system_m3l_model.register_output(inertial_forces, qst_7)
system_m3l_model.register_output(inertial_moments, qst_7)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    inertial_forces, 
    inertial_moments,
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_7,
)
system_m3l_model.register_output(total_forces, qst_7)
system_m3l_model.register_output(total_moments, qst_7)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_7_ac_states,
    design_condition=qst_7,
)
system_m3l_model.register_output(trim_residual, qst_7)
# endregion

# region qst 8
qst_8 = cd.CruiseCondition(name='qst_8')
qst_8.atmosphere_model = cd.SimpleAtmosphereModel()
qst_8.set_module_input('pitch_angle', val=0.04712265, dv_flag=True, lower=0.04712265-np.deg2rad(3), upper=np.deg2rad(5), scaler=1e1)
qst_8.set_module_input('mach_number', val=0.16485417)
qst_8.set_module_input('altitude', val=300)
qst_8.set_module_input(name='range', val=403)
qst_8.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_8_ac_states = qst_8.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_8",
        f"{htail_vlm_mesh_name}_qst_8",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)
system_m3l_model.register_output(vlm_forces, design_condition=qst_8)
system_m3l_model.register_output(vlm_moments, design_condition=qst_8)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=3000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct,qst_8_pp_Q,_ = pp_bem_model.evaluate(ac_states=qst_8_ac_states, design_condition=qst_8)

# qst 8 Motor Analysis
qst_8_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_8_pp_input_power, qst_8_pp_efficiency = qst_8_pp_motor_model.evaluate(qst_8_pp_Q, pp_motor_parameters, design_condition=qst_8)
system_m3l_model.register_output(qst_8_pp_input_power, qst_8)

# qst 8 Energy Consumption
qst_8_energy_model = cd.EnergyModelM3L()
qst_8_energy = qst_8_energy_model.evaluate(
    qst_8_pp_input_power,
    ac_states=qst_8_ac_states, design_condition=qst_8
)

system_m3l_model.register_output(qst_8_energy, qst_8)

# # acoustics
# qst_8_acoustics = Acoustics(
#     aircraft_position = np.array([0.,0., 73.])
# )

# qst_8_acoustics.add_observer(
#     name='obs1',
#     obs_position=np.array([0., 0., 0.]),
#     time_vector=np.array([0. ]),
# )

# pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_2_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_8_tonal_SPL, pp_qst_8_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_8_ac_states, design_condition=qst_8)
# system_m3l_model.register_output(pp_qst_8_tonal_SPL, qst_8)
# system_m3l_model.register_output(pp_qst_8_tonal_SPL_A_weighted, qst_8)

# pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_8_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_8_broadband_SPL, pp_qst_8_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_8_ac_states, pp_Ct, design_condition=qst_8)
# system_m3l_model.register_output(pp_qst_8_broadband_SPL, qst_8)
# system_m3l_model.register_output(pp_qst_8_broadband_SPL_A_weighted, qst_8)

# total_noise_model_qst_8 = TotalAircraftNoise(
#     acoustics_data=qst_8_acoustics,
#     component_list=[pp_disk],
# )
# noise_components = [
#     pp_qst_8_tonal_SPL, #pp_qst_8_broadband_SPL,

# ]
# A_weighted_noise_components = [
#     pp_qst_8_tonal_SPL_A_weighted, # pp_qst_8_broadband_SPL_A_weighted,
# ]

# qst_8_total_SPL, qst_8_total_SPL_A_weighted = total_noise_model_qst_8.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_8)
# system_m3l_model.register_output(qst_8_total_SPL, qst_8)
# system_m3l_model.register_output(qst_8_total_SPL_A_weighted, qst_8)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_8
)

system_m3l_model.register_output(total_mass, qst_8)
system_m3l_model.register_output(total_cg, qst_8)
system_m3l_model.register_output(total_inertia, qst_8)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_8_ac_states, 
    design_condition=qst_8
)
system_m3l_model.register_output(inertial_forces, qst_8)
system_m3l_model.register_output(inertial_moments, qst_8)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    # rlo_bem_forces, 
    # rlo_bem_moments, 
    # rli_bem_forces, 
    # rli_bem_moments,
    # rri_bem_forces, 
    # rri_bem_moments, 
    # rro_bem_forces, 
    # rro_bem_moments,  
    # flo_bem_forces, 
    # flo_bem_moments, 
    # fli_bem_forces, 
    # fli_bem_moments,
    # fri_bem_forces, 
    # fri_bem_moments, 
    # fro_bem_forces, 
    # fro_bem_moments,  
    inertial_forces, 
    inertial_moments,
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_8,
)
system_m3l_model.register_output(total_forces, qst_8)
system_m3l_model.register_output(total_moments, qst_8)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_8_ac_states,
    design_condition=qst_8,
)
system_m3l_model.register_output(trim_residual, qst_8)
# endregion

# region qst 9
qst_9 = cd.CruiseCondition(name='qst_9')
qst_9.atmosphere_model = cd.SimpleAtmosphereModel()
qst_9.set_module_input('pitch_angle', val=0.03981101, dv_flag=True, lower=0.03981101-np.deg2rad(4),upper=np.deg2rad(5), scaler=1e1)
qst_9.set_module_input('mach_number', val=0.16937793)
qst_9.set_module_input('altitude', val=300)
qst_9.set_module_input(name='range', val=414)
qst_9.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_9_ac_states = qst_9.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_9",
        f"{htail_vlm_mesh_name}_qst_9",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)
system_m3l_model.register_output(vlm_forces, design_condition=qst_9)
system_m3l_model.register_output(vlm_moments, design_condition=qst_9)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct, qst_9_pp_Q,_ = pp_bem_model.evaluate(ac_states=qst_9_ac_states, design_condition=qst_9)

# qst 9 Motor Analysis
qst_9_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_9_pp_input_power, qst_9_pp_efficiency = qst_9_pp_motor_model.evaluate(qst_9_pp_Q, pp_motor_parameters, design_condition=qst_9)
system_m3l_model.register_output(qst_9_pp_input_power, qst_9)

# qst 8 Energy Consumption
qst_9_energy_model = cd.EnergyModelM3L()
qst_9_energy = qst_9_energy_model.evaluate(
    qst_9_pp_input_power,
    ac_states=qst_9_ac_states, design_condition=qst_9
)

system_m3l_model.register_output(qst_9_energy, qst_9)


# # acoustics
# qst_9_acoustics = Acoustics(
#     aircraft_position = np.array([0.,0., 73.])
# )

# qst_9_acoustics.add_observer(
#     name='obs1',
#     obs_position=np.array([0., 0., 0.]),
#     time_vector=np.array([0. ]),
# )

# pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_2_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_9_tonal_SPL, pp_qst_9_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_9_ac_states, design_condition=qst_9)
# system_m3l_model.register_output(pp_qst_9_tonal_SPL, qst_9)
# system_m3l_model.register_output(pp_qst_9_tonal_SPL_A_weighted, qst_9)

# pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_9_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_9_broadband_SPL, pp_qst_9_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_9_ac_states, pp_Ct, design_condition=qst_9)
# system_m3l_model.register_output(pp_qst_9_broadband_SPL, qst_9)
# system_m3l_model.register_output(pp_qst_9_broadband_SPL_A_weighted, qst_9)

# total_noise_model_qst_9 = TotalAircraftNoise(
#     acoustics_data=qst_9_acoustics,
#     component_list=[pp_disk],
# )
# noise_components = [
#     pp_qst_9_tonal_SPL, #pp_qst_9_broadband_SPL,

# ]
# A_weighted_noise_components = [
#     pp_qst_9_tonal_SPL_A_weighted, # pp_qst_9_broadband_SPL_A_weighted,
# ]

# qst_9_total_SPL, qst_9_total_SPL_A_weighted = total_noise_model_qst_9.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_9)
# system_m3l_model.register_output(qst_9_total_SPL, qst_9)
# system_m3l_model.register_output(qst_9_total_SPL_A_weighted, qst_9)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_9
)

system_m3l_model.register_output(total_mass, qst_9)
system_m3l_model.register_output(total_cg, qst_9)
system_m3l_model.register_output(total_inertia, qst_9)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_9_ac_states, 
    design_condition=qst_9
)
system_m3l_model.register_output(inertial_forces, qst_9)
system_m3l_model.register_output(inertial_moments, qst_9)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(  
    inertial_forces, 
    inertial_moments,
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_9,
)
system_m3l_model.register_output(total_forces, qst_9)
system_m3l_model.register_output(total_moments, qst_9)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_9_ac_states,
    design_condition=qst_9,
)
system_m3l_model.register_output(trim_residual, qst_9)
# endregion

# region qst 10
qst_10 = cd.CruiseCondition(name='qst_10')
qst_10.atmosphere_model = cd.SimpleAtmosphereModel()
qst_10.set_module_input('pitch_angle', val=0.03369678, dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(5), scaler=1e1)
qst_10.set_module_input('mach_number', val=0.17354959)
qst_10.set_module_input('altitude', val=300)
qst_10.set_module_input(name='range', val=424)
qst_10.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

qst_10_ac_states = qst_10.evaluate_ac_states()

vlm_model = VASTFluidSover(
    surface_names=[
        f"{wing_vlm_mesh_name}_qst_10",
        f"{htail_vlm_mesh_name}_qst_10",
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='ft',
    cl0=[0.25, 0]
)

forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)
system_m3l_model.register_output(vlm_forces, design_condition=qst_10)
system_m3l_model.register_output(vlm_moments, design_condition=qst_10)

pp_bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
pp_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
pp_bem_forces, pp_bem_moments, pp_dT ,pp_dQ ,pp_dD, pp_Ct,qst_10_pp_Q ,_ = pp_bem_model.evaluate(ac_states=qst_10_ac_states, design_condition=qst_10)

# qst 10 Motor Analysis
qst_10_pp_motor_model = MotorAnalysis(component=pp_disk)
qst_10_pp_input_power, qst_10_pp_efficiency = qst_10_pp_motor_model.evaluate(qst_10_pp_Q, pp_motor_parameters, design_condition=qst_10)
system_m3l_model.register_output(qst_10_pp_input_power, qst_10)

# qst 10 Energy Consumption
qst_10_energy_model = cd.EnergyModelM3L()
qst_10_energy = qst_10_energy_model.evaluate(
    qst_10_pp_input_power,
    ac_states=qst_10_ac_states, design_condition=qst_10
)

system_m3l_model.register_output(qst_10_energy, qst_10)


# # acoustics
# qst_10_acoustics = Acoustics(
#     aircraft_position = np.array([0.,0., 73.])
# )

# qst_10_acoustics.add_observer(
#     name='obs1',
#     obs_position=np.array([0., 0., 0.]),
#     time_vector=np.array([0. ]),
# )

# pp_lowson_model = Lowson(component=pp_disk, mesh=pusher_bem_mesh, acoustics_data=qst_2_acoustics, disk_prefix='pp_disk', blade_prefix='pp')
# pp_qst_10_tonal_SPL, pp_qst_10_tonal_SPL_A_weighted = pp_lowson_model.evaluate_tonal_noise(pp_dT, pp_dD, qst_10_ac_states, design_condition=qst_10)
# system_m3l_model.register_output(pp_qst_10_tonal_SPL, qst_10)
# system_m3l_model.register_output(pp_qst_10_tonal_SPL_A_weighted, qst_10)

# pp_gl_model = GL(component=pp_disk, mesh=bem_mesh_lift, acoustics_data=qst_10_acoustics, disk_prefix='10_pp_disk', blade_prefix='pp')
# pp_qst_10_broadband_SPL, pp_qst_10_broadband_SPL_A_weighted = pp_gl_model.evaluate_broadband_noise(qst_10_ac_states, pp_Ct, design_condition=qst_10)
# system_m3l_model.register_output(pp_qst_10_broadband_SPL, qst_10)
# system_m3l_model.register_output(pp_qst_10_broadband_SPL_A_weighted, qst_10)

# total_noise_model_qst_10 = TotalAircraftNoise(
#     acoustics_data=qst_10_acoustics,
#     component_list=[pp_disk],
# )
# noise_components = [
#     pp_qst_10_tonal_SPL, #pp_qst_10_broadband_SPL,

# ]
# A_weighted_noise_components = [
#     pp_qst_10_tonal_SPL_A_weighted, # pp_qst_10_broadband_SPL_A_weighted,
# ]

# qst_10_total_SPL, qst_10_total_SPL_A_weighted = total_noise_model_qst_10.evaluate(noise_components, A_weighted_noise_components, design_condition=qst_10)
# system_m3l_model.register_output(qst_10_total_SPL, qst_10)
# system_m3l_model.register_output(qst_10_total_SPL_A_weighted, qst_10)


total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=qst_10)

system_m3l_model.register_output(total_mass, qst_10)
system_m3l_model.register_output(total_cg, qst_10)
system_m3l_model.register_output(total_inertia, qst_10)

inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=total_cg, 
    totoal_mass=total_mass, 
    ac_states=qst_10_ac_states, 
    design_condition=qst_10
)
system_m3l_model.register_output(inertial_forces, qst_10)
system_m3l_model.register_output(inertial_moments, qst_10)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    inertial_forces, 
    inertial_moments,
    pp_bem_forces,
    pp_bem_moments,
    vlm_forces,
    vlm_moments,
    design_condition=qst_10,
)
system_m3l_model.register_output(total_forces, qst_10)
system_m3l_model.register_output(total_moments, qst_10)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=qst_10_ac_states,
    design_condition=qst_10,
)
system_m3l_model.register_output(trim_residual, qst_10)
# endregion

# region climb 1
climb_1 = cd.ClimbCondition(name='climb_1')
climb_1.atmosphere_model = cd.SimpleAtmosphereModel()
climb_1.set_module_input(name='altitude', val=1000)
climb_1.set_module_input(name='mach_number', val=0.17)
climb_1.set_module_input(name='initial_altitude', val=300)
climb_1.set_module_input(name='final_altitude', val=1000)
climb_1.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(10), scaler=1e1)
climb_1.set_module_input(name='flight_path_angle', val=np.deg2rad(4.588))
climb_1.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

ac_states = climb_1.evaluate_ac_states()
system_m3l_model.register_output(ac_states)

vlm_model = VASTFluidSover(
    surface_names=[
        f'{wing_vlm_mesh_name}_climb',
        f'{htail_vlm_mesh_name}_climb',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.25, 0.],
    ML=False,
)

# aero forces and moments
# cl_distribution, re_spans, vlm_panel_forces, panel_areas, evaluation_pt, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, ML=True, design_condition=climb_1)
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=climb_1)


system_m3l_model.register_output(vlm_force, design_condition=climb_1)
system_m3l_model.register_output(vlm_moment, design_condition=climb_1)
# system_m3l_model.register_output(cl_distribution, design_condition=climb_1)
# system_m3l_model.register_output(re_spans, design_condition=climb_1)

# ml_pressures = PressureProfile(
#     airfoil_name='NASA_langley_ga_1',
#     use_inverse_cl_map=True,
# )

# cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans, design_condition=climb_1) #, mach_number, reynolds_number)
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

# cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[], design_condition=climb_1)
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

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=900, upper=2000, scaler=1e-3)
bem_forces, bem_moments,_ ,_ ,_, _,climb_1_pp_Q ,_ = bem_model.evaluate(ac_states=ac_states, design_condition=climb_1)

system_m3l_model.register_output(bem_forces, design_condition=climb_1)
system_m3l_model.register_output(bem_moments, design_condition=climb_1)

# climb 1 Motor Analysis
climb_1_pp_motor_model = MotorAnalysis(component=pp_disk)
climb_1_pp_input_power, climb_1_pp_efficiency = climb_1_pp_motor_model.evaluate(climb_1_pp_Q, pp_motor_parameters, design_condition=climb_1)
system_m3l_model.register_output(climb_1_pp_input_power, climb_1)

# climb 1 Energy Consumption
climb_1_energy_model = cd.EnergyModelM3L()
climb_1_energy = climb_1_energy_model.evaluate(
    climb_1_pp_input_power,
    ac_states=ac_states, design_condition=climb_1
)

system_m3l_model.register_output(climb_1_energy, climb_1)


total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=climb_1
)

system_m3l_model.register_output(total_mass, climb_1)
system_m3l_model.register_output(total_cg, climb_1)
system_m3l_model.register_output(total_inertia, climb_1)

inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=climb_1)
system_m3l_model.register_output(inertial_forces, climb_1)
system_m3l_model.register_output(inertial_moments, climb_1)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_F, vlm_M, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=climb_1)
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=climb_1)
system_m3l_model.register_output(total_forces, climb_1)
system_m3l_model.register_output(total_moments, climb_1)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states,
    design_condition=climb_1,
)

system_m3l_model.register_output(trim_residual, climb_1)
# endregion

# region cruise condition
cruise_condition = cd.CruiseCondition(name="cruise")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.173, dv_flag=False, lower=0.17, upper=0.19)
cruise_condition.set_module_input(name='range', val=50000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(6), scaler=1e1)
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
system_m3l_model.register_output(ac_states)

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
cl_distribution, re_spans, vlm_panel_forces, panel_areas, evaluation_pt, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, ML=True, design_condition=cruise_condition)
# vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)
system_m3l_model.register_output(vlm_force, design_condition=cruise_condition)
system_m3l_model.register_output(vlm_moment, design_condition=cruise_condition)
system_m3l_model.register_output(cl_distribution, design_condition=cruise_condition)
system_m3l_model.register_output(re_spans, design_condition=cruise_condition)

ml_pressures = PressureProfile(
    airfoil_name='NASA_langley_ga_1',
    use_inverse_cl_map=True,
)

cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans, design_condition=cruise_condition) #, mach_number, reynolds_number)
system_m3l_model.register_output(cp_upper, design_condition=cruise_condition)
system_m3l_model.register_output(cp_lower, design_condition=cruise_condition)

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
system_m3l_model.register_output(vlm_F, design_condition=cruise_condition)
system_m3l_model.register_output(vlm_M, design_condition=cruise_condition)

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

cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[], design_condition=cruise_condition)
wing_oml_pressure_upper = cp_upper_oml[0]
htail_oml_pressure_upper = cp_upper_oml[1]
wing_oml_pressure_lower = cp_lower_oml[0]
htail_oml_pressure_lower = cp_lower_oml[1]

system_m3l_model.register_output(wing_oml_pressure_upper, design_condition=cruise_condition)
system_m3l_model.register_output(htail_oml_pressure_upper, design_condition=cruise_condition)
system_m3l_model.register_output(wing_oml_pressure_lower, design_condition=cruise_condition)
system_m3l_model.register_output(htail_oml_pressure_lower, design_condition=cruise_condition)

build_cp_index_function(wing, 10, cruise_condition, system_m3l_model, wing_cp_index_functions, wing_oml_pressure_upper, wing_oml_pressure_lower)


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

oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh], design_condition=cruise_condition)
wing_forces = oml_forces[0]
htail_forces = oml_forces[1]

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=650, upper=2000, scaler=1e-3)
bem_forces, bem_moments,_ ,_ ,_,_,cruise_pp_Q,_ = bem_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)

system_m3l_model.register_output(bem_forces, design_condition=cruise_condition)
system_m3l_model.register_output(bem_moments, design_condition=cruise_condition)

# cruise Motor Analysis
cruise_pp_motor_model = MotorAnalysis(component=pp_disk)
cruise_pp_input_power, cruise_pp_efficiency = cruise_pp_motor_model.evaluate(cruise_pp_Q, pp_motor_parameters, design_condition=cruise_condition)
system_m3l_model.register_output(cruise_pp_input_power, cruise_condition)

# cruise Energy Consumption
cruise_energy_model = cd.EnergyModelM3L()
cruise_energy = cruise_energy_model.evaluate(
    cruise_pp_input_power,
    ac_states=ac_states, design_condition=cruise_condition
)

system_m3l_model.register_output(cruise_energy, cruise_condition)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=cruise_condition
)

system_m3l_model.register_output(total_mass, cruise_condition)
system_m3l_model.register_output(total_cg, cruise_condition)
system_m3l_model.register_output(total_inertia, cruise_condition)

inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=cruise_condition)
system_m3l_model.register_output(inertial_forces, cruise_condition)
system_m3l_model.register_output(inertial_moments, cruise_condition)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_F, vlm_M, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=cruise_condition)
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
system_m3l_model.register_output(total_forces, cruise_condition)
system_m3l_model.register_output(total_moments, cruise_condition)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states,
    design_condition=cruise_condition,
)

system_m3l_model.register_output(trim_residual, cruise_condition)
# endregion

# region descent 1
descent_1 = cd.ClimbCondition(name='descent_1')
descent_1.atmosphere_model = cd.SimpleAtmosphereModel()
descent_1.set_module_input(name='altitude', val=1000)
descent_1.set_module_input(name='mach_number', val=0.17)
descent_1.set_module_input(name='initial_altitude', val=1000)
descent_1.set_module_input(name='final_altitude', val=300)
descent_1.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1e1)
descent_1.set_module_input(name='flight_path_angle', val=np.deg2rad(-4), dv_flag=False)
descent_1.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

ac_states = descent_1.evaluate_ac_states()
system_m3l_model.register_output(ac_states)

vlm_model = VASTFluidSover(
    surface_names=[
        f'{wing_vlm_mesh_name}_descent',
        f'{htail_vlm_mesh_name}_descent',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.25, 0.],
    ML=False,
)

# aero forces and moments
# cl_distribution, re_spans, vlm_panel_forces, panel_areas, evaluation_pt, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, ML=True, design_condition=descent_1)
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states, design_condition=descent_1)
system_m3l_model.register_output(vlm_force, design_condition=descent_1)
system_m3l_model.register_output(vlm_moment, design_condition=descent_1)

# system_m3l_model.register_output(cl_distribution, design_condition=descent_1)
# system_m3l_model.register_output(re_spans, design_condition=descent_1)

# ml_pressures = PressureProfile(
#     airfoil_name='NASA_langley_ga_1',
#     use_inverse_cl_map=True,
# )

# cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans, design_condition=descent_1) #, mach_number, reynolds_number)
# system_m3l_model.register_output(cp_upper, design_condition=descent_1)
# system_m3l_model.register_output(cp_lower, design_condition=descent_1)

# viscous_drag_correction = ViscousCorrectionModel(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_descent',
#         f'{htail_vlm_mesh_name}_descent',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
# )
# moment_point = None
# vlm_F, vlm_M = viscous_drag_correction.evaluate(ac_states=ac_states, forces=vlm_panel_forces, cd_v=Cd, panel_area=panel_areas, moment_pt=moment_point, evaluation_pt=evaluation_pt, design_condition=descent_1)
# system_m3l_model.register_output(vlm_F, design_condition=descent_1)
# system_m3l_model.register_output(vlm_M, design_condition=descent_1)

# ml_pressures_oml_map = NodalPressureProfile(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_descent',
#         f'{htail_vlm_mesh_name}_descent',
#     ],
#     surface_shapes=[
#         wing_upper_surface_ml.value.shape,
#         htail_upper_surface_ml.value.shape,
#     ]
# )

# cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[], design_condition=descent_1)
# wing_oml_pressure_upper = cp_upper_oml[0]
# htail_oml_pressure_upper = cp_upper_oml[1]
# wing_oml_pressure_lower = cp_lower_oml[0]
# htail_oml_pressure_lower = cp_lower_oml[1]

# system_m3l_model.register_output(wing_oml_pressure_upper, design_condition=descent_1)
# system_m3l_model.register_output(htail_oml_pressure_upper, design_condition=descent_1)
# system_m3l_model.register_output(wing_oml_pressure_lower, design_condition=descent_1)
# system_m3l_model.register_output(htail_oml_pressure_lower, design_condition=descent_1)

# vlm_force_mapping_model = VASTNodalForces(
#     surface_names=[
#         f'{wing_vlm_mesh_name}_descent',
#         f'{htail_vlm_mesh_name}_descent',
#     ],
#     surface_shapes=[
#         (1, ) + wing_camber_surface.evaluate().shape[1:],
#         (1, ) + htail_camber_surface.evaluate().shape[1:],
#     ],
#     initial_meshes=[
#         wing_camber_surface,
#         htail_camber_surface]
# )

# oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[wing_oml_mesh, wing_oml_mesh], design_condition=descent_1)
# wing_forces = oml_forces[0]
# htail_forces = oml_forces[1]

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=600, upper=2000, scaler=1e-3)
bem_forces, bem_moments,_ ,_ ,_, _,descent_1_pp_Q,_ = bem_model.evaluate(ac_states=ac_states, design_condition=descent_1)

system_m3l_model.register_output(bem_forces, design_condition=descent_1)
system_m3l_model.register_output(bem_moments, design_condition=descent_1)

# descent 1 Motor Analysis
descent_1_pp_motor_model = MotorAnalysis(component=pp_disk)
descent_1_pp_input_power, descent_1_pp_efficiency = descent_1_pp_motor_model.evaluate(descent_1_pp_Q, pp_motor_parameters, design_condition=descent_1)
system_m3l_model.register_output(descent_1_pp_input_power, descent_1)

# descent_1 Energy Consumption
descent_1_energy_model = cd.EnergyModelM3L()
descent_1_energy = descent_1_energy_model.evaluate(
    descent_1_pp_input_power,
    ac_states=ac_states, design_condition=descent_1
)

system_m3l_model.register_output(descent_1_energy, descent_1)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(
    mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, 
    m_mass_pp, m_cg_pp, m_I_pp, 
    m_mass_rlo, m_cg_rlo, m_I_rlo,
    m_mass_rli, m_cg_rli, m_I_rli,
    m_mass_rri, m_cg_rri, m_I_rri,
    m_mass_rro, m_cg_rro, m_I_rro,
    m_mass_flo, m_cg_flo, m_I_flo,
    m_mass_fli, m_cg_fli, m_I_fli,
    m_mass_fri, m_cg_fri, m_I_fri,
    m_mass_fro, m_cg_fro, m_I_fro,
    design_condition=descent_1)

system_m3l_model.register_output(total_mass, descent_1)
system_m3l_model.register_output(total_cg, descent_1)
system_m3l_model.register_output(total_inertia, descent_1)

inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=descent_1)
system_m3l_model.register_output(inertial_forces, descent_1)
system_m3l_model.register_output(inertial_moments, descent_1)

total_forces_moments_model = cd.TotalForcesMomentsM3L()
# total_forces, total_moments = total_forces_moments_model.evaluate(vlm_F, vlm_M, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=descent_1)
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=descent_1)
system_m3l_model.register_output(total_forces, descent_1)
system_m3l_model.register_output(total_moments, descent_1)

eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states,
    design_condition=descent_1,
)

system_m3l_model.register_output(trim_residual, descent_1)
# endregion

# endregion


# region total energy and soc
total_energy_model = cd.TotalEnergyModelM3L()
total_energy = total_energy_model.evaluate(
    hover_energy, 
    climb_1_energy, 
    cruise_energy, 
    descent_1_energy,
    qst_1_energy,
    qst_2_energy,
    qst_3_energy,
    qst_4_energy,
    qst_5_energy,
    qst_6_energy,
    qst_7_energy,
    qst_8_energy,
    qst_9_energy,
    qst_10_energy,
)
# total_energy = total_energy_model.evaluate(climb_energy)
system_m3l_model.register_output(total_energy)

soc_model = cd.SOCModelM3L(battery_energy_density=400) # Note, either turn this parameter into csdl variable or connect it from battery sizing 
final_SoC = soc_model.evaluate(battery_mass=battery_mass, total_energy_consumption=total_energy, mission_multiplier=2.2)
system_m3l_model.register_output(final_SoC)
# endregion

# region add design conditions to design scenario
# wing sizing conditions
design_scenario.add_design_condition(plus_3g_condition)
design_scenario.add_design_condition(minus_1g_condition)

# Off design (OEI )
design_scenario.add_design_condition(hover_1_oei_flo)
design_scenario.add_design_condition(hover_1_oei_fli)

# On design 
design_scenario.add_design_condition(hover_1)
design_scenario.add_design_condition(qst_1)
design_scenario.add_design_condition(qst_2)
design_scenario.add_design_condition(qst_3)
design_scenario.add_design_condition(qst_4)
design_scenario.add_design_condition(qst_5)
design_scenario.add_design_condition(qst_6)
design_scenario.add_design_condition(qst_7)
design_scenario.add_design_condition(qst_8)
design_scenario.add_design_condition(qst_9)
design_scenario.add_design_condition(qst_10)
design_scenario.add_design_condition(climb_1)
design_scenario.add_design_condition(cruise_condition)
design_scenario.add_design_condition(descent_1)
# endregion

system_model.add_design_scenario(design_scenario)
system_model.add_m3l_model('system_m3l_model', system_m3l_model)


caddee_csdl_model = caddee.assemble_csdl()

# region connections
caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tweb', 'system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.wing_beam_tweb')
caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tcap', 'system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.wing_beam_tcap')
caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tweb', 'system_model.system_m3l_model.minus_1g_sizing_wing_eb_beam_model.Aframe.wing_beam_tweb')
caddee_csdl_model.connect('system_model.system_m3l_model.mass_model.wing_beam_tcap', 'system_model.system_m3l_model.minus_1g_sizing_wing_eb_beam_model.Aframe.wing_beam_tcap')


lrps = ['rlo', 'rli', 'rri', 'rro', 'flo', 'fli', 'fri', 'fro']
for i in range(5):
    for j in range(8):
        lrp = lrps[j]
        if i == 0: # hover and qst 1
            connect_phi_from_bem_hover = f'system_model.system_m3l_model.hover_{i+1}_{lrp}_disk_bem_model.phi_bracketed_search_group.phi_distribution'
            connect_phi_to_noise_hover = f'system_model.system_m3l_model.hover_{i+1}_{lrp}_disk_KS_tonal_model.ks_spl_model.phi'
            
            connect_phi_from_bem_qst_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_bem_model.phi_bracketed_search_group.phi_distribution'
            connect_phi_to_noise_qst_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_KS_tonal_model.ks_spl_model.phi'

            connect_rpm_from_bem_hover = f'system_model.system_m3l_model.hover_{i+1}_{lrp}_disk_bem_model.rpm'
            connect_rpm_to_noise_1_hover = f'system_model.system_m3l_model.hover_{i+1}_{lrp}_disk_KS_tonal_model.rpm'
            connect_rpm_to_noise_2_hover = f'system_model.system_m3l_model.hover_{i+1}_{lrp}_disk_GL_broadband_model.rpm'

            connect_rpm_from_bem_qst_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_bem_model.rpm'
            connect_rpm_to_noise_1_qst_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_KS_tonal_model.rpm'
            connect_rpm_to_noise_2_qst_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_GL_broadband_model.rpm'

            caddee_csdl_model.connect(connect_phi_from_bem_hover, connect_phi_to_noise_hover)
            caddee_csdl_model.connect(connect_phi_from_bem_qst_1, connect_phi_to_noise_qst_1)

            caddee_csdl_model.connect(connect_rpm_from_bem_hover, connect_rpm_to_noise_1_hover)
            caddee_csdl_model.connect(connect_rpm_from_bem_hover, connect_rpm_to_noise_2_hover)

            caddee_csdl_model.connect(connect_rpm_from_bem_qst_1, connect_rpm_to_noise_1_qst_1)
            caddee_csdl_model.connect(connect_rpm_from_bem_qst_1, connect_rpm_to_noise_2_qst_1)
        
        else: # qst 2-5
            connect_from_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_pitt_peters_model.pitt_peters_external_inputs_model.in_plane_ex'
            connect_to_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_Lowson_tonal_model.lowson_spl_model.in_plane_ex'
            
            connect_from_1_pp = f'system_model.system_m3l_model.qst_{i+1}_pp_disk_bem_model.BEM_external_inputs_model.in_plane_ex'
            connect_to_1_pp = f'system_model.system_m3l_model.qst_{i+1}_pp_disk_Lowson_tonal_model.lowson_spl_model.in_plane_ex'

            connect_from_rpm = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_pitt_peters_model.rpm'
            connect_to_rpm_1 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_GL_broadband_model.rpm'
            connect_to_rpm_2 = f'system_model.system_m3l_model.qst_{i+1}_{lrp}_disk_Lowson_tonal_model.rpm'

            connect_from_rpm_pp = f'system_model.system_m3l_model.qst_{i+1}_pp_disk_bem_model.rpm'
            # connect_to_rpm_1_pp = f'system_model.system_m3l_model.qst_{i+1}_pp_disk_GL_broadband_model.rpm'
            connect_to_rpm_2_pp = f'system_model.system_m3l_model.qst_{i+1}_pp_disk_Lowson_tonal_model.rpm'
        
            caddee_csdl_model.connect(connect_from_1, connect_to_1)
            caddee_csdl_model.connect(connect_from_1_pp, connect_to_1_pp)
            
            caddee_csdl_model.connect(connect_from_rpm, connect_to_rpm_1)
            caddee_csdl_model.connect(connect_from_rpm, connect_to_rpm_2)
            caddee_csdl_model.connect(connect_from_rpm_pp, connect_to_rpm_2_pp)

# for i in range(4):
#     connect_rpm_from_bem = f'system_model.system_m3l_model.qst_{i+6}_pp_disk_bem_model.rpm'
#     connect_rpm_to_noise_1 = f'system_model.system_m3l_model.qst_{i+6}_pp_disk_Lowson_tonal_model.rpm'
#     # connect_rpm_to_noise_2 = f'broad band'
    
#     caddee_csdl_model.connect(connect_rpm_from_bem, connect_rpm_to_noise_1)
    # caddee_csdl_model.connect('system_model.system_m3l_model.qst_6_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_6_pp_disk_Lowson_tonal_model.rpm')

# caddee_csdl_model.connect('system_model.system_m3l_model.qst_7_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_7_pp_disk_GL_broadband_model.rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.qst_7_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_7_pp_disk_Lowson_tonal_model.rpm')

# caddee_csdl_model.connect('system_model.system_m3l_model.qst_8_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_8_pp_disk_GL_broadband_model.rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.qst_8_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_8_pp_disk_Lowson_tonal_model.rpm')

# caddee_csdl_model.connect('system_model.system_m3l_model.qst_9_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_9_pp_disk_GL_broadband_model.rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.qst_9_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_9_pp_disk_Lowson_tonal_model.rpm')

# caddee_csdl_model.connect('system_model.system_m3l_model.qst_10_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_10_pp_disk_GL_broadband_model.rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.qst_10_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.qst_10_pp_disk_Lowson_tonal_model.rpm')

# # Motor analysis connections
lrps = ['rlo', 'rli', 'rri', 'rro', 'flo', 'fli', 'fri', 'fro']

# # Hover + OEIs
for pref in lrps:
    # hover
    caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.hover_1_{pref}_disk_motor_analysis_model.motor_diameter')
    caddee_csdl_model.connect(f'system_model.system_m3l_model.hover_1_{pref}_disk_bem_model.rpm', f'system_model.system_m3l_model.hover_1_{pref}_disk_motor_analysis_model.rpm')

    # oei flo
    if pref == 'flo':
        pass
    else:
        caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.hover_1_oei_flo_{pref}_disk_motor_analysis_model.motor_diameter')
        caddee_csdl_model.connect(f'system_model.system_m3l_model.hover_1_oei_flo_{pref}_disk_bem_model.rpm', f'system_model.system_m3l_model.hover_1_oei_flo_{pref}_disk_motor_analysis_model.rpm')
        caddee_csdl_model.add_constraint(f'system_model.system_m3l_model.hover_1_oei_flo_{pref}_disk_motor_analysis_model.torque_delta', lower=0.1)

    # oei fli
    if pref == 'fli':
        pass
    else:
        caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.hover_1_oei_fli_{pref}_disk_motor_analysis_model.motor_diameter')
        caddee_csdl_model.connect(f'system_model.system_m3l_model.hover_1_oei_fli_{pref}_disk_bem_model.rpm', f'system_model.system_m3l_model.hover_1_oei_fli_{pref}_disk_motor_analysis_model.rpm')
        caddee_csdl_model.add_constraint(f'system_model.system_m3l_model.hover_1_oei_fli_{pref}_disk_motor_analysis_model.torque_delta', lower=0.1)

    # qst 1
    caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.qst_1_{pref}_disk_motor_analysis_model.motor_diameter')
    caddee_csdl_model.connect(f'system_model.system_m3l_model.qst_1_{pref}_disk_bem_model.rpm', f'system_model.system_m3l_model.qst_1_{pref}_disk_motor_analysis_model.rpm')

    # qst 2
    caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.qst_2_{pref}_disk_motor_analysis_model.motor_diameter')
    caddee_csdl_model.connect(f'system_model.system_m3l_model.qst_2_{pref}_disk_pitt_peters_model.rpm', f'system_model.system_m3l_model.qst_2_{pref}_disk_motor_analysis_model.rpm')

    # # qst 3
    # caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.qst_3_{pref}_disk_motor_analysis_model.motor_diameter')
    # caddee_csdl_model.connect(f'system_model.system_m3l_model.qst_3_{pref}_disk_pitt_peters_model.rpm', f'system_model.system_m3l_model.qst_3_{pref}_disk_motor_analysis_model.rpm')

    # # qst 4
    # caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.qst_4_{pref}_disk_motor_analysis_model.motor_diameter')
    # caddee_csdl_model.connect(f'system_model.system_m3l_model.qst_4_{pref}_disk_pitt_peters_model.rpm', f'system_model.system_m3l_model.qst_4_{pref}_disk_motor_analysis_model.rpm')

    # # qst 5
    # caddee_csdl_model.connect(f'system_model.system_m3l_model.{pref}_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.qst_5_{pref}_disk_motor_analysis_model.motor_diameter')
    # caddee_csdl_model.connect(f'system_model.system_m3l_model.qst_5_{pref}_disk_pitt_peters_model.rpm', f'system_model.system_m3l_model.qst_5_{pref}_disk_motor_analysis_model.rpm')

prfxs = ['qst_3', 'qst_4', 'qst_5', 'qst_6', 'qst_7', 'qst_8', 'qst_9', 'qst_10']

for prfx in prfxs:
    caddee_csdl_model.connect(f'system_model.system_m3l_model.pp_disk_motor_sizing_model.motor_diameter', f'system_model.system_m3l_model.{prfx}_pp_disk_motor_analysis_model.motor_diameter')
    caddee_csdl_model.connect(f'system_model.system_m3l_model.{prfx}_pp_disk_bem_model.rpm', f'system_model.system_m3l_model.{prfx}_pp_disk_motor_analysis_model.rpm')


# +3g
caddee_csdl_model.connect('system_model.system_m3l_model.pp_disk_motor_sizing_model.motor_diameter', 'system_model.system_m3l_model.plus_3g_sizing_pp_disk_motor_analysis_model.motor_diameter')
caddee_csdl_model.connect('system_model.system_m3l_model.plus_3g_sizing_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.plus_3g_sizing_pp_disk_motor_analysis_model.rpm')
caddee_csdl_model.add_constraint(f'system_model.system_m3l_model.plus_3g_sizing_pp_disk_motor_analysis_model.torque_delta', lower=0.1)

# -1g
caddee_csdl_model.connect('system_model.system_m3l_model.pp_disk_motor_sizing_model.motor_diameter', 'system_model.system_m3l_model.minus_1g_sizing_pp_disk_motor_analysis_model.motor_diameter')
caddee_csdl_model.connect('system_model.system_m3l_model.minus_1g_sizing_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.minus_1g_sizing_pp_disk_motor_analysis_model.rpm')
caddee_csdl_model.add_constraint(f'system_model.system_m3l_model.minus_1g_sizing_pp_disk_motor_analysis_model.torque_delta', lower=0.1)

# climb
caddee_csdl_model.connect('system_model.system_m3l_model.pp_disk_motor_sizing_model.motor_diameter', 'system_model.system_m3l_model.climb_1_pp_disk_motor_analysis_model.motor_diameter')
caddee_csdl_model.connect('system_model.system_m3l_model.climb_1_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.climb_1_pp_disk_motor_analysis_model.rpm')
caddee_csdl_model.add_constraint(f'system_model.system_m3l_model.climb_1_pp_disk_motor_analysis_model.torque_delta', lower=0.1)

# cruise
caddee_csdl_model.connect('system_model.system_m3l_model.pp_disk_motor_sizing_model.motor_diameter', 'system_model.system_m3l_model.cruise_pp_disk_motor_analysis_model.motor_diameter')
caddee_csdl_model.connect('system_model.system_m3l_model.cruise_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.cruise_pp_disk_motor_analysis_model.rpm')

# descent
caddee_csdl_model.connect('system_model.system_m3l_model.pp_disk_motor_sizing_model.motor_diameter', 'system_model.system_m3l_model.descent_1_pp_disk_motor_analysis_model.motor_diameter')
caddee_csdl_model.connect('system_model.system_m3l_model.descent_1_pp_disk_bem_model.rpm', 'system_model.system_m3l_model.descent_1_pp_disk_motor_analysis_model.rpm')

# endregion


tilt_1 = np.deg2rad(0)
tilt_2 = np.deg2rad(0)

# region actuations
h_tail_act_plus_3g = caddee_csdl_model.create_input('plus_3g_tail_actuation', val=-0.36170858)
caddee_csdl_model.add_design_variable('plus_3g_tail_actuation', 
                                lower=np.deg2rad(-25),
                                upper=np.deg2rad(25),
                                scaler=2e1,
                            )
wing_act_plus_3g = caddee_csdl_model.create_input('plus_3g_wing_actuation', val=np.deg2rad(3.2))

h_tail_act_minus_1g = caddee_csdl_model.create_input('minus_1g_tail_actuation', val=0.19485233)
caddee_csdl_model.add_design_variable('minus_1g_tail_actuation', 
                                lower=np.deg2rad(-25),
                                upper=np.deg2rad(25),
                                scaler=2e1,
                            )
wing_act_minis_1g = caddee_csdl_model.create_input('minus_1g_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_1_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_1_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_1_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_1_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_1_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


caddee_csdl_model.create_input('qst_2_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_2_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
caddee_csdl_model.create_input('qst_2_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_2_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_2_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_2_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_2_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_2_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


caddee_csdl_model.create_input('qst_3_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_3_tail_actuation', lower=np.deg2rad(-30), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_3_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_3_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_3_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_3_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_3_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_3_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


caddee_csdl_model.create_input('qst_4_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_4_tail_actuation', lower=np.deg2rad(-25), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_4_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_4_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_4_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_4_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_4_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_4_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


caddee_csdl_model.create_input('qst_5_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_5_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_5_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_5_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('qst_5_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('qst_5_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('qst_5_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('qst_5_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))


caddee_csdl_model.create_input('qst_6_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_6_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_6_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_7_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_7_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_7_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_8_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_8_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_8_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_9_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_9_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_9_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('qst_10_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('qst_10_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('qst_10_wing_actuation', val=np.deg2rad(3.2))


caddee_csdl_model.create_input('hover_1_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('climb_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('climb_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('climb_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('cruise_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('cruise_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('cruise_wing_actuation', val=np.deg2rad(3.2))

caddee_csdl_model.create_input('descent_tail_actuation', val=np.deg2rad(0.5))
caddee_csdl_model.add_design_variable('descent_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=2e1)
caddee_csdl_model.create_input('descent_wing_actuation', val=np.deg2rad(3.2))

# OEI flo
caddee_csdl_model.create_input('hover_1_oei_flo_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_flo_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_flo_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_flo_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_flo_fli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_fli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_fli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_fli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_flo_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_flo_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_flo_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_flo_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_flo_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

# OEI fli
caddee_csdl_model.create_input('hover_1_oei_fli_rlo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_rlo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rlo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rlo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_fli_rli_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_rli_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rli_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rli_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_fli_rri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_rri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_fli_rro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_rro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_rro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_fli_flo_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_flo_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_flo_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_flo_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_fli_fri_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_fri_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_fri_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_fri_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))

caddee_csdl_model.create_input('hover_1_oei_fli_fro_disk_actuation_1', val=tilt_1)
caddee_csdl_model.create_input('hover_1_oei_fli_fro_disk_actuation_2', val=tilt_2)
caddee_csdl_model.add_design_variable('hover_1_oei_fli_fro_disk_actuation_1', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('hover_1_oei_fli_fro_disk_actuation_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# endregion

# region geometric constraints/dvs
wing_twist = caddee_csdl_model.create_input('wing_twist_distribution', val=np.zeros((10, )))
caddee_csdl_model.add_design_variable('wing_twist_distribution', lower=np.deg2rad(-2), upper=np.deg2rad(2))


# pp_radius = caddee_csdl_model.create_input('pp_radius', val=4.6)
# pp_blade_twist = caddee_csdl_model.create_input('pp_blade_twist', np.array([0., 0., 0., 0., 0.]))
pp_blade_chord = caddee_csdl_model.create_input('pp_blade_chord', np.array([0., 0., 0., 0.]))
# caddee_csdl_model.add_design_variable('pp_radius', lower=4, upper=6., scaler=1e-1)
# caddee_csdl_model.add_design_variable('pp_blade_twist', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('pp_blade_chord', lower=-0.5, upper=0.5)

# caddee_csdl_model.connect('pp_radius', 'pp_in_plane_r1')
# caddee_csdl_model.connect('pp_radius', 'pp_in_plane_r2')
# caddee_csdl_model.connect('pp_radius', 'pp_in_plane_r3')
# caddee_csdl_model.connect('pp_radius', 'pp_in_plane_r4')
# caddee_csdl_model.connect('pp_radius', 'pp_in_plane_r5')
# caddee_csdl_model.connect('pp_radius', 'pp_in_plane_r6')
caddee_csdl_model.connect('pp_blade_chord', 'pp_blade_1_chord')
caddee_csdl_model.connect('pp_blade_chord', 'pp_blade_2_chord')
caddee_csdl_model.connect('pp_blade_chord', 'pp_blade_3_chord')
caddee_csdl_model.connect('pp_blade_chord', 'pp_blade_4_chord')
# caddee_csdl_model.connect('pp_blade_twist', 'pp_blade_1_twist')
# caddee_csdl_model.connect('pp_blade_twist', 'pp_blade_2_twist')
# caddee_csdl_model.connect('pp_blade_twist', 'pp_blade_3_twist')
# caddee_csdl_model.connect('pp_blade_twist', 'pp_blade_4_twist')


rear_outer_radius = caddee_csdl_model.create_input('rear_outer_radius', val=5)
rear_outer_blade_twist = caddee_csdl_model.create_input('rear_outer_blade_twist', np.deg2rad(np.array([0., 0., 0., 0., 0.])))
rear_outer_blade_twist = caddee_csdl_model.create_input('rear_outer_blade_twist_2', np.deg2rad(np.array([0., 0., 0., 0., 0.])))
rear_outer_blade_chord = caddee_csdl_model.create_input('rear_outer_blade_chord', np.array([0., 0., 0., 0.]))
rear_outer_blade_chord = caddee_csdl_model.create_input('rear_outer_blade_chord_2', np.array([0., 0., 0., 0.]))
caddee_csdl_model.add_design_variable('rear_outer_radius', lower=4., upper=6, scaler=1e-1)
# caddee_csdl_model.add_design_variable('rear_outer_blade_twist', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('rear_outer_blade_twist_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('rear_outer_blade_chord', lower=-0.7, upper=0.7)
caddee_csdl_model.add_design_variable('rear_outer_blade_chord_2', lower=-0.7, upper=0.7)

caddee_csdl_model.connect('rear_outer_blade_twist', 'rlo_blade_1_twist')
caddee_csdl_model.connect('rear_outer_blade_twist', 'rlo_blade_2_twist')
caddee_csdl_model.connect('rear_outer_blade_twist_2', 'rro_blade_1_twist')
caddee_csdl_model.connect('rear_outer_blade_twist_2', 'rro_blade_2_twist')
caddee_csdl_model.connect('rear_outer_blade_chord', 'rlo_blade_1_chord')
caddee_csdl_model.connect('rear_outer_blade_chord', 'rlo_blade_2_chord')
caddee_csdl_model.connect('rear_outer_blade_chord_2', 'rro_blade_1_chord')
caddee_csdl_model.connect('rear_outer_blade_chord_2', 'rro_blade_2_chord')
caddee_csdl_model.connect('rear_outer_radius', 'rlo_in_plane_r1')
caddee_csdl_model.connect('rear_outer_radius', 'rlo_in_plane_r2')
caddee_csdl_model.connect('rear_outer_radius', 'rlo_in_plane_r3')
caddee_csdl_model.connect('rear_outer_radius', 'rlo_in_plane_r4')
caddee_csdl_model.connect('rear_outer_radius', 'rro_in_plane_r1')
caddee_csdl_model.connect('rear_outer_radius', 'rro_in_plane_r2')
caddee_csdl_model.connect('rear_outer_radius', 'rro_in_plane_r3')
caddee_csdl_model.connect('rear_outer_radius', 'rro_in_plane_r4')


rear_inner_radius = caddee_csdl_model.create_input('rear_inner_radius', val=5)
rear_inner_blade_twist = caddee_csdl_model.create_input('rear_inner_blade_twist', np.array([0., 0., 0., 0., 0.]))
rear_inner_blade_twist = caddee_csdl_model.create_input('rear_inner_blade_twist_2', np.array([0., 0., 0., 0., 0.]))
rear_inner_blade_chord = caddee_csdl_model.create_input('rear_inner_blade_chord', np.array([0., 0., 0., 0.]))
rear_inner_blade_chord = caddee_csdl_model.create_input('rear_inner_blade_chord_2', np.array([0., 0., 0., 0.]))
caddee_csdl_model.add_design_variable('rear_inner_radius', lower=4, upper=6, scaler=1e-1)
# caddee_csdl_model.add_design_variable('rear_inner_blade_twist', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('rear_inner_blade_twist_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('rear_inner_blade_chord', lower=-0.7, upper=0.7)
caddee_csdl_model.add_design_variable('rear_inner_blade_chord_2', lower=-0.7, upper=0.7)

caddee_csdl_model.connect('rear_inner_blade_twist', 'rli_blade_1_twist')
caddee_csdl_model.connect('rear_inner_blade_twist', 'rli_blade_2_twist')
caddee_csdl_model.connect('rear_inner_blade_twist_2', 'rri_blade_1_twist')
caddee_csdl_model.connect('rear_inner_blade_twist_2', 'rri_blade_2_twist')
caddee_csdl_model.connect('rear_inner_blade_chord', 'rli_blade_1_chord')
caddee_csdl_model.connect('rear_inner_blade_chord', 'rli_blade_2_chord')
caddee_csdl_model.connect('rear_inner_blade_chord_2', 'rri_blade_1_chord')
caddee_csdl_model.connect('rear_inner_blade_chord_2', 'rri_blade_2_chord')
caddee_csdl_model.connect('rear_inner_radius', 'rli_in_plane_r1')
caddee_csdl_model.connect('rear_inner_radius', 'rli_in_plane_r2')
caddee_csdl_model.connect('rear_inner_radius', 'rli_in_plane_r3')
caddee_csdl_model.connect('rear_inner_radius', 'rli_in_plane_r4')
caddee_csdl_model.connect('rear_inner_radius', 'rri_in_plane_r1')
caddee_csdl_model.connect('rear_inner_radius', 'rri_in_plane_r2')
caddee_csdl_model.connect('rear_inner_radius', 'rri_in_plane_r3')
caddee_csdl_model.connect('rear_inner_radius', 'rri_in_plane_r4')


front_outer_radius = caddee_csdl_model.create_input('front_outer_radius', val=5)
front_outer_blade_twist = caddee_csdl_model.create_input('front_outer_blade_twist', np.array([0., 0., 0., 0., 0.]))
front_outer_blade_twist = caddee_csdl_model.create_input('front_outer_blade_twist_2', np.array([0., 0., 0., 0., 0.]))
front_outer_blade_chord = caddee_csdl_model.create_input('front_outer_blade_chord', np.array([0., 0., 0., 0.]))
front_outer_blade_chord = caddee_csdl_model.create_input('front_outer_blade_chord_2', np.array([0., 0., 0., 0.]))
caddee_csdl_model.add_design_variable('front_outer_radius', lower=4, upper=6, scaler=1e-1)
# caddee_csdl_model.add_design_variable('front_outer_blade_twist', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('front_outer_blade_twist_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('front_outer_blade_chord', lower=-0.7, upper=0.7)
caddee_csdl_model.add_design_variable('front_outer_blade_chord_2', lower=-0.7, upper=0.7)

caddee_csdl_model.connect('front_outer_blade_twist', 'flo_blade_1_twist')
caddee_csdl_model.connect('front_outer_blade_twist', 'flo_blade_2_twist')
caddee_csdl_model.connect('front_outer_blade_twist_2', 'fro_blade_1_twist')
caddee_csdl_model.connect('front_outer_blade_twist_2', 'fro_blade_2_twist')
caddee_csdl_model.connect('front_outer_blade_chord', 'flo_blade_1_chord')
caddee_csdl_model.connect('front_outer_blade_chord', 'flo_blade_2_chord')
caddee_csdl_model.connect('front_outer_blade_chord_2', 'fro_blade_1_chord')
caddee_csdl_model.connect('front_outer_blade_chord_2', 'fro_blade_2_chord')
caddee_csdl_model.connect('front_outer_radius', 'flo_in_plane_r1')
caddee_csdl_model.connect('front_outer_radius', 'flo_in_plane_r2')
caddee_csdl_model.connect('front_outer_radius', 'flo_in_plane_r3')
caddee_csdl_model.connect('front_outer_radius', 'flo_in_plane_r4')
caddee_csdl_model.connect('front_outer_radius', 'fro_in_plane_r1')
caddee_csdl_model.connect('front_outer_radius', 'fro_in_plane_r2')
caddee_csdl_model.connect('front_outer_radius', 'fro_in_plane_r3')
caddee_csdl_model.connect('front_outer_radius', 'fro_in_plane_r4')


front_inner_radius = caddee_csdl_model.create_input('front_inner_radius', val=5)
front_inner_blade_twist = caddee_csdl_model.create_input('front_inner_blade_twist', np.array([0., 0., 0., 0., 0.]))
front_inner_blade_twist = caddee_csdl_model.create_input('front_inner_blade_twist_2', np.array([0., 0., 0., 0., 0.]))
front_inner_blade_chord = caddee_csdl_model.create_input('front_inner_blade_chord', np.array([0., 0., 0., 0.]))
front_inner_blade_chord = caddee_csdl_model.create_input('front_inner_blade_chord_2', np.array([0., 0., 0., 0.]))
caddee_csdl_model.add_design_variable('front_inner_radius', lower=4, upper=6, scaler=1e-1)
# caddee_csdl_model.add_design_variable('front_inner_blade_twist', lower=np.deg2rad(-10), upper=np.deg2rad(10))
# caddee_csdl_model.add_design_variable('front_inner_blade_twist_2', lower=np.deg2rad(-10), upper=np.deg2rad(10))
caddee_csdl_model.add_design_variable('front_inner_blade_chord', lower=-0.7, upper=0.7)
caddee_csdl_model.add_design_variable('front_inner_blade_chord_2', lower=-0.7, upper=0.7)

caddee_csdl_model.connect('front_inner_blade_twist', 'fli_blade_1_twist')
caddee_csdl_model.connect('front_inner_blade_twist', 'fli_blade_2_twist')
caddee_csdl_model.connect('front_inner_blade_twist_2', 'fri_blade_1_twist')
caddee_csdl_model.connect('front_inner_blade_twist_2', 'fri_blade_2_twist')
caddee_csdl_model.connect('front_inner_blade_chord', 'fli_blade_1_chord')
caddee_csdl_model.connect('front_inner_blade_chord', 'fli_blade_2_chord')
caddee_csdl_model.connect('front_inner_blade_chord_2', 'fri_blade_1_chord')
caddee_csdl_model.connect('front_inner_blade_chord_2', 'fri_blade_2_chord')
caddee_csdl_model.connect('front_inner_radius', 'fli_in_plane_r1')
caddee_csdl_model.connect('front_inner_radius', 'fli_in_plane_r2')
caddee_csdl_model.connect('front_inner_radius', 'fli_in_plane_r3')
caddee_csdl_model.connect('front_inner_radius', 'fli_in_plane_r4')
caddee_csdl_model.connect('front_inner_radius', 'fri_in_plane_r1')
caddee_csdl_model.connect('front_inner_radius', 'fri_in_plane_r2')
caddee_csdl_model.connect('front_inner_radius', 'fri_in_plane_r3')
caddee_csdl_model.connect('front_inner_radius', 'fri_in_plane_r4')


caddee_csdl_model.create_input('pp_twist_cp', val=np.deg2rad(np.array([80, 65, 45, 25])))
caddee_csdl_model.create_input('flo_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('fli_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('fri_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('fro_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('rlo_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('rli_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('rri_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))
caddee_csdl_model.create_input('rro_twist_cp', val=np.deg2rad(np.array([30, 20, 10, 5])))

caddee_csdl_model.add_design_variable('pp_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('flo_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('fli_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('fri_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('fro_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('rlo_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('rli_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('rri_twist_cp', lower=0, upper=np.deg2rad(85))
caddee_csdl_model.add_design_variable('rro_twist_cp', lower=0, upper=np.deg2rad(85))

# endregion

# region rpm constraints
# hover_1_rlo_rpm = caddee_csdl_model.declare_variable('hover_1_rlo_disk_bem_model_rpm', shape=(1, 1))
# hover_1_rro_rpm = caddee_csdl_model.declare_variable('hover_1_rro_disk_bem_model_rpm', shape=(1, 1))
# hover_1_ro_rpm_constraint = hover_1_rlo_rpm * 1 - hover_1_rro_rpm * 1

# hover_1_rli_rpm = caddee_csdl_model.declare_variable('hover_1_rli_disk_bem_model_rpm', shape=(1, 1))
# hover_1_rri_rpm = caddee_csdl_model.declare_variable('hover_1_rri_disk_bem_model_rpm', shape=(1, 1))
# hover_1_ri_rpm_constraint = hover_1_rli_rpm * 1 - hover_1_rri_rpm * 1

# hover_1_flo_rpm = caddee_csdl_model.declare_variable('hover_1_flo_disk_bem_model_rpm', shape=(1, 1))
# hover_1_fro_rpm = caddee_csdl_model.declare_variable('hover_1_fro_disk_bem_model_rpm', shape=(1, 1))
# hover_1_fo_rpm_constraint = hover_1_flo_rpm * 1 - hover_1_fro_rpm * 1

# hover_1_fli_rpm = caddee_csdl_model.declare_variable('hover_1_fli_disk_bem_model_rpm', shape=(1, 1))
# hover_1_fri_rpm = caddee_csdl_model.declare_variable('hover_1_fri_disk_bem_model_rpm', shape=(1, 1))
# hover_1_fi_rpm_constraint = hover_1_fli_rpm * 1 - hover_1_fri_rpm * 1

# caddee_csdl_model.register_output('hover_1_ro_rpm_constraint', hover_1_ro_rpm_constraint *  1)
# caddee_csdl_model.register_output('hover_1_ri_rpm_constraint', hover_1_ri_rpm_constraint)
# caddee_csdl_model.register_output('hover_1_fo_rpm_constraint', hover_1_fo_rpm_constraint)
# caddee_csdl_model.register_output('hover_1_fi_rpm_constraint', hover_1_fi_rpm_constraint)

# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rlo_disk_bem_model.rpm','hover_1_rlo_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rro_disk_bem_model.rpm','hover_1_rro_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rli_disk_bem_model.rpm','hover_1_rli_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_rri_disk_bem_model.rpm','hover_1_rri_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_flo_disk_bem_model.rpm','hover_1_flo_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fro_disk_bem_model.rpm','hover_1_fro_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fli_disk_bem_model.rpm','hover_1_fli_disk_bem_model_rpm')
# caddee_csdl_model.connect('system_model.system_m3l_model.hover_1_fri_disk_bem_model.rpm','hover_1_fri_disk_bem_model_rpm')
# endregion

# region system level constraints and objective
caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.minus_1g_sizing_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.hover_1_oei_flo_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.hover_1_oei_fli_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)

caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_1_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_2_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_4_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_5_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_6_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_7_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_8_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_9_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_10_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)

caddee_csdl_model.add_constraint('system_model.system_m3l_model.hover_1_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.climb_1_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.cruise_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.descent_1_euler_eom_gen_ref_pt.trim_residual', equals=0, scaler=5)

# caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.new_stress',upper=427E6/1.,scaler=1E-8)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.new_stress', upper=600E6/1.5, scaler=1E-8) # Quasi isotropic 
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.new_stress_composite',lower=-1/1.5, upper=1/1.5, scaler=1) # Quasi isotropic 
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.new_stress', upper=350E6/1.5, scaler=1e-8) # Aluminum alloy
caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.wing_beam_displacement', lower=-0.5, upper=0.5, scaler=0.5)
# caddee_csdl_model.add_constraint('system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.wing_beam_displacement', upper=0.25, scaler=1)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.hover_1_total_noise_model.A_weighted_total_spl', upper=74, scaler=1e-2)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_1_total_noise_model.A_weighted_total_spl', upper=73, scaler=1e-2)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_2_total_noise_model.A_weighted_total_spl', upper=70, scaler=1e-2)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_3_total_noise_model.A_weighted_total_spl', upper=68, scaler=1e-2)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_4_total_noise_model.A_weighted_total_spl', upper=67, scaler=1e-2)
caddee_csdl_model.add_constraint('system_model.system_m3l_model.qst_5_total_noise_model.A_weighted_total_spl', upper=67, scaler=1e-2)

caddee_csdl_model.add_constraint('system_model.system_m3l_model.SOC_model.finalSoC', lower=0.2, scaler=5)

caddee_csdl_model.add_objective('system_model.system_m3l_model.total_constant_mass_properties.total_constant_mass', scaler=1e-3)

# t1 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.plus_3g_sizing_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t2 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.minus_1g_sizing_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t3 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.hover_1_oei_flo_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t4 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.hover_1_oei_fli_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))

# t5 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t6 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_2_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t7 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t8 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_4_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t9 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_5_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t10 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_6_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t11 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_7_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t12 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_8_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t13 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_9_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t14 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.qst_10_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))

# t15 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.hover_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t16 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.climb_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t17 =caddee_csdl_model.declare_variable('system_model.system_m3l_model.cruise_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))
# t18 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.descent_1_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))

# dummy_input = caddee_csdl_model.create_input('dummy_input', val=10.)
# dummy_objective = caddee_csdl_model.register_output('dummy_objective/', dummy_input*1.)

# caddee_csdl_model.add_objective('dummy_input')
# # caddee_csdl_model.add_objective('system_model.system_m3l_model.total_constant_mass_properties.total_mass', scaler=1e-3)

# combined_trim = caddee_csdl_model.register_output('combined_trim', 
#     t1*1 +  t2*1 + t3*1 + t4*1 + t5*1 + t6*1 + t7*1 + t8*1 + t9*1 + t10*1 + t11*1 + t12*1 + t13 *1+ t14*1 + t15*1 + t16*1 + t17*1 + t18*1)

# combined_trim = caddee_csdl_model.register_output('combined_trim', t5*1 + t6*1 + t7*1 + t8*1 + t9*1 + t15*1)

# combined_trim = caddee_csdl_model.register_output('combined_trim', trim_1 *1 + trim_2*1 + trim_3*1 + trim_4*1 + trim_5 * 1 + trim_6 * 1 + trim_7 * 1 + trim_8 * 1 + trim_9 * 1 + trim_10*1)
# caddee_csdl_model.add_objective('combined_trim',  scaler=1e1)
# endregion


# region TC 2 csdl model
import csdl
upstream_model = csdl.Model()
wing_area = upstream_model.create_input('wing_area', val=200.)
wing_taper_ratio = upstream_model.create_input('wing_taper_ratio', val=0.45)
aspect_ratio = upstream_model.create_input('wing_aspect_ratio', val=13)

wing_span = (aspect_ratio * wing_area)**0.5
wing_root_chord = 2 * wing_area/((1 + wing_taper_ratio) * wing_span)
wing_tip_chord = wing_root_chord * wing_taper_ratio

tm = upstream_model.create_input('tail_moment_arm_input', val=17.23)

tail_area = upstream_model.create_input('tail_area', val=30)
tail_taper_ratio = upstream_model.create_input('tail_taper_ratio', val=0.6)
tail_aspect_ratio = upstream_model.create_input('tail_aspect_ratio', val=5)

upstream_model.add_design_variable('wing_area', upper=250, lower=150, scaler=5e-3)
upstream_model.add_design_variable('wing_aspect_ratio', upper=15, lower=9, scaler=1e-1)
upstream_model.add_design_variable('tail_area', upper=40, lower=20, scaler=5e-2)
upstream_model.add_design_variable('tail_aspect_ratio', upper=7, lower=3.5, scaler=1e-1)
upstream_model.add_design_variable('tail_moment_arm_input', upper=20, lower=15.5, scaler=5e-2)



front_outer_r = upstream_model.declare_variable('front_outer_radius_up', shape=(1, ))
front_inner_r = upstream_model.declare_variable('front_inner_radius_up', shape=(1, ))
rear_outer_r = upstream_model.declare_variable('rear_outer_radius_up', shape=(1, ))
rear_inner_r = upstream_model.declare_variable('rear_inner_radius_up', shape=(1, ))

radii_front_wing_ratio = (front_outer_r * 1 + front_inner_r * 1) / (0.5 * wing_span)
radii_rear_wing_ratio = (rear_outer_r * 1 + rear_inner_r * 1) / (0.5 * wing_span)
upstream_model.register_output('radii_front_wing_ratio', radii_front_wing_ratio * 1)
upstream_model.register_output('radii_rear_wing_ratio', radii_rear_wing_ratio)
upstream_model.add_constraint('radii_front_wing_ratio', equals=0.4)
upstream_model.add_constraint('radii_rear_wing_ratio', equals=0.4)

tail_span = (tail_aspect_ratio * tail_area)**0.5
tail_root_chord = 2 * tail_area/((1 + tail_taper_ratio) * tail_span)
tail_tip_chord = tail_root_chord * tail_taper_ratio

upstream_model.register_output('tail_moment_arm', tm * 1)
upstream_model.register_output('wing_root_chord', wing_root_chord)
upstream_model.register_output('wing_tip_chord_left', wing_tip_chord)
upstream_model.register_output('wing_tip_chord_right', wing_tip_chord * 1)
upstream_model.register_output('wing_span', wing_span)

upstream_model.register_output('tail_root_chord', tail_root_chord)
upstream_model.register_output('tail_tip_chord_left', tail_tip_chord)
upstream_model.register_output('tail_tip_chord_right', tail_tip_chord * 1)
upstream_model.register_output('tail_span', tail_span)

tc2_model = csdl.Model()
tc2_model.add(submodel=upstream_model, name='geometry_processing_model', promotes=[])
tc2_model.add(submodel=caddee_csdl_model, name='caddee_csdl_model', promotes=[])

tc2_model.connect('caddee_csdl_model.rear_outer_radius', 'geometry_processing_model.rear_outer_radius_up')
tc2_model.connect('caddee_csdl_model.rear_inner_radius', 'geometry_processing_model.rear_inner_radius_up')
tc2_model.connect('caddee_csdl_model.front_outer_radius', 'geometry_processing_model.front_outer_radius_up')
tc2_model.connect('caddee_csdl_model.front_inner_radius', 'geometry_processing_model.front_inner_radius_up')

tc2_model.connect('geometry_processing_model.wing_root_chord', 'caddee_csdl_model.wing_root_chord')
tc2_model.connect('geometry_processing_model.wing_tip_chord_left', 'caddee_csdl_model.wing_tip_chord_left')
tc2_model.connect('geometry_processing_model.wing_tip_chord_right', 'caddee_csdl_model.wing_tip_chord_right')
tc2_model.connect('geometry_processing_model.wing_span', 'caddee_csdl_model.wing_span')
tc2_model.connect('geometry_processing_model.tail_moment_arm', 'caddee_csdl_model.tail_moment_arm')

tc2_model.connect('geometry_processing_model.tail_root_chord', 'caddee_csdl_model.tail_root_chord')
tc2_model.connect('geometry_processing_model.tail_tip_chord_left', 'caddee_csdl_model.tail_tip_chord_left')
tc2_model.connect('geometry_processing_model.tail_tip_chord_right', 'caddee_csdl_model.tail_tip_chord_right')
tc2_model.connect('geometry_processing_model.tail_span', 'caddee_csdl_model.tail_span')

tc2_model.add_constraint('caddee_csdl_model.system_representation.outputs_model.design_outputs_model.fuselage_length', lower=27, scaler=1e-2)
tc2_model.connect('caddee_csdl_model.system_representation.outputs_model.design_outputs_model.fuselage_length', 'caddee_csdl_model.system_model.system_m3l_model.m4_regression.fuselage_length')
tc2_model.connect('geometry_processing_model.wing_aspect_ratio', 'caddee_csdl_model.system_model.system_m3l_model.m4_regression.wing_AR')
tc2_model.connect('geometry_processing_model.wing_area', 'caddee_csdl_model.system_model.system_m3l_model.m4_regression.wing_area')
tc2_model.connect('geometry_processing_model.tail_area', 'caddee_csdl_model.system_model.system_m3l_model.m4_regression.tail_area')

wing_beam_tcap = tc2_model.declare_variable('caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tcap', shape=(20,))
wing_beam_tweb = tc2_model.declare_variable('caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tweb', shape=(20,))
left_tcap = wing_beam_tcap[0:10]
left_tweb = wing_beam_tweb[0:10]
# right_tcap = wing_beam_tcap[-1:-10]
right_tcap = tc2_model.create_output('right_tcap', shape=(10, ))
right_tweb = tc2_model.create_output('right_tweb', shape=(10, ))
indices = np.flip(np.arange(10, 20, 1, dtype=int))
for i in range(10):
    right_tcap[i] = wing_beam_tcap[int(indices[i])]
    right_tweb[i] = wing_beam_tweb[int(indices[i])]

tc2_model.register_output('tcap_symmetry', right_tcap-left_tcap)
tc2_model.register_output('tweb_symmetry', right_tweb-left_tweb)

tc2_model.add_constraint('tcap_symmetry', equals=0, scaler=1e-2)
tc2_model.add_constraint('tweb_symmetry', equals=0, scaler=1e-2)

tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_3_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_4_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_5_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_6_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_7_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_8_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_9_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.qst_10_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.descent_1_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.climb_1_pp_disk_bem_model.T', lower=20, scaler=1e-2)
tc2_model.add_constraint('caddee_csdl_model.system_model.system_m3l_model.cruise_pp_disk_bem_model.T', lower=20, scaler=1e-2)
# endregion

design_configuration_map = {
   'minus_1g_sizing':"minus_1g_configuration", 
   'plus_3g_sizing':"plus_3g_configuration",
   'hover_1':"hover_1_configuration", 
   'cruise':"cruise_configuration", 
   'climb_1':"climb_configuration",
   'qst_1':"quasi_steady_transition_1",    
   'qst_2':"quasi_steady_transition_2",    
   'qst_3':"quasi_steady_transition_3",    
   'qst_4':"quasi_steady_transition_4",    
   'qst_5':"quasi_steady_transition_5",    
   'qst_6':"quasi_steady_transition_6",    
   'qst_7':"quasi_steady_transition_7",    
   'qst_8':"quasi_steady_transition_8",    
   'qst_9':"quasi_steady_transition_9",    
   'qst_10':"quasi_steady_transition_10",    
   'hover_1_oei_flo':"hover_configuration_oei_flo", 
   'hover_1_oei_fli':"hover_configuration_oei_fli", 
#    :"quasi_steady_transition_1_oei_flo",    
#    :"quasi_steady_transition_2_oei_flo",
#    :"quasi_steady_transition_3_oei_flo",
#    :"quasi_steady_transition_4_oei_flo",
}

import lsdo_dash.api as ld
caddee_viz = ld.caddee_plotters.CaddeeViz(
    caddee = caddee,
    system_m3l_model = system_m3l_model,
    design_configuration_map=design_configuration_map,
    system_prefix='caddee_csdl_model',
)
import time
start = time.time()
# import cProfile
# profiler = cProfile.Profile()
# profiler.enable()
rep = csdl.GraphRepresentation(tc2_model)
# profiler.disable()
# profiler.dump_stats('output')
end = time.time()
compile_time = end - start
print('compile time: ', compile_time)
# exit()

class TC2DB(ld.DashBuilder):
    def define(self, *args):
        # rep = self.load_cached_vars() # Uncomment when runing visualization

        # opt_frame[1, 0] = ld.default_plotters.build_constraints_plotter(rep, False)
        # opt_frame[1, 1] = ld.default_plotters.build_objective_plotter(rep)
        # opt_frame[1, 2] = ld.default_plotters.build_design_var_plotter(rep, False)

        # geo_frame = self.add_frame(
        #     'geometry_perturb_frame',
        #     height_in=8.,
        #     width_in=12.,
        #     ncols=120,
        #     nrows = 90,
        #     wspace=0.4,
        #     hspace=0.4,
        # )

        # center_x = 20  # eyeballing center x coordinate of geometry
        # center_z = 11  # eyeballing center z coordinate of geometry
        # camera_settings = {
        #     'pos': (-32, -22, 32),
        #     'viewup': (0, 0, 1),
        #     'focalPoint': (center_x+8, 0+6, center_z-10)
        # }
        # # geo_frame[30:60,0:30] = caddee_viz.build_geometry_plotter(show = False, camera_settings = camera_settings)

        # geo_elements = []
        # geo_elements.append(caddee_viz.build_geometry_plotter(show = 0, design_condition_name = 'qst_2'))
        # # geo_elements.append(caddee_viz.build_state_plotter(displacements_plus_3g, rep = rep, displacements = 100))
        # geo_frame[:,:] = caddee_viz.build_vedo_renderer(geo_elements, camera_settings = camera_settings)
        # geo_elements = []
        # geo_elements.append(caddee_viz.build_geometry_plotter(show = 0))
        # geo_elements.append(caddee_viz.build_state_plotter(displacements_minus_1g, rep = rep, displacements = 100))
        # geo_frame[:,60:120] = caddee_viz.build_vedo_renderer(geo_elements, camera_settings = camera_settings)
        # geo_elements = []
        # # geo_elements.append(caddee_viz.build_geometry_plotter(show = 0))
        # geo_elements.append(caddee_viz.build_state_plotter(wing_cp_index_functions['descent_1'], rep = rep))
        # geo_frame[:,60:120] = caddee_viz.build_vedo_renderer(geo_elements, camera_settings = camera_settings)

        # center_x = 20  # eyeballing center x coordinate of geometry
        # center_z = 11  # eyeballing center z coordinate of geometry
        # camera_settings = {
        #     'pos': (-46, -36, 30),
        #     # 'pos': (-26, -26, 30),
        #     'viewup': (0, 0, 1),
        #     'focalPoint': (center_x+8, 0+6, center_z-15)
        # }
        # geo_frame[30:60,0:30] = caddee_viz.build_geometry_plotter('climb_1',show = False, camera_settings = camera_settings)
        # geo_frame[30:60,30:60] = caddee_viz.build_geometry_plotter('cruise', camera_settings = camera_settings)
        # geo_frame[30:60,60:90] = caddee_viz.build_geometry_plotter('hover_1', camera_settings = camera_settings)

        # TC 2 full geo frame
        geo_frame = self.add_frame(
            'full_geometry_frame',
            height_in=16.,
            width_in=30.,
            ncols=200,
            nrows = 110,
            wspace=0.4,
            hspace=0.4,
        )

        center_x = 20  # eyeballing center x coordinate of geometry
        center_z = 11  # eyeballing center z coordinate of geometry
        position = (-49.71375366736947/ 1.6, -26.09937777096867/ 1.6, 100.3252841988041 / 1.6)
        camera_settings = {
            'pos' : position,
            # 'pos': (-32, -22, 32),
            'viewup': (0, 0, 1),
            'focalPoint': (center_x+8, -2, center_z-12)
        }
        # geo_frame[30:60,0:30] = caddee_viz.build_geometry_plotter(show = False, camera_settings = camera_settings)

        wing_surfaces = ld.caddee_plotters.get_index_function_surfaces(wing_cp_index_functions['cruise'])
        rotor_surfaces = []
        for rindex in rotor_index_functions_qst_1:
            rotor_surfaces = rotor_surfaces + ld.caddee_plotters.get_index_function_surfaces(rindex)
        remove_surfaces = wing_surfaces + rotor_surfaces

        # :::::::::::::::::::::::::::: Cruise ::::::::::::::::::::::::::::
        geo_elements = []
        geo_elements.append(caddee_viz.build_geometry_plotter(show = 0, remove_primitives = remove_surfaces, opacity=0.3))
        geo_elements.append(caddee_viz.build_state_plotter(
            wing_cp_index_functions['cruise'],
            rep = rep,
            vmin = -2,
            vmax = 1.5,
            normal_arrows = False,
            colormap='coolwarm',
            grid_num=40,
        ))
        def add_axes(ax_subplot, data_dict, data_dict_history):
            import matplotlib
            import matplotlib.pyplot as plt
            plt.rcParams.update({'font.size': 10})
            plt.colorbar(matplotlib.cm.ScalarMappable(norm=matplotlib.colors.Normalize(vmin=-2, vmax=1.5), cmap='coolwarm'), cax = ax_subplot, location="bottom")
            ax_subplot.set_title(r'$C_p$')
        
        geo_frame[55:57, 20:50] = ld.BaseAxesPlotter(
            plot_function = add_axes,
        )
        rotor_primitives = []
        # for rindex in rotor_index_functions_hover_1:
        #     geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
        #     for primitive_name in rindex.coefficients:
        #         rotor_primitives.append(primitive_name)
        geo_frame[0:60, 0:80] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Cruise',
        )

        import matplotlib.pyplot as plt

        def plot_2D(ax_subplot, data_dict, data_dict_history):
            rlo = data_dict['caddee_csdl_model.system_model.system_m3l_model.rlo_disk_motor_sizing_model.mass']
            rli = data_dict['caddee_csdl_model.system_model.system_m3l_model.rli_disk_motor_sizing_model.mass']
            rri = data_dict['caddee_csdl_model.system_model.system_m3l_model.rri_disk_motor_sizing_model.mass']
            rro = data_dict['caddee_csdl_model.system_model.system_m3l_model.rro_disk_motor_sizing_model.mass']
            flo = data_dict['caddee_csdl_model.system_model.system_m3l_model.flo_disk_motor_sizing_model.mass']
            fli = data_dict['caddee_csdl_model.system_model.system_m3l_model.fli_disk_motor_sizing_model.mass']
            fri = data_dict['caddee_csdl_model.system_model.system_m3l_model.fri_disk_motor_sizing_model.mass']
            fro = data_dict['caddee_csdl_model.system_model.system_m3l_model.fro_disk_motor_sizing_model.mass']
            pp = data_dict['caddee_csdl_model.system_model.system_m3l_model.pp_disk_motor_sizing_model.mass']
            battery = data_dict['caddee_csdl_model.system_model.system_m3l_model.battery_simple_battery_sizing.battery_mass']
            wing_mass = data_dict['caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.MassProp.mass']
            empennage = data_dict['caddee_csdl_model.system_model.system_m3l_model.m4_regression.empennage_struct_mass']
            fus_boom = data_dict['caddee_csdl_model.system_model.system_m3l_model.m4_regression.wing_boom_fuselage_structural_mass']
            nsm = data_dict['caddee_csdl_model.system_model.system_m3l_model.m4_regression.nsm_mass']
            
            lift_rotor_mass = rlo + rli + rri + rro + flo + fli + fri + fro
            pusher_motor_mass = pp
            data = [lift_rotor_mass[0], pusher_motor_mass[0], battery[0], wing_mass[0], empennage[0], fus_boom[0], nsm[0]]      
            # print(data)    

            labels = ['lift motors', 'pusher motor', 'battery', 'wing mass', 'empennage', 'fuselage & booms', 'nsm']
            ax_subplot.pie(data, labels=labels, autopct='%.0f%%')
            # x_axis = data_dict_history['global_ind']
            # cur_ind = x_axis[-1]
            # sns.lineplot(x=list(range(len(y_axis))), y=np.array(y_axis).flatten(), ax=ax_subplot)
            # sns.lineplot(x=list(range(len(y_axis))), y=b, ax=ax_subplot, linestyle='--' )
            # ax_subplot.set_ylabel('constraint')
            # ax_subplot.set_ylim([0.0, 8.0])
            # ax_subplot.set_xlabel('index')
            # ax_subplot.set_title(f'Constraint at iteration {cur_ind}')

        geo_frame[65:100, 20:50] = ld.BaseAxesPlotter(
            required_vars = [
                'caddee_csdl_model.system_model.system_m3l_model.rlo_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.rli_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.rri_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.rro_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.flo_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.fli_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.fri_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.fro_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.pp_disk_motor_sizing_model.mass',
                'caddee_csdl_model.system_model.system_m3l_model.battery_simple_battery_sizing.battery_mass',
                'caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.Aframe.MassProp.mass',
                'caddee_csdl_model.system_model.system_m3l_model.m4_regression.empennage_struct_mass',
                'caddee_csdl_model.system_model.system_m3l_model.m4_regression.empennage_struct_mass',
                'caddee_csdl_model.system_model.system_m3l_model.m4_regression.wing_boom_fuselage_structural_mass',
                'caddee_csdl_model.system_model.system_m3l_model.m4_regression.nsm_mass',

            ],
            history = False,
            plot_function = plot_2D,
        )


        # :::::::::::::::::::::::::::: + 3 ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        def z_reverser(locations, state):
            # Reverse the z axis of the geometry because beam solver
            state[:,2] = -state[:,2]
            return locations, state
        center_x = 20  # eyeballing center x coordinate of geometry
        center_z = 11  # eyeballing center z coordinate of geometry
        # position = (-49.71375366736947/1.65, -26.09937777096867/1.65, 100.3252841988041/1.65)
        camera_settings = {
            # 'pos': position,
            'pos': (-40, -22, 37),
            'viewup': (0, 0, 1),
            'focalPoint': (center_x+8, -3, center_z-5)
        }
        geo_elements = []
        geo_elements.append(caddee_viz.build_geometry_plotter(show = False, remove_primitives = rotor_surfaces, opacity = 0.3))
        geo_elements.append(caddee_viz.build_state_plotter(
            displacements_plus_3g,
            rep = rep,
            displacements= 20,
            state_callback = z_reverser,
            color='#bacdff00', #'#182B49',
            opacity=1.,
        ))

        geo_elements.append(caddee_viz.build_state_plotter(
            displacements_minus_1g,
            rep = rep,
            displacements= 20,
            state_callback = z_reverser,
            color='#ffefba00', #'#FFCD00', #(0.859, 0.843, 0.514),
            opacity=1.,
        ))

        geo_frame[0:60, 60:140] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = '+3g',
        )
        
        disk_prexixes = ['rlo', 'rli', 'rri', 'rro', 'flo', 'fli', 'fri', 'fro']
        bem_conditions = ['hover_1', 'qst_1']
        pitt_peters_conditions = ['qst_2', 'qst_3', 'qst_4', 'qst_5']
        oeis = ['hover_1_oei_flo', 'hover_1_oei_fli']
        other_cond = ['plus_3g_sizing', 'minus_1g_sizing', 'hover_1', 'climb_1', 'descent_1', 'cruise']
        total_cond = bem_conditions + pitt_peters_conditions + oeis + other_cond
        
        on_design_energy_cond = pitt_peters_conditions + bem_conditions + ['qst_6', 'qst_6', 'qst_6', 'qst_6', 'qst_6', 'hover_1', 'climb_1', 'descent_1', 'cruise']

        for cond in on_design_energy_cond:
            if cond == 'hover_1' or cond == 'qst_1' or cond == 'qst_2':
                for i in range(8):
                    self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_energy_model.input_power_{i+1}', history=True)
                    self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_energy_model.{cond}_energy', history=True)
            else:
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_energy_model.input_power_1', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_energy_model.{cond}_energy', history=True)

        for prf in disk_prexixes:
            self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.hover_1_{prf}_disk_bem_model.chord_profile', history=True)
            self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.hover_1_{prf}_disk_bem_model.twist_profile', history=True)
            self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.hover_1_{prf}_disk_bem_model.propeller_radius', history=True)
            self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{prf}_disk_motor_sizing_model.mass', history=True)
            for cond in bem_conditions:
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.thrust_vector', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.thrust_origin', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.rpm', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._dT', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model.Q', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._ux', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model.FOM', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._dD', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.phi_bracketed_search_group.phi_distribution', history=True)
        
        for cond in oeis:
            if cond == 'hover_1_oei_flo':
                for prf in disk_prexixes:
                    if prf == 'flo':
                        pass
                    else:    
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.thrust_vector', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.thrust_origin', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.rpm', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._dT', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model.Q', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._ux', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model.FOM', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.phi_bracketed_search_group.phi_distribution', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_motor_analysis_model.implicit_em_torque_model.load_torque', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_motor_analysis_model.input_power', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_motor_analysis_model.torque_delta', history=True)
            else:
                for prf in disk_prexixes:
                    if prf == 'fli':
                        pass
                    else:    
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.thrust_vector', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.thrust_origin', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.rpm', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._dT', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model.Q', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model._ux', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.induced_velocity_model.FOM', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_bem_model.phi_bracketed_search_group.phi_distribution', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_motor_analysis_model.implicit_em_torque_model.load_torque', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_motor_analysis_model.input_power', history=True)
                        self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_motor_analysis_model.torque_delta', history=True)

        
        for cond in pitt_peters_conditions:
            self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_wing_vlm_mesh_{cond}h_tail_vlm_mesh_{cond}_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.F', history=True)
            for prf in disk_prexixes:
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.thrust_vector', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.thrust_origin', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.rpm', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.pitt_peters_post_process_model_2._dT', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.pitt_peters_post_process_model_2.total_torque', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.pitt_peters_post_process_model_2._ux', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.pitt_peters_post_process_model_2._dD', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_{prf}_disk_pitt_peters_model.F', history=True)


        for cond in total_cond:
            if 'hover' in cond:
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_euler_eom_gen_ref_pt.trim_residual', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_time', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.u', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.v', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.w', history=True)
            elif 'climb' in cond or 'descent' in cond:
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_euler_eom_gen_ref_pt.trim_residual', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_pitch_angle', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_speed', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_time', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.u', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.v', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.w', history=True)
            else:
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_euler_eom_gen_ref_pt.trim_residual', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_pitch_angle', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_range', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_speed', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.{cond}_time', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.u', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.v', history=True)
                self.save_variables(f'caddee_csdl_model.system_model.system_m3l_model.{cond}_ac_states_operation.w', history=True)

        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.cruise_total_constant_mass_properties.total_mass', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tweb', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tcap', history=True)

        self.save_variables('geometry_processing_model.wing_area', history=True)
        self.save_variables('geometry_processing_model.wing_aspect_ratio', history=True)
        self.save_variables('geometry_processing_model.tail_area', history=True)
        self.save_variables('geometry_processing_model.tail_aspect_ratio', history=True)

        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.pp_disk_motor_sizing_model.mass', history=True)

        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.wing_beam_displacement', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_wing_eb_beam_model.new_stress', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_pp_disk_motor_analysis_model.torque_delta', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_pp_disk_motor_analysis_model.input_power', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.plus_3g_sizing_pp_disk_motor_analysis_model.implicit_em_torque_model.load_torque', history=True)


        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.minus_1g_sizing_wing_eb_beam_model.wing_beam_displacement', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.minus_1g_sizing_wing_eb_beam_model.new_stress', history=True)
     
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.cruise_airfoil_ml_model.wing_vlm_mesh_cruise_cl_span_total_reynolds_number', history=True)
        self.save_variables('caddee_csdl_model.system_model.system_m3l_model.cruise_airfoil_ml_model.wing_vlm_mesh_cruise_cl_span_total_lift_coefficient', history=True)
       

        # :::::::::::::::::::::::::::: OEI FLO ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'hover_1_oei_flo')
        )
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))

        rotor_primitives = []
        for rindex in rotor_index_functions_hover_1_oei_flo:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[40:, :] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 1 / 10)',
        )

        # :::::::::::::::::::::::::::: OEI FLI ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'hover_1_oei_fli')
        )
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))

        rotor_primitives = []
        for rindex in rotor_index_functions_hover_1_oei_fli:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[40:, :] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 1 / 10)',
        )


        # :::::::::::::::::::::::::::: QST 1 ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'qst_1')
        )
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))

        rotor_primitives = []
        for rindex in rotor_index_functions_qst_1:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[40:, :] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 1 / 10)',
        )
       
        # :::::::::::::::::::::::::::: QST 2 ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'qst_2')
        )
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))

        rotor_primitives = []
        for rindex in rotor_index_functions_qst_2:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[40:, :] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 1 / 10)',
        )

        # :::::::::::::::::::::::::::: QST 3 ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'qst_3'))
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))

        rotor_primitives = []
        for rindex in rotor_index_functions_qst_3:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[0:60,120:200] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 5 / 10)',
        )

        # :::::::::::::::::::::::::::: QST 4 ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'qst_4'))
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))

        rotor_primitives = []
        for rindex in rotor_index_functions_qst_4:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[0:60,120:200] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 5 / 10)',
        )

        # :::::::::::::::::::::::::::: QST 5 ::::::::::::::::::::::::::::
        # get geometry elements to plot:
        # - full geometry plus
        # - wing pressure coefficient
        # - rotor ux
        geo_elements = []
        geo_elements.append(
            caddee_viz.build_geometry_plotter(show = 0, remove_primitives = rotor_surfaces, design_condition_name = 'qst_5'))
        # geo_elements.append(caddee_viz.build_state_plotter(
        #     wing_cp_index_functions['cruise'],
        #     rep = rep,
        #     vmin = -2,
        #     vmax = 1,
        #     normal_arrows = True,
        # ))
        camera_settings = {
            # 'pos': position,
            'pos': (-40, -22, 37),
            'viewup': (0, 0, 1),
            'focalPoint': (center_x+8, 2, center_z-5.5)
        }
        rotor_primitives = []
        for rindex in rotor_index_functions_qst_5:
            geo_elements.append(caddee_viz.build_state_plotter(rindex, rep = rep))
            for primitive_name in rindex.coefficients:
                rotor_primitives.append(primitive_name)
        geo_frame[0:60,120:200] = caddee_viz.build_vedo_renderer(
            geo_elements,
            camera_settings = camera_settings,
            show = 0,
            # title = 'Transition (point 2/10)',
        )


       


        # # TC 2 full geo frame
        # dv_and_constraint_frame = self.add_frame(
        #     'dv_and_constraints',
        #     height_in=16.,
        #     width_in=24.,
        #     ncols=120,
        #     nrows = 80,
        #     wspace=0.4,
        #     hspace=0.4,
        # )


        # center_x = 20  # eyeballing center x coordinate of geometry
        # center_z = 11  # eyeballing center z coordinate of geometry
        # camera_settings = {
        #     'pos': (-32, -22, 32),
        #     'viewup': (0, 0, 1),
        #     'focalPoint': (center_x+8, -2, center_z-10)
        # }
        # # geo_frame[30:60,0:30] = caddee_viz.build_geometry_plotter(show = False, camera_settings = camera_settings)
        sub_system_frame = self.add_frame(
            'sub_system_frame',
            height_in=16.,
            width_in=30.,
            ncols=200,
            nrows = 110,
            wspace=0.4,
            hspace=0.4,
        )
        
        import matplotlib.pyplot as plt
        import seaborn as sns

        def plot_2D(ax_subplot, data_dict, data_dict_history):
            soc = data_dict_history['caddee_csdl_model.system_model.system_m3l_model.SOC_model.finalSoC']
            noise5 = data_dict_history['caddee_csdl_model.system_model.system_m3l_model.qst_5_total_noise_model.A_weighted_total_spl']
            noise4 = data_dict_history['caddee_csdl_model.system_model.system_m3l_model.qst_4_total_noise_model.A_weighted_total_spl']
            noise3 = data_dict_history['caddee_csdl_model.system_model.system_m3l_model.qst_3_total_noise_model.A_weighted_total_spl']
            noise2 = data_dict_history['caddee_csdl_model.system_model.system_m3l_model.qst_2_total_noise_model.A_weighted_total_spl']
            noise1 = data_dict_history['caddee_csdl_model.system_model.system_m3l_model.qst_1_total_noise_model.A_weighted_total_spl']


            x_axis = data_dict_history['global_ind']
            cur_ind = x_axis[-1]
            # y=np.array(soc).flatten()
            y= 0.2 * np.ones((cur_ind+1, ))
            print(len(y))
            print(len(list(range(len(soc)))))
            sns.lineplot(x=list(range(len(soc))), y=np.array(soc).flatten(), ax=ax_subplot)
            sns.lineplot(x=list(range(len(soc))), y=y, ax=ax_subplot, linestyle='--' )
            ax_subplot.set_ylabel('final SoC')
            ax_subplot.set_ylim([-0.4, 0.4])
            ax_subplot.set_xlabel('iteration')
            # ax_subplot.set_title(f'Constraint at iteration {cur_ind}')

        sub_system_frame[70:75, 20:40] = ld.BaseAxesPlotter(
            required_vars = [
                'caddee_csdl_model.system_model.system_m3l_model.SOC_model.finalSoC',
                'caddee_csdl_model.system_model.system_m3l_model.qst_5_total_noise_model.A_weighted_total_spl',
                'caddee_csdl_model.system_model.system_m3l_model.qst_4_total_noise_model.A_weighted_total_spl',
                'caddee_csdl_model.system_model.system_m3l_model.qst_3_total_noise_model.A_weighted_total_spl',
                'caddee_csdl_model.system_model.system_m3l_model.qst_2_total_noise_model.A_weighted_total_spl',
                'caddee_csdl_model.system_model.system_m3l_model.qst_1_total_noise_model.A_weighted_total_spl',


            ],
            history = True,
            plot_function = plot_2D,
        )
        


        # # endregion

        # end dashboard
def get_mesh_average(spatial_rep, primitive_names):
    grid_num = 10
    transfer_para_mesh = []
    surface_names_to_indices_dict = {}
    end_index = 0
    for name in primitive_names:
        start_index = end_index
        for u in np.linspace(0,1,grid_num):
            for v in np.linspace(0,1,grid_num):
                transfer_para_mesh.append((name, np.array([u,v]).reshape((1,2))))
                end_index = end_index + 1
    locations = spatial_rep.evaluate_parametric(transfer_para_mesh).value
    mesh_average_x = np.average(locations[:,0])
    mesh_average_y = np.average(locations[:,1])
    mesh_average_z = np.average(locations[:,2])
    return mesh_average_x, mesh_average_y, mesh_average_z


if __name__ == '__main__':
    dashbuilder = TC2DB()
    dashbuilder.assemble_basedash(cache_vars=True)
    from mpi4py import MPI
    comm = MPI.COMM_WORLD
    # dashbuilder = None

    sim = Simulator(
        rep, 
        analytics=True,
        comm=comm,
        dashboard = dashbuilder
    )

    import pickle
    # with open('full_MDO_with_motor_dvs_0.pickle', 'rb') as handle:
    with open('quasi_isotropic_full_MDO_dvs_3.pickle', 'rb') as handle:
        trim_dvs = pickle.load(handle)

    for key, val in trim_dvs.items():
        # if 'cruise' in key:
        #     pass
        # elif 'climb' in key:
        #     pass
        # elif 'descent' in key:
        #     pass
        # elif 'qst_6' in key:
        #     pass
        # elif 'qst_7' in key:
        #     pass
        # elif 'qst_8' in key:
        #     pass
        # elif 'qst_9' in key:
        #     pass
        # elif 'qst_10' in key: 
        #     pass
        # elif 'oei' in key:
        #     pass
        # else:
        sim[key] = val
        print(key, val)

    # sim = Simulator(tc2_model, analytics=True)
    sim.run()
    # cruise_geometry = sim['caddee_csdl_model.design_geometry']    
    # updated_primitives_names = list(lpc_rep.spatial_representation.primitives.keys()).copy()
    # # cruise_geometry = sim['design_geometry']
    # lpc_rep.spatial_representation.update(cruise_geometry, updated_primitives_names)
    # lpc_rep.spatial_representation.plot()
    # print('\n')
    # sim.check_totals(of='system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', wrt='system_model.system_m3l_model.qst_3_pp_disk_bem_model.rpm')
    # sim.check_totals()
    # import matplotlib.pyplot as plt
    # surfaces = valid_surfaces_ml
    # oml_cp_para_mesh = []
    # grid_num = 50
    # for name in surfaces:
    #     for u in np.linspace(0,1,grid_num):
    #         for v in np.linspace(0,1,grid_num):
    #             oml_cp_para_mesh.append((name, np.array([u,v]).reshape((1,2))))

    # oml_cp_geo_mesh = lpc_rep.spatial_representation.evaluate_parametric(oml_cp_para_mesh)
    # locations = oml_cp_geo_mesh.value
    # x = locations[:,0]
    # y = locations[:,1]
    # z = locations[:,2]

    # coefficients = {}
    # for name in valid_surfaces_ml:
    #     c_name = f'caddee_csdl_model.system_model.system_m3l_model.wing_cp_indexed_function_inverse_evaluation.{name}_cp_coefficients'
    #     # c_name = 'system_model.structural_sizing.cruise_3.cruise_3.wing_cp_function_inverse_evaluation.'+name+'_wing_cp_coefficients'
    #     coefficients[name] = sim[c_name]
    
    # cp = wing_cp_index_functions['cruise'].compute(oml_cp_para_mesh, coefficients)
    # v = cp
    # # plot(x,y,z,v)
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # cmap = plt.get_cmap("coolwarm")
    # cax = ax.scatter(x, y, z, s=50, c=v, cmap=cmap)
    # ax.set_aspect('equal')
    # plt.colorbar(cax)
    # fig.set_figwidth(24)
    # fig.set_figheight(12)
    # plt.show()

    # exit()



    prob = CSDLProblem(problem_name='TC_2_problem_full_mdo', simulator=sim)

    optimizer = SNOPT(
        prob, 
        Major_iterations=600, 
        Major_optimality=5e-5, 
        Major_feasibility=1e-4,
        append2file=True,
        Iteration_limit=500000,
        Major_step_limit= 1.0,
        Linesearch_tolerance=0.9,
    )

    # optimizer = SLSQP(prob, maxiter=5000, ftol=1E-7)
    # optimizer.solve()
    # optimizer.print_results()

    optimizer.solve()
    optimizer.print_results()
    dv_dictionary = {}
    for dv_name, dv_dict in sim.dvs.items():
        print(dv_name, dv_dict['index_lower'], dv_dict['index_upper'])
        print(sim[dv_name])
        dv_dictionary[dv_name] = sim[dv_name]
    # dv_dictionary['caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tcap'] = sim['caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tcap']
    # dv_dictionary['caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tweb'] = sim['caddee_csdl_model.system_model.system_m3l_model.mass_model.wing_beam_tweb']
    print('\n')
    print('\n')
    with open('full_MDO_with_motor_dvs_3.pickle', 'wb') as handle:
        pickle.dump(dv_dictionary, handle, protocol=pickle.HIGHEST_PROTOCOL)

    c_dictionary = {}
    for c_name, c_dict in sim.cvs.items():
        print(c_name, c_dict['index_lower'], c_dict['index_upper'])
        print(sim[c_name])
        c_dict[c_name] = sim[c_name]
        with open('full_MDO_with_motor_constraints_3.pickle', 'wb') as handle:
            pickle.dump(c_dictionary, handle, protocol=pickle.HIGHEST_PROTOCOL)

    cruise_geometry = sim['caddee_csdl_model.design_geometry']    
    updated_primitives_names = list(lpc_rep.spatial_representation.primitives.keys()).copy()
    # cruise_geometry = sim['design_geometry']
    lpc_rep.spatial_representation.update(cruise_geometry, updated_primitives_names)
    lpc_rep.spatial_representation.plot()

    print('\n')


    # prob = CSDLProblem(problem_name='lpc', simulator=sim)
    # optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
    # optimizer.solve()
    # optimizer.print_results()