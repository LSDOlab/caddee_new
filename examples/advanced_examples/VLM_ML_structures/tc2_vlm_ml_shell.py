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
from caddee import PROJECTIONS_FOLDER
import pickle
import sys
sys.setrecursionlimit(100000)


caddee = cd.CADDEE()

# Import representation and the geometry from another file for brevity
from examples.advanced_examples.TC2_problem.ex_tc2_geometry_setup import lpc_rep, lpc_param, wing_camber_surface, htail_camber_surface, \
    wing_vlm_mesh_name, htail_vlm_mesh_name, wing_oml_mesh, wing_upper_surface_ml, htail_upper_surface_ml, wing_oml_mesh_ml, \
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


# region cruise condition
cruise_condition = cd.CruiseCondition(name="cruise")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.173, dv_flag=False, lower=0.17, upper=0.19)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(5))
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
system_m3l_model.register_output(vlm_force)
system_m3l_model.register_output(vlm_moment)
system_m3l_model.register_output(cl_distribution)
system_m3l_model.register_output(re_spans)

ml_pressures = PressureProfile(
    airfoil_name='NASA_langley_ga_1',
    use_inverse_cl_map=True,
)

cp_upper, cp_lower, Cd = ml_pressures.evaluate(cl_distribution, re_spans) #, mach_number, reynolds_number)
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

cp_upper_oml, cp_lower_oml = ml_pressures_oml_map.evaluate(cp_upper, cp_lower, nodal_pressure_mesh=[])
wing_oml_pressure_upper = cp_upper_oml[0]
htail_oml_pressure_upper = cp_upper_oml[1]
wing_oml_pressure_lower = cp_lower_oml[0]
htail_oml_pressure_lower = cp_lower_oml[1]

vstack = m3l.VStack()
wing_oml_pressure = vstack.evaluate(wing_oml_pressure_upper, wing_oml_pressure_lower)

system_m3l_model.register_output(wing_oml_pressure_upper, design_condition=cruise_condition)
system_m3l_model.register_output(htail_oml_pressure_upper, design_condition=cruise_condition)
system_m3l_model.register_output(wing_oml_pressure_lower, design_condition=cruise_condition)
system_m3l_model.register_output(htail_oml_pressure_lower, design_condition=cruise_condition)

# region B-spline fitting
pressure_spaces = {}
coefficients = {}


nodes_parametric = []
targets = lpc_rep.spatial_representation.get_primitives(wing.get_primitives())
num_targets = len(targets.keys())

from m3l.utils.utils import index_functions
order = 3
shape = 5
space_u = lg.BSplineSpace(name='cp_sapce',
                        order=(2, 3),
                        control_points_shape=(3, 15))
wing_cp_function = index_functions(list(wing.get_primitives().keys()), 'cp', space_u, 1)





projected_points_on_each_target = []
target_names = []
nodes = wing_oml_mesh_ml.value.reshape((200*24, 3))
# print(nodes.shape)
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
# with open(PROJECTIONS_FOLDER /  'wing_cp_projections.pcikle', 'wb') as f:
#         pickle.dump(nodes_parametric, f)
# exit()
with open(PROJECTIONS_FOLDER /  'wing_cp_projections.pcikle', 'rb') as f:
        nodes_parametric = pickle.load(f)

counter = 0
for item in nodes_parametric:
    new_tuple = (item[0], item[1].reshape(1, 2)) 
    nodes_parametric[counter] = new_tuple
    counter += 1


wing_cp_function.inverse_evaluate(nodes_parametric, wing_oml_pressure)
system_m3l_model.register_output(wing_cp_function.coefficients, design_condition=cruise_condition)

# endregion
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

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=500, upper=2000, scaler=1e-3)
bem_forces, bem_moments,_ ,_ ,_,_,_,_ = bem_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)

system_m3l_model.register_output(bem_forces, design_condition=cruise_condition)
system_m3l_model.register_output(bem_moments, design_condition=cruise_condition)

# total_mass_properties = cd.TotalMassPropertiesM3L()
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_m4, cg_battery, wing_inertia_tensor, I_m4, I_battery, design_condition=cruise_condition)

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


design_scenario.add_design_condition(cruise_condition)


system_model.add_design_scenario(design_scenario)
system_model.add_m3l_model('system_m3l_model', system_m3l_model)


caddee_csdl_model = caddee.assemble_csdl()


# region actuations
caddee_csdl_model.create_input('cruise_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('cruise_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))
caddee_csdl_model.create_input('cruise_wing_actuation', val=np.deg2rad(3.2))
# endregion

# region system level constraints and objective

# trim_3 = caddee_csdl_model.declare_variable('system_model.system_m3l_model.cruise_euler_eom_gen_ref_pt.trim_residual', shape=(1, ))

# combined_trim = caddee_csdl_model.register_output('combined_trim', trim_1 * 1)
# combined_trim = caddee_csdl_model.register_output('combined_trim', trim_1 *1 + trim_2*1 + trim_3*1 + trim_4*1 + trim_5 * 1 + trim_6 * 1 + trim_7 * 1 + trim_8 * 1 + trim_9 * 1 + trim_10*1)
# caddee_csdl_model.add_objective('combined_trim')
# endregion


# region TC 2 csdl model
import csdl
upstream_model = csdl.Model()
wing_area = upstream_model.create_input('wing_area', val=200.)
wing_taper_ratio = upstream_model.create_input('wing_taper_ratio', val=0.45)
aspect_ratio = upstream_model.create_input('wing_aspect_ratio', val=20)

wing_span = (aspect_ratio * wing_area)**0.5
wing_root_chord = 2 * wing_area/((1 + wing_taper_ratio) * wing_span)
wing_tip_chord = wing_root_chord * wing_taper_ratio

tm = upstream_model.create_input('tail_moment_arm_input', val=17.23)

tail_area = upstream_model.create_input('tail_area', val=30)
tail_taper_ratio = upstream_model.create_input('tail_taper_ratio', val=0.6)
tail_aspect_ratio = upstream_model.create_input('tail_aspect_ratio', val=5)

upstream_model.add_design_variable('wing_area', upper=300, lower=100, scaler=5e-3)
upstream_model.add_design_variable('wing_aspect_ratio', upper=16, lower=8, scaler=1e-1)
upstream_model.add_design_variable('tail_area', upper=80, lower=10, scaler=5e-2)
upstream_model.add_design_variable('tail_aspect_ratio', upper=9, lower=1, scaler=1e-1)
upstream_model.add_design_variable('tail_moment_arm_input', upper=20, lower=14, scaler=5e-2)



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



tc2_model.connect('geometry_processing_model.wing_root_chord', 'caddee_csdl_model.wing_root_chord')
tc2_model.connect('geometry_processing_model.wing_tip_chord_left', 'caddee_csdl_model.wing_tip_chord_left')
tc2_model.connect('geometry_processing_model.wing_tip_chord_right', 'caddee_csdl_model.wing_tip_chord_right')
tc2_model.connect('geometry_processing_model.wing_span', 'caddee_csdl_model.wing_span')
tc2_model.connect('geometry_processing_model.tail_moment_arm', 'caddee_csdl_model.tail_moment_arm')

tc2_model.connect('geometry_processing_model.tail_root_chord', 'caddee_csdl_model.tail_root_chord')
tc2_model.connect('geometry_processing_model.tail_tip_chord_left', 'caddee_csdl_model.tail_tip_chord_left')
tc2_model.connect('geometry_processing_model.tail_tip_chord_right', 'caddee_csdl_model.tail_tip_chord_right')
tc2_model.connect('geometry_processing_model.tail_span', 'caddee_csdl_model.tail_span')

# endregion

# run commond: mpirun -n 2 python tc2_main_script
from mpi4py import MPI
comm = MPI.COMM_WORLD
rep = csdl.GraphRepresentation(tc2_model)
sim = Simulator(
    rep, 
    analytics=True,
    comm=comm,
)

# sim = Simulator(tc2_model, analytics=True)
sim.run()
# exit()
# print('\n')
# sim.check_totals(of='system_model.system_m3l_model.qst_3_euler_eom_gen_ref_pt.trim_residual', wrt='system_model.system_m3l_model.qst_3_pp_disk_bem_model.rpm')
# sim.check_totals()

cruise_geometry = sim['caddee_csdl_model.design_geometry']    
updated_primitives_names = list(lpc_rep.spatial_representation.primitives.keys()).copy()
# cruise_geometry = sim['design_geometry']
lpc_rep.spatial_representation.update(cruise_geometry, updated_primitives_names)
lpc_rep.spatial_representation.plot()

coefficients = {}
for name in rep.unpromoted_to_promoted:
    if 'cp_coefficients' in name:
        dict_name = name.split(".")[-1].split("_cp_coefficients")[0]
        print(dict_name)
        coefficients[dict_name] = sim[f'{name}']
    # print(sim[name])
    # if 'coefficients' in name:
        # print(name)
        # coefficients[name] = sim[name]


# # After sim.run()
# coefficients = sim['someting_coefficients'].turn_into_dictionary()
grid_num = 50
transfer_para_mesh = []
# for name in wing.get_primitives():
for name in coefficients.keys():
    for u in np.linspace(0,1,grid_num):
        for v in np.linspace(0,1,grid_num):
            transfer_para_mesh.append((name, np.array([u,v]).reshape(1, 2)))
# print(transfer_para_mesh)
cp_points_1 = wing_cp_function.compute(nodes_parametric, coefficients) # at nodes
cp_points_2 = wing_cp_function.compute(transfer_para_mesh, coefficients) # at nodes

locations = lpc_rep.spatial_representation.evaluate_parametric(transfer_para_mesh).value

import matplotlib.pyplot as plt 

x = locations[:,0]
y = locations[:,1]
z = locations[:,2]
v = cp_points_2

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
cmap = plt.get_cmap("coolwarm")
cax = ax.scatter(x, y, z, s=50, c=v, cmap=cmap, vmin=-1.5, vmax=1.5)
ax.set_aspect('equal')

plt.colorbar(cax)

plt.show()
# print(cp_points)

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

prob = CSDLProblem(problem_name='TC_2_problem_full', simulator=sim)

optimizer = SNOPT(
    prob, 
    Major_iterations=2000, 
    Major_optimality=1e-5, 
    Major_feasibility=1e-5,
    append2file=True,
    Iteration_limit=500000
)

optimizer.solve()
optimizer.print_results()

for dv_name, dv_dict in sim.dvs.items():
    print(dv_name, dv_dict['index_lower'], dv_dict['index_upper'])
    print(sim[dv_name])

print('\n')
print('\n')

for c_name, c_dict in sim.cvs.items():
    print(c_name, c_dict['index_lower'], c_dict['index_upper'])
    print(sim[c_name])
# print(sim['system_model.system_m3l_model.qst_1_rlo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_rro_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_flo_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fli_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fri_disk_bem_model.thrust_vector'])
# print(sim['system_model.system_m3l_model.qst_1_fro_disk_bem_model.thrust_vector'])

cruise_geometry = sim['caddee_csdl_model.design_geometry']    
updated_primitives_names = list(lpc_rep.spatial_representation.primitives.keys()).copy()
# cruise_geometry = sim['design_geometry']
lpc_rep.spatial_representation.update(cruise_geometry, updated_primitives_names)
lpc_rep.spatial_representation.plot()

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