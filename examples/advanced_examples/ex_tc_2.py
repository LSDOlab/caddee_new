##skip
import caddee.api as cd
# from csdl_om import Simulator
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEMMesh, BEM
from VLM_package.vlm_solver_copy import VLM, VLMMesh
# from aframe.linear_beam_module import LinearBeam
from modopt.snopt_library import SNOPT
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
import time


caddee = cd.CADDEE()
caddee.system_configuration = sys_config = cd.SystemConfiguration()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_configuration=sys_config)

file_path = '/home/marius/Desktop/packages/lsdo_lab/caddee_new/caddee/core/geometry_files/UAM_configs/'
file_name = 'LPC_test.stp'

mech = sys_config.mechanical_structure
mech.import_file(file_name=file_path + file_name)
# mech.plot(plot_types=['mesh'])
# exit()

# Wing
wing_primitive_names = list(mech.get_primitives(search_names=['Wing']).keys())
wing = cd.LiftingSurface(name='wing', mechanical_structure=mech, primitive_names=wing_primitive_names)

# Horizontal tail
tail_primitive_names = list(mech.get_primitives(search_names=['Tail_1']).keys())
horizontal_stabilizer = cd.LiftingSurface(name='tail', mechanical_structure=mech, primitive_names=tail_primitive_names)  # TODO add material arguments

# Rotor: front left outer (flo)
rotor_flo_primitive_names = list(mech.get_primitives(search_names=['Rotor_1_disk']).keys())
rotor_flo = cd.Rotor(name='rotor_flo', mechanical_structure=mech, primitive_names=rotor_flo_primitive_names)

# Rotor: front right outer (fro)
rotor_fro_primitive_names = list(mech.get_primitives(search_names=['Rotor_7_disk']).keys())
rotor_fro = cd.Rotor(name='rotor_fro', mechanical_structure=mech, primitive_names=rotor_fro_primitive_names)

# Rotor: rear left outer (rlo)
rotor_rlo_primitive_names = list(mech.get_primitives(search_names=['Rotor_2_disk']).keys())
rotor_rlo = cd.Rotor(name='rotor_rlo', mechanical_structure=mech, primitive_names=rotor_rlo_primitive_names)

# Rotor: rear right outer (rro)
rotor_rro_primitive_names = list(mech.get_primitives(search_names=['Rotor_8_disk']).keys())
rotor_rro = cd.Rotor(name='rotor_rro', mechanical_structure=mech, primitive_names=rotor_rro_primitive_names)

# Rotor: pusher
pusher_prop_primitive_names = list(mech.get_primitives(search_names=['Rotor-9-disk']).keys())
pusher_prop = cd.Rotor(name='pusher_prop', mechanical_structure=mech, primitive_names=pusher_prop_primitive_names)


# Adding components
sys_config.add_component(wing)
sys_config.add_component(horizontal_stabilizer)
sys_config.add_component(rotor_flo)
sys_config.add_component(rotor_fro)
sys_config.add_component(rotor_rlo)
sys_config.add_component(rotor_rro)
sys_config.add_component(pusher_prop)

# Parameterization
# Wing FFD
# wing_geometry_primitives = wing.get_geometry_primitives()
# wing_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(wing_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
# wing_ffd_block = cd.SRBGFFDBlock(name='wing_ffd_block', primitive=wing_ffd_bspline_volume, embedded_entities=wing_geometry_primitives)
# wing_ffd_block.add_scale_v(name='linear_taper', order=2, num_dof=3, value=np.array([0., 1., 0.]), cost_factor=1.)
# wing_ffd_block.add_rotation_u(name='twist_distribution', order=4, num_dof=10, value=-1/2*np.array([0., 0.11, 0.22, 0.33, 0.44, 0.44, 0.33, 0.22, 0.11, 0.]))

# Tail FFD
horizontal_stabilizer_geometry_primitives = horizontal_stabilizer.get_geometry_primitives()
horizontal_stabilizer_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(horizontal_stabilizer_geometry_primitives, num_control_points=(11, 2, 2), order=(4,2,2), xyz_to_uvw_indices=(1,0,2))
horizontal_stabilizer_ffd_block = cd.SRBGFFDBlock(name='horizontal_stabilizer_ffd_block', primitive=horizontal_stabilizer_ffd_bspline_volume, embedded_entities=horizontal_stabilizer_geometry_primitives)
horizontal_stabilizer_ffd_block.add_scale_v(name='horizontal_stabilizer_linear_taper', order=2, num_dof=3, value=np.array([0., 0., 0.]), cost_factor=1.)
horizontal_stabilizer_ffd_block.add_rotation_u(name='horizontal_stabilizer_twist_distribution', connection_name='h_tail_act', order=1, num_dof=1, value=np.array([0]))

# ffd_set = cd.SRBGFFDSet(name='ffd_set', ffd_blocks={wing_ffd_block.name : wing_ffd_block, horizontal_stabilizer_ffd_block.name : horizontal_stabilizer_ffd_block})
ffd_set = cd.SRBGFFDSet(name='ffd_set', ffd_blocks={horizontal_stabilizer_ffd_block.name : horizontal_stabilizer_ffd_block})


# Mesh definitions
# Wing mesh
num_spanwise_vlm = 15
num_chordwise_vlm = 15
leading_edge = wing.project(np.linspace(np.array([8., -26., 7.5]), np.array([8., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), plot=False)  # returns MappedArray
trailing_edge = wing.project(np.linspace(np.array([15., -26., 7.5]), np.array([15., 26., 7.5]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), plot=False)   # returns MappedArray
chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
# mech.plot_meshes([chord_surface])
wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15, plot=False)
wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15, plot=False)
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1
# mech.plot_meshes([wing_camber_surface])
# exit()
# Tail mesh
num_spanwise_vlm = 5
num_chordwise_vlm = 3
leading_edge = horizontal_stabilizer.project(np.linspace(np.array([27., -6.5, 6.]), np.array([27., 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=15)  # returns MappedArray
trailing_edge = horizontal_stabilizer.project(np.linspace(np.array([31.5, -6.5, 6.]), np.array([31.5, 6.75, 6.]), num_spanwise_vlm), direction=np.array([0., 0., -1.]), grid_search_n=15)   # returns MappedArray
chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
horizontal_stabilizer_upper_surface_wireframe = horizontal_stabilizer.project(chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=15)
horizontal_stabilizer_lower_surface_wireframe = horizontal_stabilizer.project(chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=15)
horizontal_stabilizer_camber_surface = am.linspace(horizontal_stabilizer_upper_surface_wireframe, horizontal_stabilizer_lower_surface_wireframe, 1) # this linspace will return average when n=1
# print(horizontal_stabilizer_camber_surface)
# print(wing_camber_surface)
# exit()
# VLM mesh


vlm_mesh = VLMMesh(
    surface_names=[
        wing.parameters['name'],
        horizontal_stabilizer.parameters['name'],
    ],
    surface_shapes=[
        wing_camber_surface.evaluate().shape[1:],
        horizontal_stabilizer_camber_surface.evaluate().shape[1:],
    ],
    meshes = dict(
        wing=wing_camber_surface,
        tail=horizontal_stabilizer_camber_surface,
    ),
)

# print([
#         wing_camber_surface.evaluate().shape,
#         horizontal_stabilizer_camber_surface.evaluate().shape,
#     ])

# Rotor mesh: rear left outer (rlo)
y11 = rotor_rlo.project(np.array([19.2, -13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rotor_rlo.project(np.array([19.2, -23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rotor_rlo.project(np.array([14.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rotor_rlo.project(np.array([24.2, -18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
rlo_in_plane_y = am.subtract(y11, y12)
rlo_in_plane_x = am.subtract(y21, y22)
rlo_origin = rotor_rlo.project(np.array([19.2, -18.75, 9.01]))

# Rotor mesh: rear right outer (rro)
y11 = rotor_rro.project(np.array([19.2, 23.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rotor_rro.project(np.array([19.2, 13.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rotor_rro.project(np.array([14.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rotor_rro.project(np.array([24.2, 18.75, 9.01]), direction=np.array([0., 0., -1.]), plot=False)
rro_in_plane_y = am.subtract(y11, y12)
rro_in_plane_x = am.subtract(y21, y22)
rro_origin = rotor_rro.project(np.array([19.2, 18.75, 9.01]))

# Rotor mesh: fron left outer (flo)
y11 = rotor_flo.project(np.array([5.07, -13.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rotor_flo.project(np.array([5.07, -23.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rotor_flo.project(np.array([0.07, -18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rotor_flo.project(np.array([10.07, -18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
flo_in_plane_y = am.subtract(y11, y12)
flo_in_plane_x = am.subtract(y21, y22)
flo_origin = rotor_flo.project(np.array([5.07, -18.75, 6.73]))

# Rotor mesh: front right outer (fro)
y11 = rotor_fro.project(np.array([5.07, 23.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y12 = rotor_fro.project(np.array([5.07, 13.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y21 = rotor_fro.project(np.array([0.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
y22 = rotor_fro.project(np.array([10.07, 18.75, 6.73]), direction=np.array([0., 0., -1.]), plot=False)
fro_in_plane_y = am.subtract(y11, y12)
fro_in_plane_x = am.subtract(y21, y22)
fro_origin = rotor_fro.project(np.array([5.07, 18.75, 6.73]))

# Rotor mesh: pusher
y11 = pusher_prop.project(np.array([31.94, 0.00, 3.29]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = pusher_prop.project(np.array([31.94, 0.00, 12.29]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = pusher_prop.project(np.array([31.94, -4.50, 7.79]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = pusher_prop.project(np.array([31.94, 4.45, 7.79]), direction=np.array([-1., 0., 0.]), plot=False)
pusher_prop_in_plane_y = am.subtract(y11, y12)
pusher_prop_in_plane_x = am.subtract(y21, y22)
pusher_prop_origin = pusher_prop.project(np.array([32.625, 0, 7.79]))


rotor_rro_mesh = BEMMesh(
    meshes=dict(
        rotor_rro_in_plane_y=rro_in_plane_y,
        rotor_rro_in_plane_x=rro_in_plane_x,
        rotor_rro_origin=rro_origin,
    ),
    num_blades=3,
    num_radial=30,
    airfoil='NACA_4412',
    airfoil_polar=None,
    chord_b_spline_rep=True,
    twist_b_spline_rep=True,
)

rotor_rlo_mesh = BEMMesh(
    meshes=dict(
        rotor_rlo_in_plane_y=rlo_in_plane_y,
        rotor_rlo_in_plane_x=rlo_in_plane_x,
        rotor_rlo_origin=rlo_origin,
    ),
    num_blades=3,
    num_radial=30,
    airfoil='NACA_4412',
    airfoil_polar=None,
    chord_b_spline_rep=True,
    twist_b_spline_rep=True,
)

rotor_fro_mesh = BEMMesh(
    meshes=dict(
        rotor_fro_in_plane_y=fro_in_plane_y,
        rotor_fro_in_plane_x=fro_in_plane_x,
        rotor_fro_origin=fro_origin,
    ),
    num_blades=3,
    num_radial=30,
    airfoil='NACA_4412',
    airfoil_polar=None,
    chord_b_spline_rep=True,
    twist_b_spline_rep=True,
)

rotor_flo_mesh = BEMMesh(
    meshes=dict(
        rotor_flo_in_plane_y=flo_in_plane_y,
        rotor_flo_in_plane_x=flo_in_plane_x,
        rotor_flo_origin=flo_origin,
    ),
    num_blades=3,
    num_radial=30,
    airfoil='NACA_4412',
    airfoil_polar=None,
    chord_b_spline_rep=True,
    twist_b_spline_rep=True,
)

pusher_prop_mesh = BEMMesh(
    meshes=dict(
        pusher_prop_in_plane_y=pusher_prop_in_plane_y,
        pusher_prop_in_plane_x=pusher_prop_in_plane_x,
        pusher_prop_origin=pusher_prop_origin,
    ),
    num_blades=3,
    num_radial=30,
    airfoil='NACA_4412',
    airfoil_polar=None,
    chord_b_spline_rep=True,
    twist_b_spline_rep=True,
)

sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()

# ffd_set.setup()
affine_section_properties = ffd_set.evaluate_affine_section_properties()
rotational_section_properties = ffd_set.evaluate_rotational_section_properties()
affine_ffd_control_points_local_frame = ffd_set.evaluate_affine_block_deformations(plot=False)
ffd_control_points_local_frame = ffd_set.evaluate_rotational_block_deformations(plot=False)
ffd_control_points = ffd_set.evaluate_control_points(plot=False)
updated_geometry = ffd_set.evaluate_embedded_entities(plot=False)
updated_primitives_names = horizontal_stabilizer.primitive_names.copy()
# updated_primitives_names.extend(horizontal_stabilizer.primitive_names.copy())

# mech.update(updated_geometry, updated_primitives_names)
# mech.plot()
# exit()


motor_comp = cd.MotorComp(name='motor_1')
battery_comp = cd.BatteryComp(name='battery')
sys_config.add_component(motor_comp)
sys_config.add_component(battery_comp)

# sys_config.power_systems_architecture = psa = cd.PowerSystemsArchitecture()
# psa.connect(
#     rotor_rro, ['Q'],
#     motor_comp, ['Q'],
# )
# psa.connect(
#     motor_comp, ['power'],
#     battery_comp, ['power_profile'],
# )

system_model = cd.SystemModel()

system_model.sizing_group = sizing_group = cd.SizingGroup()

simple_battery_sizing = cd.SimpleBatterySizing(
    battery_packaging_fraction=0.15,
)
sizing_group.add_module(simple_battery_sizing)

m4_regressions = cd.M4Regressions()
sizing_group.add_module(m4_regressions)

sizing_group.connect(
    simple_battery_sizing, ['battery_pack_mass'],
    m4_regressions, ['battery_mass']
)

design_scenario = cd.DesignScenario(name='mission')
design_scenario.equations_of_motion_csdl = cd.EulerFlatEarth6DoFGenRef

# region cruise
cruise_condition = cd.AircraftCondition(
    name='cruise',
    stability_flag=False,
    dynamic_flag=False,
)
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input('mach_number', 0.17, dv_flag=True, lower=0.15)
cruise_condition.set_module_input('time', 3600)
cruise_condition.set_module_input('roll_angle', 0)
cruise_condition.set_module_input('pitch_angle', np.deg2rad(3), dv_flag=True, upper=np.deg2rad(5), lower=np.deg2rad(-5),  scaler=10)
cruise_condition.set_module_input('yaw_angle', 0)
cruise_condition.set_module_input('flight_path_angle', np.deg2rad(0))
cruise_condition.set_module_input('wind_angle', 0)
cruise_condition.set_module_input('observer_location', np.array([0, 0, 1000]))
cruise_condition.set_module_input('altitude', 1000)

bem = BEM(component=pusher_prop, mesh=pusher_prop_mesh)
bem.set_module_input('rpm', 1300, dv_flag=True, lower=400, upper=1500, scaler=1e-3)

vlm = VLM(component=wing, mesh=vlm_mesh)
# beam = LinearBeam()

cruise_condition.mechanics_group = mechanics_group = cd.MechanicsGroup()
mechanics_group.add_module(bem)
mechanics_group.add_module(vlm)
# mechanics_group.add_module(beam)


# cruise_condition.power_group = power_group = cd.PowerGroup()
# motor_model = cd.DummyMotorModel(component=motor_comp)
# battery_model = cd.DummyBatteryModel(component=battery_comp)
# power_group.add_module(motor_model)
# power_group.add_module(battery_model)

design_scenario.add_design_condition(design_condition=cruise_condition)

system_model.connect(cruise_condition, ['speed'], m4_regressions, ['cruise_speed'])
# endregion

# region hover
hover_condition = cd.AircraftCondition(
    name='hover',
    stability_flag=False,
    dynamic_flag=False,
)
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
hover_condition.set_module_input('time', 120)
hover_condition.set_module_input('speed', 1e-3)
hover_condition.set_module_input('roll_angle', 0)
hover_condition.set_module_input('pitch_angle', np.deg2rad(-1), dv_flag=False)
hover_condition.set_module_input('yaw_angle', 0)
hover_condition.set_module_input('flight_path_angle', 0)
hover_condition.set_module_input('wind_angle', 0)
hover_condition.set_module_input('observer_location', np.array([0, 0, 500]))
hover_condition.set_module_input('altitude', 500)

bem_1 = BEM(component=rotor_rro, mesh=rotor_rro_mesh)
bem_1.set_module_input('rpm', 1000, dv_flag=True, lower=500, upper=2000, scaler=1e-3)

bem_2 = BEM(component=rotor_rlo, mesh=rotor_rlo_mesh)
bem_2.set_module_input('rpm', 1000, dv_flag=True, lower=500, upper=2000, scaler=1e-3)

bem_3 = BEM(component=rotor_fro, mesh=rotor_fro_mesh)
bem_3.set_module_input('rpm', 1000, dv_flag=True, lower=500, upper=2000, scaler=1e-3)

bem_4 = BEM(component=rotor_flo, mesh=rotor_flo_mesh)
bem_4.set_module_input('rpm', 1000, dv_flag=True, lower=500, upper=2000, scaler=1e-3)

hover_condition.mechanics_group = hover_mechanics_group = cd.MechanicsGroup()
hover_mechanics_group.add_module(bem_1)
hover_mechanics_group.add_module(bem_2)
hover_mechanics_group.add_module(bem_3)
hover_mechanics_group.add_module(bem_4)

# hover_condition.power_group = power_group = cd.PowerGroup()
# battery_model = cd.DummyBatteryModel(component=battery_comp)
# power_group.add_module(battery_model)

design_scenario.add_design_condition(design_condition=hover_condition)
# endregion

# Connect battery sizing model to battery analysis
# system_model.connect(simple_battery_sizing, ['total_energy'], battery_comp, ['total_energy'])

system_model.add_design_scenario(design_scenario=design_scenario)
caddee.system_model = system_model

t1 = time.time()

caddee_csdl = caddee.assemble_csdl_modules()


h_tail_act = caddee_csdl.create_input('h_tail_act', val=np.deg2rad(0))
caddee_csdl.add_design_variable('h_tail_act', 
                                lower=np.deg2rad(-10),
                                upper=np.deg2rad(10),
                                scaler=1,
                            )

# caddee_csdl.add_objective('system_model.mission.eom.obj_r', scaler=1e-1)
caddee_csdl.add_constraint('system_model.mission.eom.obj_r', equals=0, scaler=1e-2)
# caddee_csdl.add_objective('system_model.mass_properties.m_total', scaler=1e-3)


# Before code
# import cProfile
# profiler = cProfile.Profile()

# profiler.enable()

sim = Simulator(caddee_csdl, analytics=True, display_scripts=True)

# After code
# profiler.disable()
# profiler.dump_stats('output')


t2 = time.time()

sim.run()




print(t2-t1)
# sim.check_totals()
exit()
prob = CSDLProblem(problem_name='LPC_trim', simulator=sim)
optimizer = SNOPT(
    prob, 
    Major_iterations=100, 
    Major_optimality=1e-5, 
    # Major_feasibility=1e-8,
    append2file=True,
)
# # # optimizer = SLSQP(
# # #     prob
# # # )
optimizer.solve()

# print(sim['system_model.mission.cruise.atmosphere_model.cruise_density'])
# print(sim['system_model.mission.hover.atmosphere_model.hover_density'])

# print(sim['system_model.mission.mechanics_group.rotor_1_bem.rpm'])
# print(sim['system_model.mission.mechanics_group.rotor_2_bem.rpm'])

# print(sim['system_model.mission.mechanics_group.pusher_prop_bem.F'])
# print(sim['system_model.mission.mechanics_group.wing_vlm.F'])
# print(sim['system_model.mission.mechanics_group.wing_vlm.frame_vel'])
# print(sim['system_model.mission.mechanics_group.wing_vlm.beta'])
# print(sim['system_model.mission.mechanics_group.wing_vlm.alpha'])
print('\n')
print(sim['system_model.mission.eom.Fx_total'])
print(sim['system_model.mission.eom.Fy_total'])
print(sim['system_model.mission.eom.Fz_total'])
# print(sim['system_model.mission.mechanics_group.rotor_2_bem.F'])

# wing_camber_surface_csdl = sim['system_model.mission.mesh_evaluation.wing'][0, :, :, :].reshape(wing_camber_surface.shape)
# horizontal_stabilizer_camber_surface_csdl = sim['system_model.mission.mesh_evaluation.tail'][0, :, :, :].reshape(horizontal_stabilizer_camber_surface.shape)
# print("Python and CSDL difference: wing camber surface", np.linalg.norm(wing_camber_surface_csdl - wing_camber_surface.value))
# print("Python and CSDL difference: horizontal stabilizer camber surface", np.linalg.norm(horizontal_stabilizer_camber_surface_csdl - horizontal_stabilizer_camber_surface.value))

# mech.plot_meshes([wing_camber_surface_csdl, horizontal_stabilizer_camber_surface_csdl], mesh_plot_types=['wireframe'], mesh_opacity=1.)

# print(sim['system_model.mission.cruise.cruise_u'])
# print(sim['system_model.mission.cruise.cruise_rpm_rotor_1'])
# print(sim['system_model.sizing_group.m4_regressions.cruise_speed'])

# print(sim['system_model.mission.eom.m_total'])
