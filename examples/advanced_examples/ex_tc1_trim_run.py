import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee import IMPORTS_FILES_FOLDER
import array_mapper as am
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem


caddee = cd.CADDEE()

# Import representation and the geometry from another file for brevity
from examples.advanced_examples.TC2_problem.ex_tc2_geometry_setup import lpc_rep, lpc_param, wing_camber_surface, htail_camber_surface, \
    pp_disk_in_plane_x, pp_disk_in_plane_y, pp_disk_origin, pp_disk, \
    rlo_in_plane_x, rlo_in_plane_y, rlo_origin, rlo_disk, \
    rli_in_plane_x, rli_in_plane_y, rli_origin, rli_disk, \
    rri_in_plane_x, rri_in_plane_y, rri_origin, rri_disk, \
    rro_in_plane_x, rro_in_plane_y, rro_origin, rro_disk, \
    flo_in_plane_x, flo_in_plane_y, flo_origin, flo_disk, \
    fli_in_plane_x, fli_in_plane_y, fli_origin, fli_disk, \
    fri_in_plane_x, fri_in_plane_y, fri_origin, fri_disk, \
    fro_in_plane_x, fro_in_plane_y, fro_origin, fro_disk \

# set system representation and parameterization
caddee.system_representation = lpc_rep
caddee.system_parameterization = lpc_param

# system model
caddee.system_model = system_model = cd.SystemModel()

# m3l sizing model
sizing_model = m3l.Model()

# Battery sizing
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

simple_battery_sizing.set_module_input('battery_mass', val=800)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.5, 0, 0.5]))
simple_battery_sizing.set_module_input('battery_energy_density', val=400)

battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()
sizing_model.register_output(battery_mass)
sizing_model.register_output(cg_battery)
sizing_model.register_output(I_battery)


# M4 regressions
m4_regression = cd.M4RegressionsM3L()

mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)
sizing_model.register_output(mass_m4)
sizing_model.register_output(cg_m4)
sizing_model.register_output(I_m4)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

sizing_model.register_output(total_mass)
sizing_model.register_output(total_cg)
sizing_model.register_output(total_inertia)

system_model.add_m3l_model('sizing_model', sizing_model)

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# region cruise condition
cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=False, lower=0.17, upper=0.19)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0))
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='wind_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(ac_states)


vlm_model = VASTFluidSover(
    surface_names=[
        'wing',
        'h_tail',
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        (1, ) + htail_camber_surface.evaluate().shape[1:],
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.43, 0]
)

# aero forces and moments
_, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=ac_states)
cruise_model.register_output(vlm_forces)
cruise_model.register_output(vlm_moments)

# BEM prop forces and moments
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh

airfoil_polar = {
    'Cl_0': 0.25,
    'Cl_alpha': 5.1566,
    'Cd_0': 0.01,
    'Cl_stall': [-1, 1.5], 
    'Cd_stall': [0.02, 0.06],
    'alpha_Cl_stall': [-10, 15],
}

pusher_bem_mesh = BEMMesh(
    meshes=dict(
    pp_disk_in_plane_1=pp_disk_in_plane_y,
    pp_disk_in_plane_2=pp_disk_in_plane_x,
    pp_disk_origin=pp_disk_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=4,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=2000, scaler=1e-3)
bem_forces, bem_moments = bem_model.evaluate(ac_states=ac_states)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_forces, vlm_moments, bem_forces, bem_moments, inertial_forces, inertial_moments)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
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

# region hover condition
hover_model = m3l.Model()
hover_condition = cd.HoverCondition(name='hover_1')
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
hover_condition.set_module_input('altitude', val=500)
hover_condition.set_module_input('hover_time', val=90)
hover_condition.set_module_input('observer_location', val=np.array([0, 0 ,0]))

ac_states = hover_condition.evaluate_ac_states()
hover_model.register_output(ac_states)

# BEM instances

# rlo
rlo_bem_mesh = BEMMesh(
    meshes=dict(
    rlo_disk_in_plane_1=rlo_in_plane_y,
    rlo_disk_in_plane_2=rlo_in_plane_x,
    rlo_disk_origin=rlo_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
rlo_bem_model = BEM(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=rlo_bem_mesh)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments = rlo_bem_model.evaluate(ac_states=ac_states)

# rli
rli_bem_mesh = BEMMesh(
    meshes=dict(
    rli_disk_in_plane_1=rli_in_plane_y,
    rli_disk_in_plane_2=rli_in_plane_x,
    rli_disk_origin=rli_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
rli_bem_model = BEM(disk_prefix='rli_disk', blade_prefix='rlo', component=rli_disk, mesh=rli_bem_mesh)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments = rli_bem_model.evaluate(ac_states=ac_states)

# rri
rri_bem_mesh = BEMMesh(
    meshes=dict(
    rri_disk_in_plane_1=rri_in_plane_y,
    rri_disk_in_plane_2=rri_in_plane_x,
    rri_disk_origin=rri_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
rri_bem_model = BEM(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=rri_bem_mesh)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments = rri_bem_model.evaluate(ac_states=ac_states)

# rro
rro_bem_mesh = BEMMesh(
    meshes=dict(
    rri_disk_in_plane_1=rri_in_plane_y,
    rri_disk_in_plane_2=rri_in_plane_x,
    rri_disk_origin=rri_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
rro_bem_model = BEM(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=rro_bem_mesh)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments = rro_bem_model.evaluate(ac_states=ac_states)

# flo
flo_bem_mesh = BEMMesh(
    meshes=dict(
    flo_disk_in_plane_1=flo_in_plane_y,
    flo_disk_in_plane_2=flo_in_plane_x,
    flo_disk_origin=flo_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
flo_bem_model = BEM(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=flo_bem_mesh)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments = flo_bem_model.evaluate(ac_states=ac_states)

# fli
fli_bem_mesh = BEMMesh(
    meshes=dict(
    fli_disk_in_plane_1=fli_in_plane_y,
    fli_disk_in_plane_2=fli_in_plane_x,
    fli_disk_origin=fli_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
fli_bem_model = BEM(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=fli_bem_mesh)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments = fli_bem_model.evaluate(ac_states=ac_states)

# fri
fri_bem_mesh = BEMMesh(
    meshes=dict(
    fri_disk_in_plane_1=fri_in_plane_y,
    fri_disk_in_plane_2=fri_in_plane_x,
    fri_disk_origin=fri_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
fri_bem_model = BEM(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=fri_bem_mesh)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments = fri_bem_model.evaluate(ac_states=ac_states)

# fro
fro_bem_mesh = BEMMesh(
    meshes=dict(
    fro_disk_in_plane_1=fro_in_plane_y,
    fro_disk_in_plane_2=fro_in_plane_x,
    fro_disk_origin=fro_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    airfoil_polar=airfoil_polar,
    # units='ft',
)
fro_bem_model = BEM(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=fro_bem_mesh)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments = fro_bem_model.evaluate(ac_states=ac_states)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
hover_model.register_output(inertial_forces)
hover_model.register_output(inertial_moments)

# total forces and moments 
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
    inertial_moments

)
hover_model.register_output(total_forces)
hover_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states
)
hover_model.register_output(trim_residual)

# Add cruise m3l model to cruise condition
hover_condition.add_m3l_model('hover_model', hover_model)

# Add design condition to design scenario
design_scenario.add_design_condition(hover_condition)
# endregion



# Add design scenario to system_model
system_model.add_design_scenario(design_scenario=design_scenario)

# get final caddee csdl model
caddee_csdl_model = caddee.assemble_csdl()

# h_tail_act = caddee_csdl_model.create_input('h_tail_act', val=np.deg2rad(-0.5))
# caddee_csdl_model.add_design_variable('h_tail_act', 
#                                 lower=np.deg2rad(-10),
#                                 upper=np.deg2rad(10),
#                                 scaler=1,
#                             )

# wing_incidence = caddee_csdl_model.create_input('wing_incidence', val=np.deg2rad(0))
# caddee_csdl_model.add_design_variable('wing_incidence', 
#                                 lower=np.deg2rad(0),
#                                 upper=np.deg2rad(5),
#                                 scaler=1,
#                             )

# pp_blade_twist = caddee_csdl_model.create_input('pp_blade_4_twist', val=np.zeros((5, )))
# caddee_csdl_model.add_design_variable('pp_blade_4_twist', 
#                                 lower=np.deg2rad(-20),
#                                 upper=np.deg2rad(5),
#                                 scaler=1,
#                             )

# cruise_trim_res = caddee_csdl_model.declare_variable('system_model.aircraft_trim.cruise_1.cruise_1.EulerEoMGenRefPt.trim_residual', shape=(1, ))
# hover_trim_res = caddee_csdl_model.declare_variable('system_model.aircraft_trim.hover_1.hover_1.EulerEoMGenRefPt.trim_residual', shape=(1, ))

# total_trim = cruise_trim_res + hover_trim_res
# caddee_csdl_model.register_output('total_trim', total_trim)
# caddee_csdl_model.add_objective('total_trim')



caddee_csdl_model.add_objective('system_model.aircraft_trim.hover_1.hover_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True, display_scripts=True)
sim.run()

exit()
# sim.check_totals(step=1e-10)


prob = CSDLProblem(problem_name='LPC_trim', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=100, 
    ftol=1e-8,
)
optimizer.solve()
optimizer.print_results()




# csdl_test_model = test_m3l_model._assemble_csdl()
# sim = Simulator(csdl_test_model, analytics=True)
# sim.run()

exit()





# design scenario
design_scenario = cd.DesignScenario(name="aircraft_trim")

# design condition
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()

cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='wing_incidence_angle', val=np.deg2rad(1), dv_flag=True)
cruise_condition.set_module_input(name='pitch_angel', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='observer_loacation', val=np.array([0, 0, 500]))


# m3l api 
cruise_model = m3l.Model()



# region future code
# order_u = 3
# num_control_points_u = 35
# knots_u_beginning = np.zeros((order_u-1,))
# knots_u_middle = np.linspace(0., 1., num_control_points_u+2)
# knots_u_end = np.ones((order_u-1,))
# knots_u = np.hstack((knots_u_beginning, knots_u_middle, knots_u_end))
# order_v = 1
# knots_v = np.array([0., 0.5, 1.])

# dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(order_u,1), knots=(knots_u,knots_v))
# dummy_function_space = lg.BSplineSetSpace(name='dummy_space', b_spline_spaces={'dummy_b_spline_space': dummy_b_spline_space})

# cruise_wing_pressure_coefficients = m3l.Variable(name='cruise_wing_pressure_coefficients', shape=(num_control_points_u,1,3))
# cruise_wing_pressure = m3l.Function(name='cruise_wing_pressure', function_space=dummy_function_space, coefficients=cruise_wing_pressure_coefficients)

# vlm = VLMM3L(vlm_mesh)
# vlm_forces = vlm.evaluate(displacements=None)

# bem_forces = bem.evaluate(input=None)

# total_forces = vlm_forces + bem_forces
# endregion

mass_properties = cd.TotalMPs()
total_mass, total_inertia, cg_location = mass_properties.evaluate(m4_mass)



eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(total_forces=total_forces, total_moments=None)
cruise_model.register_output(trim_residual)





# ...

# add model group to design condition
cruise_condition.add_model_group(cruise_model_group)


# add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)

# get final caddee csdl model
caddee_csdl_model = caddee.assemble_csdl()

# create and run simulator
sim = Simulator(caddee_csdl_model)
sim.run()


# TODO:
# 2) Strategy for geometry/parameterization/representation m3l models
# 3) num_nodes: important!
# 4) mesh_evaluation -> How about one per (vectorized) mission segment? Seems most straight-forward 

############################################## Geometry definition/setup ##############################################
#   Step 1) geometry import                                                                                           #
#   Step 2) creating and adding components                                                                            #
#           TODO: rethink "stock components" (shouldn't need separate rotor, motor, battery component)                #
#   Step 3) defining free form deformation blocks and actuations                                                      #
#   Step 4) defining solvers meshes                                                                                   #
#######################################################################################################################