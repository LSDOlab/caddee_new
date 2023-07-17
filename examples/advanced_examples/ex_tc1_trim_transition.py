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
    wing_vlm_mesh_name, htail_vlm_mesh_name, \
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
system_m3l_model = m3l.Model()

# Battery sizing
battery_component = cd.Component(name='battery')
simple_battery_sizing = cd.SimpleBatterySizingM3L(component=battery_component)

simple_battery_sizing.set_module_input('battery_mass', val=800)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3.5, 0, 0.5]))
simple_battery_sizing.set_module_input('battery_energy_density', val=400)

battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()
system_m3l_model.register_output(battery_mass)
system_m3l_model.register_output(cg_battery)
system_m3l_model.register_output(I_battery)


# M4 regressions
m4_regression = cd.M4RegressionsM3L()

mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)
system_m3l_model.register_output(mass_m4)
system_m3l_model.register_output(cg_m4)
system_m3l_model.register_output(I_m4)

total_mass_properties = cd.TotalConstantMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

system_m3l_model.register_output(total_mass)
system_m3l_model.register_output(total_cg)
system_m3l_model.register_output(total_inertia)


# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# region transition condition
# system_m3l_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=False)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0))
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
system_m3l_model.register_output(ac_states, design_condition=cruise_condition)


vlm_model = VASTFluidSover(
    surface_names=[
        wing_vlm_mesh_name,
        htail_vlm_mesh_name,
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
forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)
system_m3l_model.register_output(vlm_forces, design_condition=cruise_condition)
system_m3l_model.register_output(vlm_moments, design_condition=cruise_condition)

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
)
bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=ac_states, design_condition=cruise_condition)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=cruise_condition)
system_m3l_model.register_output(inertial_forces, design_condition=cruise_condition)
system_m3l_model.register_output(inertial_moments, design_condition=cruise_condition)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_forces, vlm_moments, bem_forces, bem_moments, inertial_forces, inertial_moments, design_condition=cruise_condition)
system_m3l_model.register_output(total_forces, design_condition=cruise_condition)
system_m3l_model.register_output(total_moments, design_condition=cruise_condition)

# pass total forces/moments + mass properties into EoM model
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
system_m3l_model.register_output(trim_residual, design_condition=cruise_condition)

# Add cruise m3l model to cruise condition
# cruise_condition.add_m3l_model('transition_model', transition_model)

# Add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)
# endregion

# region hover condition
# system_m3l_model = m3l.Model()
hover_condition = cd.HoverCondition(name='hover_1')
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
hover_condition.set_module_input('altitude', val=500)
hover_condition.set_module_input('hover_time', val=90)
hover_condition.set_module_input('observer_location', val=np.array([0, 0 ,0]))

ac_states = hover_condition.evaluate_ac_states()
system_m3l_model.register_output(ac_states)

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
    mesh_units='ft',
)
rlo_bem_model = BEM(disk_prefix='rlo_disk', blade_prefix='rlo', component=rlo_disk, mesh=rlo_bem_mesh)
rlo_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
rlo_bem_forces, rlo_bem_moments,_ ,_ ,_ = rlo_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

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
    mesh_units='ft',
)
rli_bem_model = BEM(disk_prefix='rli_disk', blade_prefix='rlo', component=rli_disk, mesh=rli_bem_mesh)
rli_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
rli_bem_forces, rli_bem_moments,_ ,_ ,_ = rli_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

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
    mesh_units='ft',
)
rri_bem_model = BEM(disk_prefix='rri_disk', blade_prefix='rri', component=rri_disk, mesh=rri_bem_mesh)
rri_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
rri_bem_forces, rri_bem_moments,_ ,_ ,_ = rri_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

# rro
rro_bem_mesh = BEMMesh(
    meshes=dict(
    rri_disk_in_plane_1=rro_in_plane_y,
    rri_disk_in_plane_2=rro_in_plane_x,
    rri_disk_origin=rro_origin,
    ),
    airfoil='NACA_4412', 
    num_blades=2,
    num_radial=25,
    mesh_units='ft',
)
rro_bem_model = BEM(disk_prefix='rro_disk', blade_prefix='rro', component=rro_disk, mesh=rro_bem_mesh)
rro_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
rro_bem_forces, rro_bem_moments,_ ,_ ,_ = rro_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

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
    mesh_units='ft',
)
flo_bem_model = BEM(disk_prefix='flo_disk', blade_prefix='flo', component=flo_disk, mesh=flo_bem_mesh)
flo_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
flo_bem_forces, flo_bem_moments,_ ,_ ,_ = flo_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

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
    mesh_units='ft',
)
fli_bem_model = BEM(disk_prefix='fli_disk', blade_prefix='fli', component=fli_disk, mesh=fli_bem_mesh)
fli_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
fli_bem_forces, fli_bem_moments,_ ,_ ,_ = fli_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

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
    mesh_units='ft',
)
fri_bem_model = BEM(disk_prefix='fri_disk', blade_prefix='fri', component=fri_disk, mesh=fri_bem_mesh)
fri_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
fri_bem_forces, fri_bem_moments,_ ,_ ,_ = fri_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

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
    mesh_units='ft',
)
fro_bem_model = BEM(disk_prefix='fro_disk', blade_prefix='fro', component=fro_disk, mesh=fro_bem_mesh)
fro_bem_model.set_module_input('rpm', val=1350, dv_flag=False, lower=800, upper=4000, scaler=1e-3)
fro_bem_forces, fro_bem_moments,_ ,_ ,_ = fro_bem_model.evaluate(ac_states=ac_states, design_condition=hover_condition)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states, design_condition=hover_condition)
system_m3l_model.register_output(inertial_forces, hover_condition)
system_m3l_model.register_output(inertial_moments, hover_condition)

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
    inertial_moments,
    design_condition=hover_condition,
)
system_m3l_model.register_output(total_forces,hover_condition)
system_m3l_model.register_output(total_moments, hover_condition)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states,
    design_condition=hover_condition,
)
system_m3l_model.register_output(trim_residual, hover_condition)

# Add cruise m3l model to cruise condition
# hover_condition.add_m3l_model('hover_model', hover_model)

# Add design condition to design scenario
design_scenario.add_design_condition(hover_condition)
# endregion



# Add design scenario to system_model
system_model.add_design_scenario(design_scenario=design_scenario)
system_model.add_m3l_model('system_m3l_model', system_m3l_model)

# get final caddee csdl model
caddee_csdl_model = caddee.assemble_csdl()

cruise_htail_actuation = caddee_csdl_model.create_input('cruise_tail_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('cruise_tail_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))

cruise_htail_actuation = caddee_csdl_model.create_input('cruise_wing_actuation', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('cruise_wing_actuation', lower=np.deg2rad(-15), upper=np.deg2rad(15))

caddee_csdl_model.connect('cruise_tail_actuation', 'system_representation.system_configurations_model.cruise_tail_actuation.cruise_tail_actuation')
caddee_csdl_model.connect('cruise_wing_actuation', 'system_representation.system_configurations_model.cruise_wing_actuation.cruise_wing_actuation')
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



caddee_csdl_model.add_objective('system_model.system_m3l_model.cruise_1_euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True, display_scripts=True)
sim.run()

sim.check_totals(step=1e-5)


# prob = CSDLProblem(problem_name='LPC_trim', simulator=sim)
# optimizer = SLSQP(
#     prob,
#     maxiter=100, 
#     ftol=1e-8,
# )
# optimizer.solve()
# optimizer.print_results()

