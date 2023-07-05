import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee import IMPORTS_FILES_FOLDER
from caddee import GEOMETRY_FILES_FOLDER
import array_mapper as am
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem


caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

file_name = 'lift_plus_cruise_final.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)
# spatial_rep.plot(plot_types=['mesh'])

# wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing_1']).keys())
wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
# wing.plot()

# Adding components
sys_rep.add_component(wing)

# sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()

# Import representation and the geometry from another file for brevity
from ex_tc1_geometry_setup import lpc_rep, lpc_param, wing_camber_surface, htail_camber_surface, \
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
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True, lower=0.17, upper=0.19)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(6))
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
vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=ac_states)
cruise_model.register_output(vlm_forces)
cruise_model.register_output(vlm_moments)

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
)
bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
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



# caddee_csdl_model.add_objective('system_model.aircraft_trim.hover_1.hover_1.euler_eom_gen_ref_pt.trim_residual')
caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
# sim = Simulator(caddee_csdl_model, analytics=True, display_scripts=True)
sim = Simulator(caddee_csdl_model, analytics=True)
# sim.run()

# sim.check_totals(step=1e-10)

prob = CSDLProblem(problem_name='LPC_trim', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=100, 
    ftol=1e-8,
)
optimizer.solve()
optimizer.print_results()