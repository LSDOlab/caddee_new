import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee import IMPORTS_FILES_FOLDER
import array_mapper as am
from modopt.scipy_library import SLSQP, BFGS, COBYLA
from modopt.csdl_library import CSDLProblem


caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()


# m3l sizing model
sizing_model = m3l.Model()

# Battery sizing
c172_sizing = cd.C172MassProperties()
c172_mass, c172_cg, c172_I = c172_sizing.evaluate()
sizing_model.register_output(c172_mass)
sizing_model.register_output(c172_cg)
sizing_model.register_output(c172_I)

system_model.add_m3l_model('sizing_model', sizing_model)

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# cruise condition
cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()

# cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='wing_incidence_angle', val=np.deg2rad(0), dv_flag=False)
cruise_condition.set_module_input(name='pitch_angle', val=0, dv_flag=True, lower=np.deg2rad(0), upper=np.deg2rad(5))
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='wind_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(ac_states)

# aero forces and moments
c172_aero_model = cd.C172AeroM3L()
c172_aero_model.set_module_input('delta_a', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
c172_aero_model.set_module_input('delta_r', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
c172_aero_model.set_module_input('delta_e', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
c172_forces, c172_moments = c172_aero_model.evaluate(ac_states=ac_states)
cruise_model.register_output(c172_forces)
cruise_model.register_output(c172_moments)

# prop forces and moments
c172_prop_model = cd.C172PropulsionModel()
c172_prop_model.set_module_input('propeller_radius', val=1.)
c172_prop_model.set_module_input('omega', val=2800, dv_flag=True, lower=2200, scaler=1e-3)
c172_prop_model.set_module_input('thrust_origin', val=np.array([0., 0., 0.]))
c172_prop_model.set_module_input('ref_pt', val=np.array([0., 0., 0.]))
c172_prop_forces, c172_prop_moments = c172_prop_model.evaluate(ac_states=ac_states)
cruise_model.register_output(c172_prop_forces)
cruise_model.register_output(c172_prop_forces)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=c172_cg, totoal_mass=c172_mass, ac_states=ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    c172_forces, 
    c172_moments, 
    c172_prop_forces,
    c172_prop_moments,
    inertial_forces, 
    inertial_moments
)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=c172_mass, 
    total_cg_vector=c172_cg, 
    total_inertia_tensor=c172_I, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states
)
cruise_model.register_output(trim_residual)

caddee_csdl_model = cruise_model._assemble_csdl()
caddee_csdl_model.add_objective('EulerEoMGenRefPt.trim_residual')
# # Add cruise m3l model to cruise condition
# cruise_condition.add_m3l_model('cruise_model', cruise_model)

# # Add design condition to design scenario
# design_scenario.add_design_condition(cruise_condition)

# # Add design scenario to system_model
# system_model.add_design_scenario(design_scenario=design_scenario)

# # get final caddee csdl model
# caddee_csdl_model = caddee.assemble_csdl()

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

sim.check_totals()

# exit()

prob = CSDLProblem(problem_name='c172_trim', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=100, 
    ftol=1e-15,
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