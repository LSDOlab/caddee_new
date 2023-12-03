'''Example 2 : Description of example 2'''
import numpy as np
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem


caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()

# m3l sizing model
m3l_model = m3l.Model()


# Aircraft sizing
c172_sizing = cd.C172MassProperties()
mass_properties = c172_sizing.evaluate()
m3l_model.register_output(mass_properties)

ref_pt = np.array([4.5, 0., 5.]) * 0.3024
ref_pt = m3l_model.create_input('ref_pt', val=ref_pt)


# cruise condition
cruise_condition = cd.CruiseCondition(
    name="cruise_1",
    stability_flag=True,
    num_nodes=1,
)

# cruise_speed = m3l_model.create_input('cruise_speed', val=67.9)
mach_number = m3l_model.create_input('mach_number', val=np.array([0.14]))
altitude = m3l_model.create_input('cruise_altitude', val=np.array([1500]))
pitch_angle = m3l_model.create_input('pitch_angle', val=np.array([np.deg2rad(1)]), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
range = m3l_model.create_input('cruise_range', val=np.array([40000]))

ac_states, atmosphere = cruise_condition.evaluate(
    mach_number=mach_number, 
    pitch_angle=pitch_angle, 
    altitude=altitude, 
    cruise_range=range
)
m3l_model.register_output(ac_states)

# aero forces and moments
c172_aero_model = cd.C172AeroM3L(
    name='cruise_c172_aero_regression',
    num_nodes=1,
)
aileron_deflection = m3l_model.create_input(name='delta_a', val=np.array([0]))
rudder_deflection = m3l_model.create_input(name='delta_r', val=np.array([0]))
elevator_deflection = m3l_model.create_input(name='delta_e', val=np.array([np.deg2rad(-1.32724268)]), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))

c172_aero_outputs = c172_aero_model.evaluate(
    ac_states=ac_states,
    delta_a=aileron_deflection,
    delta_r=rudder_deflection,
    delta_e=elevator_deflection,
)

m3l_model.register_output(c172_aero_outputs)


# prop forces and moments
c172_prop_model = cd.C172PropulsionModel(
    name='cruise_c172_prop_regression',
    num_nodes=1,
)


prop_radius = m3l_model.create_input('prop_radius', val=1.)
omega = m3l_model.create_input('omega', val=np.array([2109.07445251]), dv_flag=True, lower=1000, upper=2800, scaler=1e-3)
thrust_origin = m3l_model.create_input('thrust_origin', val=np.array([4.5, 0., 5.]) * 0.3024)
thrust_vector = m3l_model.create_input('thrust_vector', val=np.array([1., 0., 0.]))

c172_prop_outputs = c172_prop_model.evaluate(
    ac_states=ac_states,
    prop_radius=prop_radius,
    rpm=omega,
    thrust_origin=thrust_origin,
    thrust_vector=thrust_vector,
    ref_pt=ref_pt,
)
m3l_model.register_output(c172_prop_outputs)


trim_variables = cruise_condition.assemble_trim_residual(
    mass_properties=[mass_properties], 
    aero_propulsive_outputs=[c172_aero_outputs, c172_prop_outputs],
    ac_states=ac_states,
    ref_pt=ref_pt,
)
m3l_model.register_output(trim_variables)


caddee_csdl_model = m3l_model.assemble_csdl()
caddee_csdl_model.add_objective('cruise_1_eom_model.accelerations')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

sim.check_totals(of='cruise_1_linear_stability_model_long_accelerations_jacobian_eig.e_imag', wrt='cruise_1.pitch_angle')
sim.check_totals(of='cruise_1_linear_stability_model_long_accelerations_jacobian_eig.e_real', wrt='cruise_1.pitch_angle')
sim.check_totals(of='cruise_1_linear_stability_model_lat_accelerations_jacobian_eig.e_imag', wrt='cruise_1.pitch_angle')
sim.check_totals(of='cruise_1_linear_stability_model_lat_accelerations_jacobian_eig.e_real', wrt='cruise_1.pitch_angle')
prob = CSDLProblem(problem_name='TC2_new_caddee_test', simulator=sim)
optimizer = SLSQP(prob, maxiter=100, ftol=1E-7)
optimizer.solve()

print(sim['cruise_1_eom_model.A_long'])
print(sim['cruise_1_eom_model.A_lat'])
print(sim['delta_e'])

cd.print_caddee_outputs(m3l_model, sim)
