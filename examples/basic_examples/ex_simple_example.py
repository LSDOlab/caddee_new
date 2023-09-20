import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
import numpy as np
from lsdo_rotor import BEM, BEMMesh
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem


bem_mesh = BEMMesh(
    num_radial=30,
    num_tangential=1,
    num_blades=2,
    use_rotor_geometry=False,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
)

m3l_model = m3l.Model()

test_condition = cd.CruiseCondition(
    name='test_condition'
)


M = test_condition.create_input('mach_number', val=0.2)
altitude = test_condition.create_input('altitude', val=5000)
theta = test_condition.create_input('pitch_angle', val=0)
cruise_range = test_condition.create_input('cruise_range', val=40000)
observer_location = test_condition.create_input('observer_location', val=np.array([0., 0., 0.,]))


ac_states, atmosphere = test_condition.evaluate(mach_number=M, pitch_angle=theta, 
                                    altitude=altitude, cruise_range=cruise_range, 
                                    observer_location=observer_location)

m3l_model.register_output(ac_states)

bem_model = BEM(
    name='single_BEM',
    mesh=bem_mesh,
    disk_prefix='disk',
    blade_prefix='blade',
)

chord_cp = bem_model.create_input('chord_cp', val=np.linspace(0.3, 0.1, 4), dv_flag=True, lower=0.01, upper=0.4)
twist_cp = bem_model.create_input('twist_cp', val=np.deg2rad(np.linspace(75, 10, 4)), dv_flag=True, lower=np.deg2rad(5), upper=np.deg2rad(85))
thrust_vector = bem_model.create_input('thrust_vector', val=np.array([1, 0, 0]))
thrust_origin = bem_model.create_input('thrust_origin', val=np.array([0, 0, 0]))
rotor_radius = bem_model.create_input('rotor_radius', val=0.8)
rpm = bem_model.create_input('rpm', val=2000)

bem_outputs = bem_model.evaluate(ac_states=ac_states, rpm=rpm, rotor_radius=rotor_radius, 
                thrust_origin=thrust_origin, thrust_vector=thrust_vector, atmosphere=atmosphere,
                blade_chord_cp=chord_cp, blade_twist_cp=twist_cp)

bem_model.add_constraint(bem_outputs.T, equals=1000, scaler=1e-3)
bem_model.add_constraint(bem_outputs.eta, equals=0.80)
bem_model.add_objective(bem_outputs.Q, scaler=1e-2)

m3l_model.register_output(bem_outputs)


csdl_model = m3l_model.assemble_csdl()

sim = Simulator(csdl_model, analytics=True)
sim.run()

print(sim['test_condition.atmosphere_model.density'])
print(sim['test_condition.atmosphere_model.speed_of_sound'])
print(sim['test_condition.atmosphere_model.dynamic_viscosity'])
print(sim['test_condition.cruise_speed'])
print(sim['single_BEM.BEM_external_inputs_model.rpm'])
print(sim['single_BEM.T'])
print(sim['single_BEM.eta'])
print(sim['single_BEM.Q'])


prob = CSDLProblem(problem_name='blade_shape_opt', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=150, 
    ftol=1e-4,
)
optimizer.solve()
optimizer.print_results()

print(sim['single_BEM.T'])
print(sim['single_BEM.eta'])
print(sim['single_BEM.Q'])


# TODO: 
# 1) convert lsdo_rotor to new api
#       - remove any lsdo_modules dependence
#       - remove atmosphere model (have the lsdo_rotor internal model be optional)
# 2) Convet hover and climb conditions to new api 
# 3) look more into helper functions 
# 4) think about vectorized design conditions 

# 5) Ask Andrew about nonlinear mapped arrays
# 6) Num nodes issue for vectorizaed design conditions

# 7) Think about default kwargs (meaning name= for operations)