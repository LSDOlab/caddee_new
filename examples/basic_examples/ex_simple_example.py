'''Example 5 : Description of example 2'''

import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
import numpy as np
from lsdo_rotor import BEM, BEMMesh
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem


m3l_model = m3l.Model()


bem_mesh = BEMMesh(
    num_radial=30,
    num_tangential=1,
    num_blades=2,
    use_rotor_geometry=False,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
)

bem_model_cruise = BEM(
    name='BEM_cruise',
    num_nodes=1,
    mesh=bem_mesh,
    disk_prefix='disk',
    blade_prefix='blade',
)

bem_model_hover = BEM(
    name='BEM_hover',
    num_nodes=1,
    mesh=bem_mesh,
    disk_prefix='disk',
    blade_prefix='blade',
)

chord_cp = m3l_model.create_input('chord_cp', val=np.linspace(0.3, 0.1, 4), dv_flag=True, lower=0.01, upper=0.4)
twist_cp = m3l_model.create_input('twist_cp', val=np.deg2rad(np.linspace(75, 10, 4)), dv_flag=True, lower=np.deg2rad(5), upper=np.deg2rad(85))
thrust_vectors_cruise = m3l_model.create_input('thrust_vector_cruise', val=np.array([1, 0, 0])[np.newaxis, :].repeat(10, axis=0), shape=(10, 3))
thrust_origins_cruise = m3l_model.create_input('thrust_origin_cruise', val=np.array([0, 0, 0])[np.newaxis, :].repeat(10, axis=0), shape=(10, 3))

thrust_vector_hover = m3l_model.create_input('thrust_vector_hover', val=np.array([1, 0, 0]))
thrust_origin_hover = m3l_model.create_input('thrust_origin_hover', val=np.array([0, 0, 0]))
rotor_radius = m3l_model.create_input('rotor_radius', val=0.8)


cruise_condition = cd.CruiseCondition(
    name='cruise_condition',
    num_nodes=1,
    stability_flag=True,
)


speed = m3l_model.create_input('cruise_speed', val=60)
altitude = m3l_model.create_input('cruise_altitude', val=1000)
theta = m3l_model.create_input('pitch_angle', val=np.deg2rad(5))
cruise_range = m3l_model.create_input('cruise_range', val=40000)
observer_location = m3l_model.create_input('cruise_observer_location', val=np.array([0., 0., 0.,]))


ac_states, atmosphere = cruise_condition.evaluate(mach_number=None, pitch_angle=theta, cruise_speed=speed,
                                    altitude=altitude, cruise_range=cruise_range, 
                                    observer_location=observer_location)

m3l_model.register_output(ac_states)

cruise_rpm = m3l_model.create_input('cruise_rpm', val=2000)

cruise_bem_outputs = bem_model_cruise.evaluate(ac_states=ac_states, rpm=cruise_rpm, rotor_radius=rotor_radius, 
                thrust_origin=thrust_origin_hover, thrust_vector=thrust_vector_hover, atmosphere=atmosphere,
                blade_chord_cp=chord_cp, blade_twist_cp=twist_cp)


m3l_model.register_output(cruise_bem_outputs)

stability_analysis = cd.LinearStabilityAnalysis(design_condition=cruise_condition)
# damping_ratios = stability_analysis.evaluate(ac_states=ac_states, cg_vector=None, vehicle_mass=None, solvers=[bem_model_cruise])
A_long, A_lat = stability_analysis.evaluate(vehicle_mass=None, aerodynamic_outputs=[cruise_bem_outputs])
m3l_model.register_output(A_long)
m3l_model.register_output(A_lat)


# hover_condition = cd.HoverCondition(name='hover_condition')

# hover_altitude = m3l_model.create_input('hover_altitude', val=1000.)
# hover_time = m3l_model.create_input('time', val=100.)
# observer_location = m3l_model.create_input('hover_observer_location', val=np.array([0., 0., 0.]))

# hover_ac_state, hover_atmosphere = hover_condition.evaluate(
#     altitude=hover_altitude, hover_time=hover_time, observer_location=observer_location
# )
# m3l_model.register_output(hover_ac_state)

# hover_rpm = m3l_model.create_input('hover_rpm', val=1500)

# hover_bem_outputs = bem_model_hover.evaluate(ac_states=hover_ac_state, rpm=hover_rpm, rotor_radius=rotor_radius, thrust_origin=thrust_origin_hover, thrust_vector=thrust_vector_hover, atmosphere=hover_atmosphere, blade_chord_cp=chord_cp, blade_twist_cp=twist_cp)
# m3l_model.register_output(hover_bem_outputs)


# m3l_model.add_constraint(cruise_bem_outputs.T, equals=1000, scaler=1e-3)
# m3l_model.add_constraint(cruise_bem_outputs.eta, equals=0.80)
# m3l_model.add_objective(cruise_bem_outputs.Q, scaler=1e-2)



csdl_model = m3l_model.assemble_csdl()

# csdl_model.connect('BEM_cruise.BEM_cruise.F', 'stability_analysis.BEM_cruise_F')

sim = Simulator(csdl_model, analytics=True)
sim.run()

print(sim['cruise_condition.atmosphere_model.density'])
print(sim['cruise_condition.atmosphere_model.speed_of_sound'])
print(sim['cruise_condition.atmosphere_model.dynamic_viscosity'])
print(sim['cruise_condition.cruise_speed'])

print('\n')

print(sim['BEM_cruise.BEM_cruise.BEM_external_inputs_model.rpm'])
print(sim['BEM_cruise.BEM_cruise.T'])
print(sim['BEM_cruise.BEM_cruise.eta'])
print(sim['BEM_cruise.BEM_cruise.Q'])

print('\n')

# print(sim['BEM_cruise.inflow_velocity'])
# print(sim['BEM_cruise.BEM_external_inputs_model.thrust_vector'])
# print(sim['BEM_cruise.BEM_external_inputs_model.velocity_vector'])
# print(sim['BEM_cruise.F'])
# print(sim['BEM_cruise.u'])
print(sim['BEM_cruise.BEM_cruise.u'])
print(sim['BEM_cruise.BEM_cruise.v'])
print(sim['BEM_cruise.BEM_cruise.w'])
print(sim['BEM_cruise.BEM_cruise.M'])
print(sim['BEM_cruise.BEM_cruise.F'])
print(sim['BEM_cruise.BEM_cruise.F_perturbed'])
print('\n')
A_long = sim['stability_analysis.A_long']

w, v = np.linalg.eig(A_long)
print(w)
# print(sim['stability_analysis.u_plus'])

# print(sim['BEM_hover.BEM_external_inputs_model.rpm'])
# print(sim['BEM_hover.T'])
# print(sim['BEM_hover.eta'])
# print(sim['BEM_hover.Q'])

exit()

prob = CSDLProblem(problem_name='blade_shape_opt', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=150, 
    ftol=1e-5,
)
optimizer.solve()
optimizer.print_results()

print(sim['BEM_cruise.T'])
print(sim['BEM_cruise.eta'])
print(sim['BEM_cruise.Q'])


print(sim['BEM_hover.T'])
print(sim['BEM_hover.FOM'])
print(sim['BEM_hover.Q'])

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


# TODO: 
# Write docs
# Strategy for stability analysis:
#   1) Produce perturbed ac_states
#       [   u,   v,   w,   p,   q,   r,  theta,   phi]

#       [u + du,   0,     0,      0,   0,   0,    0,      0]
#       [   0,   v + dv,  0,      0,   0,   0,    0,      0]
#       [   0,     0,   w + dw,   0,   0,   0,    0,      0]
#       The question is how does the api for this look 
#       Ideas:
#           - Design condition has a subclass LinearStabilityAnalysis
#           - cruise_condition.linear_stability_analysis.evaluate(ac_states=ac_states, cg_vector=cg_vector, vehicle_mass=mass, solvers=[bem, vlm])
#           - NOTE: At this point, if executed properly, the design condition should have all the information about which solvers are used so the 'solvers=[]' argument shouldn't be necessary


# dynamic_sisr_dev_2
# m3l advanced examples

# Stability analysis: main challenge is how to re-use the same models 