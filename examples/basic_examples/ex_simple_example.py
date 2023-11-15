'''Example 5 : Minimal BEM only example'''

import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
import numpy as np
from lsdo_rotor import BEM, BEMParameters


m3l_model = m3l.Model()


bem_parameters = BEMParameters(
    num_radial=30,
    num_tangential=1,
    num_blades=2,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
)

bem_model_cruise = BEM(
    name='BEM_cruise',
    num_nodes=1,
    BEM_parameters=bem_parameters,
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
    stability_flag=False,
)


speed = m3l_model.create_input('cruise_speed', val=60)
altitude = m3l_model.create_input('cruise_altitude', val=1000)
theta = m3l_model.create_input('pitch_angle', val=np.deg2rad(5))
cruise_range = m3l_model.create_input('cruise_range', val=40000)


ac_states, atmosphere = cruise_condition.evaluate(mach_number=None, pitch_angle=theta, cruise_speed=speed,
                                    altitude=altitude, cruise_range=cruise_range, 
                                    )

m3l_model.register_output(ac_states)

cruise_rpm = m3l_model.create_input('cruise_rpm', val=2000)

cruise_bem_outputs = bem_model_cruise.evaluate(ac_states=ac_states, rpm=cruise_rpm, rotor_radius=rotor_radius, 
                thrust_origin=thrust_origin_hover, thrust_vector=thrust_vector_hover, atmosphere=atmosphere,
                blade_chord_cp=chord_cp, blade_twist_cp=twist_cp)


m3l_model.register_output(cruise_bem_outputs)


csdl_model = m3l_model.assemble_csdl()

# csdl_model.connect('BEM_cruise.BEM_cruise.F', 'stability_analysis.BEM_cruise_F')

sim = Simulator(csdl_model, analytics=True)
sim.run()

cd.print_caddee_outputs(m3l_model=m3l_model, sim=sim)

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