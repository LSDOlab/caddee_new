import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
import numpy as np


m3l_model = m3l.Model()

test_condition = cd.CruiseCondition(
    name='test_condition'
)


M = test_condition.create_input('mach_number', val=0.3)
altitude = test_condition.create_input('altitude', val=10000)
theta = test_condition.create_input('pitch_angle', val=0)
cruise_range = test_condition.create_input('cruise_range', val=40000)
observer_location = test_condition.create_input('observer_location', val=np.array([0., 0., 0.,]))


ac_states, atmosphere = test_condition.evaluate(mach_number=M, pitch_angle=theta, 
                                    altitude=altitude, cruise_range=cruise_range, 
                                    observer_location=observer_location)

m3l_model.register_output(ac_states)

csdl_model = m3l_model.assemble_csdl()

sim = Simulator(csdl_model, analytics=True)
sim.run()

print(sim['test_condition.atmosphere_model.density'])
print(sim['test_condition.atmosphere_model.speed_of_sound'])
print(sim['test_condition.atmosphere_model.dynamic_viscosity'])
print(sim['test_condition.cruise_speed'])