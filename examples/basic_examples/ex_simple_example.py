import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
import numpy as np


m3l_model = m3l.Model()


test_condition = cd.CruiseCondition(
    name='test_condition'
)

M = test_condition.create_input('mach_number', val=0.3)
theta = test_condition.create_input('pitch_angle', val=0)
altitude = test_condition.create_input('altitude', val=1000)
cruise_range = test_condition.create_input('cruise_range', val=40000)
observer_location = test_condition.create_input('observer_location', val=np.array([0., 0., 0.,]))



ac_states = test_condition.evaluate(mach_number=M, pitch_angle=theta, 
                                    altitude=altitude, cruise_range=cruise_range, 
                                    observer_location=observer_location)

m3l_model.register_output(ac_states)

csdl_model = m3l_model.assemble_csdl()

sim = Simulator(csdl_model)
sim.run()