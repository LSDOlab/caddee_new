import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
import numpy as np


m3l_model = m3l.Model()


test_condition = cd.CruiseCondition(
    name='test_condition'
)

M = test_condition.create_input('mach_number', val=0.3, shape=(1, ))

# test_condition.set_module_input('pitch_angle', val=0.03369678, dv_flag=True, lower=np.deg2rad(-5), upper=np.deg2rad(5))
# test_condition.set_module_input('mach_number', val=0.17354959)
# test_condition.set_module_input('altitude', val=300)
# test_condition.set_module_input(name='range', val=20)
# test_condition.set_module_input(name='observer_location', val=np.array([0, 0, 0]))


ac_states = test_condition.evaluate_ac_states()

m3l_model.register_output(ac_states, test_condition)

csdl_model = m3l_model.assemble_csdl()

sim = Simulator(csdl_model)
sim.run()