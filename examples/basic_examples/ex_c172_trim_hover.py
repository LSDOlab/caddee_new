import numpy as np
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem


caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()

# m3l sizing model
sizing_model = m3l.Model()

# Aircraft sizing
c172_sizing = cd.C172MassProperties()
c172_mass, c172_cg, c172_I = c172_sizing.evaluate()
sizing_model.register_output(c172_mass)
sizing_model.register_output(c172_cg)
sizing_model.register_output(c172_I)

system_model.add_m3l_model('sizing_model', sizing_model)

ref_pt = np.array([4.5, 0., 5])

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# hover condition
hover_model = m3l.Model()
hover_condition = cd.HoverCondition(name="hover")
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()

hover_condition.set_module_input(name='range', val=40000)
hover_condition.set_module_input(name='altitude', val=500)
hover_condition.set_module_input(name='hover_time', val=60)
hover_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = hover_condition.evaluate_ac_states()
hover_model.register_output(ac_states)

# Inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=c172_cg,
    totoal_mass=c172_mass,
    ac_states=ac_states
)
hover_model.register_output(inertial_forces)
hover_model.register_output(inertial_moments)

# prop forces and moments
c172_prop_model = cd.C172PropulsionModel()
c172_prop_model.set_module_input('propeller_radius', val=1.)
c172_prop_model.set_module_input('omega', val=2800, dv_flag=True, lower=2000, upper=2800, scaler=1e-3)
c172_prop_model.set_module_input('thrust_origin', val=np.array([0., 0., 5.]))
c172_prop_model.set_module_input('ref_pt', val=ref_pt)
c172_prop_forces, c172_prop_moments = c172_prop_model.evaluate(ac_states=ac_states)
hover_model.register_output(c172_prop_forces)
hover_model.register_output(c172_prop_forces)

# total forces and moments
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(
    c172_prop_forces,
    c172_prop_moments,
    # inertial_forces,
    # inertial_moments
)
hover_model.register_output(total_forces)
hover_model.register_output(total_moments)

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
hover_model.register_output(trim_residual)

caddee_csdl_model = hover_model._assemble_csdl()
caddee_csdl_model.add_objective('EulerEoMGenRefPt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

exit()

sim.check_totals()

prob = CSDLProblem(problem_name='c172_hover_trim', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=100,
    ftol=1e-15,
)
optimizer.solve()
optimizer.print_results()
print("Hover lift rotor RPM", optimizer.outputs['x'][-1, 1]*1000)