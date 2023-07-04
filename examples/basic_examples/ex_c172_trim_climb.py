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

# Climb condition
climb_model = m3l.Model()
climb_condition = cd.ClimbCondition(name="Climb")
climb_condition.atmosphere_model = cd.SimpleAtmosphereModel()

climb_condition.set_module_input(name='mach_number', val=0.17)
climb_condition.set_module_input(name='pitch_angle', val=np.deg2rad(5), dv_flag=True, lower=np.deg2rad(0), upper=np.deg2rad(5))
climb_condition.set_module_input(name='initial_altitude', val=0)
climb_condition.set_module_input(name='final_altitude', val=500)
climb_condition.set_module_input(name='altitude', val=500)
climb_condition.set_module_input(name='flight_path_angle', val=np.deg2rad(3.))
# climb_condition.set_module_input(name='flight_path_angle', val=np.deg2rad(2.8395444552006524))
climb_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = climb_condition.evaluate_ac_states()
climb_model.register_output(ac_states)

# Inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(
    total_cg_vector=c172_cg,
    totoal_mass=c172_mass,
    ac_states=ac_states
)
climb_model.register_output(inertial_forces)
climb_model.register_output(inertial_moments)

# aero forces and moments
c172_aero_model = cd.C172AeroM3L()
c172_aero_model.set_module_input('delta_a', val=np.deg2rad(0))
c172_aero_model.set_module_input('delta_r', val=np.deg2rad(0))
c172_aero_model.set_module_input('delta_e', val=np.deg2rad(0),
                                 dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
c172_forces, c172_moments = c172_aero_model.evaluate(ac_states=ac_states)
climb_model.register_output(c172_forces)
climb_model.register_output(c172_moments)

# prop forces and moments
c172_prop_model = cd.C172PropulsionModel()
c172_prop_model.set_module_input('propeller_radius', val=1.)
c172_prop_model.set_module_input('omega', val=2800, dv_flag=True, lower=2000, upper=2800, scaler=1e-3)
c172_prop_model.set_module_input('thrust_origin', val=np.array([0., 0., 5.]))
c172_prop_model.set_module_input('ref_pt', val=ref_pt)
c172_prop_forces, c172_prop_moments = c172_prop_model.evaluate(ac_states=ac_states)
climb_model.register_output(c172_prop_forces)
climb_model.register_output(c172_prop_forces)

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
climb_model.register_output(total_forces)
climb_model.register_output(total_moments)

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
climb_model.register_output(trim_residual)

caddee_csdl_model = climb_model._assemble_csdl()
caddee_csdl_model.add_objective('EulerEoMGenRefPt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

# exit()

sim.check_totals()

prob = CSDLProblem(problem_name='c172_climb_trim', simulator=sim)
optimizer = SLSQP(
    prob,
    maxiter=100,
    ftol=1e-15,
)
optimizer.solve()
optimizer.print_results()
print("Climb angle of attack", np.rad2deg(optimizer.outputs['x'][-1, 0]))
print("Climb elevator deflection", np.rad2deg(optimizer.outputs['x'][-1, 2]))
print("Climb pusher rotor RPM", optimizer.outputs['x'][-1, 1]*1000)