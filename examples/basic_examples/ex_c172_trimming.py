##skip
import caddee.api as cd
from python_csdl_backend import Simulator
import numpy as np

from caddee_execution_scripts.c172.c172_weights import C172MassProperties

caddee = cd.CADDEE()



# region System Representation

caddee.system_representation = sys_rep = cd.SystemRepresentation()

caddee.system_parameterization = sys_param = None
wing = cd.LiftingSurface(name='aircraft')
push_rotor = cd.Rotor(name='pusher_rotor')

sys_rep.add_component(wing)
sys_rep.add_component(push_rotor)
# endregion


# region System Model

system_model = cd.SystemModel()

# region Sizing Group
system_model.sizing_group = sizing_group = cd.SizingGroup()

c172_mp = C172MassProperties()
sizing_group.add_module(c172_mp)
# endregion

# region Design Scenario
design_scenario = cd.DesignScenario(name='mission')
design_scenario.equations_of_motion_csdl = cd.EulerFlatEarth6DoFGenRef

# region Hover Design Condition
hover_condition = cd.AircraftCondition(
    name='hover',
    stability_flag=False,
    dynamic_flag=False,
)
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
hover_condition.set_module_input('time', 120)
hover_condition.set_module_input('speed', 1e-3)
hover_condition.set_module_input('roll_angle', 0)
hover_condition.set_module_input('pitch_angle', np.deg2rad(0), dv_flag=False)
hover_condition.set_module_input('yaw_angle', 0)
hover_condition.set_module_input('flight_path_angle', 0)
hover_condition.set_module_input('wind_angle', 0)
hover_condition.set_module_input('altitude', 500.)
hover_condition.set_module_input('observer_location', np.array([0., 0., 500.]))

design_scenario.add_design_condition(design_condition=hover_condition)
dummy_computation = cd.MechanicsGroup()
hover_condition.mechanics_group = dummy_computation
# endregion

system_model.add_design_scenario(design_scenario=design_scenario)
# endregion

# endregion

caddee.system_model = system_model
# endregion

caddee_csdl = caddee.assemble_csdl_modules()

# caddee_csdl.add_objective('system_model.mission.eom.obj_r', scaler=1e-1)

sim = Simulator(caddee_csdl, analytics=True, display_scripts=True)
sim.run()