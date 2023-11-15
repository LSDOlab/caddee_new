from caddee.core.caddee_core.caddee import CADDEE
from caddee.core.caddee_core.system_model.system_model import SystemModel

from caddee.core.caddee_core.system_model.sizing_group.sizing_models.m4_regressions import M4Regressions
from caddee.core.caddee_core.system_model.design_scenario.design_scenario import DesignScenario
from caddee.core.caddee_core.system_model.design_scenario.design_condition.design_condition import (
    CruiseCondition,
    ClimbCondition,
    HoverCondition,
    AcStates,
    AtmosphericProperties,
)
from caddee.core.caddee_core.system_model.design_scenario.design_condition.atmosphere.atmosphere import Atmosphere

from caddee.utils.regression_models.c172_aerodynamics import C172AeroM3L
from caddee.utils.regression_models.c172_propulsion import C172PropulsionModel
from caddee.utils.regression_models.c172_weights import C172MassProperties
from caddee.utils.helper_functions.print_caddee_output import print_caddee_outputs


from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.equations_of_motion_csdl.eom_6dof_module import EoMEuler6DOF
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_m3l import M4RegressionsM3L
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.simple_battery_sizing_m3l import SimpleBatterySizingM3L
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.constant_mass_properties_csdl import TotalMassPropertiesM3L, TotalConstantMassM3L
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.inertial_loads_csdl import InertialLoads
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.total_forces_moments_csdl import TotalForcesMoments

from caddee.core.csdl_core.system_model_csdl.submodels.power_density_based_motor_analysis_m3l import ConstantPowerDensityMotorM3L

from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.design_condition_energy_m3l import EnergyModelM3L
from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.total_energy_m3l import TotalEnergyModelM3L
from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.final_SoC_m3l import SOCModelM3L

from caddee.utils.helper_functions.geometry_helpers import make_rotor_mesh, make_vlm_camber_mesh, make_1d_box_beam_mesh, BladeParameters
from caddee.utils.helper_functions.caddee_helper_functions import create_multiple_inputs
from caddee.utils.aircraft_models.drag_models.drag_build_up import DragBuildUpModel
