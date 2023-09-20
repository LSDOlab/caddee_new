from caddee.core.caddee_core.caddee import CADDEE
from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization, SystemRepresentation
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
from caddee.core.caddee_core.system_representation.component.component import Component, LiftingSurface, Rotor

from caddee.utils.regression_models.c172_aerodynamics import C172AeroM3L
from caddee.utils.regression_models.c172_propulsion import C172PropulsionModel
from caddee.utils.regression_models.c172_weights import C172MassProperties


from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.equations_of_motion_csdl.eom_6dof_module import EoMM3LEuler6DOF
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.M4_regressions_m3l import M4RegressionsM3L
from caddee.core.csdl_core.system_model_csdl.sizing_group_csdl.sizing_models_csdl.simple_battery_sizing_m3l import SimpleBatterySizingM3L
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.constant_mass_properties_csdl import TotalMassPropertiesM3L, TotalConstantMassM3L
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.inertial_loads_csdl import InertialLoadsM3L
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.total_forces_moments_csdl import TotalForcesMomentsM3L

from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_functions import create_cartesian_enclosure_volume
from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_block import SRBGFFDBlock
from caddee.core.caddee_core.system_parameterization.free_form_deformation.ffd_set import SRBGFFDSet

from caddee.core.csdl_core.system_model_csdl.submodels.power_density_based_motor_analysis_m3l import ConstantPowerDensityMotorM3L

from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.design_condition_energy_m3l import EnergyModelM3L
from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.total_energy_m3l import TotalEnergyModelM3L
from caddee.core.csdl_core.system_model_csdl.energy_group_csdl.final_SoC_m3l import SOCModelM3L

# from core.system_configuration.networks.networks import PowerSystemsArchitecture
# from core.system_configuration.component.component import Component, Rotor, MotorComp, BatteryComp, LiftingSurface
# from core.system_model.system_model import SystemModel
# from core.system_model.sizing_group.sizing_group import SizingGroup
# from core.system_model.sizing_group.sizing_models.m4_regressions import M4Regressions
# from core.system_model.sizing_group.sizing_models.simple_battery_sizing import SimpleBatterySizing
# from core.system_model.design_scenario.design_scenario import DesignScenario
# from core.system_model.design_scenario.design_condition.design_condition import AircraftCondition
# from core.system_model.design_scenario.design_condition.atmosphere.atmosphere import SimpleAtmosphereModel

# from utils.dummy_solvers.dummy_bem import BEMDummyModel, BEMDummyMesh, DummyBEMCSDLModules, DummyBEMModules
# from utils.dummy_solvers.dummy_motor import DummyMotorModel, DummyMotorMesh
# from utils.dummy_solvers.dummy_battery import DummyBatteryModel, DummyBatteryMesh
# from csdl_core_modules.system_model_csdl.design_scenario_csdl.design_condition_csdl.equations_of_motion_csdl.equations_of_motion_csdl import EulerFlatEarth6DoFGenRef