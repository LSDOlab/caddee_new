from pint import UnitRegistry
from pathlib import Path

# from caddee.system_analysis.scenario.condition.static_condition.static_conditions import CruiseCondition

ureg = UnitRegistry()
Q_ = ureg.Quantity

_REPO_ROOT_FOLDER = Path(__file__).parents[1]
_PACKAGE_ROOT_FOLDER = Path(__file__).parents[0]

RESULTS_ROOT_FOLDER = _REPO_ROOT_FOLDER / 'results'
STP_FILES_FOLDER = _REPO_ROOT_FOLDER / 'models' / 'stp'
