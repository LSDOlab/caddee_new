import pickle
import numpy as np
from caddee.core.caddee_core.system_configuration.spatial_material.spatial_material import SpatialMaterial
import caddee.core.primitives.bsplines.bspline_functions as bsf
from caddee.core.caddee_core.system_configuration.system_primitive.system_primitive import SystemPrimitive
from caddee.core.caddee_core.system_configuration.mechanical_structure import MechanicalStructure
import meshio
import pathlib
import scipy.io
import time
from caddee.core.caddee_core.system_configuration.utils.mesh_utils import import_mesh
import vedo
from caddee.core.csdl_core.system_configuration_csdl.heaviside import PiecewiseHeavisideModel
import csdl
from python_csdl_backend import Simulator


ls_fitting_points = np.array([[[.25],[.25]],[[-.25],[-.25]]])
ls_primitive = bsf.fit_bspline(ls_fitting_points, num_control_points=(2,2),order=(1,))

s_mat = SpatialMaterial('test', 0, level_set_primitive=ls_primitive)

x = np.linspace(0,1,10)
y = np.linspace(0,1,10)
X, Y = np.meshgrid(x, y)

# sim = Simulator(PiecewiseHeavisideModel(eps=1e-4, shape=(int(1e8),)))
# sim.run()
#print(sim['output'])

points = s_mat.evaluate_points(X.flatten(),Y.flatten())
# print(points)
