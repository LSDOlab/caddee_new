import numpy as np
import csdl
from python_csdl_backend import Simulator

import pickle
from lsdo_acoustics import Acoustics, ROOT
from lsdo_acoustics.core.models.tonal.Lowson.Lowson_model import LowsonModel
from lsdo_acoustics.core.models.tonal.KS.KvurtStalnov_model import KvurtStalnovModel

'''
This file is a validation test for the Lowson and KS tonal noise models.

The input files currently supported for Lowson are:
    - loading_Jamaluddin

The input files currently supported for KS are:
    - idealtwist_out
'''

# dummy mesh object
class DummyMesh(object):
    def __init__(self, num_radial, num_tangential):
        self.parameters = {
            'num_radial': num_radial,
            'num_tangential': num_tangential
        }

num_radial = 40
mode = 'KS'

# region inputs
if mode == 'Lowson':
    
    file_name = ROOT / 'core' / 'validation' / 'data_files' / 'loading_Jamaluddin.pkl'
    file = open(str(file_name), 'rb')
    data = pickle.load(file)
    file.close()
    # exit()

    inputs = {
        'num_blades': 2,
        'radius': 0.1524, # m
        # 'radius': 0.1904, # m
        'obs_loc': np.array([0., 0., 1.75]),
        # 'obs_loc': np.array([1.75, 0., 0.]),
        'mach': 0.0705,
        # 'mach': 0.0 + 1e-6,
        # 'obs_loc': np.array([0., 2.2755, -2.7118]),
        # 'mach': 0.045,
        'RPM': 5500.,
        # 'RPM': 4047.,
        'num_radial': 40,
        'num_tangential': len(data['time'])
    }

    model = LowsonModel

elif mode == 'KS':

    file_name = ROOT / 'core' / 'validation' / 'data_files' / 'idealtwist_out.pkl'
    file = open(str(file_name), 'rb')
    data = pickle.load(file)
    file.close()
    # exit()

    inputs = {
        'num_blades': 4,
        'radius': 0.1588, # m
        # 'obs_loc': np.array([1.91, 0., 0.]),
        'obs_loc': np.array([0., 0., 1.91]),
        'mach': 0 + 1.e-7,
        'RPM': 5500.,
        'num_radial': 40,
        'num_tangential': 1
    }
    model = KvurtStalnovModel

mesh = DummyMesh(
    num_radial=inputs['num_radial'],
    num_tangential=inputs['num_tangential']
)

# endregion 

# region assemble observer info
a = Acoustics(inputs['obs_loc'])
a.setup_directivity_plot(
    name='dir_plot',
    center_point=[0., 0., 0.,],
    radius=4.,
    num_azim=50
)

observer_data = a.assemble_observers()
# endregion
if mode == 'Lowson':
    m = model(
        component_name='verif',
        disk_prefix='rotor_disk',
        # blade_prefix='rotor_blade',
        mesh=mesh,
        observer_data=observer_data,
        num_blades=inputs['num_blades'],
        modes=[1],
        # load_harmonics=[0,1],
        debug=True,
        # use_geometry=False
    )

elif mode == 'KS':
    m = model(
        component_name='verif',
        disk_prefix='rotor_disk',
        blade_prefix='rotor_blade',
        mesh=mesh,
        observer_data=observer_data,
        num_blades=inputs['num_blades'],
        modes=[1],
        load_harmonics=[0,1],
        debug=True,
        # use_geometry=False
    )


def plot_acoustics():
    sim = Simulator(m, analytics=True)

    sim['rpm'] = inputs['RPM']
    sim['propeller_radius'] = inputs['radius']
    sim['altitude'] = 0.

    if mode == 'Lowson':
        sim['_dD'] = np.reshape(
            data['fx'],
            newshape=(1, data['fx'].shape[0], data['fx'].shape[1])
        )
        sim['_dT'] = np.reshape(
            data['fz'],
            newshape=(1, data['fz'].shape[0], data['fz'].shape[1])
        )
        sim['mach_number'] = inputs['mach']
        sim['thrust_dir'] = np.array([0., 0., 1.])
        sim['in_plane_ex'] = np.array([[1., 0., 0.,]])

    elif mode == 'KS': # all of shape (num_nodes, num_radial)
        sim['chord_profile'] = 0.03176 * np.ones((num_radial,))
        sim['lambda_i'] = np.array(data['lambda_i'])
        sim['nondim_sectional_radius'] = np.array(data['non_dim_rad'])
        rho = 1.225
        R = inputs['radius']
        A = np.pi*R**2
        omega = inputs['RPM']*2.*np.pi/60.

        sim['dTdR_real'] = rho*A*R*(omega**2)*np.array(data['dCTdr'])
        sigma = 0.03176*R*inputs['num_blades'] / (np.pi*R**2)
        dCQdr = 0.5*sigma*np.array(data['Cl'])*np.array(data['lambda_i'])*np.array(data['non_dim_rad'])
        dQdr = rho*A*R*(omega**2*R**2)*dCQdr
        sim['dDdR_real'] = dQdr / (np.array(data['non_dim_rad'])*R)

        sim['thrust_dir'] = np.array([0., 0., 1.])
        # sim['in_plane_ex'] = np.array([[1., 0., 0.,]])

    sim.run()

    return sim

'''
================================= DIRECTIVITY PLOT EXAMPLE =================================
'''

# obs_data = sim['verif_tonal_spl'] # verif is the prefix here
# import matplotlib.pyplot as plt

# x = sim['init_obs_x_loc'] # x-location
# y = sim['init_obs_y_loc'] # y-location

# r = np.sqrt(x**2 + y**2)
# theta = np.arctan2(y,x)

# fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
# ax.plot(theta, np.reshape(obs_data, theta.shape))
# # ax.set_rmax(2)
# # ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
# # ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
# ax.grid(True)

# ax.set_title("It's SHO-TIME", va='bottom')
# plt.show()