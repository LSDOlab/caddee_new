import pandas as pd
import m3l
import textwrap
import numpy as np


def print_caddee_outputs(m3l_model : m3l.Model, sim):
    """
    Funciton to print all (m3l) outputs a user registers inside their run script.
    The outputs is written to Pandas dataframe and organized in the following way:

    """
    mass_properties = {}

    stability_modes = ['short period', 'phugoid', 'dutch roll', 'roll', 'spiral']
    stability_measure = ['real eigenvalue', 'imaginary eigenvalue', 'natural frequency', 'damping ratio', 'time to double']
    stability_list = []
    print('\n')
    print('----------------------------CADDEE output summary----------------------------')
    print('\n')
    for output, var in m3l_model.outputs.items():
        try:
            var.value = sim[f"{var.operation.name}.{var.name}"]
            if 'stability' in var.operation.name:
                stability_list.append(var.value.flatten())
            else:
                print('Variable name:\t\t', var.name)
                print('Variable operation:\t', var.operation.name)
                print('Variable value:',  '\t' + str(var.value).replace('\n', '\n\t\t\t'))
                print('\n')
        except:
            pass

    if stability_list:
        stability_data_frame = pd.DataFrame(data=np.array(stability_list).reshape(5, 5).T, index=stability_measure, columns=stability_modes)
        print('-----------------------Linear stability analysis summary-----------------------')
        print(stability_data_frame)
        
