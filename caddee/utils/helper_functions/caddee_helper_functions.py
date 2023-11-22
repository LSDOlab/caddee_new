import m3l
from typing import List, Union
import numpy as np


def create_multiple_inputs(num_vars: int, m3l_model : m3l.Model, variable_prefix : str, value:Union[int, float, List[Union[int, float]]],  
                           shape: Union[tuple, None] = None, dv_flag: bool=False, lower : Union[int, float, np.ndarray]=None, 
                           upper: Union[int, float, np.ndarray]=None, scaler: Union[int, float, np.ndarray]=None)-> List[m3l.Variable]:
    """
    Simple helper function to create multiple m3l variables
    """
    var_list = []
    if not isinstance(value, (int, float, list)):
        raise ValueError(f"Value of type {type(value)} unacceptable")
    elif isinstance(value, list):
        if len(value) != num_vars:
            raise ValueError(f"Number of values (length of value list) not equal to 'num_vars'")
        else:
            for i in range(num_vars):
                var_list.append(
                    m3l_model.create_input(name=f'{variable_prefix}_{i}', val=value[i], shape=shape, dv_flag=dv_flag, lower=lower, upper=upper, scaler=scaler)
                )

    else:
        for i in range(num_vars):
            var_list.append(
                m3l_model.create_input(name=f'{variable_prefix}_{i}', val=value, shape=shape, dv_flag=dv_flag, lower=lower, upper=upper, scaler=scaler)
            )

    return var_list


def flatten_list(nested_list):
    flattened_list = []
    for element in nested_list:
        if isinstance(element, list):
            flattened_list.extend(flatten_list(element))
        else:
            flattened_list.append(element)
    return flattened_list