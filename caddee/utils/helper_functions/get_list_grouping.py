import numpy as np 

test_list = ['mission_1_cruise_2_rotor_1_rpm', 'mission_1_hover_1_rotor_1_rpm', 'mission_1_hover_1_rotor_2_rpm']
contains_list = ['rotor_1', 'rotor_2','rotor_3']


def get_list_grouping(long_list, short_list):
    parent_list = []
    for i in short_list:
        empty_list = []
        for j in long_list:
            if i in j:
                empty_list.append(j)
            else:
                pass
        parent_list.append(empty_list)
    filtered_list = [x for x in parent_list if x]
    return filtered_list

# print(get_list_grouping(test_list,contains_list))
