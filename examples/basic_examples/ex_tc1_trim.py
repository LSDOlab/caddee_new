import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee import IMPORTS_FILES_FOLDER


caddee = cd.CADDEE()

caddee.system_representation = system_rep = cd.SystemRepresentation()
caddee.system_parameterization = system_param = cd.SystemParameterization(system_representation=system_rep)

file_name = 'LPC_test.stp'
spatial_rep = system_rep.spatial_representation
spatial_rep.import_file(file_name=file_name)

############################################## Geometry definition/setup ##############################################
#   Step 1) geometry import                                                                                           #
#   Step 2) creating and adding components                                                                            #
#           TODO: rethink "stock components" (shouldn't need separate rotor, motor, battery component)                #
#   Step 3) defining free form deformation blocks and actuations                                                      #
#   Step 4) defining solvers meshes                                                                                   #
#######################################################################################################################


############################################ system model definition/setup ############################################
caddee.system_model = system_model = cd.SystemModel()

#   Step 1) set up sizing model:        TBD: Will this still be part of caddee or also come from m3l? 
#   Step 2) set up design scenario:     This will be part of caddee
#   Step 3) set up design condtions:    This will be part of caddee and a have an 'add_model_group' method where model group is an m3l model group
#                                       Most significant changes to api (in terms of m3l)

# Sizing group
sizing_group = m3l.ModelGroup()
system_model.add_model_group(sizing_group)

# design scenario
design_scenario = cd.DesignScenario(name="aircraft_trim")

# design condition
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()

cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='wing_incidence_angle', val=np.deg2rad(1), dv_flag=True)
cruise_condition.set_module_input(name='pitch_angel', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='observer_loacation', val=np.array([0, 0, 500]))


# m3l api 
cruise_model_group = m3l.Model()

# ...

# add model group to design condition
cruise_condition.add_model_group(cruise_model_group)


# add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)

# get final caddee csdl model
caddee_csdl_model = caddee.assemble_csdl()

# create and run simulator
sim = Simulator(caddee_csdl_model)
sim.run()