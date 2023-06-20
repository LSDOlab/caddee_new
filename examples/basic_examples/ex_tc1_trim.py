import caddee.api as cd 
import lsdo_geo as lg
import m3l


caddee = cd.CADDEE()

caddee.system_representation = system_rep = cd.SystemRepresentation()
caddee.system_parameterization = system_param = cd.SystemParameterization()

############################################## Geometry definition/setup ##############################################
#   Step 1) geometry import                                                                                           #
#   Step 2) creating and adding components                                                                            #
#   Step 3) defining free form deformation blocks and actuations                                                      #
#   Step 4) defining solvers meshes                                                                                   #
#######################################################################################################################



############################################ system model definition/setup ############################################
caddee.system_model = system_model = cd.SystemModel()

#   Step 1) set up sizing model:        TBD: Will this still be part of caddee or also come from m3l? 
#   Step 2) set up design scenario:     This will be part of caddee
#   Step 3) set up design condtions:    This will be part of caddee and a have an 'add_model_group' method where model group is an m3l model group
#                                       Most significant changes to api (in terms of m3l)


sizing_group = m3l.ModelGroup()


design_scenario = cd.DesignScenario(name="aircraft_trim")

cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()

cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(nmae='mach_number', )




