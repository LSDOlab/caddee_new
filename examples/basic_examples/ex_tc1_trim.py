import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from python_csdl_backend import Simulator
from caddee import IMPORTS_FILES_FOLDER
import array_mapper as am


caddee = cd.CADDEE()

# caddee.system_representation = system_rep = cd.SystemRepresentation()
# caddee.system_parameterization = system_param = cd.SystemParameterization(system_representation=system_rep)

# file_name = 'LPC_test.stp'
# spatial_rep = system_rep.spatial_representation
# spatial_rep.import_file(file_name=file_name)


# # Main wing
# wing_primitive_names = list(spatial_rep.get_geometry_primitives(search_names=['Wing']))
# wing = cd.LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

# # Horizontal tail
# tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Tail_1']).keys())
# horizontal_stabilizer = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)

# # Rotor: pusher
# pusher_prop_primitive_names = list(spatial_rep.get_primitives(search_names=['Rotor-9-disk']).keys())
# pusher_prop = cd.Rotor(name='pusher_prop', spatial_representation=spatial_rep, primitive_names=pusher_prop_primitive_names)


# # add components
# system_rep.add_component(wing)

# num_spanwise_vlm = 22
# num_chordwise_vlm = 5
# leading_edge = wing.project(am.linspace(am.array([7.5, -13.5, 2.5]), am.array([7.5, 13.5, 2.5]), num_spanwise_vlm),
#                             direction=am.array([0., 0., -1.]))  # returns MappedArray
# trailing_edge = wing.project(np.linspace(np.array([13., -13.5, 2.5]), np.array([13., 13.5, 2.5]), num_spanwise_vlm),
#                              direction=np.array([0., 0., -1.]))   # returns MappedArray
# chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
# wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.5]), direction=np.array([0., 0., -1.]), grid_search_n=25)
# wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.5]), direction=np.array([0., 0., 1.]), grid_search_n=25)
# wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1
# wing_camber_surface = wing_camber_surface.reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
# system_rep.add_output(name='chord_distribution', quantity=am.norm(leading_edge-trailing_edge))

# from VAST.vlm import VLMMesh, VLMM3L
# vlm_mesh = VLMMesh(wing_camber_surface)

############################################## Geometry definition/setup ##############################################
#   Step 1) geometry import                                                                                           #
#   Step 2) creating and adding components                                                                            #
#           TODO: rethink "stock components" (shouldn't need separate rotor, motor, battery component)                #
#   Step 3) defining free form deformation blocks and actuations                                                      #
#   Step 4) defining solvers meshes                                                                                   #
#######################################################################################################################


############################################ system model definition/setup ############################################
caddee.system_model = system_model = cd.SystemModel()


# m3l sizing model
sizing_model = m3l.Model()

# Battery sizing
simple_battery_sizing = cd.SimpleBatterySizingM3L()

simple_battery_sizing.set_module_input('battery_mass', val=800)
simple_battery_sizing.set_module_input('battery_position', val=np.array([3., 0, 0.5]))
simple_battery_sizing.set_module_input('battery_energy_density', val=400)

battery_mass, cg_battery, I_battery = simple_battery_sizing.evaluate()
sizing_model.register_output(battery_mass)
sizing_model.register_output(cg_battery)
sizing_model.register_output(I_battery)


# M4 regressions
m4_regression = cd.M4RegressionsM3L()

mass_m4, cg_m4, I_m4 = m4_regression.evaluate(battery_mass=battery_mass)
sizing_model.register_output(mass_m4)
sizing_model.register_output(cg_m4)
sizing_model.register_output(I_m4)

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

sizing_model.register_output(total_mass)
sizing_model.register_output(total_cg)
sizing_model.register_output(total_inertia)

system_model.add_m3l_model('sizing_model', sizing_model)

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_trim')

# design condition
cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()

cruise_condition.set_module_input(name='altitude', val=1000)
cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='wing_incidence_angle', val=np.deg2rad(1), dv_flag=True)
cruise_condition.set_module_input(name='pitch_angle', val=0)
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='wind_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(ac_states)

# aero forces and moments
c172_aero_model = cd.C172AeroM3L()
c172_aero_model.set_module_input('delta_a', val=np.deg2rad(0))
c172_aero_model.set_module_input('delta_r', val=np.deg2rad(0))
c172_aero_model.set_module_input('delta_e', val=np.deg2rad(0))
c172_forces, c172_moments = c172_aero_model.evaluate(ac_states=ac_states)
cruise_model.register_output(c172_forces)
cruise_model.register_output(c172_moments)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L()
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(c172_forces, c172_moments, inertial_forces, inertial_moments)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass, 
    total_cg_vector=total_cg, 
    total_inertia_tensor=total_inertia, 
    total_forces=total_forces, 
    total_moments=total_moments,
    ac_states=ac_states
)
cruise_model.register_output(trim_residual)

caddee_csdl_model = cruise_model._assemble_csdl()

# # Add cruise m3l model to cruise condition
# cruise_condition.add_m3l_model('cruise_model', cruise_model)

# # Add design condition to design scenario
# design_scenario.add_design_condition(cruise_condition)

# # Add design scenario to system_model
# system_model.add_design_scenario(design_scenario=design_scenario)

# # get final caddee csdl model
# caddee_csdl_model = caddee.assemble_csdl()

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()


# csdl_test_model = test_m3l_model._assemble_csdl()
# sim = Simulator(csdl_test_model, analytics=True)
# sim.run()

exit()





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
cruise_model = m3l.Model()



# region future code
# order_u = 3
# num_control_points_u = 35
# knots_u_beginning = np.zeros((order_u-1,))
# knots_u_middle = np.linspace(0., 1., num_control_points_u+2)
# knots_u_end = np.ones((order_u-1,))
# knots_u = np.hstack((knots_u_beginning, knots_u_middle, knots_u_end))
# order_v = 1
# knots_v = np.array([0., 0.5, 1.])

# dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(order_u,1), knots=(knots_u,knots_v))
# dummy_function_space = lg.BSplineSetSpace(name='dummy_space', b_spline_spaces={'dummy_b_spline_space': dummy_b_spline_space})

# cruise_wing_pressure_coefficients = m3l.Variable(name='cruise_wing_pressure_coefficients', shape=(num_control_points_u,1,3))
# cruise_wing_pressure = m3l.Function(name='cruise_wing_pressure', function_space=dummy_function_space, coefficients=cruise_wing_pressure_coefficients)

# vlm = VLMM3L(vlm_mesh)
# vlm_forces = vlm.evaluate(displacements=None)

# bem_forces = bem.evaluate(input=None)

# total_forces = vlm_forces + bem_forces
# endregion

mass_properties = cd.TotalMPs()
total_mass, total_inertia, cg_location = mass_properties.evaluate(m4_mass)



eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(total_forces=total_forces, total_moments=None)
cruise_model.register_output(trim_residual)





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