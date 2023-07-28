# region Imports
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem

# Geometry
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
import array_mapper as am
import lsdo_geo as lg

# Solvers
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from caddee.utils.aircraft_models.pav.pav_weight import PavMassProperties
from aframe.core.mass import Mass, MassMesh
from VAST.core.generate_mappings_m3l import VASTNodalForces
import aframe.core.beam_module as ebbeam

from caddee import GEOMETRY_FILES_FOLDER

import numpy as np
import pandas as pd
# endregion


ft2m = 0.3048

def setup_geometry(include_wing_flag=False, num_wing_spanwise_vlm = 21, num_wing_chordwise_vlm = 5,
                   include_tail_flag=False, num_htail_vlm = 21, num_chordwise_vlm = 5,
                   include_tail_actuation_flag=False,
                   include_wing_beam_flag=False, num_wing_beam_nodes=21,
                   debug_geom_flag = False, visualize_flag = False):
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()
    caddee.system_representation = sys_rep = cd.SystemRepresentation()
    caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

    # region Geometry
    file_name = 'pav.stp'

    spatial_rep = sys_rep.spatial_representation
    spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
    spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

    # region Lifting surfaces
    if include_wing_flag:
        # Wing
        wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
        wing = LiftingSurface(name='Wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
        if debug_geom_flag:
            wing.plot()
        sys_rep.add_component(wing)

    # Horizontal tail
    if include_tail_flag:
        tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Stabilizer']).keys())
        htail = cd.LiftingSurface(name='HTail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
        if debug_geom_flag:
            htail.plot()
        sys_rep.add_component(htail)
    # endregion

    # endregion

    # region Meshes

    # region Wing
    if include_wing_flag:
        point00 = np.array([8.796, 14.000, 1.989])  # * ft2m # Right tip leading edge
        point01 = np.array([11.300, 14.000, 1.989])  # * ft2m # Right tip trailing edge
        point10 = np.array([8.800, 0.000, 1.989])  # * ft2m # Center Leading Edge
        point11 = np.array([15.170, 0.000, 1.989])  # * ft2m # Center Trailing edge
        point20 = np.array([8.796, -14.000, 1.989])  # * ft2m # Left tip leading edge
        point21 = np.array([11.300, -14.000, 1.989])  # * ft2m # Left tip

        leading_edge_points = np.concatenate(
            (np.linspace(point00, point10, int(num_wing_spanwise_vlm / 2 + 1))[0:-1, :],
             np.linspace(point10, point20, int(num_wing_spanwise_vlm / 2 + 1))),
            axis=0)
        trailing_edge_points = np.concatenate(
            (np.linspace(point01, point11, int(num_wing_spanwise_vlm / 2 + 1))[0:-1, :],
             np.linspace(point11, point21, int(num_wing_spanwise_vlm / 2 + 1))),
            axis=0)

        wing_leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
        wing_trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

        # Chord Surface
        wing_chord_surface = am.linspace(wing_leading_edge, wing_trailing_edge, num_wing_chordwise_vlm)
        if debug_geom_flag:
            spatial_rep.plot_meshes([wing_chord_surface])

        # Upper and lower surface
        wing_upper_surface_wireframe = wing.project(wing_chord_surface.value + np.array([0., 0., 0.5]),
                                                    direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                    plot=debug_geom_flag, max_iterations=200)
        wing_lower_surface_wireframe = wing.project(wing_chord_surface.value - np.array([0., 0., 0.5]),
                                                    direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                    plot=debug_geom_flag, max_iterations=200)

        # Chamber surface
        wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
        wing_vlm_mesh_name = 'wing_vlm_mesh'
        sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
        if debug_geom_flag:
            spatial_rep.plot_meshes([wing_camber_surface])

        # OML mesh
        wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
        wing_oml_mesh_name = 'wing_oml_mesh'
        sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
        if debug_geom_flag:
            spatial_rep.plot_meshes([wing_oml_mesh])
    # endregion

    # region Tail
    if include_tail_flag:

        point00 = np.array([20.713 - 4., 8.474 + 1.5, 0.825 + 1.5])  # * ft2m # Right tip leading edge
        point01 = np.array([22.916, 8.474, 0.825])  # * ft2m # Right tip trailing edge
        point10 = np.array([18.085, 0.000, 0.825])  # * ft2m # Center Leading Edge
        point11 = np.array([23.232, 0.000, 0.825])  # * ft2m # Center Trailing edge
        point20 = np.array([20.713 - 4., -8.474 - 1.5, 0.825 + 1.5])  # * ft2m # Left tip leading edge
        point21 = np.array([22.916, -8.474, 0.825])  # * ft2m # Left tip trailing edge

        leading_edge_points = np.linspace(point00, point20, num_htail_vlm)
        trailing_edge_points = np.linspace(point01, point21, num_htail_vlm)

        leading_edge = htail.project(leading_edge_points, direction=np.array([0., 0., -1.]), plot=debug_geom_flag)
        trailing_edge = htail.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

        # Chord Surface
        htail_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
        if debug_geom_flag:
            spatial_rep.plot_meshes([htail_chord_surface])

        # Upper and lower surface
        htail_upper_surface_wireframe = htail.project(htail_chord_surface.value + np.array([0., 0., 0.5]),
                                                      direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                      plot=debug_geom_flag, max_iterations=200)
        htail_lower_surface_wireframe = htail.project(htail_chord_surface.value - np.array([0., 0., 0.5]),
                                                      direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                      plot=debug_geom_flag, max_iterations=200)

        # Chamber surface
        htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
        htail_vlm_mesh_name = 'htail_vlm_mesh'
        sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
        if debug_geom_flag:
            spatial_rep.plot_meshes([htail_camber_surface])

        # OML mesh
        htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
        htail_oml_mesh_name = 'htail_oml_mesh'
        sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
        if debug_geom_flag:
            spatial_rep.plot_meshes([htail_oml_mesh])
        # endregion
    # endregion

    # region Wing Beam Mesh
    if include_wing_flag and include_wing_beam_flag:

        wing_beam = am.linear_combination(wing_leading_edge, wing_trailing_edge, 1,
                                          start_weights=np.ones((num_wing_beam_nodes,)) * 0.75,
                                          stop_weights=np.ones((num_wing_beam_nodes,)) * 0.25)
        width = am.norm((wing_leading_edge - wing_trailing_edge) * 0.5)

        if debug_geom_flag:
            spatial_rep.plot_meshes([wing_beam])

        offset = np.array([0, 0, 0.5])
        top = wing.project(wing_beam.value + offset, direction=np.array([0., 0., -1.]), plot=debug_geom_flag)
        bot = wing.project(wing_beam.value - offset, direction=np.array([0., 0., 1.]), plot=debug_geom_flag)
        height = am.norm((top.reshape((-1,3)) - bot.reshape((-1,3))) * 1)

        sys_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
        sys_rep.add_output(name='wing_beam_width', quantity=width)
        sys_rep.add_output(name='wing_beam_height', quantity=height)

        # pass the beam meshes to aframe:
        beam_mesh = ebbeam.LinearBeamMesh(
            meshes=dict(
                wing_beam=wing_beam,
                wing_beam_width=width,
                wing_beam_height=height, ))

        # pass the beam meshes to the aframe mass model:
        beam_mass_mesh = MassMesh(
            meshes=dict(
                wing_beam=wing_beam,
                wing_beam_width=width,
                wing_beam_height=height, ))
    # endregion

    if visualize_flag:
        if include_wing_flag and not include_tail_flag and \
                not include_tail_actuation_flag and not include_wing_beam_flag:  # Wing-Only VLM Analysis
            spatial_rep.plot_meshes([wing_camber_surface])
        elif include_tail_flag and not include_wing_flag:
            raise NotImplementedError
        elif include_wing_flag and include_tail_flag and include_tail_actuation_flag \
                 and not include_wing_beam_flag:  # Trimming
            spatial_rep.plot_meshes([wing_camber_surface, htail_camber_surface])
        elif include_wing_flag and not include_tail_flag and not include_tail_actuation_flag \
                and include_wing_beam_flag:  # Wing-only structural analysis
            spatial_rep.plot_meshes([wing_camber_surface, wing_beam])
        else:
            raise NotImplementedError
    # endregion

    # region Actuations
    if include_tail_flag and include_tail_actuation_flag:
        # Tail FFD
        htail_geometry_primitives = htail.get_geometry_primitives()
        htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
            htail_geometry_primitives,
            num_control_points=(11, 2, 2), order=(4, 2, 2),
            xyz_to_uvw_indices=(1, 0, 2)
        )
        htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block',
                                          primitive=htail_ffd_bspline_volume,
                                          embedded_entities=htail_geometry_primitives)
        htail_ffd_block.add_scale_v(name='htail_linear_taper',
                                    order=2, num_dof=3, value=np.array([0., 0., 0.]),
                                    cost_factor=1.)
        htail_ffd_block.add_rotation_u(name='htail_twist_distribution',
                                       connection_name='h_tail_act', order=1,
                                       num_dof=1, value=np.array([np.deg2rad(1.75)]))
        ffd_set = cd.SRBGFFDSet(
            name='ffd_set',
            ffd_blocks={htail_ffd_block.name: htail_ffd_block}
        )
        sys_param.add_geometry_parameterization(ffd_set)
    sys_param.setup()
    # endregion

    if include_wing_flag and not include_tail_flag and \
            not include_tail_actuation_flag and not include_wing_beam_flag:  # Wing-Only VLM Analysis
        return caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface
    elif include_tail_flag and not include_wing_flag:
        raise NotImplementedError
    elif include_wing_flag and include_tail_flag and include_tail_actuation_flag \
            and not include_wing_beam_flag:  # Trimming
        return caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface, \
            htail_vlm_mesh_name, htail_camber_surface
    elif include_wing_flag and not include_tail_flag and not include_tail_actuation_flag \
            and include_wing_beam_flag:  # Wing-only structural analysis
        return caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface, wing_oml_mesh, \
            wing, beam_mesh, beam_mass_mesh
    elif include_wing_flag and include_tail_flag and \
            not include_tail_actuation_flag and not include_wing_beam_flag:  # Wing-Tail VLM Analysis without actuation
        return caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface, \
            htail_vlm_mesh_name, htail_camber_surface
    else:
        raise NotImplementedError


def vlm_as_ll(debug_geom_flag = False, visualize_flag = False):
    """
    Script that tests if the VLM when defaulted to a lifting line returns CL=0 at 0 deg pitch angle
    """
    # region Geometry and meshes
    caddee, system_model, sys_rep, sys_param, \
        wing_vlm_mesh_name, wing_camber_surface = setup_geometry(
        include_wing_flag=True,
        include_tail_flag=False,
        visualize_flag=visualize_flag,
        debug_geom_flag=debug_geom_flag,
        num_wing_spanwise_vlm=21,
        num_wing_chordwise_vlm=2
    )
    # endregion

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Cruise condition
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
    cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
    cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

    cruise_ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(cruise_ac_states)

    # region Aerodynamics
    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[0.]
    )
    vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(vlm_forces)
    cruise_model.register_output(vlm_moments)
    # endregion

    # Add cruise m3l model to cruise condition
    cruise_condition.add_m3l_model('cruise_model', cruise_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(cruise_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()

    CL = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
    print('CL when VLM is made LL: ', CL)
    return

def tuning_cl0(cl0_expected=0.55,
               debug_geom_flag = False, visualize_flag = False):
    """
    Fixing the number of chordwise VLM panels
    We know from airfoil tools what the expected cl0 expected should be
    Find the correction term that feeds into VLM
    """

    reynolds_number = 5224802

    resolution = 50
    cl0_range = np.linspace(0.0, 0.8, num=resolution)
    CL = np.empty(resolution)
    CD = np.empty(resolution)

    for idx, cl0 in enumerate(cl0_range):
        # region Geometry and meshes
        caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface = setup_geometry(
            include_wing_flag=True,
            include_tail_flag=False,
            visualize_flag=visualize_flag,
            debug_geom_flag=debug_geom_flag,
            num_wing_spanwise_vlm=21,
            num_wing_chordwise_vlm=5
        )
        # endregion

        # region Mission

        design_scenario = cd.DesignScenario(name='aircraft_trim')

        # region Cruise condition
        cruise_model = m3l.Model()
        cruise_condition = cd.CruiseCondition(name="cruise_1")
        cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
        cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
        cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
        cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
        cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0))
        cruise_condition.set_module_input(name='flight_path_angle', val=0)
        cruise_condition.set_module_input(name='roll_angle', val=0)
        cruise_condition.set_module_input(name='yaw_angle', val=0)
        cruise_condition.set_module_input(name='wind_angle', val=0)
        cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

        cruise_ac_states = cruise_condition.evaluate_ac_states()
        cruise_model.register_output(cruise_ac_states)

        # region Aerodynamics
        vlm_model = VASTFluidSover(
            surface_names=[
                wing_vlm_mesh_name,
            ],
            surface_shapes=[
                (1,) + wing_camber_surface.evaluate().shape[1:],
            ],
            fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
            mesh_unit='ft',
            cl0=[cl0]
        )
        vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
        cruise_model.register_output(vlm_forces)
        cruise_model.register_output(vlm_moments)
        # endregion

        # Add cruise m3l model to cruise condition
        cruise_condition.add_m3l_model('cruise_model', cruise_model)

        # Add design condition to design scenario
        design_scenario.add_design_condition(cruise_condition)
        # endregion

        system_model.add_design_scenario(design_scenario=design_scenario)
        # endregion

        caddee_csdl_model = caddee.assemble_csdl()

        # Create and run simulator
        sim = Simulator(caddee_csdl_model, analytics=True)
        sim.run()

        CL[idx] = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
        CD[idx] = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CD']

    # creating the dataframe
    df = pd.DataFrame(data=np.transpose(np.vstack((cl0_range, CL, CD))),
                      columns=['cl0', 'CL', 'CD'])
    df.to_excel("InfuenceOfCl0.xlsx")
    print(df)

    def find_cl0(cl0_test):
        return (cl0_expected - np.interp(cl0_test, cl0_range, CL))**2

    from scipy.optimize import minimize
    res = minimize(find_cl0, 0.35, method='Nelder-Mead', tol=1e-6)
    print(res)

    return res.x


def vlm_evaluation_wing_only_aoa_sweep(wing_cl0=0.3475,
                                       debug_geom_flag = False, visualize_flag = False):
    resolution = 21
    pitch_angle_range = np.deg2rad(np.linspace(-10, 10, num=resolution))
    CL = np.empty(resolution)
    CD = np.empty(resolution)

    for idx, pitch_angle in enumerate(pitch_angle_range):
        # region Geometry and meshes
        caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface = setup_geometry(
            include_wing_flag=True,
            include_tail_flag=False,
            visualize_flag=visualize_flag,
            debug_geom_flag=debug_geom_flag,
        )
        # endregion

        # region Mission

        design_scenario = cd.DesignScenario(name='aircraft_trim')

        # region Cruise condition
        cruise_model = m3l.Model()
        cruise_condition = cd.CruiseCondition(name="cruise_1")
        cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
        cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
        cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
        cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
        cruise_condition.set_module_input(name='pitch_angle', val=pitch_angle)
        cruise_condition.set_module_input(name='flight_path_angle', val=0)
        cruise_condition.set_module_input(name='roll_angle', val=0)
        cruise_condition.set_module_input(name='yaw_angle', val=0)
        cruise_condition.set_module_input(name='wind_angle', val=0)
        cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

        cruise_ac_states = cruise_condition.evaluate_ac_states()
        cruise_model.register_output(cruise_ac_states)

        # region Aerodynamics
        vlm_model = VASTFluidSover(
            surface_names=[
                wing_vlm_mesh_name,
            ],
            surface_shapes=[
                (1,) + wing_camber_surface.evaluate().shape[1:],
            ],
            fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
            mesh_unit='ft',
            cl0=[wing_cl0]
        )
        vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
        cruise_model.register_output(vlm_forces)
        cruise_model.register_output(vlm_moments)
        # endregion

        # Add cruise m3l model to cruise condition
        cruise_condition.add_m3l_model('cruise_model', cruise_model)

        # Add design condition to design scenario
        design_scenario.add_design_condition(cruise_condition)
        # endregion

        system_model.add_design_scenario(design_scenario=design_scenario)
        # endregion

        caddee_csdl_model = caddee.assemble_csdl()

        # Create and run simulator
        sim = Simulator(caddee_csdl_model, analytics=True)
        sim.run()

        CL[idx] = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
        CD[idx] = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CD']

    # creating the dataframe
    df = pd.DataFrame(data=np.transpose(np.vstack((np.rad2deg(pitch_angle_range), CL, CD))),
                      columns=['Pitch Angle', 'CL', 'CD'])
    df.to_excel("WingOnly_InfluenceOfAoA.xlsx")
    print(df)
    return


def vlm_evaluation_wing_tail_aoa_sweep(wing_cl0=0.3475,
                                       debug_geom_flag = False, visualize_flag = False):
    resolution = 21
    pitch_angle_range = np.deg2rad(np.linspace(-10, 10, num=resolution))
    CL = np.empty(resolution)
    CD = np.empty(resolution)

    for idx, pitch_angle in enumerate(pitch_angle_range):
        # region Geometry and meshes
        caddee, system_model, sys_rep, sys_param, \
            wing_vlm_mesh_name, wing_camber_surface, \
            htail_vlm_mesh_name, htail_camber_surface = setup_geometry(
            include_wing_flag=True,
            include_tail_flag=True,
            visualize_flag=visualize_flag,
            debug_geom_flag=debug_geom_flag,
        )
        # endregion

        # region Mission

        design_scenario = cd.DesignScenario(name='aircraft_trim')

        # region Cruise condition
        cruise_model = m3l.Model()
        cruise_condition = cd.CruiseCondition(name="cruise_1")
        cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
        cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
        cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
        cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
        cruise_condition.set_module_input(name='pitch_angle', val=pitch_angle)
        cruise_condition.set_module_input(name='flight_path_angle', val=0)
        cruise_condition.set_module_input(name='roll_angle', val=0)
        cruise_condition.set_module_input(name='yaw_angle', val=0)
        cruise_condition.set_module_input(name='wind_angle', val=0)
        cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

        cruise_ac_states = cruise_condition.evaluate_ac_states()
        cruise_model.register_output(cruise_ac_states)

        # region Aerodynamics
        vlm_model = VASTFluidSover(
            surface_names=[
                wing_vlm_mesh_name,
                htail_vlm_mesh_name
            ],
            surface_shapes=[
                (1,) + wing_camber_surface.evaluate().shape[1:],
                (1,) + htail_camber_surface.evaluate().shape[1:],
            ],
            fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
            mesh_unit='ft',
            cl0=[wing_cl0, 0.]
        )
        vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
        cruise_model.register_output(vlm_forces)
        cruise_model.register_output(vlm_moments)
        # endregion

        # Add cruise m3l model to cruise condition
        cruise_condition.add_m3l_model('cruise_model', cruise_model)

        # Add design condition to design scenario
        design_scenario.add_design_condition(cruise_condition)
        # endregion

        system_model.add_design_scenario(design_scenario=design_scenario)
        # endregion

        caddee_csdl_model = caddee.assemble_csdl()

        # Create and run simulator
        sim = Simulator(caddee_csdl_model, analytics=True)
        sim.run()

        CL[idx] = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
        CD[idx] = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CD']

    # creating the dataframe
    df = pd.DataFrame(data=np.transpose(np.vstack((np.rad2deg(pitch_angle_range), CL, CD))),
                      columns=['Pitch Angle', 'CL', 'CD'])
    df.to_excel("WingTail_InfluenceOfAoA.xlsx")
    print(df)
    return

def trim_at_cruise(wing_cl0=0.3475):

    # region Geometry and meshes

    # region Lifting surfaces
    caddee, system_model, sys_rep, sys_param, \
        wing_vlm_mesh_name, wing_camber_surface, \
        htail_vlm_mesh_name, htail_camber_surface = setup_geometry(
        include_wing_flag=True,
        include_tail_flag=True,
        include_tail_actuation_flag=True
    )
    # endregion

    spatial_rep = sys_rep.spatial_representation

    # region Rotors
    # Pusher prop
    pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropPusher']).keys())
    pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
    sys_rep.add_component(pp_disk)
    # endregion
    # endregion

    # region Sizing
    pav_wt = PavMassProperties()
    mass, cg, I = pav_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
    # endregion

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Cruise condition
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
    cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
    cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True,
                                      lower=np.deg2rad(-10), upper=np.deg2rad(10))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

    cruise_ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(cruise_ac_states)

    # region Propulsion
    pusher_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=5,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
    bem_model.set_module_input('rpm', val=4000., dv_flag=True, lower=1500., upper=5000., scaler=1e-3)
    bem_model.set_module_input('propeller_radius', val=3.97727 / 2 * ft2m)
    bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
    bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625]))
    bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                               dv_flag=True,
                               upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                               scaler=1
                               )
    bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                               dv_flag=True,
                               lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                               )
    bem_forces, bem_moments, _, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(bem_forces)
    cruise_model.register_output(bem_moments)
    # endregion

    # region Inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass,
                                                                      ac_states=cruise_ac_states)
    cruise_model.register_output(inertial_forces)
    cruise_model.register_output(inertial_moments)
    # endregion

    # region Aerodynamics
    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
            (1,) + htail_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[wing_cl0, 0.0]
    )
    vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(vlm_forces)
    cruise_model.register_output(vlm_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        inertial_forces, inertial_moments,
        vlm_forces, vlm_moments,
        bem_forces, bem_moments
    )
    cruise_model.register_output(total_forces)
    cruise_model.register_output(total_moments)

    # Equations of motions
    eom_m3l_model = cd.EoMM3LEuler6DOF()
    trim_residual = eom_m3l_model.evaluate(
        total_mass=total_mass,
        total_cg_vector=total_cg,
        total_inertia_tensor=total_inertia,
        total_forces=total_forces,
        total_moments=total_moments,
        ac_states=cruise_ac_states
    )

    cruise_model.register_output(trim_residual)

    # Add cruise m3l model to cruise condition
    cruise_condition.add_m3l_model('cruise_model', cruise_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(cruise_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
    caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1.)

    # region Optimization Setup
    caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
    caddee_csdl_model.add_constraint(
        name='system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta',
        equals=0.8)
    caddee_csdl_model.add_constraint(
        name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D',
        equals=8.,
        scaler=1e-1
    )
    # endregion

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()
    # sim.compute_total_derivatives()
    # sim.check_totals()

    print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])

    prob = CSDLProblem(problem_name='lpc', simulator=sim)
    optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
    optimizer.solve()
    optimizer.print_results()

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Pitch: ', np.rad2deg(
        sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
    print('RPM: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm'])
    print('Horizontal tail actuation: ',
          np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))

    print('Cruise propeller efficiency: ',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta'])
    print('Cruise L/D',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])
    print('Cruise prop torque', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.total_torque'])

    twist_cp = sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.twist_cp']
    print('Cruise prop twist cp: ', twist_cp)
    chord_cp = sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.chord_cp']
    print('Cruise prop chord cp: ', chord_cp)
    return twist_cp, chord_cp


def trim_at_hover(debug_geom_flag=False):
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()
    caddee.system_representation = sys_rep = cd.SystemRepresentation()
    caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

    # region Geometry
    file_name = 'pav.stp'

    spatial_rep = sys_rep.spatial_representation
    spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
    spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

    # region Rotors
    # region Lift rotor: Right 1
    lr_r1_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight1']).keys())
    lr_r1_disk = cd.Rotor(name='lr_r1_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r1_disk_prim_names)
    if debug_geom_flag:
        lr_r1_disk.plot()
    sys_rep.add_component(lr_r1_disk)
    # endregion

    # region Lift rotor: Right 2
    lr_r2_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight2']).keys())
    lr_r2_disk = cd.Rotor(name='lr_r2_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r2_disk_prim_names)
    if debug_geom_flag:
        lr_r2_disk.plot()
    sys_rep.add_component(lr_r2_disk)
    # endregion

    # region Lift rotor: Right 3
    lr_r3_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight3']).keys())
    lr_r3_disk = cd.Rotor(name='lr_r3_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r3_disk_prim_names)
    if debug_geom_flag:
        lr_r3_disk.plot()
    sys_rep.add_component(lr_r3_disk)
    # endregion

    # region Lift rotor: Right 4
    lr_r4_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight4']).keys())
    lr_r4_disk = cd.Rotor(name='lr_r4_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r4_disk_prim_names)
    if debug_geom_flag:
        lr_r4_disk.plot()
    sys_rep.add_component(lr_r4_disk)
    # endregion

    # region Lift rotor: Left 1
    lr_l1_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft1']).keys())
    lr_l1_disk = cd.Rotor(name='lr_l1_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l1_disk_prim_names)
    if debug_geom_flag:
        lr_l1_disk.plot()
    sys_rep.add_component(lr_l1_disk)
    # endregion
    
    # region Lift rotor: Left 2
    lr_l2_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft2']).keys())
    lr_l2_disk = cd.Rotor(name='lr_l2_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l2_disk_prim_names)
    if debug_geom_flag:
        lr_l2_disk.plot()
    sys_rep.add_component(lr_l2_disk)
    # endregion

    # region Lift rotor: Left 3
    lr_l3_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft3']).keys())
    lr_l3_disk = cd.Rotor(name='lr_l3_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l3_disk_prim_names)
    if debug_geom_flag:
        lr_l3_disk.plot()
    sys_rep.add_component(lr_l3_disk)
    # endregion

    # region Lift rotor: Left 4
    lr_l4_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft4']).keys())
    lr_l4_disk = cd.Rotor(name='lr_l4_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l4_disk_prim_names)
    if debug_geom_flag:
        lr_l4_disk.plot()
    sys_rep.add_component(lr_l4_disk)
    # endregion

    # endregion
    
    # endregion

    # region Sizing
    pav_wt = PavMassProperties()
    mass, cg, I = pav_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
    # endregion

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Hover condition
    hover_model = m3l.Model()
    hover_condition = cd.HoverCondition(name="hover")
    hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    hover_condition.set_module_input(name='altitude', val=50 * ft2m)
    hover_condition.set_module_input(name='hover_time', val=30)  # 30 seconds
    hover_condition.set_module_input(name='observer_location', val=np.array([0, 0, 50 * ft2m]))

    hover_ac_states = hover_condition.evaluate_ac_states()
    hover_model.register_output(hover_ac_states)

    # region Inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass,
                                                                      ac_states=hover_ac_states)
    hover_model.register_output(inertial_forces)
    hover_model.register_output(inertial_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Right 1
    lr_r1_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r1_bem_model = BEM(disk_prefix='lr_r1_disk', blade_prefix='lr_r1', 
                    component=lr_r1_disk, 
                    mesh=lr_r1_bem_mesh)
    lr_r1_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_r1_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r1_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r1_bem_model.set_module_input('thrust_origin', val=np.array([-3.000, 5.313, -0.530]))
    lr_r1_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                               dv_flag=True,
                               upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                               scaler=1
                               )
    lr_r1_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                               dv_flag=True,
                               lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                               )
    lr_r1_bem_forces, lr_r1_bem_moments, _, _, _, _ = lr_r1_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_r1_bem_forces)
    hover_model.register_output(lr_r1_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Right 2
    lr_r2_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r2_bem_model = BEM(disk_prefix='lr_r2_disk', blade_prefix='lr_r2',
                          component=lr_r2_disk,
                          mesh=lr_r2_bem_mesh)
    lr_r2_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_r2_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r2_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r2_bem_model.set_module_input('thrust_origin', val=np.array([3.500, 5.313, -0.530]))
    lr_r2_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_r2_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_r2_bem_forces, lr_r2_bem_moments, _, _, _, _ = lr_r2_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_r2_bem_forces)
    hover_model.register_output(lr_r2_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Right 3
    lr_r3_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r3_bem_model = BEM(disk_prefix='lr_r3_disk', blade_prefix='lr_r3',
                          component=lr_r3_disk,
                          mesh=lr_r3_bem_mesh)
    lr_r3_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_r3_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r3_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r3_bem_model.set_module_input('thrust_origin', val=np.array([16.000, 5.313, -0.530]))
    lr_r3_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_r3_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_r3_bem_forces, lr_r3_bem_moments, _, _, _, _ = lr_r3_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_r3_bem_forces)
    hover_model.register_output(lr_r3_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Right 4
    lr_r4_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r4_bem_model = BEM(disk_prefix='lr_r4_disk', blade_prefix='lr_r4',
                          component=lr_r4_disk,
                          mesh=lr_r4_bem_mesh)
    lr_r4_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_r4_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r4_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r4_bem_model.set_module_input('thrust_origin', val=np.array([25.000, 5.313, -0.530]))
    lr_r4_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_r4_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_r4_bem_forces, lr_r4_bem_moments, _, _, _, _ = lr_r4_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_r4_bem_forces)
    hover_model.register_output(lr_r4_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Left 1
    lr_l1_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l1_bem_model = BEM(disk_prefix='lr_l1_disk', blade_prefix='lr_l1',
                          component=lr_l1_disk,
                          mesh=lr_l1_bem_mesh)
    lr_l1_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_l1_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l1_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l1_bem_model.set_module_input('thrust_origin', val=np.array([-3.000, -5.313, -0.530]))
    lr_l1_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_l1_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_l1_bem_forces, lr_l1_bem_moments, _, _, _, _ = lr_l1_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_l1_bem_forces)
    hover_model.register_output(lr_l1_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Left 2
    lr_l2_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l2_bem_model = BEM(disk_prefix='lr_l2_disk', blade_prefix='lr_l2',
                          component=lr_l2_disk,
                          mesh=lr_l2_bem_mesh)
    lr_l2_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_l2_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l2_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l2_bem_model.set_module_input('thrust_origin', val=np.array([3.500, -5.313, -0.530]))
    lr_l2_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_l2_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_l2_bem_forces, lr_l2_bem_moments, _, _, _, _ = lr_l2_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_l2_bem_forces)
    hover_model.register_output(lr_l2_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Left 3
    lr_l3_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l3_bem_model = BEM(disk_prefix='lr_l3_disk', blade_prefix='lr_l3',
                          component=lr_l3_disk,
                          mesh=lr_l3_bem_mesh)
    lr_l3_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_l3_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l3_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l3_bem_model.set_module_input('thrust_origin', val=np.array([16.000, -5.313, -0.530]))
    lr_l3_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_l3_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_l3_bem_forces, lr_l3_bem_moments, _, _, _, _ = lr_l3_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_l3_bem_forces)
    hover_model.register_output(lr_l3_bem_moments)
    # endregion

    # region Propulsion Loads: Lift Rotor Left 4
    lr_l4_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l4_bem_model = BEM(disk_prefix='lr_l4_disk', blade_prefix='lr_l4',
                          component=lr_l4_disk,
                          mesh=lr_l4_bem_mesh)
    lr_l4_bem_model.set_module_input('rpm', val=2000, lower=500, upper=3000, scaler=1e-3, dv_flag=True)
    lr_l4_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l4_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l4_bem_model.set_module_input('thrust_origin', val=np.array([25.000, -5.313, -0.530]))
    lr_l4_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                                     dv_flag=True,
                                     upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                                     scaler=1
                                     )
    lr_l4_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                                     dv_flag=True,
                                     lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                                     )
    lr_l4_bem_forces, lr_l4_bem_moments, _, _, _, _ = lr_l4_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_l4_bem_forces)
    hover_model.register_output(lr_l4_bem_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        # lr_r1_bem_forces, lr_r1_bem_moments,
        # lr_r2_bem_forces, lr_r2_bem_moments,
        # lr_r3_bem_forces, lr_r3_bem_moments,
        # lr_r4_bem_forces, lr_r4_bem_moments,
        # lr_l1_bem_forces, lr_l1_bem_moments,
        # lr_l2_bem_forces, lr_l2_bem_moments,
        # lr_l3_bem_forces, lr_l3_bem_moments,
        # lr_l4_bem_forces, lr_l4_bem_moments,
        inertial_forces, inertial_moments
    )
    hover_model.register_output(total_forces)
    hover_model.register_output(total_moments)

    # Equations of motions
    eom_m3l_model = cd.EoMM3LEuler6DOF()
    trim_residual = eom_m3l_model.evaluate(
        total_mass=total_mass,
        total_cg_vector=total_cg,
        total_inertia_tensor=total_inertia,
        total_forces=total_forces,
        total_moments=total_moments,
        ac_states=hover_ac_states
    )

    hover_model.register_output(trim_residual)

    # Add hover m3l model to hover condition
    hover_condition.add_m3l_model('hover_model', hover_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(hover_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    # # region Optimization Setup
    # caddee_csdl_model.add_objective('system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.trim_residual')
    #
    # caddee_csdl_model.create_output(
    #     name='dss', val=9.
    # )
    # caddee_csdl_model.add_constraint()
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_r2_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_r3_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_r4_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_l1_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_l2_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_l3_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # caddee_csdl_model.add_constraint(
    # #     name='system_model.aircraft_trim.hover.hover.lr_l4_disk_bem_model.induced_velocity_model.FOM',
    # #     equals=0.76,
    # #     scaler=1
    # # )
    # # endregion

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()

    # prob = CSDLProblem(problem_name='lpc', simulator=sim)
    # optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
    # optimizer.solve()
    # optimizer.print_results()

    print('Trim residual: ', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.trim_residual'])

    print('Total forces: ', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.total_moments'])

    # print('Lift rotor right 1 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.rpm'])
    # print('Lift rotor right 1 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor right 2 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r2_disk_bem_model.rpm'])
    # print('Lift rotor right 2 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_r2_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor right 3 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r3_disk_bem_model.rpm'])
    # print('Lift rotor right 3 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_r3_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor right 4 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r4_disk_bem_model.rpm'])
    # print('Lift rotor right 4 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_r4_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor left 1 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l1_disk_bem_model.rpm'])
    # print('Lift rotor left 1 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_l1_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor left 2 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l2_disk_bem_model.rpm'])
    # print('Lift rotor left 2 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_l2_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor left 3 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l3_disk_bem_model.rpm'])
    # print('Lift rotor left 3 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_l3_disk_bem_model.induced_velocity_model.FOM'])
    #
    # print('Lift rotor left 4 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l4_disk_bem_model.rpm'])
    # print('Lift rotor left 4 FoM: ',
    #       sim['system_model.aircraft_trim.hover.hover.lr_l4_disk_bem_model.induced_velocity_model.FOM'])

    return


def structural_wingbox_beam_evaluation(wing_cl0=0.3475,
                                       pitch_angle=np.deg2rad(6.),
                                       num_wing_beam_nodes=21,
                                       debug_geom_flag = False, visualize_flag = False):
    # region Geometry and meshes
    caddee, system_model, sys_rep, sys_param, \
        wing_vlm_mesh_name, wing_camber_surface, \
        wing_oml_mesh, wing_component, \
        beam_mesh, beam_mass_mesh = setup_geometry(
        include_wing_flag=True,
        include_tail_flag=False,
        include_wing_beam_flag=True,
        visualize_flag=visualize_flag,
        debug_geom_flag=debug_geom_flag,
        num_wing_spanwise_vlm=21,
        num_wing_chordwise_vlm=5,
        num_wing_beam_nodes=21
    )
    # endregion

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Cruise condition
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
    cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
    cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
    cruise_condition.set_module_input(name='pitch_angle', val=pitch_angle)
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

    cruise_ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(cruise_ac_states)

    # region VLM Solver
    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[wing_cl0, ]
    )
    vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(vlm_forces)
    cruise_model.register_output(vlm_moments)

    vlm_force_mapping_model = VASTNodalForces(
        surface_names=[
            wing_vlm_mesh_name,
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
        ],
        initial_meshes=[
            wing_camber_surface,
        ]
    )

    oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces,
                                                  nodal_force_meshes=[wing_oml_mesh, ])
    wing_forces = oml_forces[0]

    # endregion

    # region Beam Solver

    # create the aframe dictionaries:
    joints, bounds, beams = {}, {}, {}
    beams['wing_beam'] = {'E': 70E9, 'G': 70E9 / (2 * (1 + 0.33)), 'rho': 2700, 'cs': 'box',
                          'nodes': list(range(num_wing_beam_nodes))}
    bounds['wing_root'] = {'beam': 'wing_beam', 'node': 10, 'fdim': [1, 1, 1, 1, 1, 1]}

    beam_mass = Mass(component=wing_component, mesh=beam_mass_mesh, beams=beams, mesh_units='ft')
    beam_mass.set_module_input('wing_beam_tcap', val=0.000508,
                               dv_flag=True, lower=0.000508, upper=0.02,
                               scaler=1E3)
    beam_mass.set_module_input('wing_beam_tweb', val=0.000508,
                               dv_flag=True, lower=0.000508, upper=0.02,
                               scaler=1E3)

    mass_model_wing_mass = beam_mass.evaluate()
    cruise_model.register_output(mass_model_wing_mass)

    dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(3, 1), control_points_shape=((35, 1)))
    dummy_function_space = lg.BSplineSetSpace(name='dummy_space', spaces={'dummy_b_spline_space': dummy_b_spline_space})

    cruise_wing_displacement_coefficients = m3l.Variable(name='cruise_wing_displacement_coefficients', shape=(35, 3))
    cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', space=dummy_function_space,
                                            coefficients=cruise_wing_displacement_coefficients)

    beam_force_map_model = ebbeam.EBBeamForces(component=wing_component,
                                               beam_mesh=beam_mesh,
                                               beams=beams,
                                               exclude_middle=True)
    cruise_structural_wing_mesh_forces = beam_force_map_model.evaluate(nodal_forces=wing_forces,
                                                                       nodal_forces_mesh=wing_oml_mesh)

    beam_displacements_model = ebbeam.EBBeam(component=wing_component, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)

    cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, \
        wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
        forces=cruise_structural_wing_mesh_forces)
    cruise_model.register_output(cruise_structural_wing_mesh_displacements)


    # endregion

    # Add cruise m3l model to cruise condition
    cruise_condition.add_m3l_model('cruise_model', cruise_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(cruise_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    caddee_csdl_model.connect('system_model.aircraft_trim.cruise_1.cruise_1.mass_model.wing_beam_tweb',
                              'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_tweb')
    caddee_csdl_model.connect('system_model.aircraft_trim.cruise_1.cruise_1.mass_model.wing_beam_tcap',
                              'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_tcap')

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()

    displ = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_displacement']
    print("Beam displacement (m): ", displ)
    print('Tip displacement (m): ', displ[-1, 2])

    print('Wingbox mass (kg): ', sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.MassProp.struct_mass'])
    print('Stress (N/m^2): ', sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.new_stress'])
    print('Wing beam forces (N): ',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.wing_beam_forces'])

    # beam_forces_from_vlm = np.reshape(
    #     sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.wing_beam_forces'],
    #     newshape=(num_wing_beam_nodes, 3))
    # np.sum(beam_forces_from_vlm, axis=0)

    print('Web thickness (m)', sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_tweb'])
    print('Cap thickness (m)', sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_tcap'])
    return


def trim_at_3g(pusher_prop_twist_cp=np.array([1.10595917, 0.71818285, 0.47990602, 0.35717703]),
               pusher_prop_chord_cp=np.array([0.09891285, 0.15891845, 0.14555978, 0.06686854]),
               wing_cl0=0.3475):

    # region Geometry and meshes

    # region Lifting surfaces
    caddee, system_model, sys_rep, sys_param, \
        wing_vlm_mesh_name, wing_camber_surface, \
        htail_vlm_mesh_name, htail_camber_surface = setup_geometry(
        include_wing_flag=True,
        include_tail_flag=True,
        include_tail_actuation_flag=True
    )
    # endregion

    spatial_rep = sys_rep.spatial_representation

    # region Rotors
    # Pusher prop
    pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropPusher']).keys())
    pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
    sys_rep.add_component(pp_disk)
    # endregion
    # endregion

    # region Sizing
    pav_wt = PavMassProperties()
    mass, cg, I = pav_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
    # endregion

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Cruise condition
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
    cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
    cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True,
                                      lower=np.deg2rad(-10), upper=np.deg2rad(15))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

    cruise_ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(cruise_ac_states)

    # region Propulsion
    pusher_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=5,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
    bem_model.set_module_input('rpm', val=4000., dv_flag=True, lower=1500., upper=5000., scaler=1e-3)
    bem_model.set_module_input('propeller_radius', val=3.97727 / 2 * ft2m)
    bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
    bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625]))
    bem_model.set_module_input('chord_cp', val=pusher_prop_chord_cp)
    bem_model.set_module_input('twist_cp', val=pusher_prop_twist_cp)
    bem_forces, bem_moments, _, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(bem_forces)
    cruise_model.register_output(bem_moments)
    # endregion

    # region Inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=3.)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass,
                                                                      ac_states=cruise_ac_states)
    cruise_model.register_output(inertial_forces)
    cruise_model.register_output(inertial_moments)
    # endregion

    # region Aerodynamics
    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
            (1,) + htail_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[wing_cl0, 0.0]
    )
    vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(vlm_forces)
    cruise_model.register_output(vlm_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        inertial_forces, inertial_moments,
        vlm_forces, vlm_moments,
        bem_forces, bem_moments
    )
    cruise_model.register_output(total_forces)
    cruise_model.register_output(total_moments)

    # Equations of motions
    eom_m3l_model = cd.EoMM3LEuler6DOF()
    trim_residual = eom_m3l_model.evaluate(
        total_mass=total_mass,
        total_cg_vector=total_cg,
        total_inertia_tensor=total_inertia,
        total_forces=total_forces,
        total_moments=total_moments,
        ac_states=cruise_ac_states
    )

    cruise_model.register_output(trim_residual)

    # Add cruise m3l model to cruise condition
    cruise_condition.add_m3l_model('cruise_model', cruise_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(cruise_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
    caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-30), upper=np.deg2rad(30), scaler=1.)

    # region Optimization Setup
    caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
    # endregion

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()
    # sim.compute_total_derivatives()
    # sim.check_totals()

    print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])

    prob = CSDLProblem(problem_name='lpc', simulator=sim)
    optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
    optimizer.solve()
    optimizer.print_results()

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Pitch: ', np.rad2deg(
        sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
    print('RPM: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm'])
    print('Horizontal tail actuation: ',
          np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))

    print('Cruise propeller efficiency: ',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta'])
    print('Cruise L/D',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])
    print('Cruise prop torque', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.total_torque'])

    return


if __name__ == '__main__':
    vlm_as_ll()
    cl0 = tuning_cl0()
    vlm_evaluation_wing_only_aoa_sweep()
    vlm_evaluation_wing_tail_aoa_sweep(debug_geom_flag=False, visualize_flag=False)
    pusher_prop_twist_cp, pusher_prop_chord_cp = trim_at_cruise()
    trim_at_3g()
    structural_wingbox_beam_evaluation(pitch_angle=np.deg2rad(0.50921594))

    # trim_at_hover()
