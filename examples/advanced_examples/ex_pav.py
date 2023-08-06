# region Imports
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem

# Geometry
from caddee.utils.aircraft_models.pav.pav_geom_mesh import PavGeomMesh
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

from caddee.utils.aircraft_models.pav.ex_pav_visualize_function import visualize_ex_pav

from caddee import GEOMETRY_FILES_FOLDER

import numpy as np
import pandas as pd
import sys
sys.setrecursionlimit(100000)
import unittest


# endregion


ft2m = 0.3048
force_reprojection = False


def vlm_as_ll():
    """
    Script that tests if the VLM when defaulted to a lifting line returns CL=0 at 0 deg pitch angle
    """
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()

    # region Geometry and meshes
    pav_geom_mesh = PavGeomMesh()
    pav_geom_mesh.setup_geometry(
        include_wing_flag=True,
        force_reprojection=force_reprojection
    )
    pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=2,
                             force_reprojection=force_reprojection)

    caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
    caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param

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
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
        ],
        surface_shapes=[
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='m',
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

    wing_surface_name = pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing']
    CL_total = sim[
            'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
    CL_surface = sim[
            f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_surface_name}_C_L_total']
    print('CL when VLM is made LL: ', CL_total)
    if CL_total > 0.02 and (CL_total-CL_surface)**2 > 1e-6:
        raise ValueError
    return

def tuning_cl0(cl0_expected=0.61753,
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

        caddee = cd.CADDEE()
        caddee.system_model = system_model = cd.SystemModel()

        # region Geometry and meshes
        pav_geom_mesh = PavGeomMesh()
        pav_geom_mesh.setup_geometry(
            include_wing_flag=True,
            debug_geom_flag=debug_geom_flag,
            force_reprojection=force_reprojection
        )
        pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                                 visualize_flag=visualize_flag, force_reprojection=force_reprojection)

        caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
        caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
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
                pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
            ],
            surface_shapes=[
                (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
            ],
            fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
            mesh_unit='m',
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

    if (res.x - 0.3662)**2 > 1e-4:
        raise ValueError

    return res.x


def vlm_evaluation_wing_only_aoa_sweep(wing_cl0=0.3662):
    resolution = 27
    pitch_angle_range = np.deg2rad(np.linspace(-10, 16, num=resolution))
    CL = np.empty(resolution)
    CD = np.empty(resolution)

    for idx, pitch_angle in enumerate(pitch_angle_range):
        caddee = cd.CADDEE()
        caddee.system_model = system_model = cd.SystemModel()

        # region Geometry and meshes
        pav_geom_mesh = PavGeomMesh()
        pav_geom_mesh.setup_geometry(
            include_wing_flag=True,
            force_reprojection=force_reprojection
        )
        pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                                 force_reprojection=False)

        caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
        caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
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
                pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
            ],
            surface_shapes=[
                (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
            ],
            fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
            mesh_unit='m',
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


def vlm_evaluation_wing_tail_aoa_sweep(wing_cl0=0.3662,
                                       debug_geom_flag = False, visualize_flag = False):
    resolution = 27
    pitch_angle_range = np.deg2rad(np.linspace(-10, 16, num=resolution))
    CL = np.empty(resolution)
    CD = np.empty(resolution)

    for idx, pitch_angle in enumerate(pitch_angle_range):
        caddee = cd.CADDEE()
        caddee.system_model = system_model = cd.SystemModel()

        # region Geometry and meshes
        pav_geom_mesh = PavGeomMesh()
        pav_geom_mesh.setup_geometry(
            include_wing_flag=True,
            include_htail_flag=True,
            debug_geom_flag=debug_geom_flag,
            force_reprojection=force_reprojection
        )
        pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                                 include_htail_flag=True, num_htail_spanwise_vlm=21, num_htail_chordwise_vlm=5,
                                 force_reprojection=force_reprojection, visualize_flag=visualize_flag)

        caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
        caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
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
                pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
                pav_geom_mesh.mesh_data['vlm']['mesh_name']['htail']
            ],
            surface_shapes=[
                (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
                (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['htail'].evaluate().shape[1:],
            ],
            fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
            mesh_unit='m',
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

def trim_at_cruise(wing_cl0=0.3662):

    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()

    # region Geometry and meshes

    # region Lifting surfaces
    pav_geom_mesh = PavGeomMesh()
    pav_geom_mesh.setup_geometry(
        include_wing_flag=True,
        include_htail_flag=True,
        force_reprojection=force_reprojection
    )
    pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                             include_htail_flag=True, num_htail_spanwise_vlm=21, num_htail_chordwise_vlm=5,
                             force_reprojection=force_reprojection)
    pav_geom_mesh.actuations(include_tail_actuation_flag=True)

    caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
    caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
    sys_param.setup()
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
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(-0.02403531), dv_flag=True,
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
    bem_model.set_module_input('rpm', val=4000., dv_flag=True, lower=1500., upper=5000., scaler=1e-3)
    bem_model.set_module_input('propeller_radius', val=3.97727 / 2 * ft2m)
    bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
    bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625])* ft2m)
    bem_model.set_module_input('chord_cp', val=np.array([0.09891285, 0.15891845, 0.14555978, 0.06686854]),
                               dv_flag=True,
                               upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                               scaler=1
                               )
    bem_model.set_module_input('twist_cp', val=np.array([1.10595917, 0.71818285, 0.47990602, 0.35717703]),
                               dv_flag=True,
                               lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                               )
    bem_forces, bem_moments, _, _, _, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
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
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['htail']
        ],
        surface_shapes=[
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['htail'].evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='m',
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

    pitch_angle = np.rad2deg(sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle'])
    rpm = sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm']
    tail_actuation = np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act'])

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Pitch: ', pitch_angle)
    print('RPM: ', rpm)
    print('Horizontal tail actuation: ', tail_actuation)

    print('Cruise propeller efficiency: ',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta'])
    print('Cruise L/D',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])
    print('Cruise prop torque', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.total_torque'])

    twist_cp = sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.twist_cp']
    print('Cruise prop twist cp: ', twist_cp)
    chord_cp = sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.chord_cp']
    print('Cruise prop chord cp: ', chord_cp)

    tolerence = 1e-4
    if (pitch_angle - (-0.8008331)) ** 2 > tolerence or \
            (rpm - 2880.90131068) ** 2 > tolerence or \
            (tail_actuation - (0.3148777)) ** 2 > tolerence:
        raise ValueError
    return twist_cp, chord_cp


def trim_at_n1g(wing_cl0=0.3662,
                pusher_prop_twist_cp=np.array([1.10595917, 0.71818285, 0.47990602, 0.35717703]),
                pusher_prop_chord_cp=np.array([0.09891285, 0.15891845, 0.14555978, 0.06686854])):

    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()

    # region Geometry and meshes

    # region Lifting surfaces
    pav_geom_mesh = PavGeomMesh()
    pav_geom_mesh.setup_geometry(
        include_wing_flag=True,
        include_htail_flag=True,
        force_reprojection=force_reprojection
    )
    pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                             include_htail_flag=True, num_htail_spanwise_vlm=21, num_htail_chordwise_vlm=5,
                             force_reprojection=force_reprojection)
    pav_geom_mesh.actuations(include_tail_actuation_flag=True)

    caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
    caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
    sys_param.setup()
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
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(-0.02403531), dv_flag=True,
                                      lower=np.deg2rad(-15), upper=np.deg2rad(10))
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
    bem_model.set_module_input('rpm', val=4000., dv_flag=True, lower=1500., upper=5000., scaler=1e-3)
    bem_model.set_module_input('propeller_radius', val=3.97727 / 2 * ft2m)
    bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
    bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625])* ft2m)
    bem_model.set_module_input('chord_cp', val=pusher_prop_chord_cp)
    bem_model.set_module_input('twist_cp', val=pusher_prop_twist_cp)
    bem_forces, bem_moments, _, _, _, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(bem_forces)
    cruise_model.register_output(bem_moments)
    # endregion

    # region Inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=-1.)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass,
                                                                      ac_states=cruise_ac_states)
    cruise_model.register_output(inertial_forces)
    cruise_model.register_output(inertial_moments)
    # endregion

    # region Aerodynamics
    vlm_model = VASTFluidSover(
        surface_names=[
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['htail']
        ],
        surface_shapes=[
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['htail'].evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='m',
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
    caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-15), upper=np.deg2rad(15), scaler=1.)

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
    optimizer = SLSQP(prob, maxiter=500, ftol=1E-10)
    optimizer.solve()
    optimizer.print_results()

    pitch_angle = np.rad2deg(sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle'])
    rpm = sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm']
    tail_actuation = np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act'])

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Pitch: ', pitch_angle)
    print('RPM: ', rpm)
    print('Horizontal tail actuation: ', tail_actuation)

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


def optimize_lift_rotor_blade(expected_thrust=980.665,
                              rotor_rpm=2400.,
                              debug_geom_flag=True):
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

    # region Propulsion Loads: Lift Rotor Right 1
    lr_r1_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_cp=4,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r1_bem_model = BEM(disk_prefix='lr_r1_disk', blade_prefix='lr_r1',
                          component=lr_r1_disk,
                          mesh=lr_r1_bem_mesh)
    lr_r1_bem_model.set_module_input('rpm', val=rotor_rpm)
    lr_r1_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r1_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r1_bem_model.set_module_input('thrust_origin', val=np.array([-1.146, 1.619, -0.162]))  # m
    lr_r1_bem_model.set_module_input('chord_cp', val=np.array([0.24830086, 0.14683384, 0.1215227, 0.05676139]),
                                     dv_flag=True,
                                     upper=np.array([0.3, 0.3, 0.3, 0.3]), lower=np.array([0.05, 0.05, 0.02, 0.02]),
                                     scaler=1
                                     )
    lr_r1_bem_model.set_module_input('twist_cp', val=np.array([0.43293954, 0.37960366, 0.21316869, 0.13607267]),
                                     dv_flag=True,
                                     lower=np.deg2rad(0), upper=np.deg2rad(85), scaler=1
                                     )
    lr_r1_bem_forces, lr_r1_bem_moments, _, _, _, _, _, _ = lr_r1_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_r1_bem_forces)
    hover_model.register_output(lr_r1_bem_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        lr_r1_bem_forces, lr_r1_bem_moments
    )
    hover_model.register_output(total_forces)
    hover_model.register_output(total_moments)

    # Add hover m3l model to hover condition
    hover_condition.add_m3l_model('hover_model', hover_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(hover_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    expected_thrust = caddee_csdl_model.create_input(name='expected_thrust', val=expected_thrust)
    computed_thrust = caddee_csdl_model.declare_variable(name='computed_thrust')
    caddee_csdl_model.connect('system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.T', 'computed_thrust')
    thrust_residual = (computed_thrust + -1*expected_thrust)**2
    caddee_csdl_model.register_output(name='thrust_residual', var=thrust_residual)

    # # region Optimization Setup
    caddee_csdl_model.add_objective('thrust_residual', scaler=1e-4)
    caddee_csdl_model.add_constraint(
        name='system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.induced_velocity_model.FOM',
        lower=0.76)

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()

    prob = CSDLProblem(problem_name='lpc', simulator=sim)
    optimizer = SLSQP(prob, maxiter=250, ftol=1E-5)
    optimizer.solve()
    optimizer.print_results()

    print('Total forces (N): ', sim['system_model.aircraft_trim.hover.hover.total_forces_moments_model.total_forces'])
    print('Thrust (N): ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.T'])
    print('FoM: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.induced_velocity_model.FOM'])
    print('Top-level model computed thrust (N):', sim['computed_thrust'])
    print('Top-level model expected thrust (N):', sim['expected_thrust'])
    print('Thrust residual: ', sim['thrust_residual'])
    print('Twist cp: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.twist_cp'])
    print('Chord cp: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.chord_cp'])

    return


def trim_at_hover(twist_cp=np.array([0.36388453, 0.24775229, 0.1752334,  0.10173386]),
                  chord_cp=np.array([0.20088419, 0.1588741, 0.08904938, 0.04459458]),
                  rotor_rpm=2400,
                  debug_geom_flag=False):
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r1_bem_model = BEM(disk_prefix='lr_r1_disk', blade_prefix='lr_r1', 
                    component=lr_r1_disk, 
                    mesh=lr_r1_bem_mesh)
    lr_r1_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_r1_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r1_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r1_bem_model.set_module_input('thrust_origin', val=np.array([-1.146, 1.619, -0.162]))  # m
    lr_r1_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_r1_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_r1_bem_forces, lr_r1_bem_moments,_, _, _, _, _, _ = lr_r1_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r2_bem_model = BEM(disk_prefix='lr_r2_disk', blade_prefix='lr_r2',
                          component=lr_r2_disk,
                          mesh=lr_r2_bem_mesh)
    lr_r2_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_r2_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r2_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r2_bem_model.set_module_input('thrust_origin', val=np.array([1.597, 1.619, -0.162]))  # m
    lr_r2_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_r2_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_r2_bem_forces, lr_r2_bem_moments,_, _, _, _, _, _ = lr_r2_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r3_bem_model = BEM(disk_prefix='lr_r3_disk', blade_prefix='lr_r3',
                          component=lr_r3_disk,
                          mesh=lr_r3_bem_mesh)
    lr_r3_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_r3_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r3_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r3_bem_model.set_module_input('thrust_origin', val=np.array([4.877, 1.619, -0.162]))  # m
    lr_r3_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_r3_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_r3_bem_forces, lr_r3_bem_moments,_, _, _, _, _, _ = lr_r3_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r4_bem_model = BEM(disk_prefix='lr_r4_disk', blade_prefix='lr_r4',
                          component=lr_r4_disk,
                          mesh=lr_r4_bem_mesh)
    lr_r4_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_r4_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r4_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r4_bem_model.set_module_input('thrust_origin', val=np.array([7.620, 1.619, -0.162]))  # m
    lr_r4_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_r4_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_r4_bem_forces, lr_r4_bem_moments, _, _, _, _, _, _ = lr_r4_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l1_bem_model = BEM(disk_prefix='lr_l1_disk', blade_prefix='lr_l1',
                          component=lr_l1_disk,
                          mesh=lr_l1_bem_mesh)
    lr_l1_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_l1_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l1_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l1_bem_model.set_module_input('thrust_origin', val=np.array([-1.146, -1.619, -0.162]))  # m
    lr_l1_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_l1_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_l1_bem_forces, lr_l1_bem_moments,_, _, _, _, _, _ = lr_l1_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l2_bem_model = BEM(disk_prefix='lr_l2_disk', blade_prefix='lr_l2',
                          component=lr_l2_disk,
                          mesh=lr_l2_bem_mesh)
    lr_l2_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_l2_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l2_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l2_bem_model.set_module_input('thrust_origin', val=np.array([1.597, -1.619, -0.162]))  # m
    lr_l2_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_l2_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_l2_bem_forces, lr_l2_bem_moments,_, _, _, _, _, _ = lr_l2_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l3_bem_model = BEM(disk_prefix='lr_l3_disk', blade_prefix='lr_l3',
                          component=lr_l3_disk,
                          mesh=lr_l3_bem_mesh)
    lr_l3_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_l3_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l3_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l3_bem_model.set_module_input('thrust_origin', val=np.array([4.877, -1.619, -0.162]))  # m
    lr_l3_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_l3_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_l3_bem_forces, lr_l3_bem_moments,_, _, _, _, _, _ = lr_l3_bem_model.evaluate(ac_states=hover_ac_states)
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_l4_bem_model = BEM(disk_prefix='lr_l4_disk', blade_prefix='lr_l4',
                          component=lr_l4_disk,
                          mesh=lr_l4_bem_mesh)
    lr_l4_bem_model.set_module_input('rpm', val=rotor_rpm, lower=rotor_rpm - 50, upper=rotor_rpm + 50, scaler=1e-3, dv_flag=True)
    lr_l4_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_l4_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_l4_bem_model.set_module_input('thrust_origin', val=np.array([7.620, -1.619, -0.162]))  # m
    lr_l4_bem_model.set_module_input('chord_cp', val=chord_cp)
    lr_l4_bem_model.set_module_input('twist_cp', val=twist_cp)
    lr_l4_bem_forces, lr_l4_bem_moments,_, _, _, _, _, _ = lr_l4_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_l4_bem_forces)
    hover_model.register_output(lr_l4_bem_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        lr_r1_bem_forces, lr_r1_bem_moments,
        lr_r2_bem_forces, lr_r2_bem_moments,
        lr_r3_bem_forces, lr_r3_bem_moments,
        lr_r4_bem_forces, lr_r4_bem_moments,
        lr_l1_bem_forces, lr_l1_bem_moments,
        lr_l2_bem_forces, lr_l2_bem_moments,
        lr_l3_bem_forces, lr_l3_bem_moments,
        lr_l4_bem_forces, lr_l4_bem_moments,
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

    # region Optimization Setup
    caddee_csdl_model.add_objective('system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.trim_residual')
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

    prob = CSDLProblem(problem_name='lpc', simulator=sim)
    optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
    optimizer.solve()
    optimizer.print_results()

    print('Trim residual: ', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.trim_residual'])

    print('Total forces: ', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.total_moments'])

    print('Lift rotor right 1 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.rpm'])
    print('Lift rotor right 1 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor right 2 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r2_disk_bem_model.rpm'])
    print('Lift rotor right 2 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_r2_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor right 3 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r3_disk_bem_model.rpm'])
    print('Lift rotor right 3 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_r3_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor right 4 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r4_disk_bem_model.rpm'])
    print('Lift rotor right 4 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_r4_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor left 1 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l1_disk_bem_model.rpm'])
    print('Lift rotor left 1 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_l1_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor left 2 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l2_disk_bem_model.rpm'])
    print('Lift rotor left 2 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_l2_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor left 3 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l3_disk_bem_model.rpm'])
    print('Lift rotor left 3 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_l3_disk_bem_model.induced_velocity_model.FOM'])

    print('Lift rotor left 4 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_l4_disk_bem_model.rpm'])
    print('Lift rotor left 4 FoM: ',
          sim['system_model.aircraft_trim.hover.hover.lr_l4_disk_bem_model.induced_velocity_model.FOM'])

    return


def structural_wingbox_beam_evaluation(wing_cl0=0.3662,
                                       pitch_angle=np.deg2rad(6.),
                                       num_wing_beam_nodes=21,
                                       youngs_modulus=73.1E9, poissons_ratio=0.33, density=2768,  # SI
                                       min_guage=0.00127, yield_strength=324E6, FoS=1.5,  # SI 0.05 in = 0.00127 m
                                       thicknesses=0.00127,
                                       sizing_flag=False,
                                       visualize_flag = False):
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()

    # region Geometry and meshes
    pav_geom_mesh = PavGeomMesh()
    pav_geom_mesh.setup_geometry(
        include_wing_flag=True,
        include_htail_flag=False,
    )
    pav_geom_mesh.sys_rep.spatial_representation.assemble()
    pav_geom_mesh.oml_mesh(include_wing_flag=True,
                           debug_geom_flag=False, force_reprojection=force_reprojection)
    pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                             visualize_flag=visualize_flag, force_reprojection=force_reprojection)
    pav_geom_mesh.beam_mesh(include_wing_flag=True, num_wing_beam_nodes=21,
                            visualize_flag=visualize_flag, force_reprojection=force_reprojection)
    pav_geom_mesh.setup_index_functions(left_wing_shell_flag=False)
    caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
    caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
    sys_param.setup()
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
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
        ],
        surface_shapes=[
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='m',
        cl0=[wing_cl0, ]
    )
    wing_vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(vlm_forces)
    cruise_model.register_output(vlm_moments)

    vlm_force_mapping_model = VASTNodalForces(
        surface_names=[
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
        ],
        surface_shapes=[
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
        ],
        initial_meshes=[
            pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'],
        ]
    )

    wing_oml_mesh = pav_geom_mesh.mesh_data['oml']['oml_geo_nodes']['wing']
    oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=wing_vlm_panel_forces,
                                                  nodal_force_meshes=[wing_oml_mesh, ])
    wing_forces = oml_forces[0]

    # Index function
    wing_force_index_func = pav_geom_mesh.functions['wing_force']
    wing_oml_para_coords = pav_geom_mesh.mesh_data['oml']['oml_para_nodes']['wing']
    wing_force_index_func.inverse_evaluate(wing_oml_para_coords, wing_forces)
    cruise_model.register_output(wing_force_index_func.coefficients)
    # ################### Index function thing ###################
    # endregion

    # region Beam Solver
    wing_component = pav_geom_mesh.geom_data['components']['wing']
    beam_mass_mesh = pav_geom_mesh.mesh_data['beam']['mass']
    beam_mesh = pav_geom_mesh.mesh_data['beam']['ebbeam']

    # create the aframe dictionaries:
    joints, bounds, beams = {}, {}, {}
    beams['wing_beam'] = {'E': youngs_modulus,
                          'G': youngs_modulus / (2 * (1 + poissons_ratio)),
                          'rho': density, 'cs': 'box',
                          'nodes': list(range(num_wing_beam_nodes))}
    bounds['wing_root'] = {'beam': 'wing_beam', 'node': 10, 'fdim': [1, 1, 1, 1, 1, 1]}

    beam_mass = Mass(component=wing_component, mesh=beam_mass_mesh, beams=beams, mesh_units='m')
    beam_mass.set_module_input('wing_beam_tcap', val=thicknesses,
                               dv_flag=True, lower=min_guage, upper=0.02,
                               scaler=1E3)
    beam_mass.set_module_input('wing_beam_tweb', val=thicknesses,
                               dv_flag=True, lower=min_guage, upper=0.02,
                               scaler=1E3)

    mass_model_wing_mass = beam_mass.evaluate()
    cruise_model.register_output(mass_model_wing_mass)

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
    # cruise_model.register_output(wing_mass)

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

    # region Optimization
    if sizing_flag:
        caddee_csdl_model.add_constraint(
            'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.new_stress', upper=yield_strength / FoS,
            scaler=1E-8)
        caddee_csdl_model.add_objective(
            'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.MassProp.struct_mass', scaler=1e-3)
    # endregion

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()

    if sizing_flag:
        prob = CSDLProblem(problem_name='pav_wingbox_sizing_beam', simulator=sim)
        optimizer = SLSQP(prob, maxiter=1000, ftol=1E-8)
        optimizer.solve()
        optimizer.print_results()
        assert optimizer.scipy_output.success

    # region Results

    # Displacement
    displ = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_displacement']
    print("Beam displacement (m): ", displ)
    print('Tip displacement (m): ', displ[-1, 2])
    print('Tip displacement (in): ', displ[-1, 2]* 39.3701)

    print('Wingbox mass (kg): ', sim['system_model.aircraft_trim.cruise_1.cruise_1.mass_model.mass'])
    print('Mass prop mass: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.MassProp.mass'])

    # Stress
    vmstress = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.new_stress']
    print('Stress (N/m^2): ', vmstress)
    print('Max stress (N/m^2): ', np.max(np.max(vmstress)))
    print('Max stress (psi): ', np.max(np.max(vmstress))*0.000145038)

    # Thicknesses
    web_t = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_tweb']
    cap_t = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_tcap']

    # VLM forces
    vlm_panel_forces = sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.wing_vlm_mesh_total_forces']
    vlm_oml_forces = vlm_panel_forces.reshape(4, 20, 3)
    vlm_panel_forces_summed = np.sum(vlm_oml_forces, axis=0)

    # endregion

    # region Output Dataframe
    spanwise_node_y_loc = sim[
                              'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_mesh'].reshape(
        num_wing_beam_nodes, 3)[:, 1]
    spanwise_max_stress = np.max(vmstress, axis=1)
    spanwise_z_disp = displ[:, 2]
    spanwise_z_force = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.wing_beam_forces'].reshape(
        num_wing_beam_nodes, 3)[:, 2]
    print('Total load: (lbf): ', np.sum(spanwise_z_force)*0.224809)
    spanwise_width = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_width']
    spanwise_height = sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_height']
    node_dict = {'Spanwise loc (m)': spanwise_node_y_loc,
                'Width (in)': spanwise_width*3.28084*12,
                'Height (in)': spanwise_height*3.28084*12,
                'Node z force (lbf)': spanwise_z_force*0.224809,
                'Displacement (ft)': spanwise_z_disp*3.28084}
    elem_dict = {'VLM z force (lbf)': vlm_panel_forces_summed[:, 2]*0.224809,
                  'Max stress (N/m^2)': spanwise_max_stress,
                  'Web thickness (m)': web_t,
                  'Cap thickness (m)': cap_t}
    nodal_sol_df = pd.DataFrame(data=node_dict)
    nodal_sol_df.to_excel(f'BeamWingboxAnalysis_{np.rad2deg(pitch_angle)}deg_NodalSolution.xlsx')
    print(nodal_sol_df)
    elem_sol_df = pd.DataFrame(data=elem_dict)
    elem_sol_df.to_excel(f'BeamWingboxAnalysis_{np.rad2deg(pitch_angle)}deg_ElementSolution.xlsx')
    print(elem_sol_df)
    # endregion

    # region Plot data
    plot_data = {}
    plot_data['caddee'] = caddee
    plot_data['sim'] = sim
    plot_data['displ'] = displ
    plot_data['web_t'] = web_t
    plot_data['cap_t'] = cap_t
    plot_data['beam_mesh'] = sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_mesh'].reshape(
        num_wing_beam_nodes, 3)
    plot_data['spanwise_height'] = spanwise_height
    plot_data['spanwise_width'] = spanwise_width
    plot_data['forces_index_function'] = wing_force_index_func
    plot_data['vlm_mesh'] = sim['system_representation.outputs_model.design_outputs_model.wing_vlm_mesh']
    plot_data['vlm_force'] = vlm_panel_forces
    # endregion

    print('Tip Iy: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_element_0_Iy'])
    print('Root Iy: ',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_element_10_Iy'])

    print(f'Root bending moment: ', sim[
              'system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.Aframe.wing_beam_element_10StressBox.wing_beam_element_10local_loads'][10])

    print('Spanwise external load resultant: ',
          np.linalg.norm(sim['system_model.aircraft_trim.cruise_1.cruise_1.Wing_eb_beam_model.wing_beam_forces'].reshape(num_wing_beam_nodes, 3), axis=1)*0.224809)

    return plot_data


def trim_at_3g(pusher_prop_twist_cp=np.array([1.10595917, 0.71818285, 0.47990602, 0.35717703]),
               pusher_prop_chord_cp=np.array([0.09891285, 0.15891845, 0.14555978, 0.06686854]),
               wing_cl0=0.3662):
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()

    # region Geometry and meshes

    # region Lifting surfaces
    pav_geom_mesh = PavGeomMesh()
    pav_geom_mesh.setup_geometry(
        include_wing_flag=True,
        include_htail_flag=True,
    )
    pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                             include_htail_flag=True, num_htail_spanwise_vlm=21, num_htail_chordwise_vlm=5)
    pav_geom_mesh.actuations(include_tail_actuation_flag=True)

    caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
    caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
    sys_param.setup()
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
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(10), dv_flag=True,
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
        mesh_units='m',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
    bem_model.set_module_input('rpm', val=4000., dv_flag=True, lower=1500., upper=5000., scaler=1e-3)
    bem_model.set_module_input('propeller_radius', val=3.97727 / 2 * ft2m)
    bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
    bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625])* ft2m)
    bem_model.set_module_input('chord_cp', val=pusher_prop_chord_cp)
    bem_model.set_module_input('twist_cp', val=pusher_prop_twist_cp)
    bem_forces, bem_moments, _, _, _, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
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
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['wing'],
            pav_geom_mesh.mesh_data['vlm']['mesh_name']['htail']
        ],
        surface_shapes=[
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['wing'].evaluate().shape[1:],
            (1,) + pav_geom_mesh.mesh_data['vlm']['chamber_surface']['htail'].evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='m',
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

    print('3g propeller efficiency: ',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta'])
    print('3g L/D',
          sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])
    print('3g prop torque', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.total_torque'])

    return


def structural_wingbox_shell_evaluation(wing_cl0=0.3366,
                                        pitch_angle=np.deg2rad(6.),
                                        debug_geom_flag=False,
                                        visualize_flag=False):
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()

    # region Geometry and meshes
    pav_geom_mesh = PavGeomMesh()
    pav_geom_mesh.setup_geometry(
        include_wing_flag=True,
        include_htail_flag=False,
    )
    pav_geom_mesh.setup_internal_wingbox_geometry(debug_geom_flag=debug_geom_flag,
                                                  force_reprojection=force_reprojection)
    pav_geom_mesh.sys_rep.spatial_representation.assemble()
    pav_geom_mesh.oml_mesh(include_wing_flag=True,
                           debug_geom_flag=debug_geom_flag, force_reprojection=force_reprojection)
    pav_geom_mesh.vlm_meshes(include_wing_flag=True, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
                             visualize_flag=visualize_flag, force_reprojection=force_reprojection)
    pav_geom_mesh.setup_index_functions()

    caddee.system_representation = sys_rep = pav_geom_mesh.sys_rep
    caddee.system_parameterization = sys_param = pav_geom_mesh.sys_param
    sys_param.setup()
    # endregion
    return


def pav_visualization():

    m2ft = 3.28084

    plots_minus1 = structural_wingbox_beam_evaluation(pitch_angle=np.deg2rad(-12.06291905), sizing_flag=True, visualize_flag=False)
    plots_plus1 = structural_wingbox_beam_evaluation(pitch_angle=np.deg2rad(-0.38129494), sizing_flag=True, visualize_flag=False)
    plots_plus3 = structural_wingbox_beam_evaluation(pitch_angle=np.deg2rad(12.11391141), sizing_flag=True, visualize_flag=False)
    # exit()

    caddee = plots_plus3['caddee']
    sim = plots_plus3['sim']
    # list of displacements to add. The first one will plot thicknesses (3g) the rest will plot displacements
    # jig is the zero displacements
    displacements = [plots_plus3['displ'], plots_minus1['displ'], plots_plus1['displ'], np.zeros_like(plots_plus3['displ'])]
    beam_mesh = plots_plus3['beam_mesh']
    # web_t = plots['web_t']
    # cap_t = plots['cap_t']
    width = plots_plus3['spanwise_width']
    height = plots_plus3['spanwise_height']
    forces_index_function = plots_plus1['forces_index_function']  # index function of pressure
    rotor_origins = [
        np.array([-1.146, 1.619, -0.162]) * m2ft, # np.array([-3.000, 5.313, -0.530]),
        np.array([1.597, 1.619, -0.162]) * m2ft,  # np.array([3.500, 5.313, -0.530]),
        np.array([4.877, 1.619, -0.162]) * m2ft,  # np.array([16.000, 5.313, -0.530]),
        np.array([7.620, 1.619, -0.162]) * m2ft,  # np.array([25.000, 5.313, -0.530]),
        np.array([-1.146, -1.619, -0.162]) * m2ft, # np.array([-3.000, -5.313, -0.530]),
        np.array([1.597, -1.619, -0.162]) * m2ft,  # np.array([3.500, -5.313, -0.530]),
        np.array([4.877, -1.619, -0.162]) * m2ft,  # np.array([16.000, -5.313, -0.530]),
        np.array([7.620, -1.619, -0.162]) * m2ft,  # np.array([25.000, -5.313, -0.530]),
    ]  # origins of rotors to draw vector from

    print('-1g loads web thickness :', plots_minus1['web_t'])
    print('-1g loads cap thickness :', plots_minus1['cap_t'])

    print('1g loads web thickness :', plots_plus1['web_t'])
    print('1g loads cap thickness :', plots_plus1['cap_t'])

    print('3g loads web thickness :', plots_plus3['web_t'])
    print('3g loads cap thickness :', plots_plus3['cap_t'])

    web_t = plots_plus3['web_t']
    cap_t = plots_plus3['cap_t']

    vlm_mesh = plots_plus1['vlm_mesh']
    vlm_force = plots_plus1['vlm_force']

    visualize_ex_pav(
        caddee,
        sim,
        displacements,
        beam_mesh,
        web_t,
        cap_t,
        width,
        height,
        rotor_origins,
        forces_index_function,
        vlm_mesh,
        vlm_force
    )
    return


if __name__ == '__main__':
    # vlm_as_ll()
    # cl0 = tuning_cl0()
    # vlm_evaluation_wing_only_aoa_sweep()
    # vlm_evaluation_wing_tail_aoa_sweep(visualize_flag=False)
    # pusher_prop_twist_cp, pusher_prop_chord_cp = trim_at_cruise()
    # trim_at_n1g(
    #     pusher_prop_twist_cp=pusher_prop_twist_cp,
    #     pusher_prop_chord_cp=pusher_prop_chord_cp
    # )
    # trim_at_3g(
    #     pusher_prop_twist_cp=pusher_prop_twist_cp,
    #     pusher_prop_chord_cp=pusher_prop_chord_cp
    # )
    # optimize_lift_rotor_blade(debug_geom_flag=False)
    # trim_at_hover()

    # structural_wingbox_beam_evaluation(pitch_angle=np.deg2rad(12.11391141),
    #                                    visualize_flag=False,
    #                                    sizing_flag=False)
    # structural_wingbox_beam_sizing(pitch_angle=np.deg2rad(12.11391141))
    # structural_wingbox_shell_evaluation(pitch_angle=np.deg2rad(12.48100761), visualize_flag=False)

    pav_visualization()