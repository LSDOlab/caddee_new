import csdl
from caddee.core.caddee_core.system_model.design_scenario.design_scenario import DesignScenario
from caddee.core.caddee_core.system_representation.system_representation import SystemRepresentation
from caddee.core.caddee_core.system_parameterization.system_parameterization import SystemParameterization
from caddee.core.caddee_core.system_model.sizing_group.sizing_group import SizingGroup
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.design_condition_csdl import DesignConditionCSDL
from caddee.utils.base_model_csdl import BaseModelCSDL
from caddee.utils.csdl.vectorization_csdl import VectorizationCSDL
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.inertial_loads_csdl import InertialLoadsModel
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.loads_csdl.total_forces_moments_csdl import TotalForcesMomentsCSDL
from caddee.core.csdl_core.system_representation_csdl.mesh_evaluation_csdl import MeshEvaluationCSDL
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.varying_mass_properties_csdl import VaryingMassPropertiesCSDL
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.total_mass_properties_csdl import TotalMassPropertiesCSDL
from caddee.core.csdl_core.system_model_csdl.mass_properties_csdl.constant_mass_properties_csdl import ConstantMassPropertiesCSDL


class DesignScenarioCSDL(BaseModelCSDL):
    def initialize(self):
        self.parameters.declare('design_scenario', types=DesignScenario)
        self.parameters.declare('system_config', types=SystemRepresentation, allow_none=False)
        self.parameters.declare('system_param', default=None, types=SystemParameterization,
                                allow_none=True)
        self.parameters.declare('sizing_group', default=None, types=SizingGroup, allow_none=True)
        # establish a pattern where the pure python objects corresponding to 
        # csdl object are declared as parameters (including composition)
        

    def define(self):
        # Parameters
        design_scenario = self.parameters['design_scenario']
        system_config = self.parameters['system_config']
        system_param = self.parameters['system_param']
        sizing_group = self.parameters['sizing_group']
        
        name = design_scenario.parameters['name']
        design_condition_dictionary = design_scenario.design_condition_dictionary
        num_nodes = len(design_condition_dictionary)
        mechanics_group, nonmechanics_group, power_group = design_scenario._assemble_groups(design_condition_dictionary)
        system_model_connections_list = design_scenario._system_model_connections_list
        psa_connections_list = design_scenario._psa_connections_list
        """
        Structure (csdl side)
            1)  DeisgnScenarioCSDL:
                Contains 2a-e and issues connections between intermediate
                variables computed in DesignConditionCSDL and the solvers
            2a) N DesignConditionCSDL: 
                all that is happening here is creating inputs
                and intermediate computations
            2b) MechanicsGroupCSDL:
                On the pure python side, mechanics group is contained
                within design condition, however, on the csdl side of 
                things, we need to vectorize, and therefore, there can 
                only be 1 mechanics group with all the mechanics solvers
            2c) EquationsOfMotionCSDL:
            2d) NonmechanicsGroupCSDL:
            2e) PowerGroupCSDL:
            3)  Connections
        """
        # Loop over all the design conditions and call
        # _assemble_csdl() when adding submodels
        mission_time = self.register_module_output('mission_time', shape=(num_nodes, ), val=0)
        counter = 0
        for name, design_condition in design_condition_dictionary.items():
            
            self.add_module(design_condition._assemble_csdl_modules(), name)
            mission_time[counter] = self.register_module_input(f'{name}_time', shape=(1, ))
            counter += 1
        total_time = csdl.sum(mission_time)
        self.register_module_output('total_time', total_time)

        # if system_model_connections_list:
        #     self.connect_model_vars(system_model_connections_list, name=name)

        # Vectorization
        vectorization_csdl = VectorizationCSDL(
                mechanics_group=mechanics_group,
                nonmechanics_group=nonmechanics_group,
                power_group=power_group,
        )
        self.add_module(vectorization_csdl, 'vectorization')

        # Mesh Evaluation
        if system_param:
            mesh_evaluation_csdl = MeshEvaluationCSDL(
                system_representation=system_config,
                groups=[mechanics_group, nonmechanics_group, power_group]
            )
            self.add(mesh_evaluation_csdl, 'mesh_evaluation')

        # mechanics group
        if mechanics_group:
            self.add_module(mechanics_group._assemble_csdl(), 'mechanics_group', promotes=[])

            if mechanics_group.connections_list:
                self._connect(connections=mechanics_group.connections_list, mech_group=mechanics_group)

            # Connect vectorized mechanics variables to mechanics models
            self.connect_vect_vars(mechanics_group)

            # Add mass properties for mechanics models that compute 
            # mass properties such as strucutural sizing solvers
            if mechanics_group._struct_models:
                self.add_module(VaryingMassPropertiesCSDL(
                    mechanics_group=mechanics_group,
                ), 'varying_mass_properties')
                self.connect_sizing_to_mass_properties(mechanics_group=mechanics_group)


            self.add_module(TotalMassPropertiesCSDL(
                sizing_group=sizing_group,
            ), 'total_mass_properties')

            # Inertial loads model
            # TODO: only add inertial loads if sizing group
            self.add_module(InertialLoadsModel(
                num_nodes=num_nodes,
            ), 'inertial_loads') # NOTE: ok to promote here 

            # Total loads model 
            self.add_module(TotalForcesMomentsCSDL(
                num_nodes=num_nodes,
                mech_group=mechanics_group,
            ), 'total_loads') # NOTE: ok to promote here 
            f_m_models = [model for model in mechanics_group._all_models_names_list if model not in mechanics_group._struct_model_names]
            
            #print(f_m_models)
            #exit()

            for model_name in f_m_models:
                self.connect(f'mechanics_group.{model_name}.F', f'total_loads.{model_name}_F')
                self.connect(f'mechanics_group.{model_name}.M', f'total_loads.{model_name}_M')

            # NOTE: no assemble_csdl for inertial and total loads because there are no 
            # equivalent pure python classes

        # eom
        equations_of_motion_csdl = design_scenario.equations_of_motion_csdl
        if equations_of_motion_csdl is None:
            pass
        else: # TODO: fix snake case (consider having eom exist on the pure python side)
            self.add_module(equations_of_motion_csdl(num_nodes=num_nodes), 'eom')
            # NOTE: ok to promote here

        # nonmechanics group
        if nonmechanics_group:
            self.add(nonmechanics_group._assemble_csdl(), 'nonmechanics_group', promotes=[])


        # TODO: time vector -- Total time computation is done;
        #                   xx power profile (time vector) not done
        # TODO: solvers-- active_nodes vs num_nodes

        # power group
        if power_group:
            self.add_module(power_group._assemble_csdl(), 'power_group', promotes=[])
            self.connect_vect_vars(power_group, psa_connections_list) # TODO: Fix this when commented in 

        # Model-model Connections
        if psa_connections_list:
            self._connect(connections=psa_connections_list, mech_group=mechanics_group,non_mech_group=nonmechanics_group, pwr_group=power_group)


        
