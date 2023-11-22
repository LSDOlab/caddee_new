import m3l
import csdl
import caddee as cd
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.equations_of_motion_csdl.equations_of_motion_csdl import EulerFlatEarth6DoFGenRef
import copy


class EoMEuler6DOF(m3l.ExplicitOperation):
    
    def initialize(self, kwargs):
        self.parameters.declare('num_nodes', types=int, default=1)
        self.parameters.declare('name', types=str, default='eom_model')
        self._stability_flag = False

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self) -> csdl.Model:
        if self._stability_flag:
            num_nodes = self.parameters['num_nodes'] * 13
        else:
            num_nodes = self.parameters['num_nodes']

        csdl_model = EulerFlatEarth6DoFGenRef(
            num_nodes=num_nodes,
            stability_flag=self._stability_flag,
        )
        
        return  csdl_model

    def evaluate(self, total_mass, total_cg_vector, total_inertia_tensor, 
                 total_forces, total_moments, ac_states, ref_pt=None, stability=False) -> m3l.Variable:
        
        self._stability_flag = stability


        mps_forces = {
            'mass' : total_mass,
            'cg_vector' : total_cg_vector,
            'inertia_tensor' : total_inertia_tensor,
            'total_forces' : total_forces,
            'total_moments' : total_moments,
        }

        if ref_pt:
            mps_forces['ref_pt'] = ref_pt

        ac_states_dict = ac_states.__dict__
        ac_states_copy = copy.deepcopy(ac_states_dict)
        del ac_states_copy['gamma']
        del ac_states_copy['time']
        del ac_states_copy['stability_flag']
        self.arguments = {**mps_forces, **ac_states_copy}

        accelerations = m3l.Variable(name=f'accelerations', shape=(1, ), operation=self)
        lhs_long = m3l.Variable(name=f'lhs_long', shape=(4, ), operation=self)
        long_stab_state_vec = m3l.Variable(name=f'long_stab_state_vec', shape=(4, ), operation=self)
        A_long = m3l.Variable(name=f'A_long', shape=(4, 4), operation=self)

        lhs_lat = m3l.Variable(name=f'lhs_lat', shape=(4, ), operation=self)
        lat_stab_state_vec = m3l.Variable(name=f'lat_stab_state_vec', shape=(4, ), operation=self)
        A_lat = m3l.Variable(name=f'A_lat', shape=(4, 4), operation=self)

        return accelerations, lhs_long, long_stab_state_vec, A_long, lhs_lat, lat_stab_state_vec, A_lat
        
    def compute_derivatives(self): pass
        

# NOTE: for later
class EoMM3LResidual(m3l.ImplicitOperation):
    def initialize(self): pass

    def evaluate_residual(self)-> csdl.Model:
        linmodel = csdl.Model()
        a_mat = linmodel.declare_variable('mp_matrix', shape=(6, 6))
        b_mat = linmodel.declare_variable('rhs', shape=(num_nodes, 6))
        state = linmodel.declare_variable('state', shape=(6, num_nodes))
        residual = csdl.matmat(a_mat, state) - csdl.transpose(b_mat)

        return linmodel
    
    def solve_residual_equations(self)-> csdl.Model: pass

    











if __name__ == "__main__":
    
    pass