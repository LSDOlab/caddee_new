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

        ac_states_copy = {}
        ac_states_copy['u'] = ac_states.u
        ac_states_copy['v'] = ac_states.v
        ac_states_copy['w'] = ac_states.w
        ac_states_copy['p'] = ac_states.p
        ac_states_copy['q'] = ac_states.q
        ac_states_copy['r'] = ac_states.r
        ac_states_copy['phi'] = ac_states.phi
        ac_states_copy['theta'] = ac_states.theta
        ac_states_copy['psi'] = ac_states.psi
        ac_states_copy['x'] = ac_states.x
        ac_states_copy['y'] = ac_states.y
        ac_states_copy['z'] = ac_states.z
        self.arguments = {**mps_forces, **ac_states_copy}

        du_dt = m3l.Variable(name=f'du_dt', shape=(1, ), operation=self)
        dv_dt = m3l.Variable(name=f'dv_dt', shape=(1, ), operation=self)
        dw_dt = m3l.Variable(name=f'dw_dt', shape=(1, ), operation=self)
        dp_dt = m3l.Variable(name=f'dp_dt', shape=(1, ), operation=self)
        dq_dt = m3l.Variable(name=f'dq_dt', shape=(1, ), operation=self)
        dr_dt = m3l.Variable(name=f'dr_dt', shape=(1, ), operation=self)
        accelerations = m3l.Variable(name=f'accelerations', shape=(1, ), operation=self)
        lhs_long = m3l.Variable(name=f'lhs_long', shape=(4, ), operation=self)
        long_stab_state_vec = m3l.Variable(name=f'long_stab_state_vec', shape=(4, ), operation=self)
        A_long = m3l.Variable(name=f'A_long', shape=(4, 4), operation=self)

        lhs_lat = m3l.Variable(name=f'lhs_lat', shape=(4, ), operation=self)
        lat_stab_state_vec = m3l.Variable(name=f'lat_stab_state_vec', shape=(4, ), operation=self)
        A_lat = m3l.Variable(name=f'A_lat', shape=(4, 4), operation=self)

        return accelerations, lhs_long, long_stab_state_vec, A_long, lhs_lat, lat_stab_state_vec, A_lat, du_dt, dv_dt, dw_dt, dp_dt, dq_dt, dr_dt
        
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