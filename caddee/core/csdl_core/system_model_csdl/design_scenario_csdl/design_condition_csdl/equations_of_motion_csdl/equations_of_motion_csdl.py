import csdl
import numpy as np
from caddee.utils.base_model_csdl import BaseModelCSDL
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL


class EulerFlatEarth6DoFGenRef(BaseModelCSDL):
    eom_model_name = 'EulerEoMGenRefPt'

    def initialize(self):
        self.parameters.declare(name='name', default=self.eom_model_name)
        self.parameters.declare('num_nodes', default=1)


    def define(self):
        num_nodes = self.parameters['num_nodes']

        # region Inputs
        # Reference point
        ref_pt = self.register_module_input(name='ref_pt', shape=(3,), val=np.array([0, 0, 0]), units='m')
        
        # Loads
        F = self.register_module_input('total_forces', shape=(num_nodes, 3))
        M = self.register_module_input('total_moments', shape=(num_nodes, 3))
        
        self.print_var(F)
        self.print_var(M)

        Fx = F[:, 0] #self.register_module_input(name='Fx_total', shape=(num_nodes, 1), units='N')
        Fy = F[:, 1] #self.register_module_input(name='Fy_total', shape=(num_nodes, 1), units='N')
        Fz = F[:, 2] #self.register_module_input(name='Fz_total', shape=(num_nodes, 1), units='N')
        Mx = M[:, 0] #self.register_module_input(name='Mx_total', shape=(num_nodes, 1), units='N*m')
        My = M[:, 1] #self.register_module_input(name='My_total', shape=(num_nodes, 1), units='N*m')
        Mz = M[:, 2] #self.register_module_input(name='Mz_total', shape=(num_nodes, 1), units='N*m')

        #self.print_var(Fy)
        #self.print_var(Fz)

        # Mass properties
        m = self.register_module_input(
            name='total_mass',
            shape=(1,), units='kg')
        inertia_tensor = self.register_module_input(
            name='total_inertia_tensor',
            shape=(3, 3)
        )
        cg_vector = self.register_module_input(
            name='total_cg_vector',
            shape=(3, )
        )

        Ixx = csdl.reshape(inertia_tensor[0, 0], (1, ))
        Iyy = csdl.reshape(inertia_tensor[1, 1], (1, ))
        Izz = csdl.reshape(inertia_tensor[2, 2], (1, ))
        Ixy = csdl.reshape(inertia_tensor[0, 1], (1, ))
        Ixz = csdl.reshape(inertia_tensor[0, 2], (1, ))
        Iyz = csdl.reshape(inertia_tensor[1, 2], (1, ))

        cgx = cg_vector[0]
        cgy = cg_vector[1]
        cgz = cg_vector[2]

        self.print_var(cg_vector)


        # State
        u = self.register_module_input(name='u', shape=(num_nodes, 1), units='m/s')
        v = self.register_module_input(name='v', shape=(num_nodes, 1), units='m/s')
        w = self.register_module_input(name='w', shape=(num_nodes, 1), units='m/s')
        p = self.register_module_input(name='p', shape=(num_nodes, 1), units='rad/s')
        q = self.register_module_input(name='q', shape=(num_nodes, 1), units='rad/s')
        r = self.register_module_input(name='r', shape=(num_nodes, 1), units='rad/s')
        phi = self.register_module_input(name='phi', shape=(num_nodes, 1), units='rad')
        theta = self.register_module_input(name='theta', shape=(num_nodes, 1), units='rad')
        psi = self.register_module_input(name='psi', shape=(num_nodes, 1), units='rad')
        x = self.register_module_input(name='x', shape=(num_nodes, 1), units='m')
        y = self.register_module_input(name='y', shape=(num_nodes, 1), units='m')
        z = self.register_module_input(name='z', shape=(num_nodes, 1), units='m')
        # endregion

      

        Idot = self.create_input(name='Idot', val=0, shape=(3, 3))

        # CG offset from reference point
        Rbcx = cgx - ref_pt[0]
        Rbcy = cgy - ref_pt[1]
        Rbcz = cgz - ref_pt[2]

        xcgdot = self.create_input(name='xcgdot', val=0, shape=(num_nodes, 1))
        ycgdot = self.create_input(name='ycgdot', val=0, shape=(num_nodes, 1))
        zcgdot = self.create_input(name='zcgdot', val=0, shape=(num_nodes, 1))
        xcgddot = self.create_input(name='xcgddot', val=0, shape=(num_nodes, 1))
        ycgddot = self.create_input(name='ycgddot', val=0, shape=(num_nodes, 1))
        zcgddot = self.create_input(name='zcgddot', val=0, shape=(num_nodes, 1))

        mp_matrix = self.register_module_output(name='mp_matrix', val=np.zeros((6, 6)), shape=(6, 6))
        mp_matrix[0, 0] = csdl.expand(m, (1, 1))
        mp_matrix[0, 4] = csdl.expand(m * Rbcz, (1, 1))
        mp_matrix[0, 5] = csdl.expand(-m * Rbcy, (1, 1))
        mp_matrix[1, 1] = csdl.expand(m, (1, 1))
        mp_matrix[1, 3] = csdl.expand(-m * Rbcz, (1, 1))
        mp_matrix[1, 5] = csdl.expand(m * Rbcx, (1, 1))
        mp_matrix[2, 2] = csdl.expand(m, (1, 1))
        mp_matrix[2, 3] = csdl.expand(m * Rbcy, (1, 1))
        mp_matrix[2, 4] = csdl.expand(-m * Rbcx, (1, 1))
        mp_matrix[3, 1] = csdl.expand(-m * Rbcz, (1, 1))
        mp_matrix[3, 2] = csdl.expand(m * Rbcy, (1, 1))
        mp_matrix[3, 3] = csdl.expand(Ixx, (1, 1))
        mp_matrix[3, 4] = csdl.expand(Ixy, (1, 1))
        mp_matrix[3, 5] = csdl.expand(Ixz, (1, 1))
        mp_matrix[4, 0] = csdl.expand(m * Rbcz, (1, 1))
        mp_matrix[4, 2] = csdl.expand(-m * Rbcx, (1, 1))
        mp_matrix[4, 3] = csdl.expand(Ixy, (1, 1))
        mp_matrix[4, 4] = csdl.expand(Iyy, (1, 1))
        mp_matrix[4, 5] = csdl.expand(Iyz, (1, 1))
        mp_matrix[5, 0] = csdl.expand(-m * Rbcy, (1, 1))
        mp_matrix[5, 1] = csdl.expand(m * Rbcx, (1, 1))
        mp_matrix[5, 3] = csdl.expand(Ixz, (1, 1))
        mp_matrix[5, 4] = csdl.expand(Iyz, (1, 1))
        mp_matrix[5, 5] = csdl.expand(Izz, (1, 1))

        lambdax = Fx + csdl.expand(m, (num_nodes, 1)) * (r * v - q * w - xcgdot - 2 * q * zcgdot
                                                         + 2 * r * ycgdot + csdl.expand(Rbcx, (num_nodes, 1)) * (
                                                                     q ** 2 + r ** 2)
                                                         - csdl.expand(Rbcy, (num_nodes, 1)) * p * q
                                                         - csdl.expand(Rbcz, (num_nodes, 1)) * p * r) + x * 0

        lambday = Fy + csdl.expand(m, (num_nodes, 1)) * (p * w - r * u - ycgddot
                                                         - 2 * r * xcgdot + 2 * p * zcgdot
                                                         - csdl.expand(Rbcx, (num_nodes, 1)) * p * q
                                                         + csdl.expand(Rbcy, (num_nodes, 1)) * (p ** 2 + r ** 2)
                                                         - csdl.expand(Rbcz, (num_nodes, 1)) * q * r) + y * 0

        lambdaz = Fz + csdl.expand(m, (num_nodes, 1)) * (q * u - p * v - zcgddot
                                                         - 2 * p * ycgdot + 2 * q * xcgdot
                                                         - csdl.expand(Rbcx, (num_nodes, 1)) * p * r
                                                         - csdl.expand(Rbcy, (num_nodes, 1)) * q * r
                                                         + csdl.expand(Rbcz, (num_nodes, 1)) * (p ** 2 + q ** 2)) + z * 0

        angvel_vec = self.register_module_output(name='angvel_vec', shape=(num_nodes, 3))
        angvel_vec[:, 0] = 1 * p
        angvel_vec[:, 1] = 1 * q
        angvel_vec[:, 2] = 1 * r

        angvel_ssym = self.register_module_output(name='angvel_ssym', val=np.zeros((num_nodes, 3, 3)), shape=(num_nodes, 3, 3))
        angvel_ssym[:, 0, 1] = csdl.expand(-r, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 0, 2] = csdl.expand(q, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 1, 0] = csdl.expand(r, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 1, 2] = csdl.expand(-p, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 2, 0] = csdl.expand(-q, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 2, 1] = csdl.expand(p, (num_nodes, 1, 1), 'ij->ija')

        Rbc_ssym = self.register_module_output(name='Rbc_ssym', val=np.zeros((num_nodes, 3, 3)), shape=(num_nodes, 3, 3))
        Rbc_ssym[:, 0, 1] = csdl.expand(-Rbcz, (num_nodes, 1, 1))
        Rbc_ssym[:, 0, 2] = csdl.expand(Rbcy, (num_nodes, 1, 1))
        Rbc_ssym[:, 1, 0] = csdl.expand(Rbcz, (num_nodes, 1, 1))
        Rbc_ssym[:, 1, 2] = csdl.expand(-Rbcx, (num_nodes, 1, 1))
        Rbc_ssym[:, 2, 0] = csdl.expand(-Rbcy, (num_nodes, 1, 1))
        Rbc_ssym[:, 2, 1] = csdl.expand(Rbcx, (num_nodes, 1, 1))

        moment_vec = self.register_module_output(name='moment_vec', shape=(num_nodes, 3))
        moment_vec[:, 0] = 1 * Mx
        moment_vec[:, 1] = 1 * My
        moment_vec[:, 2] = 1 * Mz

        I = self.register_module_output(name='I', val=np.zeros((3, 3)), shape=(3, 3))
        I[0, 0] = csdl.expand(1 * Ixx, (1, 1))
        I[0, 1] = csdl.expand(1 * Ixy, (1, 1))
        I[0, 2] = csdl.expand(1 * Ixz, (1, 1))
        I[1, 0] = csdl.expand(1 * Ixy, (1, 1))
        I[1, 1] = csdl.expand(1 * Iyy, (1, 1))
        I[1, 2] = csdl.expand(1 * Iyz, (1, 1))
        I[2, 0] = csdl.expand(1 * Ixz, (1, 1))
        I[2, 1] = csdl.expand(1 * Iyz, (1, 1))
        I[2, 2] = csdl.expand(1 * Izz, (1, 1))

        mu_vec = self.register_module_output(name='mu_vec', shape=(num_nodes, 3))
        store_vars = []
        for i in range(num_nodes):
            t1 = csdl.matmat(angvel_vec[i, :], Idot)
            angvel_ssym_2d = csdl.reshape(angvel_ssym[i, :, :], new_shape=(3, 3))
            # print(np.shape(angvel_ssym_2d))
            var1 = csdl.matmat(angvel_ssym_2d, I)
            # print(np.shape(var1))
            var2 = csdl.matmat(angvel_vec[i, :], var1)
            # print(np.shape(var2))
            Rbc_ssym_2d = csdl.reshape(Rbc_ssym[i, :, :], new_shape=(3, 3))
            var3 = csdl.matmat(angvel_ssym_2d, Rbc_ssym_2d)
            # print(np.shape(var3))
            var4 = csdl.matmat(angvel_vec[i, :], var3)
            # print(np.shape(var4))
            m_ex = csdl.expand(m, (1, 3))
            var5 = m_ex * var4
            # print(np.shape(var5))
            store_vars.append(t1)
            store_vars.append(angvel_ssym_2d)
            store_vars.append(var1)
            store_vars.append(var2)
            store_vars.append(Rbc_ssym_2d)
            store_vars.append(var3)
            store_vars.append(var4)
            store_vars.append(m_ex)
            store_vars.append(var5)
            mu_vec[i, :] = moment_vec[i, :] - t1 - var2 - var5

        rhs = self.register_module_output(name='rhs', shape=(num_nodes, 6))
        rhs[:, 0] = 1 * lambdax
        rhs[:, 1] = 1 * lambday
        rhs[:, 2] = 1 * lambdaz
        rhs[:, 3] = mu_vec[:, 0]
        rhs[:, 4] = mu_vec[:, 1]
        rhs[:, 5] = mu_vec[:, 2]

        # custom implicit operation
        # solve the system: accelerations = np.linalg.solve(mp_matrix, rhs)
        linmodel = ModuleCSDL()
        a_mat = linmodel.register_module_input('mp_matrix', shape=(6, 6))
        b_mat = linmodel.register_module_input('rhs', shape=(num_nodes, 6))
        state = linmodel.register_module_input('state', shape=(6, num_nodes))
        residual = csdl.matmat(a_mat, state) - csdl.transpose(b_mat)
        linmodel.register_module_output('residual', residual)
        solve_linear = self.create_implicit_operation(linmodel)
        solve_linear.declare_state('state', residual='residual')
        solve_linear.nonlinear_solver = csdl.NewtonSolver(
            solve_subsystems=False,
            atol=1E-8,
            iprint=False,
        )
        solve_linear.linear_solver = csdl.ScipyKrylov()
        a_mat = self.register_module_input('mp_matrix', shape=(6, 6))
        b_mat = self.register_module_input('rhs', shape=(num_nodes, 6))
        accelerations = csdl.transpose(solve_linear(a_mat, b_mat))
        self.add_module(linmodel, 'eom_implicit_module')
        # end custom implicit op

        du_dt = accelerations[:, 0]
        dv_dt = accelerations[:, 1]
        dw_dt = accelerations[:, 2]
        dp_dt = accelerations[:, 3]
        dq_dt = accelerations[:, 4]
        dr_dt = accelerations[:, 5]

        dphi_dt = p + q * csdl.sin(phi) * csdl.tan(theta) + r * csdl.cos(phi) * csdl.tan(theta)
        dtheta_dt = q * csdl.cos(phi) - r * csdl.sin(phi)
        dpsi_dt = q * csdl.sin(phi) / csdl.cos(theta) + r * csdl.cos(phi) / csdl.cos(theta)
        dx_dt = u * csdl.cos(theta) * csdl.cos(psi) \
                + v * (csdl.sin(phi) * csdl.sin(theta) * csdl.cos(psi) - csdl.cos(phi) * csdl.sin(psi)) \
                + w * (csdl.cos(phi) * csdl.sin(theta) * csdl.cos(psi) + csdl.sin(phi) * csdl.sin(psi))
        dy_dt = u * csdl.cos(theta) * csdl.sin(psi) \
                + v * (csdl.sin(phi) * csdl.sin(theta) * csdl.sin(psi) + csdl.cos(phi) * csdl.cos(psi)) \
                + w * (csdl.cos(phi) * csdl.sin(theta) * csdl.sin(psi) - csdl.sin(phi) * csdl.cos(psi))
        dz_dt = -u * csdl.sin(theta) + v * csdl.sin(phi) * csdl.cos(theta) + w * csdl.cos(phi) * csdl.cos(theta)

        # region Outputs
        res_vector = self.register_module_output(name='eom_residual', shape=(num_nodes, 12))
        res_vector[:, 0] = du_dt
        res_vector[:, 1] = dv_dt
        res_vector[:, 2] = dw_dt
        res_vector[:, 3] = dp_dt
        res_vector[:, 4] = dq_dt
        res_vector[:, 5] = dr_dt
        res_vector[:, 6] = dphi_dt
        res_vector[:, 7] = dtheta_dt
        res_vector[:, 8] = dpsi_dt
        res_vector[:, 9] = dx_dt
        res_vector[:, 10] = dy_dt
        res_vector[:, 11] = dz_dt

        xddot = self.register_module_output(name='xddot', shape=(num_nodes, 6))
        xddot[:, 0] = du_dt
        xddot[:, 1] = dv_dt
        xddot[:, 2] = dw_dt
        xddot[:, 3] = dp_dt
        xddot[:, 4] = dq_dt
        xddot[:, 5] = dr_dt

        # obj_r = csdl.pnorm(var=xddot, axis=1) 
        obj_r = csdl.pnorm(csdl.pnorm(var=xddot, axis=1))
        self.print_var(obj_r)
        self.register_module_output(name='trim_residual', var=obj_r)
        # endregion
