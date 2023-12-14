import csdl
import numpy as np



class EulerFlatEarth6DoFGenRef(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('stability_flag', types=bool, default=False)


    def define(self):
        num_nodes = self.parameters['num_nodes']
        stability_flag = self.parameters['stability_flag']

        # region Inputs
        # Reference point
        ref_pt = self.declare_variable(name='ref_pt', shape=(3,), val=np.array([0, 0, 0]), units='m')
        # # self.print_var(ref_pt)
        
        # Loads
        F = self.declare_variable('total_forces', shape=(num_nodes, 3))
        M = self.declare_variable('total_moments', shape=(num_nodes, 3))
        
        # self.print_var(F)
        # self.print_var(M)

        Fx = F[:, 0] #self.declare_variable(name='Fx_total', shape=(num_nodes, 1), units='N')
        Fy = F[:, 1] #self.declare_variable(name='Fy_total', shape=(num_nodes, 1), units='N')
        Fz = F[:, 2] #self.declare_variable(name='Fz_total', shape=(num_nodes, 1), units='N')
        Mx = M[:, 0] #self.declare_variable(name='Mx_total', shape=(num_nodes, 1), units='N*m')
        My = M[:, 1] #self.declare_variable(name='My_total', shape=(num_nodes, 1), units='N*m')
        Mz = M[:, 2] #self.declare_variable(name='Mz_total', shape=(num_nodes, 1), units='N*m')

        ## self.print_var(Fy)
        ## self.print_var(Fz)

        # Mass properties
        m = self.declare_variable(
            name='mass',
            shape=(1,), units='kg')
        inertia_tensor = self.declare_variable(
            name='inertia_tensor',
            shape=(3, 3)
        )
        cg_vector = self.declare_variable(
            name='cg_vector',
            shape=(3, )
        )
        # # self.print_var(cg_vector)

        Ixx = csdl.reshape(inertia_tensor[0, 0], (1, ))
        Iyy = csdl.reshape(inertia_tensor[1, 1], (1, ))
        Izz = csdl.reshape(inertia_tensor[2, 2], (1, ))
        Ixy = csdl.reshape(inertia_tensor[0, 1], (1, ))
        Ixz = csdl.reshape(inertia_tensor[0, 2], (1, ))
        Iyz = csdl.reshape(inertia_tensor[1, 2], (1, ))

        cgx = cg_vector[0]
        cgy = cg_vector[1]
        cgz = cg_vector[2]

        # # self.print_var(cg_vector)


        # State
        u = self.declare_variable(name='u', shape=(num_nodes, 1), units='m/s')
        v = self.declare_variable(name='v', shape=(num_nodes, 1), units='m/s')
        w = self.declare_variable(name='w', shape=(num_nodes, 1), units='m/s')
        p = self.declare_variable(name='p', shape=(num_nodes, 1), units='rad/s')
        q = self.declare_variable(name='q', shape=(num_nodes, 1), units='rad/s')
        r = self.declare_variable(name='r', shape=(num_nodes, 1), units='rad/s')
        phi = self.declare_variable(name='phi', shape=(num_nodes, 1), units='rad')
        theta = self.declare_variable(name='theta', shape=(num_nodes, 1), units='rad')
        psi = self.declare_variable(name='psi', shape=(num_nodes, 1), units='rad')
        x = self.declare_variable(name='x', shape=(num_nodes, 1), units='m')
        y = self.declare_variable(name='y', shape=(num_nodes, 1), units='m')
        z = self.declare_variable(name='z', shape=(num_nodes, 1), units='m')
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

        mp_matrix = self.create_output(name='mp_matrix', val=np.zeros((6, 6)), shape=(6, 6))
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

        angvel_vec = self.create_output(name='angvel_vec', shape=(num_nodes, 3))
        angvel_vec[:, 0] = 1 * p
        angvel_vec[:, 1] = 1 * q
        angvel_vec[:, 2] = 1 * r

        angvel_ssym = self.create_output(name='angvel_ssym', val=np.zeros((num_nodes, 3, 3)), shape=(num_nodes, 3, 3))
        angvel_ssym[:, 0, 1] = csdl.expand(-r, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 0, 2] = csdl.expand(q, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 1, 0] = csdl.expand(r, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 1, 2] = csdl.expand(-p, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 2, 0] = csdl.expand(-q, (num_nodes, 1, 1), 'ij->ija')
        angvel_ssym[:, 2, 1] = csdl.expand(p, (num_nodes, 1, 1), 'ij->ija')

        Rbc_ssym = self.create_output(name='Rbc_ssym', val=np.zeros((num_nodes, 3, 3)), shape=(num_nodes, 3, 3))
        Rbc_ssym[:, 0, 1] = csdl.expand(-Rbcz, (num_nodes, 1, 1))
        Rbc_ssym[:, 0, 2] = csdl.expand(Rbcy, (num_nodes, 1, 1))
        Rbc_ssym[:, 1, 0] = csdl.expand(Rbcz, (num_nodes, 1, 1))
        Rbc_ssym[:, 1, 2] = csdl.expand(-Rbcx, (num_nodes, 1, 1))
        Rbc_ssym[:, 2, 0] = csdl.expand(-Rbcy, (num_nodes, 1, 1))
        Rbc_ssym[:, 2, 1] = csdl.expand(Rbcx, (num_nodes, 1, 1))

        moment_vec = self.create_output(name='moment_vec', shape=(num_nodes, 3))
        moment_vec[:, 0] = 1 * Mx
        moment_vec[:, 1] = 1 * My
        moment_vec[:, 2] = 1 * Mz

        I = self.create_output(name='I', val=np.zeros((3, 3)), shape=(3, 3))
        I[0, 0] = csdl.expand(1 * Ixx, (1, 1))
        I[0, 1] = csdl.expand(1 * Ixy, (1, 1))
        I[0, 2] = csdl.expand(1 * Ixz, (1, 1))
        I[1, 0] = csdl.expand(1 * Ixy, (1, 1))
        I[1, 1] = csdl.expand(1 * Iyy, (1, 1))
        I[1, 2] = csdl.expand(1 * Iyz, (1, 1))
        I[2, 0] = csdl.expand(1 * Ixz, (1, 1))
        I[2, 1] = csdl.expand(1 * Iyz, (1, 1))
        I[2, 2] = csdl.expand(1 * Izz, (1, 1))

        mu_vec = self.create_output(name='mu_vec', shape=(num_nodes, 3))
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

        rhs = self.create_output(name='rhs', shape=(num_nodes, 6))
        rhs[:, 0] = 1 * lambdax
        rhs[:, 1] = 1 * lambday
        rhs[:, 2] = 1 * lambdaz
        rhs[:, 3] = mu_vec[:, 0]
        rhs[:, 4] = mu_vec[:, 1]
        rhs[:, 5] = mu_vec[:, 2]


        eom_solve_model = EoMSolveModel(
            num_nodes=num_nodes,
            stability_flag=stability_flag,
        
        )
        self.add(eom_solve_model, 'eom_solve_model')

        # custom implicit operation
        # solve the system: accelerations = np.linalg.solve(mp_matrix, rhs)

class EoMSolveModel(csdl.Model):
    def initialize(self):
        self.parameters.declare('num_nodes', types=int)
        self.parameters.declare('stability_flag', types=bool)

    def define(self):
        num_nodes = self.parameters['num_nodes']
        stability_flag = self.parameters['stability_flag']
        
        linmodel = csdl.Model()
        a_mat = linmodel.declare_variable('mp_matrix', shape=(6, 6))
        b_mat = linmodel.declare_variable('rhs', shape=(num_nodes, 6))
        state = linmodel.declare_variable('state', shape=(6, num_nodes))
        residual = csdl.matmat(a_mat, state) - csdl.transpose(b_mat)
        linmodel.register_output('residual', residual)
        
        solve_linear = self.create_implicit_operation(linmodel)
        solve_linear.declare_state('state', residual='residual')
        solve_linear.nonlinear_solver = csdl.NewtonSolver(
            solve_subsystems=False,
            atol=1E-8,
            iprint=False,
        )
        solve_linear.linear_solver = csdl.DirectSolver()
        a_mat = self.declare_variable('mp_matrix', shape=(6, 6))
        b_mat = self.declare_variable('rhs', shape=(num_nodes, 6))
        accelerations = csdl.transpose(solve_linear(a_mat, b_mat))
        self.add(linmodel, 'eom_implicit_module')
        # end custom implicit op

        du_dt = accelerations[:, 0]
        dv_dt = accelerations[:, 1]
        dw_dt = accelerations[:, 2]
        dp_dt = accelerations[:, 3]
        dq_dt = accelerations[:, 4]
        dr_dt = accelerations[:, 5]

        u = self.declare_variable(name='u', shape=(num_nodes, 1), units='m/s')
        v = self.declare_variable(name='v', shape=(num_nodes, 1), units='m/s')
        w = self.declare_variable(name='w', shape=(num_nodes, 1), units='m/s')
        p = self.declare_variable(name='p', shape=(num_nodes, 1), units='rad/s')
        q = self.declare_variable(name='q', shape=(num_nodes, 1), units='rad/s')
        r = self.declare_variable(name='r', shape=(num_nodes, 1), units='rad/s')
        phi = self.declare_variable(name='phi', shape=(num_nodes, 1), units='rad')
        theta = self.declare_variable(name='theta', shape=(num_nodes, 1), units='rad')
        psi = self.declare_variable(name='psi', shape=(num_nodes, 1), units='rad')

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
        acc_vector = self.create_output(name='total_accelerations', shape=(num_nodes, 12))
        acc_vector[:, 0] = du_dt
        acc_vector[:, 1] = dv_dt
        acc_vector[:, 2] = dw_dt
        acc_vector[:, 3] = dp_dt
        acc_vector[:, 4] = dq_dt
        acc_vector[:, 5] = dr_dt
        acc_vector[:, 6] = dphi_dt
        acc_vector[:, 7] = dtheta_dt
        acc_vector[:, 8] = dpsi_dt
        acc_vector[:, 9] = dx_dt
        acc_vector[:, 10] = dy_dt
        acc_vector[:, 11] = dz_dt
        # self.print_var(acc_vector)

        self.register_output('du_dt', du_dt*1)
        self.register_output('dv_dt', dv_dt*1)
        self.register_output('dw_dt', dw_dt*1)
        self.register_output('dp_dt', dp_dt*1)
        self.register_output('dq_dt', dq_dt*1)
        self.register_output('dr_dt', dr_dt*1)

        # # self.print_var(acc_vector)

        if stability_flag:
            # unperturbed_accelerations = csdl.transpose(acc_vector[0, :])
            unperturbed_accelerations = acc_vector[0, :]
            stab_der_mat = self.create_output('A_stab_transpose', shape=(12, 12), val=0)

            # stab_der_mat[:, 0] = (acc_vector[1:, 0] - unperturbed_accelerations) / 0.05 # Column 1 of stability derivative matrix [dr1/du, dr2/du, ...]^T
            # stab_der_mat[:, 1] = (acc_vector[1:, 1] - unperturbed_accelerations) / 0.05 # Column 2 of stability derivative matrix [dr1/dv, dr2/dv, ...]^T
            # stab_der_mat[:, 2] = (acc_vector[1:, 2] - unperturbed_accelerations) / 0.05 # Column 3 of stability derivative matrix [dr1/dw, dr2/dw, ...]^T
            # stab_der_mat[:, 3] = (acc_vector[1:, 3] - unperturbed_accelerations) / np.deg2rad(0.1) # Column 4 of stability derivative matrix [dr1/dp, dr2/dp, ...]^T
            # stab_der_mat[:, 4] = (acc_vector[1:, 4] - unperturbed_accelerations) / np.deg2rad(0.1) # Column 5 of stability derivative matrix [dr1/dq, dr2/dq, ...]^T
            # stab_der_mat[:, 5] = (acc_vector[1:, 5] - unperturbed_accelerations) / np.deg2rad(0.1) # Column 6 of stability derivative matrix [dr1/dr, dr2/dvr, ...]^T
            # stab_der_mat[:, 6] = (acc_vector[1:, 6] - unperturbed_accelerations) / np.deg2rad(0.1) # Column 7 of stability derivative matrix [dr1/dphi, dr2/dphi, ...]^T
            # stab_der_mat[:, 7] = (acc_vector[1:, 7] - unperturbed_accelerations) / np.deg2rad(0.1) # Column 8 of stability derivative matrix [dr1/dtheta, dr2/dtheta, ...]^T
            # stab_der_mat[:, 8] = (acc_vector[1:, 8] - unperturbed_accelerations) / np.deg2rad(0.1) # Column 9 of stability derivative matrix [dr1/dpsi, dr2/dpsi, ...]^T
            # stab_der_mat[:, 9] = (acc_vector[1:, 9] - unperturbed_accelerations) / 0.5 # Column 10 of stability derivative matrix [dr1/dx, dr2/dx, ...]^T
            # stab_der_mat[:, 10] = (acc_vector[1:, 10] - unperturbed_accelerations) / 0.5 # Column 11 of stability derivative matrix [dr1/dy, dr2/dy, ...]^T
            # stab_der_mat[:, 11] = (acc_vector[1:, 11] - unperturbed_accelerations) / 0.5 # Column 12 of stability derivative matrix [dr1/dz, dr2/dz, ...]^T

            # Forward difference
            stab_der_mat[0, :] = (acc_vector[1, :] - unperturbed_accelerations) / 0.25 # Column 1 of stability derivative matrix [dr1/du, dr2/du, ...]^T
            stab_der_mat[1, :] = (acc_vector[2, :] - unperturbed_accelerations) / 0.25# Column 2 of stability derivative matrix [dr1/dv, dr2/dv, ...]^T
            stab_der_mat[2, :] = (acc_vector[3, :] - unperturbed_accelerations) / 0.25# Column 3 of stability derivative matrix [dr1/dw, dr2/dw, ...]^T
            stab_der_mat[3, :] = (acc_vector[4, :] - unperturbed_accelerations) / np.deg2rad(0.5) # Column 4 of stability derivative matrix [dr1/dp, dr2/dp, ...]^T
            stab_der_mat[4, :] = (acc_vector[5, :] - unperturbed_accelerations) / np.deg2rad(0.5) # Column 5 of stability derivative matrix [dr1/dq, dr2/dq, ...]^T
            stab_der_mat[5, :] = (acc_vector[6, :] - unperturbed_accelerations) / np.deg2rad(0.5) # Column 6 of stability derivative matrix [dr1/dr, dr2/dvr, ...]^T
            stab_der_mat[6, :] = (acc_vector[7, :] - unperturbed_accelerations) / np.deg2rad(0.5) # Column 7 of stability derivative matrix [dr1/dphi, dr2/dphi, ...]^T
            stab_der_mat[7, :] = (acc_vector[8, :] - unperturbed_accelerations) / np.deg2rad(0.5) # Column 8 of stability derivative matrix [dr1/dtheta, dr2/dtheta, ...]^T
            stab_der_mat[8, :] = (acc_vector[9, :] - unperturbed_accelerations) / np.deg2rad(1) # Column 9 of stability derivative matrix [dr1/dpsi, dr2/dpsi, ...]^T
            stab_der_mat[9, :] = (acc_vector[10, :] - unperturbed_accelerations) / 0.25 # Column 10 of stability derivative matrix [dr1/dx, dr2/dx, ...]^T
            stab_der_mat[10, :] = (acc_vector[11, :] - unperturbed_accelerations) / 0.25 # Column 11 of stability derivative matrix [dr1/dy, dr2/dy, ...]^T
            stab_der_mat[11, :] = (acc_vector[12, :] - unperturbed_accelerations) / 0.25 # Column 12 of stability derivative matrix [dr1/dz, dr2/dz, ...]^T

            # Backward Difference
            # stab_der_mat[0, :] = (unperturbed_accelerations -acc_vector[1, :]) / 0.5 # Column 1 of stability derivative matrix [dr1/du, dr2/du, ...]^T
            # stab_der_mat[1, :] = (unperturbed_accelerations -acc_vector[2, :]) / 0.5# Column 2 of stability derivative matrix [dr1/dv, dr2/dv, ...]^T
            # stab_der_mat[2, :] = (unperturbed_accelerations -acc_vector[3, :]) / 0.5# Column 3 of stability derivative matrix [dr1/dw, dr2/dw, ...]^T
            # stab_der_mat[3, :] = (unperturbed_accelerations -acc_vector[4, :]) / np.deg2rad(1) # Column 4 of stability derivative matrix [dr1/dp, dr2/dp, ...]^T
            # stab_der_mat[4, :] = (unperturbed_accelerations -acc_vector[5, :]) / np.deg2rad(1) # Column 5 of stability derivative matrix [dr1/dq, dr2/dq, ...]^T
            # stab_der_mat[5, :] = (unperturbed_accelerations -acc_vector[6, :]) / np.deg2rad(1) # Column 6 of stability derivative matrix [dr1/dr, dr2/dvr, ...]^T
            # stab_der_mat[6, :] = (unperturbed_accelerations -acc_vector[7, :]) / np.deg2rad(1) # Column 7 of stability derivative matrix [dr1/dphi, dr2/dphi, ...]^T
            # stab_der_mat[7, :] = (unperturbed_accelerations -acc_vector[8, :]) / np.deg2rad(1) # Column 8 of stability derivative matrix [dr1/dtheta, dr2/dtheta, ...]^T
            # stab_der_mat[8, :] = (unperturbed_accelerations -acc_vector[9, :]) / np.deg2rad(1) # Column 9 of stability derivative matrix [dr1/dpsi, dr2/dpsi, ...]^T
            # stab_der_mat[9, :] = (unperturbed_accelerations -acc_vector[10, :]) / 0.5 # Column 10 of stability derivative matrix [dr1/dx, dr2/dx, ...]^T
            # stab_der_mat[10, :] = (unperturbed_accelerations -acc_vector[11, :]) / 0.5 # Column 11 of stability derivative matrix [dr1/dy, dr2/dy, ...]^T
            # stab_der_mat[11, :] = (unperturbed_accelerations -acc_vector[12, :]) / 0.5 # Column 12 of stability derivative matrix [dr1/dz, dr2/dz, ...]^T


            A_stab = csdl.transpose(stab_der_mat)
            self.register_output('A_stab', A_stab)
            # self.print_var(A_stab)
            # Longitudinal stability matrix
            A_long = self.create_output(name='A_long', shape=(4, 4), val=0)
        
            A_long[0, 0] = A_stab[0, 0]
            A_long[0, 1] = A_stab[0, 2] * -1
            A_long[0, 2] = A_stab[0, 4] 
            A_long[0, 3] = A_stab[0, 7] 

            A_long[1, 0] = A_stab[2, 0]
            A_long[1, 1] = A_stab[2, 2] * -1
            A_long[1, 2] = A_stab[2, 4] 
            A_long[1, 3] = A_stab[2, 7]

            A_long[2, 0] = A_stab[4, 0]
            A_long[2, 1] = A_stab[4, 2] * -1
            A_long[2, 2] = A_stab[4, 4]
            A_long[2, 3] = A_stab[4, 7]

            A_long[3, 2] = self.create_input('A_long_1', shape=(1, 1), val=1)

            # Assembling stability residual for longitudinal stability 
            lhs_long = self.create_output('lhs_long', shape=(4, ), val=0)
            lhs_long[0] = csdl.reshape(du_dt[0, 0], new_shape=(1, ))
            lhs_long[1] = csdl.reshape(dw_dt[0, 0], new_shape=(1, ))
            lhs_long[2] = csdl.reshape(dq_dt[0, 0], new_shape=(1, ))
            lhs_long[3] = csdl.reshape(dtheta_dt[0, 0], new_shape=(1, ))

            long_stab_state_vec = self.create_output('long_stab_state_vec', shape=(4, ), val=0)
            long_stab_state_vec[0] = csdl.reshape(u[0, 0], new_shape=(1, ))
            long_stab_state_vec[1] = csdl.reshape(w[0, 0], new_shape=(1, ))
            long_stab_state_vec[2] = csdl.reshape(q[0, 0], new_shape=(1, ))
            long_stab_state_vec[3] = csdl.reshape(theta[0, 0], new_shape=(1, ))
            
            # Lateral stability matrix
            A_lat = self.create_output(name='A_lat', shape=(4, 4), val=0)
        
            A_lat[0, 0] = A_stab[1, 1]
            A_lat[0, 1] = A_stab[1, 3] 
            A_lat[0, 2] = A_stab[1, 5]
            A_lat[0, 3] = A_stab[1, 6]

            A_lat[1, 0] = A_stab[3, 1]
            A_lat[1, 1] = A_stab[3, 3]
            A_lat[1, 2] = A_stab[3, 5]
            A_lat[1, 3] = A_stab[3, 6]

            A_lat[2, 0] = A_stab[5, 1]
            A_lat[2, 1] = A_stab[5, 3]
            A_lat[2, 2] = A_stab[5, 5]
            A_lat[2, 3] = A_stab[5, 6]

            A_lat[3, 1] = self.create_input('A_lat_1', shape=(1, 1), val=1)
            
            # Assembling stability residual for lateral stability 
            lhs_lat = self.create_output('lhs_lat', shape=(4, ), val=0)
            lhs_lat[0] = csdl.reshape(dv_dt[0, 0], new_shape=(1, ))
            lhs_lat[1] = csdl.reshape(dp_dt[0, 0], new_shape=(1, ))
            lhs_lat[2] = csdl.reshape(dr_dt[0, 0], new_shape=(1, ))
            lhs_lat[3] = csdl.reshape(dphi_dt[0, 0], new_shape=(1, ))

            lat_stab_state_vec = self.create_output('lat_stab_state_vec', shape=(4, ), val=0)
            lat_stab_state_vec[0] = csdl.reshape(v[0, 0], new_shape=(1, ))
            lat_stab_state_vec[1] = csdl.reshape(p[0, 0], new_shape=(1, ))
            lat_stab_state_vec[2] = csdl.reshape(r[0, 0], new_shape=(1, ))
            lat_stab_state_vec[3] = csdl.reshape(phi[0, 0], new_shape=(1, ))
            
            lat_stab_residual = lhs_lat - csdl.matvec(A_lat, lat_stab_state_vec)
            self.register_output('lat_stab_residual', lat_stab_residual)


            # Assembling accelerations for steady analysis
            xddot = self.create_output(name='xddot', shape=(num_nodes, 6))
            xddot[:, 0] = du_dt
            xddot[:, 1] = dv_dt
            xddot[:, 2] = dw_dt
            xddot[:, 3] = dp_dt
            xddot[:, 4] = dq_dt
            xddot[:, 5] = dr_dt

            # obj_r = csdl.pnorm(var=xddot, axis=1) 
            obj_r = csdl.pnorm(csdl.pnorm(var=xddot[0, :], axis=1))
            self.print_var(obj_r)
            self.register_output(name='accelerations', var=obj_r)

        else:
            xddot = self.create_output(name='xddot', shape=(num_nodes, 6))
            xddot[:, 0] = du_dt
            xddot[:, 1] = dv_dt
            xddot[:, 2] = dw_dt
            xddot[:, 3] = dp_dt
            xddot[:, 4] = dq_dt
            xddot[:, 5] = dr_dt
            
            # obj_r = csdl.pnorm(var=xddot, axis=1) 
            obj_r = csdl.pnorm(csdl.pnorm(var=xddot, axis=1))
            self.print_var(obj_r)
            self.register_output(name='accelerations', var=obj_r)
        # endregion


if __name__ == "__main__":
    # thrust_vector = np.array([np.sqrt(3)/3, np.sqrt(3)/3, np.sqrt(3)/3])

    def get_rot_mat(phi, theta, psi):
        T_theta = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ])
        
        T_phi = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)],
        ])
        
        T_psi = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        # rot_mat = np.array([
        #     [np.cos(theta) * np.cos(psi), -np.cos(theta) * np.sin(psi), -np.sin(theta)],
        #     [np.sin(phi) * np.sin(theta) * np.cos(psi) + np.cos(phi) * np.sin(psi), -np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi), np.sin(phi) * np.cos(theta)],
        #     [-np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi), np.cos(phi) * np.sin(theta) * np.sin(psi) + np.sin(phi) * np.cos(psi), np.cos(phi) * np.cos(psi)]
        # ])

        # rot_mat = np.array([
        #     [np.cos(theta) * np.cos(psi), np.cos(theta) * np.sin(psi), -np.sin(theta)],
        #     [np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi), np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi), np.sin(phi) * np.cos(theta)],
        #     [np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi), np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi), np.cos(phi) * np.cos(psi)]
        # ])

        rot_mat = np.matmul(np.matmul(T_phi, T_theta), T_psi)

        return rot_mat
    
    thrust_vector = np.array([1, 0, 0])
    phi = np.deg2rad(0)
    theta = np.deg2rad(0)
    psi = np.deg2rad(-45)

    rot_mat = get_rot_mat(phi, theta, psi)
    rot_thrust_vec = np.matmul(rot_mat, thrust_vector)
    print(rot_mat)
    print(rot_thrust_vec)
    print(np.linalg.norm(rot_thrust_vec))