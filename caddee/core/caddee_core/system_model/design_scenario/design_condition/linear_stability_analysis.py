import m3l
from dataclasses import dataclass
import csdl
import numpy as np


@dataclass
class DampingRatios:
    damping_long_11 : m3l.Variable
    damping_long_12 : m3l.Variable
    damping_long_21 : m3l.Variable
    damping_long_22 : m3l.Variable

    damping_lat_11 : m3l.Variable
    damping_lat_12 : m3l.Variable
    damping_lat_21 : m3l.Variable
    damping_lat_22 : m3l.Variable

class LinearStabilityAnalysis(m3l.ImplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str, default='linear_stability_model')

    def assign_attributes(self):
        self.name = self.parameters['name']
    
    def evaluate(self, A_long, lhs_long, long_stab_state_vec, A_lat, 
                 lhs_lat, lat_stab_state_vec, long_accelerations=None, lat_accelerations=None) -> DampingRatios:
        
        self.inputs = {}
        self.arguments = {}
        self.arguments['A_long'] = A_long
        self.arguments['A_lat'] = A_lat
        # self.arguments['lhs_long'] = lhs_long
        # self.arguments['long_stab_state_vec'] = long_stab_state_vec
        # if A_lat:
        #     self.arguments['A_lat'] = A_lat
        #     self.arguments['lhs_lat'] = lhs_lat
        #     self.arguments['lat_stab_state_vec'] = lat_stab_state_vec

        self.inputs['A_long'] = A_long
        self.inputs['A_lat'] = A_lat
        # self.inputs['lhs_long'] = lhs_long
        # self.inputs['long_stab_state_vec'] = long_stab_state_vec
        # if A_lat:
        #     self.inputs['A_lat'] = A_lat
        #     self.inputs['lhs_lat'] = lhs_lat
        #     self.inputs['lat_stab_state_vec'] = lat_stab_state_vec
        
        self.outputs = {}
        if long_accelerations:
            self.outputs['long_accelerations'] = long_accelerations
        if lat_accelerations:
            self.outputs['lat_accelerations'] = lat_accelerations

        self.residual_partials = {}
        self.residual_partials['long_accelerations_jacobian'] = long_accelerations
        self.residual_partials['lat_accelerations_jacobian'] = long_accelerations
        
        self.size = 4
        long_residual = m3l.Variable(name='long_stab_residual', shape=(4, ), operation=self)
        lat_residual = m3l.Variable(name='lat_stab_residual', shape=(4, ), operation=self)

        return long_residual, lat_residual
    
    def compute_residual(self):
        csdl_model = LinearStabilityCSDL(linear_stability_analysis=self)

        return csdl_model
    
    def compute_derivatives(self):
        csdl_model = LinearStabilityDerivativeCSDL()

        return csdl_model
    

class LinearStabilityCSDL(csdl.Model):
    def define(self):
        lhs_long = self.declare_variable('lhs_long', shape=(4, ))
        A_long = self.declare_variable('A_long', shape=(4, 4))
        long_stab_state_vec = self.declare_variable('long_stab_state_vec', shape=(4, ))

        lhs_lat = self.declare_variable('lhs_lat', shape=(4, ))
        A_lat = self.declare_variable('A_lat', shape=(4, 4))
        lat_stab_state_vec = self.declare_variable('lat_stab_state_vec', shape=(4, ))

        long_stab_residual = csdl.matvec(A_long, long_stab_state_vec) - lhs_long
        self.register_output('long_stab_residual', long_stab_residual)

        lat_stab_residual = csdl.matvec(A_lat, lat_stab_state_vec) - lhs_lat
        self.register_output('lat_stab_residual', lat_stab_residual)

class LinearStabilityDerivativeCSDL(csdl.Model):
    def define(self):
        A_long = self.declare_variable('A_long', shape=(4, 4))
        A_lat = self.declare_variable('A_lat', shape=(4, 4))
        self.register_output('long_accelerations_jacobian', A_long*1)
        self.register_output('lat_accelerations_jacobian', A_lat*1)


@dataclass
class LongitudinalStability:
    short_period_real : m3l.Variable = None,
    short_period_imag : m3l.Variable = None,
    short_period_natural_frequency : m3l.Variable = None,
    short_period_damping_ratio : m3l.Variable = None,
    short_period_time_to_double : m3l.Variable = None,
    
    phugoid_real : m3l.Variable = None,
    phugoid_imag : m3l.Variable = None,
    phugoid_natural_frequency : m3l.Variable = None,
    phugoid_damping_ratio : m3l.Variable = None,
    phugoid_time_to_double : m3l.Variable = None,


@dataclass
class LateralStability:
    dutch_roll_real : m3l.Variable = None
    dutch_roll_imag : m3l.Variable = None
    dutch_roll_natural_frequency : m3l.Variable = None
    dutch_roll_damping_ratio : m3l.Variable = None
    dutch_roll_time_to_double : m3l.Variable = None

    roll_real : m3l.Variable = None
    roll_imag : m3l.Variable = None
    roll_natural_frequency : m3l.Variable = None
    roll_damping_ratio : m3l.Variable = None
    roll_time_to_double : m3l.Variable = None

    spiral_real : m3l.Variable = None
    spiral_imag : m3l.Variable = None
    spiral_natural_frequency : m3l.Variable = None
    spiral_damping_ratio : m3l.Variable = None
    spiral_time_to_double : m3l.Variable = None



class LongitudinalAircraftStability(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str, default='longitudinal_stability')
        self.parameters.declare('connect_from', types=list, default=[])
        self.parameters.declare('connect_to', types=list, default=[])
        self.parameters.declare('design_condition_name', types=str)

    def assign_attributes(self):
        self.dc_name = self.parameters['design_condition_name']
        self.name = f"{self.dc_name}_{self.parameters['name']}"

    def compute(self):
        self.parameters['connect_from'] = [
            f"{self.dc_name}_linear_stability_model_long_accelerations_jacobian_eig.e_real", 
            f"{self.dc_name}_linear_stability_model_long_accelerations_jacobian_eig.e_imag", 
        ]
        self.parameters['connect_to'] = [
            f"{self.name}.e_real_long", 
            f"{self.name}.e_imag_long",
        ]
        csdl_model = LongitudinalAircraftStabilityCSDL(
            size=4,
        )
        
        return csdl_model


    def evaluate(self) -> LongitudinalStability:
        self.arguments = {}

        short_period_real = m3l.Variable(name='short_period_real', shape=(1, ), operation=self)
        short_period_imag = m3l.Variable(name='short_period_imag', shape=(1, ), operation=self)
        short_period_natural_frequency = m3l.Variable(name='short_period_natural_frequency', shape=(1, ), operation=self)
        short_period_damping_ratio = m3l.Variable(name='short_period_damping_ratio', shape=(1, ), operation=self)
        short_period_time_to_double = m3l.Variable(name='short_period_time_to_double', shape=(1, ), operation=self)

        phugoid_real = m3l.Variable(name='phugoid_real', shape=(1, ), operation=self)
        phugoid_imag = m3l.Variable(name='phugoid_imag', shape=(1, ), operation=self)
        phugoid_natural_frequency = m3l.Variable(name='phugoid_natural_frequency', shape=(1, ), operation=self)
        phugoid_damping_ratio = m3l.Variable(name='phugoid_damping_ratio', shape=(1, ), operation=self)
        phugoid_time_to_double = m3l.Variable(name='phugoid_time_to_double', shape=(1, ), operation=self)

        long_stab = LongitudinalStability(
            short_period_real=short_period_real,
            short_period_imag=short_period_imag,
            short_period_natural_frequency=short_period_natural_frequency,
            short_period_damping_ratio=short_period_damping_ratio,
            short_period_time_to_double=short_period_time_to_double,
            phugoid_real=phugoid_real,
            phugoid_imag=phugoid_imag,
            phugoid_natural_frequency=phugoid_natural_frequency,
            phugoid_damping_ratio=phugoid_damping_ratio,
            phugoid_time_to_double=phugoid_time_to_double,
        )

        return long_stab
    

class LateralAircraftStability(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str, default='lateral_stability')
        self.parameters.declare('connect_from', types=list, default=[])
        self.parameters.declare('connect_to', types=list, default=[])
        self.parameters.declare('design_condition_name', types=str)

    def assign_attributes(self):
        self.dc_name = self.parameters['design_condition_name']
        self.name = f"{self.dc_name}_{self.parameters['name']}"

    def compute(self):
        self.parameters['connect_from'] = [
            f"{self.dc_name}_linear_stability_model_lat_accelerations_jacobian_eig.e_real", 
            f"{self.dc_name}_linear_stability_model_lat_accelerations_jacobian_eig.e_imag", 
        ]
        self.parameters['connect_to'] = [
            f"{self.name}.e_real_lat", 
            f"{self.name}.e_imag_lat",
        ]
        csdl_model = LateralAircraftStabilityCSDL(
            size=4,
        )
        
        return csdl_model


    def evaluate(self) -> LateralStability:
        self.arguments = {}

        dutch_roll_real = m3l.Variable(name='dutch_roll_real', shape=(1, ), operation=self)
        dutch_roll_imag = m3l.Variable(name='dutch_roll_imag', shape=(1, ), operation=self)
        dutch_roll_natural_frequency = m3l.Variable(name='dutch_roll_natural_frequency', shape=(1, ), operation=self)
        dutch_roll_damping_ratio = m3l.Variable(name='dutch_roll_damping_ratio', shape=(1, ), operation=self)
        dutch_roll_time_to_double = m3l.Variable(name='dutch_roll_time_to_double', shape=(1, ), operation=self)

        roll_real = m3l.Variable(name='roll_real', shape=(1, ), operation=self)
        roll_imag = m3l.Variable(name='roll_imag', shape=(1, ), operation=self)
        roll_natural_frequency = m3l.Variable(name='roll_natural_frequency', shape=(1, ), operation=self)
        roll_damping_ratio = m3l.Variable(name='roll_damping_ratio', shape=(1, ), operation=self)
        roll_time_to_double = m3l.Variable(name='roll_time_to_double', shape=(1, ), operation=self)

        spiral_real = m3l.Variable(name='spiral_real', shape=(1, ), operation=self)
        spiral_imag = m3l.Variable(name='spiral_imag', shape=(1, ), operation=self)
        spiral_natural_frequency = m3l.Variable(name='spiral_natural_frequency', shape=(1, ), operation=self)
        spiral_damping_ratio = m3l.Variable(name='spiral_damping_ratio', shape=(1, ), operation=self)
        spiral_time_to_double = m3l.Variable(name='spiral_time_to_double', shape=(1, ), operation=self)

        lat_stab = LateralStability(
            dutch_roll_real=dutch_roll_real,
            dutch_roll_imag=dutch_roll_imag,
            dutch_roll_natural_frequency=dutch_roll_natural_frequency,
            dutch_roll_damping_ratio=dutch_roll_damping_ratio,
            dutch_roll_time_to_double=dutch_roll_time_to_double,
            
            roll_real=roll_real,
            roll_imag=roll_imag,
            roll_natural_frequency=roll_natural_frequency,
            roll_damping_ratio=roll_damping_ratio,
            roll_time_to_double=roll_time_to_double,

            spiral_real=spiral_real,
            spiral_imag=spiral_imag,
            spiral_natural_frequency=spiral_natural_frequency,
            spiral_damping_ratio=spiral_damping_ratio,
            spiral_time_to_double=spiral_time_to_double,
        )

        return lat_stab
    

class LongitudinalAircraftStabilityCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('size')
    
    def define(self):
        size = self.parameters['size']
        
        e_real = self.declare_variable('e_real_long', shape=(1,size))
        e_imag = self.declare_variable('e_imag_long', shape=(1,size))
        
        # short period eigenvalue pair
        sp_e_real = e_real[0,0]
        sp_e_imag = e_imag[0,0]
        self.register_output('short_period_real', sp_e_real)
        self.register_output('short_period_imag', sp_e_imag)
        
        # get phugoid eigenvalue pair
        ph_e_real = e_real[0,2]
        ph_e_imag = e_imag[0,2]
        self.register_output('phugoid_real', ph_e_real)
        self.register_output('phugoid_imag', ph_e_imag)

        
        # compute short period natural frequency
        sp_wn = (sp_e_real**2 + sp_e_imag**2)**0.5
        self.register_output('short_period_natural_frequency', sp_wn)
        # compute phugoid natural frequency
        ph_wn = (ph_e_real**2 + ph_e_imag**2)**0.5
        self.register_output('phugoid_natural_frequency', ph_wn)
        
        # compute short period damping ratio
        sp_z = -1*sp_e_real/sp_wn
        self.register_output('short_period_damping_ratio', sp_z)
        # compute phugoid damping ratio
        ph_z = -1*ph_e_real/ph_wn
        self.register_output('phugoid_damping_ratio', ph_z)
        
        # compute short period time to double
        sp_z_abs = (sp_z**2)**0.5
        sp_t2 = np.log(2)/(sp_z_abs*sp_wn)
        self.register_output('short_period_time_to_double', sp_t2)
        # compute phugoid time to double
        ph_z_abs = (ph_z**2)**0.5
        ph_t2 = np.log(2)/(ph_z_abs*ph_wn)
        self.register_output('phugoid_time_to_double', ph_t2)


class LateralAircraftStabilityCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('size')
    
    def define(self):
        size = self.parameters['size']
        
        e_real = self.declare_variable('e_real_lat', shape=(1,size))
        e_imag = self.declare_variable('e_imag_lat', shape=(1,size))
        # order: dr, dr, rr, ss
        
        # get dutch roll eigenvalue pair
        dr_e_real = e_real[0,2]
        dr_e_imag = e_imag[0,2]
        self.register_output('dutch_roll_real', dr_e_real)
        self.register_output('dutch_roll_imag', dr_e_imag)
        
        # get roll eigenvalue pair
        rr_e_real = e_real[0,0]
        rr_e_imag = e_imag[0,0]
        self.register_output('roll_real', rr_e_real)
        self.register_output('roll_imag', rr_e_imag)
        
        # get spiral eigenvalue pair
        ss_e_real = e_real[0,3]
        ss_e_imag = e_imag[0,3]
        self.register_output('spiral_real', ss_e_real)
        self.register_output('spiral_imag', ss_e_imag)

        # compute dutch roll natural frequency
        dr_wn = (dr_e_real**2 + dr_e_imag**2)**0.5
        self.register_output('dutch_roll_natural_frequency', dr_wn)
        # compute roll natural frequency
        rr_wn = (rr_e_real**2 + rr_e_imag**2)**0.5
        self.register_output('roll_natural_frequency', rr_wn)
        # compute spiral natural frequency
        ss_wn = (ss_e_real**2 + ss_e_imag**2)**0.5
        self.register_output('spiral_natural_frequency', ss_wn)
        
        # compute dutch roll damping ratio
        dr_z = -1*dr_e_real/dr_wn
        self.register_output('dutch_roll_damping_ratio', dr_z)
        # compute roll damping ratio
        rr_z = -1*rr_e_real/rr_wn
        self.register_output('roll_damping_ratio', rr_z)
        # compute spiral damping ratio
        ss_z = -1*ss_e_real/ss_wn
        self.register_output('spiral_damping_ratio', ss_z)
        
        # compute dutch roll time to double
        dr_z_abs = (dr_z**2)**0.5
        dr_t2 = np.log(2)/(dr_z_abs*dr_wn)
        self.register_output('dutch_roll_time_to_double', dr_t2)
        # compute roll time to double
        rr_z_abs = (rr_z**2)**0.5
        rr_t2 = np.log(2)/(rr_z_abs*rr_wn)
        self.register_output('roll_time_to_double', rr_t2)
        # compute spiral time to double
        ss_z_abs = (ss_z**2)**0.5
        ss_t2 = np.log(2)/(ss_z_abs*ss_wn)
        self.register_output('spiral_time_to_double', ss_t2)