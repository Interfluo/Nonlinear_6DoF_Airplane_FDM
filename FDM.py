import numpy as np
import matplotlib.pyplot as plt
from simupy.block_diagram import BlockDiagram
import simupy_flight


class airplane(object):
    def __init__(self):
        self.m = 1400
        self.I_xx = 2240
        self.I_yy = 3423
        self.I_zz = 4760
        self.I_xy = 0
        self.I_xz = 0
        self.I_yz = 0
        self.MR = np.array([1.14, 0, 0])
        self.CG = np.array([1.20, 0, 0])
        self.Sref = 15.2361
        self.bref = 12.192
        self.cref = 1.3716

    def aero_fm(self, state):
        """
        compute aerodynamic forces and moments at the moment reference location from current vehicle state
        :param state:
        :return:
        """
        # aircraft and state parameters
        p, q, r = state[0], state[1], state[2]
        alpha, alpha_dot, beta, beta_dot = state[3], state[4], state[5], state[6]
        speed, rho = state[7], state[8]
        cg1, cg2, cg5, cg6, cg7 = state[9], state[10], state[11], state[12], state[13]
        df = state[14]

        # intermediate calc
        qbar = 0.5 * rho * speed ** 2
        ndx = self.bref / speed
        ndy = self.cref / speed
        ndz = self.bref / speed

        # Control Surfaces Coefficients
        CS = np.transpose([cg1, cg2, cg5, cg6, cg7]) * np.pi / 180
        cfxcs = np.dot(np.array([-0.0036797, -0.0036803, 0.0000346, 0.0000277, 0.0005420]), CS)
        cfycs = np.dot(np.array([-0.0394368, 0.0394970, 0.0257521, -0.0227159, 0.1435234]), CS)
        cfzcs = np.dot(np.array([0.4687106, 0.4690858, 0.2688852, 0.2671962, 0.0013564]), CS)
        cmxcs = np.dot(np.array([0.1518120, -0.1517704, 0.0103873, -0.0105649, -0.0116984]), CS)
        cmycs = np.dot(np.array([-0.0704523, -0.0710427, -0.7017262, -0.6964255, -0.0046262]), CS)
        cmzcs = np.dot(np.array([0.0001911, -0.0003257, 0.0084402, -0.0074721, 0.0556894]), CS)

        # Base coefficients
        CZ00 = -66.383 * alpha ** 4 + 1.6454 * alpha ** 3 + 0.7187 * alpha ** 2 + 5.5505 * alpha + 0.177
        CZ32 = -28.023 * alpha ** 4 - 19.622 * alpha ** 3 - 2.4053 * alpha ** 2 + 5.9096 * alpha + 0.8233
        Cz0 = (CZ32 - CZ00) * (df - 0) / 32 + CZ00
        Czq = 4.7707
        Cma = 1.5 * (-0.001166016 * df ** 2 + 0.018594 * df + 1.0220)
        Cmq = -25.7942
        Cyp = -0.1829
        Clb = 0.5 * (-.0000273440 * df ** 2 + 0.001563 * df - 0.0580)
        Clp = -0.7319
        Clr = 0.2253
        Cnb = 1.5 * (-0.000017188 * df ** 2 + 0.000625 * df + 0.0410)
        Cnp = -0.0456
        Cx00 = -34.947 * alpha ** 4 - 5.127 * alpha ** 3 + 5.193 * alpha ** 2 + 0.5549 * alpha - 0.0663
        Cx32 = -69.734 * alpha ** 4 + 16.5 * alpha ** 3 + 4.391 * alpha ** 2 - 0.0466 * alpha - 0.0265
        Cx0 = (Cx32 - Cx00) * (df - 0) / 32 + Cx00
        Cx0 = Cx0
        Cz0 = Cz0
        Cza_dot = 2 * -2.363
        Cma_dot = 2 * -7.698
        Cyb = -0.299
        Cyr = 0.0426
        Cnr = -0.1906
        cfxb = Cx0
        cfyb = Cyb * beta + Cyp * p * self.bref / (2 * speed) + Cyr * r * self.bref / (2 * speed)
        cfzb = Cz0 + Czq * q * self.cref / (2 * speed) + Cza_dot * alpha_dot * self.cref / (2 * speed)
        cmxb = Clb * beta + Clp * p * self.bref / (2 * speed) + Clr * r * self.bref / (2 * speed)
        cmyb = Cma * alpha + Cmq * q * self.cref / (2 * speed) + Cma_dot * alpha_dot * self.cref / (2 * speed)
        cmzb = Cnb * beta + Cnp * p * self.bref / (2 * speed) + Cnr * r * self.bref / (2 * speed)

        #Compute total aero FM
        Fx = qbar * self.Sref * (cfxcs + cfxb)
        Fy = qbar * self.Sref * (cfycs + cfyb)
        Fz = qbar * self.Sref * (cfzcs + cfzb)
        Mx = qbar * self.Sref * self.bref * (cmxcs + cmxb)
        My = qbar * self.Sref * self.cref * (cmycs + cmyb)
        Mz = qbar * self.Sref * self.bref * (cmzcs + cmzb)
        FM = np.array([Fx, Fy, Fz, Mx, My, Mz]).ravel()
        return FM

    def mr2cg(self, FM):
        """
        moment transfer from moment reference to center of gravity
        :param FM:
        :return:
        """
        return FM

    def dynamics_output(self, FM, omega_X, omega_Y, omega_Z):
        """
        Extremely lightly modified version of the dynamics_output_function in simupy-flight
        :param FM:
        :param omega_X:
        :param omega_Y:
        :param omega_Z:
        :return:
        """
        x0 = 1 / self.m
        x1 = self.I_yz ** 2
        x2 = self.I_xz ** 2
        x3 = self.I_xy ** 2
        x4 = self.I_yy * self.I_zz
        x5 = self.I_xy * self.I_yz
        x6 = 1 / (self.I_xx * x1 - self.I_xx * x4 + 2 * self.I_xz * x5 + self.I_yy * x2 + self.I_zz * x3)
        x7 = omega_Y ** 2
        x8 = omega_Z ** 2
        x9 = omega_X * omega_Y
        x10 = omega_Y * omega_Z
        x11 = omega_X * omega_Z
        x12 = -self.I_xy * x11 + self.I_xz * x9 + self.I_yy * x10 + self.I_yz * x7 - self.I_yz * x8 - self.I_zz * x10 + FM[3]
        x13 = self.I_xz * self.I_yy + x5
        x14 = omega_X ** 2
        x15 = self.I_xx * x9 + self.I_xy * x14 - self.I_xy * x7 - self.I_xz * x10 - self.I_yy * x9 + self.I_yz * x11 + FM[5]
        x16 = self.I_xy * self.I_zz + self.I_xz * self.I_yz
        x17 = -self.I_xx * x11 + self.I_xy * x10 - self.I_xz * x14 + self.I_xz * x8 - self.I_yz * x9 + self.I_zz * x11 + FM[4]
        x18 = self.I_xx * self.I_yz + self.I_xy * self.I_xz
        return (np.array([FM[0]*x0, FM[1]*x0, FM[2]*x0, -x6*(x12*(-x1 + x4) + x13*x15 + x16*x17), -x6*(x12*x16 + x15*x18 + x17*(self.I_xx*self.I_zz - x2)), -x6*(x12*x13 + x15*(self.I_xx*self.I_yy - x3) + x17*x18)]))


# states = [Sref, bref, cref, p, q, r, alpha, alpha_dot, beta, beta_dot, speed, rho, dar, dal, der, del, dr, df]
states = [0, 0, 0, 0, 0, 0, 0, 100, 1.225, 0, 0, 0, 0, 0, 0, 0, 0]
p_B = 0
q_B = 0
r_B = 0
alpha = 0
alpha_dot = 0
beta = 0
beta_dot = 0
v_x = 65
v_y = 0
v_z = 5
rho = 1.225
print((v_x**2+v_y**2+v_z**2)**0.5)

airplane = airplane()
FM = airplane.aero_fm(states)
FM = airplane.mr2cg(FM)
vehicle = simupy_flight.Vehicle(
    base_aero_coeffs=simupy_flight.get_constant_aero(),
        # states =                      [Sref,          bref,          cref,          p,   q,   r,   alpha, alpha_dot, beta, beta_dot, speed,                       rho, dar, dal, der, del, dr, df]
    input_force_moment=airplane.aero_fm([airplane.Sref, airplane.bref, airplane.cref, p_B, q_B, r_B, alpha, alpha_dot, beta, beta_dot, (v_x**2+v_y**2+v_z**2)**0.5, rho, 0, 0, 0, 0, 0, 0]),
    m=airplane.m,
    I_xx=airplane.I_xx,
    I_yy=airplane.I_yy,
    I_zz=airplane.I_zz,
    I_xy=airplane.I_xy,
    I_yz=airplane.I_yz,
    I_xz=airplane.I_xz,
    x_com=0,
    y_com=0,
    z_com=0,
    x_mrc=0,
    y_mrc=0,
    z_mrc=0,
    S_A=airplane.Sref,
    a_l=airplane.bref,
    b_l=airplane.cref,
    c_l=airplane.bref,
    d_l=airplane.cref,
)

planet = simupy_flight.Planet(gravity=simupy_flight.earth_J2_gravity,winds=simupy_flight.get_constant_winds(),atmosphere=simupy_flight.atmosphere_1976,planetodetics=simupy_flight.Planetodetic(a=simupy_flight.earth_equitorial_radius,omega_p=simupy_flight.earth_rotation_rate,f=simupy_flight.earth_f,),)

BD = BlockDiagram(planet, vehicle)
BD.connect(planet, vehicle, inputs=np.arange(planet.dim_output))
BD.connect(vehicle, planet, inputs=np.arange(vehicle.dim_output))


lat_ic = 0.0 * np.pi / 180
long_ic = 0.0 * np.pi / 180
h_ic = 5000
V_N_ic = 000.0
V_E_ic = 75.0
V_D_ic = 0.0
psi_ic = 90.0 * np.pi / 180
theta_ic = 0.0 * np.pi / 180
phi_ic = 0.0 * np.pi / 180
omega_X_ic = 0.0 * np.pi / 180
omega_Y_ic = -0.004178073 * np.pi / 180
omega_Z_ic = 0.0 * np.pi / 180

planet.initial_condition = planet.ic_from_planetodetic(
    long_ic, lat_ic, h_ic, V_N_ic, V_E_ic, V_D_ic, psi_ic, theta_ic, phi_ic
)
planet.initial_condition[-3:] = omega_X_ic, omega_Y_ic, omega_Z_ic

res = BD.simulate(30)

print(len(res.x[1]))
print(len(res.y[1]))

plt.figure()
#plt.plot(res.t, res.y[:, 0])
#plt.plot(res.t, res.y[:, 1])
plt.plot(res.t, res.y[:, 2])
#plt.legend(['px', 'py', 'pz'])
plt.show()

