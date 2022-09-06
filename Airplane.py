import numpy as np


class Airplane:
    def __init__(self):
        # Mass and Dimensional Properties
        self.m = 28
        self.Ib = np.array([[2.56, 0, 0.5], [0, 10.9, 0], [0.5, 0, 11.3]])
        self.S = 1.8
        self.c = 0.58
        self.b = 3.1
        self.D = 0.79

        # Thrust Derivatives
        self.CFT1 = 8.42e-2
        self.CFT2 = -1.36e-1
        self.CFT3 = -9.28e-1

        # Aero Derivatives
        self.CZ1 = 1.29e-2
        self.CZalpha = -3.25
        self.CX1 = -2.12e-2
        self.CXalpha = -2.66e-2
        self.CXalpha2 = -1.55
        self.CXbeta2 = -4.01e-1
        self.CY1 = -3.79e-1
        self.CLa = 6.79e-2
        self.CLa1 = -3.395e-2
        self.CLa2 = -self.CLa1
        self.CLe1 =-0.485e-2
        self.CLe2 = - self.CLe1
        self.CLbeta = -1.3e-2
        self.CLp = -1.92e-1
        self.CLr = 3.61e-2  # error in book
        self.CM1 = 2.08e-2
        self.CMe = 5.45e-1
        self.CMe1 = 2.725e-1
        self.CMe2 = -self.CMe1
        self.CMa1 = 0.389e-1
        self.CMa2 = - self.CMa1
        self.CMalpha = -9.03e-2
        self.CMq = -9.83
        self.CNdr = 5.34e-2
        self.CNbeta = 8.67e-2
        self.CNr = -2.14e-1
        self.Tn = 0.4

    def Aero_ForcesMoments(self, da, de, dr, alpha, beta, ph, qh, rh, qbar):
        # Aerodynamic Forces
        lift = qbar*self.S*(self.CZ1 + self.CZalpha*alpha)
        lateral = qbar*self.S*(self.CY1*beta)
        drag = qbar*self.S*(self.CX1 + self.CXalpha*alpha + self.CXalpha2*alpha**2 + self.CXbeta2*beta**2)
        # Aerodynamic Torques
        roll = qbar*self.S*self.b*(self.CLa*da + self.CLbeta*beta + self.CLp*ph + self.CLr*rh)
        pitch = qbar*self.S*self.c*(self.CM1 + self.CMe*de + self.CMq*qh + self.CMalpha*alpha)
        yaw = qbar*self.S*self.b*(self.CNdr*dr + self.CNr*rh + self.CNbeta*beta)
        # pack Forces and Moments into pseudovector
        FM = np.array([lift, lateral, drag, roll, pitch, yaw])
        return FM


airplane = Airplane()
FM = Airplane.Aero_ForcesMoments(airplane, 0, 0, 0, 0.1, 0, 0, 0, 0, 100)  # Aero_ForcesMoments(self, da, de, dr, alpha, beta, ph, qh, rh, qbar):
print(FM)
