import numpy as np
from copy import deepcopy
from scipy.integrate import ode

class CPG:
    def __init__(self):
        # CPG Parameters
        self.coupling_matrix = [
            [0, np.pi, 0, np.pi, 0, np.pi],
            [np.pi, 0, np.pi, 0, 0, np.pi],
            [0, np.pi, 0, np.pi, 0, np.pi],
            [np.pi, 0, np.pi, 0, np.pi, 0],
            [0, np.pi, 0, np.pi, 0, np.pi],
            [np.pi, 0, np.pi, 0, np.pi, 0]
        ]
        
        self.k = np.cos(self.coupling_matrix)
        # Oscillator angular velocity in swing phase
        self.omega_sw = 4
        # Oscillator angular velocity in stance phase
        self.omega_st = self.omega_sw
        self.alpha = 5
        self.beta = 50
        # Sqrt of mu is oscillator's stable amplitude
        self.mu = 1
        self.b = 100
        
        self.n = len(self.k)
        self.omega = np.zeros(self.n).tolist()
        self.r = np.zeros(self.n).tolist()
        
        x = [-np.sqrt(self.mu) * (1 - np.cos(i*np.pi/4)) for i in range(self.n)]
        y = np.zeros(len(self.k)).tolist()
        
        # Initial state
        self.s0 = x + y
        
    def _dsdt(self, t, s):
        # Nested mutable object, deepcopy needed
        ds = deepcopy(s)
        
        for i in range(self.n):
            self.omega[i] = self.omega_st / np.exp(-self.b * s[self.n+i] + 1) + self.omega_sw / np.exp(self.b * s[self.n+i] + 1)
            self.r[i] = np.sqrt(s[i]**2 + s[self.n + i]**2)
            
            ds[i] = self.alpha * (self.mu - self.r[i]**2) * s[i] - self.omega[i] * s[self.n+i]
            ds[self.n+i] = self.beta * (self.mu - self.r[i]**2) * s[self.n+i] + self.omega[i] * s[i] + np.array(self.k[i]).dot(np.array(s[self.n:]))
        
        return ds
    
    def simulate(self, t):
        """
        Simulate's the cpg for t seconds from initial state, s0.
        """
        solver = ode(self._dsdt).set_integrator("dopri5", rtol=1e-3, nsteps=1000)
        
        solver.set_initial_value(self.s0, 0)
        s = solver.integrate(t)
        
        self.s0 = s
        
        return s.tolist()