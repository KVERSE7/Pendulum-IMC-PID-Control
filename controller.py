import time
import math
import numpy as np

class IMCPendulumController:
    def __init__(self, imc_lambda=0.15, mass=1.0, length=1.0):
        self.imc_lambda = imc_lambda
        self.m = mass
        self.l = length
        
        # --- TOPIC: IMC AUTO-TUNING ---
        g = 9.81
        I = (1/3) * mass * (length**2)
        Kg = 0.5 * mass * g * length

        self.kp = Kg + (I / (imc_lambda**2))
        self.kd = (2 * I) / imc_lambda
        self.ki = 0.8
        
        # --- DYNAMIC ENERGY TARGET ---
        # Theoretical energy to reach the top: m * g * (l/2)
        theoretical_peak = mass * g * (length / 2.0)
        self.target_energy = 0.9 * theoretical_peak

        self.integral_error = 0.0
        self.prev_time = time.time()
        self.is_stabilizing = False
        
        print(f"\n[SYSTEM READY]")
        print(f"Calculated Energy Target: {self.target_energy:.2f}")
        print(f"IMC Gains -> Kp: {self.kp:.2f}, Kd: {self.kd:.2f}")

    def compute(self, theta, theta_dot, current_time):
        dt = current_time - self.prev_time
        if dt <= 0.0: dt = 1e-4
        self.prev_time = current_time

        # --- MODE SWITCHING (The Catch) ---
        if abs(theta) < 0.6: 
            self.is_stabilizing = True
        elif abs(theta) > 1.0:
            self.is_stabilizing = False

        if self.is_stabilizing:
            # ZONE: PID CONTROL
            error = 0.0 - theta
            error = (error + np.pi) % (2 * np.pi) - np.pi
            self.integral_error = np.clip(self.integral_error + (error * dt), -1.0, 1.0)
            
            torque = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * (0.0 - theta_dot))
            return np.clip(torque, -2.0, 2.0), "STABILIZE"
        
        else:
            # ZONE: SWING-UP
            current_energy = (1/6)*self.m*(self.l**2)*(theta_dot**2) + (0.5*self.m*9.81*self.l*np.cos(theta))
            
            if current_energy < self.target_energy and abs(theta) > 1.57:
                torque = 2.0 if theta_dot > 0 else -2.0
            else:
                torque = 0.0 # COASTING
                
            return torque, "SWING/COAST"