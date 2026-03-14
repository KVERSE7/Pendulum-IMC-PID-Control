import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
import pygame
import math
import time

# --- PHYSICAL PARAMETERS ---
MASS = 2.5
LENGTH = 1.5
LAMBDA_VALUES = [0.1, 0.2, 0.4, 0.6, 0.8]

class IMCPendulumController:
    def __init__(self, imc_lambda=0.15, mass=1.0, length=1.0):
        self.imc_lambda = imc_lambda
        self.m = mass
        self.l = length
        
        # IMC AUTO-TUNING CALCULATION
        g = 9.81
        I = (1/3) * mass * (length**2)
        Kg = 0.5 * mass * g * length

        self.kp = Kg + (I / (imc_lambda**2))
        self.kd = (2 * I) / imc_lambda
        self.ki = 0.6 
        
        theoretical_peak = mass * g * (length / 2.0)
        self.target_energy = 0.9 * theoretical_peak

        self.integral_error = 0.0
        self.prev_time = None
        self.is_stabilizing = False
        
        print(f"LAMBDA {imc_lambda:<4} | Gains -> Kp: {self.kp:>7.2f}, Kd: {self.kd:>6.2f}")

    def compute(self, theta, theta_dot, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
            return 0.0, "START"
            
        dt = current_time - self.prev_time
        if dt <= 0.0: dt = 1e-4
        self.prev_time = current_time

        # MODE SWITCHING
        if abs(theta) < 0.6: 
            self.is_stabilizing = True
        elif abs(theta) > 1.0:
            self.is_stabilizing = False

        if self.is_stabilizing:
            # PID CONTROL ZONE
            error = 0.0 - theta
            error = (error + np.pi) % (2 * np.pi) - np.pi
            self.integral_error = np.clip(self.integral_error + (error * dt), -1.0, 1.0)
            
            torque = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * (0.0 - theta_dot))
            return np.clip(torque, -2.0, 2.0), "STABILIZE"
        else:
            # SWING-UP ZONE
            g = 9.81
            current_energy = (1/6)*self.m*(self.l**2)*(theta_dot**2) + (0.5*self.m*g*self.l*np.cos(theta))
            if current_energy < self.target_energy and abs(theta) > 1.57:
                torque = 2.0 if theta_dot > 0 else -2.0
            else:
                torque = 0.0
            return torque, "SWING"

def run_multi_lambda_analysis():
    pygame.init() # Initialized once to avoid driver errors
    all_results = {}

    for l_val in LAMBDA_VALUES:
        # Set render=True if you want to see the animations one by one
        RENDER = False 
        
        env = gym.make("Pendulum-v1", render_mode="human" if RENDER else None)
        env.unwrapped.m = MASS
        env.unwrapped.l = LENGTH
        
        controller = IMCPendulumController(imc_lambda=l_val, mass=MASS, length=LENGTH)

        # Reset environment: theta=pi (hanging down), theta_dot=0
        observation, _ = env.reset()
        env.unwrapped.state = np.array([np.pi, 0.0], dtype=np.float32)
        
        theta, theta_dot = np.pi, 0.0
        data = {"t": [], "theta": [], "torque": [], "energy": []}

        step = 0
        max_steps = 600 # Approx 30 seconds of simulation
        running = True

        while step < max_steps and running:
            t = step * 0.05
            
            if RENDER:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT: running = False

            torque, mode = controller.compute(theta, theta_dot, t)
            
            # Physics logging
            I = (1/3) * MASS * (LENGTH**2)
            current_energy = (0.5 * MASS * 9.81 * LENGTH * np.cos(theta)) + (0.5 * I * (theta_dot**2))

            # Step simulation
            obs, reward, terminated, truncated, info = env.step(np.array([torque], dtype=np.float32))
            
            # Extract theta/theta_dot from observation [cos(th), sin(th), th_dot]
            theta = math.atan2(obs[1], obs[0])
            theta_dot = obs[2]

            data["t"].append(t)
            data["theta"].append(theta)
            data["torque"].append(torque)
            data["energy"].append(current_energy)

            step += 1

        env.close()
        all_results[l_val] = data

    pygame.quit()
    generate_combined_plots(all_results)

def generate_combined_plots(results):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    colors = plt.cm.plasma(np.linspace(0, 0.8, len(LAMBDA_VALUES)))

    for i, l_val in enumerate(LAMBDA_VALUES):
        d = results[l_val]
        lbl = f"$\lambda$ = {l_val}"
        
        ax1.plot(d["t"], d["theta"], label=lbl, color=colors[i], linewidth=1.5)
        ax2.step(d["t"], d["torque"], color=colors[i], alpha=0.6)
        ax3.plot(d["t"], d["energy"], color=colors[i], alpha=0.8)

    ax1.set_title(f"Inverted Pendulum: IMC-PID Scaling Comparison (Mass={MASS}kg)")
    ax1.set_ylabel("Theta (rad)")
    ax1.axhline(0, color='red', linestyle='--', alpha=0.3, label="Target (Upright)")
    ax1.legend(loc='upper right', ncol=2, fontsize='small')
    
    ax2.set_ylabel("Torque (Nm)")
    ax2.set_ylim(-2.5, 2.5)
    
    ax3.set_ylabel("Total Energy (J)")
    ax3.set_xlabel("Time (s)")
    
    for ax in [ax1, ax2, ax3]:
        ax.grid(True, linestyle=':', alpha=0.6)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_multi_lambda_analysis()