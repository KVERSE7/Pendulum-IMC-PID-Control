import numpy as np
import matplotlib.pyplot as plt
from controller import IMCPendulumController
from simulation import PendulumSimulation
import pygame  # Added to handle window events
import time

# -- TEST PARAMETERS --
MASS = 2.5
LENGTH = 1.5
LAMBDA = 0.2
def run_analysis():
    sim = PendulumSimulation(render=True, mass=MASS, length=LENGTH)
    controller = IMCPendulumController(
        imc_lambda=LAMBDA, mass=MASS, length=LENGTH)

    theta, theta_dot = sim.reset(initial_state=[np.pi, 0.0])
    data = {"t": [], "theta": [], "torque": [], "energy": [], "mode": []}

    print("\n" + "="*60)
    print("SIMULATION RUNNING")
    print("TO TERMINATE: Close the window or press any key on the keyboard.")
    print("="*60)

    running = True
    step = 0

    while running and step < 600:
        t = step * 0.05

        # --- NEW: KEYBOARD/WINDOW TERMINATION LOGIC ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # Clicking the 'X'
                running = False
            if event.type == pygame.KEYDOWN:  # Pressing any key
                print(
                    f"\n[USER STOP] Key '{pygame.key.name(event.key)}' pressed.")
                running = False

        if not running:
            break

        # Control Logic
        torque, mode = controller.compute(theta, theta_dot, t)

        # Physics for Energy Tracking
        I = (1/3) * MASS * (LENGTH**2)
        pot = 0.5 * MASS * 9.81 * LENGTH * np.cos(theta)
        kin = 0.5 * I * (theta_dot**2)
        current_energy = pot + kin

        theta, theta_dot, _, _ = sim.step(torque)

        # Log Data
        data["t"].append(t)
        data["theta"].append(theta)
        data["torque"].append(torque)
        data["energy"].append(current_energy)
        data["mode"].append(mode)

        if step % 20 == 0:
            print(f"Time: {t:>4.1f}s | Angle: {theta:>5.2f} | Mode: {mode}")

        step += 1
        time.sleep(0.01)

    sim.close()
    print("[INFO] Simulation ended. Generating Analysis Graphs...")

    if len(data["t"]) > 5:
        generate_plots(data, controller.target_energy)

def generate_plots(data, target_e):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    ax1.plot(data["t"], data["theta"], label="Angle (rad)")
    ax1.axhline(0, color='r', linestyle='--')
    ax1.set_title("Trajectory Analysis (Gain Scheduling Zones)")
    ax2.step(data["t"], data["torque"], color='g', label="Control Effort (Nm)")
    ax3.plot(data["t"], data["energy"], color='m', label="Total Energy")
    ax3.axhline(target_e, color='orange', linestyle='--',
                label="Calculated Target E")
    ax3.set_xlabel("Time (s)")
    for ax in [ax1, ax2, ax3]:
        ax.grid(True)
        ax.legend()
    plt.tight_layout()
    plt.show()
if __name__ == "__main__":
    run_analysis()