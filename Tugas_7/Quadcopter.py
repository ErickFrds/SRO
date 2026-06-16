import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# =========================
# 1. CONNECT TO COPPELIASIM
# =========================
client = RemoteAPIClient()
sim = client.require("sim")

# =========================
# 2. GET OBJECT HANDLES
# =========================
body = sim.getObject("/Quadcopter")
propeller = sim.getObject("/Quadcopter/propeller[1]")

# =========================
# 3. START SIMULATION
# =========================
sim.startSimulation()
print("Simulation Started")

sim.addLog(1, "Hello from Python!")

try:
    start_time = time.time()
    elapsed_prev = 0.0

    while (time.time() - start_time) < 45:
        elapsed = time.time() - start_time
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        # Get propeller matrix relative to world
        M = sim.getObjectMatrix(propeller, sim.handle_world)

        # Zero translation part
        M[3] = 0
        M[7] = 0
        M[11] = 0

        # Force and torque values
        f = 5.0
        T = 0.1

        # Convert local Z force/torque to world frame
        force = sim.multiplyVector(M, [0, 0, f])
        torque = sim.multiplyVector(M, [0, 0, T])

        # Apply force and torque to the body shape
        sim.addForceAndTorque(body, force, torque)

        time.sleep(0.01)

finally:
    sim.stopSimulation()
    print("\nSimulation Stopped")