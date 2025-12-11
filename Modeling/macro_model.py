import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. PARAMETERS (Must match Micro model)
# ==========================================
DT = 0.1
SIM_DURATION = 180.0
NUM_ROBOTS = 5.0  # Float because macroscopic represents averages

# Transition Parameters
P_BID_WON = 0.02  # P_in

# Calculate Average Active Duration (T_active)
AVG_TRAVEL_TIME = 15.0
AVG_WORK_TIME = (2 * 3.0 + 3 * 9.0) / NUM_ROBOTS
AVG_OBSAVOID_TIME = 5.0  
TOTAL_ACTIVE_TIME = AVG_TRAVEL_TIME + AVG_WORK_TIME + AVG_OBSAVOID_TIME

# P_out: Probability of finishing task per step
P_FINISH = DT / TOTAL_ACTIVE_TIME 

# Battery Parameters
MAX_BATTERY_LIFE = 120.0
P_DIE = 0.0 

# ==========================================
# 2. MACROSCOPIC SIMULATION LOOP
# ==========================================
def run_macroscopic():
    steps = int(SIM_DURATION / DT)
    time_axis = np.arange(0, SIM_DURATION, DT)
    
    # State Arrays
    n_idle = np.zeros(steps)
    n_active = np.zeros(steps)
    n_completed = np.zeros(steps) # New Array for Cumulative Tasks
    
    # Initial Conditions: All robots start IDLE
    n_idle[0] = NUM_ROBOTS
    n_active[0] = 0.0
    n_completed[0] = 0.0
    
    for k in range(steps - 1):
        # 1. Calculate Flows 
        # Flow from Idle to Active
        flow_i_to_a = P_BID_WON * n_idle[k]
        
        # Flow from Active to Idle (Task Finished)
        flow_a_to_i = P_FINISH * n_active[k]
        
        # Battery Flow (Active to Dead)
        flow_a_to_dead = P_DIE * n_active[k]
        
        # 2. Update States (Difference Equations)
        n_active[k+1] = n_active[k] + flow_i_to_a - flow_a_to_i - flow_a_to_dead
        n_idle[k+1] = n_idle[k] - flow_i_to_a + flow_a_to_i
        
        # 3. Accumulate Completed Tasks
        # The flow "flow_a_to_i" represents the number of tasks finished in this time step
        n_completed[k+1] = n_completed[k] + flow_a_to_i
        
    return time_axis, n_active, n_completed

# ==========================================
# 3. RUN & PLOT
# ==========================================
t, n_active_macro, n_completed_macro = run_macroscopic()

# Create 2 subplots sharing the X axis
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Plot 1: Active Robots
ax1.plot(t, n_active_macro, label='Macroscopic Model', color='black', linewidth=3)
ax1.axhline(y=NUM_ROBOTS, color='black', linestyle=':', label='Total Capacity', alpha=0.5)
ax1.set_title('Macroscopic Model: Average Active Robots')
ax1.set_ylabel('Number of Active Robots')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, NUM_ROBOTS + 0.5)

# Plot 2: Completed Tasks
ax2.plot(t, n_completed_macro, label='Cumulative Tasks', color='blue', linewidth=3)
ax2.set_title('Macroscopic Model: Total Completed Tasks')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Count')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()