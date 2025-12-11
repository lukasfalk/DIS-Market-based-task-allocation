import numpy as np
import matplotlib.pyplot as plt
import random

# ==========================================
# 1. UNIFIED CONFIGURATION
# ==========================================
DT = 0.1
SIM_DURATION = 180.0
NUM_RUNS = 50  # Number of Monte Carlo runs for Micro model

# --- Robot & Task Parameters ---
NUM_ROBOTS = 5
NUM_ROBOTA = 2
NUM_ROBOTB = 3

# Times (Microscopic definitions)
AVG_TRAVEL_TIME = 4.52
TIME_ROBOTA = 3
TIME_ROBOTB = 1
TIME_OBSTACLE_AVOID = 1.82 
TIME_IDLE_PER_TASK = 6.83

# Probability (Microscopic definition)
BID_PROBABILITY = DT / (TIME_IDLE_PER_TASK)  # Based on average idle time per task

# ==========================================
# 2. MACROSCOPIC PARAMETER DERIVATION
# ==========================================
# We calculate Macro parameters based on Micro constants 
# to ensure the models represent the same physical system.

# 1. Flow In Probability (Idle -> Active)
P_BID_WON = BID_PROBABILITY

# 2. Average Work Duration
# Weighted average based on the population of Robot A vs Robot B
avg_work_time = ((NUM_ROBOTA * TIME_ROBOTA) + (NUM_ROBOTB * TIME_ROBOTB)) / NUM_ROBOTS

# 3. Total Active Duration (Average)
# T_active = T_travel + T_work + T_obstacle_avoid
total_active_duration = AVG_TRAVEL_TIME + avg_work_time + TIME_OBSTACLE_AVOID

# 4. Flow Out Probability (Active -> Idle)
# This is the rate at which robots finish tasks (inverse of duration)
P_FINISH = DT / total_active_duration

# ==========================================
# 3. MICROSCOPIC MODEL (Agent-Based)
# ==========================================
class Robot:
    def __init__(self, id, is_robota):
        self.id = id
        self.is_robota = is_robota
        self.state = 0 # 0: IDLE, 1: ACTIVE
        self.timer = 0.0
        self.work_duration = TIME_ROBOTA if is_robota else TIME_ROBOTB

    def reset(self):
        self.state = 0
        self.timer = 0.0

    def step(self, dt):
        """Returns True if a task was just finished."""
        task_finished = False
        
        # IDLE -> ACTIVE (Stochastic)
        if self.state == 0:
            if random.random() < BID_PROBABILITY:
                self.start_active()
        
        # ACTIVE -> IDLE (Deterministic Timer)
        elif self.state == 1:
            self.timer -= dt
            if self.timer <= 0:
                self.state = 0 # Return to Idle
                task_finished = True
                
        return task_finished

    def start_active(self):
        self.state = 1
        # Add noise to travel time (Micro-only feature)
        travel_time = np.random.normal(AVG_TRAVEL_TIME, 2.0)
        if travel_time < 0: travel_time = 0
        self.timer = travel_time + self.work_duration + TIME_OBSTACLE_AVOID

    def is_active(self):
        return self.state == 1

class World:
    def __init__(self):
        self.robots = []
        for i in range(NUM_ROBOTS):
            is_robota = (i < NUM_ROBOTA)
            self.robots.append(Robot(i, is_robota))

    def reset(self):
        for r in self.robots:
            r.reset()

    def run(self, duration):
        self.reset()
        steps = int(duration / DT)
        active_history = np.zeros(steps)
        task_history = np.zeros(steps)
        current_total_tasks = 0

        for i in range(steps):
            active_now = 0
            tasks_this_step = 0
            
            for r in self.robots:
                if r.step(DT):
                    tasks_this_step += 1
                if r.is_active():
                    active_now += 1
            
            current_total_tasks += tasks_this_step
            active_history[i] = active_now
            task_history[i] = current_total_tasks

        return active_history, task_history

# ==========================================
# 4. MACROSCOPIC MODEL (Flow-Based)
# ==========================================
def run_macroscopic():
    steps = int(SIM_DURATION / DT)
    
    n_idle = np.zeros(steps)
    n_active = np.zeros(steps)
    n_completed = np.zeros(steps)
    
    # Initial Conditions
    n_idle[0] = NUM_ROBOTS
    n_active[0] = 0.0
    n_completed[0] = 0.0
    
    for k in range(steps - 1):
        # Calculate Flows
        flow_i_to_a = P_BID_WON * n_idle[k]
        flow_a_to_i = P_FINISH * n_active[k]
        
        # Update States
        n_active[k+1] = n_active[k] + flow_i_to_a - flow_a_to_i
        n_idle[k+1] = n_idle[k] - flow_i_to_a + flow_a_to_i
        
        # Accumulate Completed Tasks
        n_completed[k+1] = n_completed[k] + flow_a_to_i
        
    return n_active, n_completed

# ==========================================
# 5. EXECUTION AND COMPARISON PLOT
# ==========================================
def main():
    # --- A. Run Microscopic (Monte Carlo) ---
    print(f"Running {NUM_RUNS} Microscopic simulations...")
    world = World()
    steps = int(SIM_DURATION / DT)
    
    # Store all runs to calculate stats
    all_active = np.zeros((NUM_RUNS, steps))
    all_tasks = np.zeros((NUM_RUNS, steps))

    for i in range(NUM_RUNS):
        act, tsk = world.run(SIM_DURATION)
        all_active[i, :] = act
        all_tasks[i, :] = tsk

    # Calculate Statistics
    micro_mean_active = np.mean(all_active, axis=0)
    micro_std_active = np.std(all_active, axis=0)
    micro_mean_tasks = np.mean(all_tasks, axis=0)
    micro_std_tasks = np.std(all_tasks, axis=0)

    # --- B. Run Macroscopic (Analytical) ---
    print("Running Macroscopic simulation...")
    macro_active, macro_tasks = run_macroscopic()

    # --- C. Plotting ---
    time_axis = np.arange(0, SIM_DURATION, DT)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # Plot 1: Active Robots Comparison
    ax1.plot(time_axis, micro_mean_active, label='Micro Mean', color='blue', linewidth=2)
    # Shaded area for Standard Deviation
    ax1.fill_between(time_axis, 
                     micro_mean_active - micro_std_active, 
                     micro_mean_active + micro_std_active, 
                     color='blue', alpha=0.1, label='Micro StdDev')
    
    # Dashed Red Line for Macro
    ax1.plot(time_axis, macro_active, label='Macro Model', color='red', linestyle='--', linewidth=2.5)
    
    ax1.set_ylabel('Number of Active Robots')
    ax1.set_title('Comparison: Active Robots (Micro vs Macro)')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, NUM_ROBOTS + 0.5)

    # Plot 2: Completed Tasks Comparison
    ax2.plot(time_axis, micro_mean_tasks, label='Micro Mean', color='green', linewidth=2)
    ax2.fill_between(time_axis, 
                     micro_mean_tasks - micro_std_tasks, 
                     micro_mean_tasks + micro_std_tasks, 
                     color='green', alpha=0.1, label='Micro StdDev')
    
    # Dashed Orange Line for Macro
    ax2.plot(time_axis, macro_tasks, label='Macro Model', color='orange', linestyle='--', linewidth=2.5)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Total Completed Tasks')
    ax2.set_title('Comparison: Cumulative Tasks Completed')
    ax2.legend(loc='lower right')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()