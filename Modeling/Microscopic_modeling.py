import random
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. CONFIGURATION & CALIBRATION
# ==========================================
DT = 0.1 

# --- MOVEMENT & WORK ---
AVG_TRAVEL_TIME = 15.0 
TIME_ROBOTA = 3.0  
TIME_ROBOTB = 9.0   
TIME_OBSTACLE_AVOID = 2.0

# --- BIDDING MECHANISM ---
# Probabilistic (Dice Roll) - Chance per time step
# Value of 0.02 per step (dt=0.1s) implies an average wait of 5.0 seconds.
BID_PROBABILITY = 0.02 

# Project Constraints
NUM_ROBOTS = 5
NUM_ROBOTA = 2 
NUM_ROBOTB = 3 
SIM_DURATION = 180.0 
NUM_RUNS = 50 

# States
STATE_IDLE = 0
STATE_ACTIVE = 1   

# ==========================================
# 2. AGENT CLASS
# ==========================================
class Robot:
    def __init__(self, id, is_robota):
        self.id = id
        self.is_robota = is_robota
        self.state = STATE_IDLE
        self.timer = 0.0
        self.work_duration = TIME_ROBOTA if is_robota else TIME_ROBOTB

    def reset(self):
        self.state = STATE_IDLE
        self.timer = 0.0

    def step(self, dt):
        """
        Advances state. Returns True if a task was JUST completed.
        Logic: IDLE -> (Dice Roll) -> ACTIVE -> IDLE
        """
        task_finished = False

        # 1. IDLE -> ACTIVE (Pure Probabilistic Check)
        if self.state == STATE_IDLE:
            # Roll the dice. If < 0.02, we win the bid immediately.
            if random.random() < BID_PROBABILITY:
                self.start_active()

        # 2. ACTIVE -> IDLE (Travel + Work)
        elif self.state == STATE_ACTIVE:
            self.timer -= dt
            if self.timer <= 0:
                self.finish_task()
                task_finished = True # Signal completion
        
        return task_finished

    def start_active(self):
        """Transition directly from IDLE to ACTIVE."""
        self.state = STATE_ACTIVE
        # Add noise to travel time for realism
        travel_time = np.random.normal(AVG_TRAVEL_TIME, 2.0)
        if travel_time < 0: travel_time = 0
        self.timer = travel_time + self.work_duration + TIME_OBSTACLE_AVOID

    def finish_task(self):
        self.state = STATE_IDLE

    def is_active(self):
        return self.state == STATE_ACTIVE

# ==========================================
# 3. WORLD CLASS
# ==========================================
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
        """
        Runs a simulation. 
        Returns TWO arrays: 
          1. Active history (how many robots working)
          2. Task history (cumulative tasks finished)
        """
        self.reset()
        steps = int(duration / DT)
        
        active_history = np.zeros(steps)
        task_history = np.zeros(steps)
        
        current_total_tasks = 0

        for i in range(steps):
            active_now = 0
            tasks_finished_this_step = 0
            
            for r in self.robots:
                # Check if this robot finished a task this step
                if r.step(DT):
                    tasks_finished_this_step += 1
                
                if r.is_active():
                    active_now += 1
            
            current_total_tasks += tasks_finished_this_step
            
            # Store data
            active_history[i] = active_now
            task_history[i] = current_total_tasks

        return active_history, task_history

# ==========================================
# 4. MAIN EXECUTION
# ==========================================
def main():
    print(f"--- Microscopic Model: Topic 4 (Task 3) ---")
    print(f"Simulating {NUM_RUNS} runs...")
    print(f"Bidding Probability per step: {BID_PROBABILITY}")

    world = World()
    steps = int(SIM_DURATION / DT)
    
    # Storage for all runs
    all_active = np.zeros((NUM_RUNS, steps))
    all_tasks = np.zeros((NUM_RUNS, steps))

    for i in range(NUM_RUNS):
        act_hist, task_hist = world.run(SIM_DURATION)
        all_active[i, :] = act_hist
        all_tasks[i, :] = task_hist

    # --- AGGREGATION ---
    mean_active = np.mean(all_active, axis=0)
    std_active = np.std(all_active, axis=0)
    
    mean_tasks = np.mean(all_tasks, axis=0)
    std_tasks = np.std(all_tasks, axis=0)
    
    time_axis = np.arange(0, SIM_DURATION, DT)

    print("-" * 40)
    print(f"FINAL RESULTS (Averages at t={SIM_DURATION}s):")
    print(f"Active Robots: {mean_active[-1]:.2f}")
    print(f"Total Tasks Completed: {mean_tasks[-1]:.2f}")
    print("-" * 40)

    # ==========================================
    # 5. PLOTTING
    # ==========================================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # Plot 1: Active Robots
    ax1.plot(time_axis, mean_active, label='Mean Active', color='blue', linewidth=2)
    ax1.fill_between(time_axis, mean_active - std_active, mean_active + std_active, color='blue', alpha=0.2)
    ax1.axhline(y=NUM_ROBOTS, color='red', linestyle='--', label='Max Robots', alpha=0.5)
    ax1.set_ylabel('Active Robots')
    ax1.set_title('Metric 1: Average Active Robots over Time')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='lower right')

    # Plot 2: Cumulative Tasks
    ax2.plot(time_axis, mean_tasks, label='Mean Tasks Completed', color='green', linewidth=2)
    ax2.fill_between(time_axis, mean_tasks - std_tasks, mean_tasks + std_tasks, color='green', alpha=0.2)
    ax2.set_ylabel('Total Tasks Completed')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_title('Metric 2: Cumulative Tasks Completed')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='lower right')

    plt.tight_layout()
    print("Displaying plots...")
    plt.show()

if __name__ == "__main__":
    main()