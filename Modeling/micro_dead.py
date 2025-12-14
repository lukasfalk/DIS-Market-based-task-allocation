import random
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. CONFIGURATION & CALIBRATION
# ==========================================
DT = 0.064

# --- MOVEMENT & WORK ---
AVG_TRAVEL_TIME = 15
#4.65+1.64 # Sum of travel, obstacle avoid, and average work times 
TIME_ROBOTA = 3  
TIME_ROBOTB = 1   

# --- BIDDING MECHANISM ---
TIME_IDLE_PER_TASK=7.27

#calculate the probability to end up as the time idle per task, given dt
BID_PROBABILITY = DT / TIME_IDLE_PER_TASK

# --- BATTERY CONSTRAINTS ---
# "it can move or complete a task only for 2 minutes"
MAX_BATTERY_LIFE = 120 

# Project Constraints
NUM_ROBOTS = 5
NUM_ROBOTA = 2 
NUM_ROBOTB = 3 
SIM_DURATION = 180.0 
NUM_RUNS = 50

# States
STATE_IDLE = 0
STATE_ACTIVE = 1
STATE_DEAD = 2   

# ==========================================
# 2. AGENT CLASS
# ==========================================
class Robot:
    def __init__(self, id, is_robota):
        self.id = id
        self.is_robota = is_robota
        self.state = STATE_IDLE
        self.timer = 0.0
        self.battery_used = 0.0 
        self.work_duration = TIME_ROBOTA if is_robota else TIME_ROBOTB

    def reset(self):
        self.state = STATE_IDLE
        self.timer = 0.0
        self.battery_used = 0.0

    def step(self, dt):
        """
        Advances state. Returns True if a task was JUST completed.
        """
        task_finished = False

        # 0. Check Battery Life (Global Check)
        if self.state == STATE_DEAD:
            return False

        # 1. IDLE -> ACTIVE
        if self.state == STATE_IDLE:
            if random.random() < BID_PROBABILITY:
                self.start_active()

        # 2. ACTIVE -> IDLE (Travel + Work)
        elif self.state == STATE_ACTIVE:
            self.timer -= dt
            self.battery_used += dt # Only active time consumes battery
            
            # Check for death DURING task
            if self.battery_used >= MAX_BATTERY_LIFE:
                self.state = STATE_DEAD # Shut down immediately
            
            # Check for task completion
            elif self.timer <= 0:
                self.finish_task()
                task_finished = True 
        
        return task_finished

    def start_active(self):
        self.state = STATE_ACTIVE
        travel_time = np.random.normal(AVG_TRAVEL_TIME, 2.0)
        if travel_time < 0: travel_time = 0
        self.timer = travel_time + self.work_duration 

    def finish_task(self):
        self.state = STATE_IDLE

    def is_active(self):
        return self.state == STATE_ACTIVE
    
    def is_dead(self):
        return self.state == STATE_DEAD

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
        self.reset()
        steps = int(duration / DT)
        
        active_history = np.zeros(steps)
        dead_history = np.zeros(steps)
        task_history = np.zeros(steps)
        
        current_total_tasks = 0

        for i in range(steps):
            active_now = 0
            dead_now = 0
            tasks_finished_this_step = 0
            
            for r in self.robots:
                if r.step(DT):
                    tasks_finished_this_step += 1
                
                if r.is_active():
                    active_now += 1
                if r.is_dead():
                    dead_now += 1
            
            current_total_tasks += tasks_finished_this_step
            
            active_history[i] = active_now
            dead_history[i] = dead_now
            task_history[i] = current_total_tasks

        return active_history, task_history, dead_history

# ==========================================
# 4. MAIN EXECUTION
# ==========================================
def main():
    print(f"--- Microscopic Model: Topic 4 (Task 3) ---")
    print(f"Battery Limit: {MAX_BATTERY_LIFE}s active time")

    world = World()
    steps = int(SIM_DURATION / DT)
    
    all_active = np.zeros((NUM_RUNS, steps))
    all_tasks = np.zeros((NUM_RUNS, steps))
    all_dead = np.zeros((NUM_RUNS, steps))

    for i in range(NUM_RUNS):
        act_hist, task_hist, dead_hist = world.run(SIM_DURATION)
        all_active[i, :] = act_hist
        all_tasks[i, :] = task_hist
        all_dead[i, :] = dead_hist

    # Aggregation
    mean_active = np.mean(all_active, axis=0)
    std_active = np.std(all_active, axis=0)
    
    mean_tasks = np.mean(all_tasks, axis=0)
    std_tasks = np.std(all_tasks, axis=0)
    
    mean_dead = np.mean(all_dead, axis=0)
    
    time_axis = np.linspace(0, SIM_DURATION, steps)

    # ==========================================
    # 5. PLOTTING
    # ==========================================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # Plot 1: Active vs Dead Robots
    ax1.plot(time_axis, mean_active, label='Active Robots', color='blue', linewidth=2)
    ax1.fill_between(time_axis, mean_active - std_active, mean_active + std_active, color='blue', alpha=0.2)
            
    ax1.set_ylabel('Robot Count')
    ax1.set_title('Metric 1: Robot Activity & Attrition')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='center right')
    
    # Set x-limits for the top plot (technically shared, but being explicit)
    ax1.set_xlim(0, SIM_DURATION)

    # Plot 2: Cumulative Tasks
    ax2.plot(time_axis, mean_tasks, label='Tasks Completed', color='green', linewidth=2)
    ax2.fill_between(time_axis, mean_tasks - std_tasks, mean_tasks + std_tasks, color='green', alpha=0.2)
    ax2.set_ylabel('Total Tasks')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_title('Metric 2: Cumulative Tasks Completed')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='lower right')
    
    # Set x-limits for the bottom plot
    ax2.set_xlim(0, SIM_DURATION)

    plt.tight_layout()
    print("Displaying plots...")
    plt.show()

if __name__ == "__main__":
    main()