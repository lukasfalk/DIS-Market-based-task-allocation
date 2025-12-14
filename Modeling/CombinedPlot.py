import re
import random
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

# ==========================================
# PART 1: MICRO_DEAD MODEL CONFIGURATION
# ==========================================
DT = 0.064

# --- MOVEMENT & WORK ---
AVG_TRAVEL_TIME = 8
TIME_ROBOTA = 3  
TIME_ROBOTB = 1   

# --- BIDDING MECHANISM ---
TIME_IDLE_PER_TASK = 3
BID_PROBABILITY = DT / TIME_IDLE_PER_TASK

# --- BATTERY CONSTRAINTS ---
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
# MICRO_DEAD AGENT CLASS
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
        task_finished = False
        if self.state == STATE_DEAD:
            return False
        if self.state == STATE_IDLE:
            if random.random() < BID_PROBABILITY:
                self.start_active()
        elif self.state == STATE_ACTIVE:
            self.timer -= dt
            self.battery_used += dt
            if self.battery_used >= MAX_BATTERY_LIFE:
                self.state = STATE_DEAD
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
# PART 2: RUN MICRO_DEAD MODEL
# ==========================================
def run_micro_dead():
    print(f"--- Running Microscopic Model with Battery Limit ({MAX_BATTERY_LIFE}s) ---")
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

    mean_active = np.mean(all_active, axis=0)
    std_active = np.std(all_active, axis=0)
    mean_tasks = np.mean(all_tasks, axis=0)
    std_tasks = np.std(all_tasks, axis=0)
    mean_dead = np.mean(all_dead, axis=0)
    time_axis = np.linspace(0, SIM_DURATION, steps)

    print(f"  Micro Dead - Avg Active: {np.mean(mean_active):.2f}, Tasks: {mean_tasks[-1]:.1f}")
    
    return time_axis, mean_active, std_active, mean_tasks, std_tasks, mean_dead

# ==========================================
# PART 3: READ MULTIPLE RUNS (SUBMICROSCOPIC)
# ==========================================
def run_submicroscopic():
    print("--- Reading Submicroscopic Run Files ---")
    
    # Parameters
    time_step = 100
    expected_robot_count = 5
    
    filenames = glob.glob('run*.csv')
    if not filenames:
        print("No run files found!")
        return None, None, None, None, None
    
    print(f"Processing {len(filenames)} files: {filenames}\n")
    
    all_active_arrays = []
    all_completed_arrays = []
    max_duration_global = 0
    
    # Regex Patterns
    p_robot_tag = re.compile(r'\[Robot (\d+) @ t=(\d+)ms\]')
    p_travel_res = re.compile(r'used (\d+)ms traveling')
    p_work_res = re.compile(r'used (\d+)ms at task')
    p_start_travel = re.compile(r'Started travelling')
    
    for filename in filenames:
        with open(filename, 'r') as f:
            content = f.read()
        lines = content.split('\n')
        
        all_timestamps = p_robot_tag.findall(content)
        max_t = max(int(t) for _, t in all_timestamps) if all_timestamps else 0
        sim_duration = max_t
        
        intervals = []
        task_finish_times = []
        pending_travels = {}
        robot_stats = {r: {'move': 0, 'work': 0} for r in range(expected_robot_count)}
        
        for line in lines:
            m_tag = p_robot_tag.search(line)
            if m_tag:
                r_id = int(m_tag.group(1))
                t = int(m_tag.group(2))
                if r_id not in robot_stats:
                    robot_stats[r_id] = {'move': 0, 'work': 0}
                if p_start_travel.search(line):
                    pending_travels[r_id] = t
                m_trav = p_travel_res.search(line)
                if m_trav:
                    duration = int(m_trav.group(1))
                    intervals.append((r_id, t - duration, t))
                    robot_stats[r_id]['move'] += duration
                    if r_id in pending_travels:
                        del pending_travels[r_id]
                m_work = p_work_res.search(line)
                if m_work:
                    duration = int(m_work.group(1))
                    intervals.append((r_id, t, t + duration))
                    robot_stats[r_id]['work'] += duration
                    task_finish_times.append(t + duration)
        
        for r_id, start_t in pending_travels.items():
            if start_t < sim_duration:
                intervals.append((r_id, start_t, sim_duration))
        
        max_duration_global = max(max_duration_global, sim_duration)
        
        run_time_axis = np.arange(0, sim_duration + time_step, time_step)
        max_r_id = max(robot_stats.keys()) if robot_stats else expected_robot_count - 1
        activity_grid = np.zeros((max_r_id + 1, len(run_time_axis)))
        
        for (r, start, end) in intervals:
            s_idx = max(0, int(start / time_step))
            e_idx = min(len(run_time_axis) - 1, int(end / time_step))
            if s_idx <= e_idx:
                activity_grid[r, s_idx:e_idx + 1] = 1
        
        n_active = np.sum(activity_grid, axis=0)
        n_completed = np.zeros(len(run_time_axis))
        task_finish_times.sort()
        curr = 0
        for i, t in enumerate(run_time_axis):
            while curr < len(task_finish_times) and task_finish_times[curr] <= t:
                curr += 1
            n_completed[i] = curr
        
        all_active_arrays.append(n_active)
        all_completed_arrays.append(n_completed)
    
    # Aggregate
    common_time_axis = np.arange(0, max_duration_global + time_step, time_step)
    final_len = len(common_time_axis)
    
    padded_active = []
    padded_completed = []
    
    for arr in all_active_arrays:
        pad_width = final_len - len(arr)
        if pad_width > 0:
            padded_arr = np.pad(arr, (0, pad_width), 'constant', constant_values=0)
        else:
            padded_arr = arr[:final_len]
        padded_active.append(padded_arr)
    
    for arr in all_completed_arrays:
        pad_width = final_len - len(arr)
        if pad_width > 0:
            padded_arr = np.pad(arr, (0, pad_width), 'edge')
        else:
            padded_arr = arr[:final_len]
        padded_completed.append(padded_arr)
    
    mean_active = np.mean(padded_active, axis=0)
    mean_completed = np.mean(padded_completed, axis=0)
    std_active = np.std(padded_active, axis=0)
    std_completed = np.std(padded_completed, axis=0)
    
    # Apply moving average
    window_size = 20
    mean_active_smooth = np.convolve(mean_active, np.ones(window_size) / window_size, mode='same')
    
    print(f"  Submicro - Avg Active: {np.mean(mean_active):.2f}, Tasks: {mean_completed[-1]:.1f}")
    
    return common_time_axis / 1000, mean_active_smooth, std_active, mean_completed, std_completed, len(padded_active)

# ==========================================
# PART 4: COMBINED PLOTTING
# ==========================================
def main():
    # Run both models
    micro_time, micro_active, micro_std_active, micro_tasks, micro_std_tasks, micro_dead = run_micro_dead()
    submicro_result = run_submicroscopic()
    
    if submicro_result[0] is None:
        print("Submicroscopic data not available. Plotting micro_dead only.")
        submicro_time = None
    else:
        submicro_time, submicro_active, submicro_std_active, submicro_tasks, submicro_std_tasks, num_runs = submicro_result
    
    # ==========================================
    # PLOTTING
    # ==========================================
    fig, ax1 = plt.subplots(figsize=(12, 7))
    
    # Colors: Submicroscopic = Blue/Green, Microscopic = Purple/Orange
    color_submicro_active = 'blue'
    color_submicro_tasks = 'green'
    color_micro_active = 'purple'
    color_micro_tasks = 'darkorange'
    
    # --- Left Y-Axis: Active Robots ---
    # Submicroscopic (if available)
    if submicro_time is not None:
        ax1.plot(submicro_time, submicro_active, label='Submicro Active Robots', 
                 color=color_submicro_active, linewidth=2)
        ax1.fill_between(submicro_time, submicro_active - submicro_std_active, 
                         submicro_active + submicro_std_active, 
                         color=color_submicro_active, alpha=0.1)
    
    # Microscopic
    ax1.plot(micro_time, micro_active, label='Micro Active Robots', 
             color=color_micro_active, linewidth=2, linestyle='--')
    ax1.fill_between(micro_time, micro_active - micro_std_active, 
                     micro_active + micro_std_active, 
                     color=color_micro_active, alpha=0.1)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Active Robots')
    ax1.tick_params(axis='y')
    ax1.set_ylim(0, NUM_ROBOTS + 0.5)
    ax1.set_xlim(0, SIM_DURATION)
    ax1.legend(loc='lower left')
    
    # --- Right Y-Axis: Completed Tasks ---
    ax2 = ax1.twinx()
    
    # Submicroscopic (if available)
    if submicro_time is not None:
        ax2.plot(submicro_time, submicro_tasks, label='Submicro Tasks', 
                 color=color_submicro_tasks, linewidth=2)
        ax2.fill_between(submicro_time, submicro_tasks - submicro_std_tasks, 
                         submicro_tasks + submicro_std_tasks, 
                         color=color_submicro_tasks, alpha=0.1)
    
    # Microscopic
    ax2.plot(micro_time, micro_tasks, label='Micro Tasks', 
             color=color_micro_tasks, linewidth=2, linestyle='--')
    ax2.fill_between(micro_time, micro_tasks - micro_std_tasks, 
                     micro_tasks + micro_std_tasks, 
                     color=color_micro_tasks, alpha=0.1)
    
    ax2.set_ylabel('Total Completed Tasks')
    ax2.tick_params(axis='y')
    ax2.legend(loc='lower right')
    ax2.grid(True, alpha=0.3)
    
    # --- Textboxes ---
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    
    # Active Robots textbox (both models)
    if submicro_time is not None:
        textstr_active = f'Active Robots\nSubmicro: {np.mean(submicro_active):.2f} ± {np.mean(submicro_std_active):.2f}\nMicro: {np.mean(micro_active):.2f} ± {np.mean(micro_std_active):.2f}'
    else:
        textstr_active = f'Active Robots\nMicro: {np.mean(micro_active):.2f} ± {np.mean(micro_std_active):.2f}'
    ax1.text(0.02, 0.98, textstr_active, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', bbox=props)
    
    # Completed Tasks textbox (both models)
    if submicro_time is not None:
        textstr_tasks = f'Completed Tasks\nSubmicro: {submicro_tasks[-1]:.1f} ± {submicro_std_tasks[-1]:.1f}\nMicro: {micro_tasks[-1]:.1f} ± {micro_std_tasks[-1]:.1f}'
    else:
        textstr_tasks = f'Completed Tasks\nMicro: {micro_tasks[-1]:.1f} ± {micro_std_tasks[-1]:.1f}'
    ax2.text(0.65, 0.98, textstr_tasks, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', bbox=props)
    
    plt.title('Submicroscopic vs Microscopic Model with battery life')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
