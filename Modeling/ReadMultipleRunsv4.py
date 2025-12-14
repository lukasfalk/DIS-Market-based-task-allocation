import re
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

# 1. Setup: Find files
filenames = glob.glob('run*.csv')
if not filenames:
    print("No run files found!")
    exit()

print(f"Processing {len(filenames)} files: {filenames}\n")

# 2. Regex Patterns
p_robot_tag = re.compile(r'\[Robot (\d+) @ t=(\d+)ms\]')
p_travel_res = re.compile(r'used (\d+)ms traveling')
p_work_res = re.compile(r'used (\d+)ms at task')
p_start_travel = re.compile(r'Started travelling')
p_avoid_total = re.compile(r'Total time any robot was near another robot or a wall: ([\d\.]+) seconds')

# 3. Parameters
time_step = 100
expected_robot_count = 5

# Aggregation for V3-style metrics
global_move_per_task = []
global_avoid_per_task = []
global_work_per_task = []
global_idle_per_task = []

# Aggregation for V1-style plots
all_active_arrays = []
all_completed_arrays = []
max_duration_global = 0

run_labels = []

# 4. Processing Loop
for filename in filenames:
    with open(filename, 'r') as f:
        content = f.read()
    
    lines = content.split('\n')
    
    # --- V3 Metrics Parsing ---
    travel_matches = p_travel_res.findall(content)
    total_travel_logged = sum(int(d) for d in travel_matches)
    
    work_matches = p_work_res.findall(content)
    total_work_logged = sum(int(d) for d in work_matches)
    num_tasks = len(work_matches)
    
    if num_tasks == 0:
        print(f"Skipping {filename}: No tasks found.")
        continue

    m_avoid = p_avoid_total.search(content)
    total_avoid_ms = float(m_avoid.group(1)) * 1000.0 if m_avoid else 0.0
    
    all_timestamps = p_robot_tag.findall(content)
    max_t = max(int(t) for _, t in all_timestamps) if all_timestamps else 0
    sim_duration = max_t
    
    # Derive V3 metrics
    pure_move_ms = max(0, total_travel_logged - total_avoid_ms)
    total_capacity = sim_duration * expected_robot_count
    total_active_ms = total_travel_logged + total_work_logged
    total_idle_ms = max(0, total_capacity - total_active_ms)
    
    move_per_task = pure_move_ms / num_tasks
    avoid_per_task = total_avoid_ms / num_tasks
    work_per_task = total_work_logged / num_tasks
    idle_per_task = total_idle_ms / num_tasks
    
    global_move_per_task.append(move_per_task)
    global_avoid_per_task.append(avoid_per_task)
    global_work_per_task.append(work_per_task)
    global_idle_per_task.append(idle_per_task)
    
    run_labels.append(os.path.basename(filename))
    
    print(f"Run: {filename}")
    print(f"  Tasks: {num_tasks}")
    print(f"  Breakdown (per Task): Move={move_per_task:.0f}ms, Avoid={avoid_per_task:.0f}ms, Work={work_per_task:.0f}ms, Idle={idle_per_task:.0f}ms")

    # --- V1 Timeline Parsing ---
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

    # Handle pending travels
    for r_id, start_t in pending_travels.items():
        if start_t < sim_duration:
            intervals.append((r_id, start_t, sim_duration))

    # Prepare timeline data for V1-style plots
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

# 5. Final Average Metrics (V3 style)
if not global_move_per_task:
    print("No valid runs to average.")
    exit()

avg_move = np.mean(global_move_per_task)
avg_avoid = np.mean(global_avoid_per_task)
avg_work = np.mean(global_work_per_task)
avg_idle = np.mean(global_idle_per_task)

print("\n" + "=" * 40)
print("      FINAL AVERAGE METRICS (PER TASK)")
print("=" * 40)
print(f"Time Idle per Task:     {avg_idle / 1000:.2f} s")
print(f"Time Moving per Task:   {avg_move / 1000:.2f} s")
print(f"Time Working per Task:  {avg_work / 1000:.2f} s")
print(f"Time Avoiding per Task: {avg_avoid / 1000:.2f} s")
print("=" * 40 + "\n")

# 6. Plotting Aggregation (V1 style)
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

# Apply moving average to smooth the active robots curve
window_size = 20  # Adjust this value to control smoothing (higher = smoother)
mean_active_smooth = np.convolve(mean_active, np.ones(window_size) / window_size, mode='same')

# 7. Generate Plots (V1 style)
fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot 1: Active Robots (left y-axis)
ax1.plot(common_time_axis / 1000, mean_active_smooth, label='Avg Active Robots (Smoothed)', color='blue', linewidth=2)
ax1.fill_between(common_time_axis / 1000, mean_active_smooth - std_active, mean_active_smooth + std_active, color='blue', alpha=0.1, label='Std Dev (Active)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Active Robots', color='blue')
ax1.tick_params(axis='y', labelcolor='blue')
ax1.set_ylim(0, expected_robot_count + 0.5)
ax1.grid(True, alpha=0.3)

# Plot 2: Completed Tasks (right y-axis)
ax2 = ax1.twinx()
ax2.plot(common_time_axis / 1000, mean_completed, label='Avg Completed Tasks', color='green', linewidth=2)
ax2.fill_between(common_time_axis / 1000, mean_completed - std_completed, mean_completed + std_completed, color='green', alpha=0.1, label='Std Dev (Tasks)')
ax2.set_ylabel('Total Completed Tasks', color='green')
ax2.set_ylim(0, 75)
ax2.tick_params(axis='y', labelcolor='green')

# Combined legend
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='lower center')

plt.title(f'Active Robots & Cumulative Tasks Completed (over {len(filenames)} runs)')
plt.tight_layout()
plt.show()
