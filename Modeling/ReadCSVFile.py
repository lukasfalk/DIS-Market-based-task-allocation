import re
import numpy as np
import matplotlib.pyplot as plt

filename = 'run1.csv'
sim_duration = 0
intervals = [] 
task_finish_times = []

p_robot_tag = re.compile(r'\[Robot (\d+) @ t=(\d+)ms\]')
p_travel_res = re.compile(r'used (\d+)ms traveling')
p_work_res = re.compile(r'used (\d+)ms at task')
p_start_travel = re.compile(r'Started travelling')

pending_travels = {}

with open(filename, 'r') as f:
    for line in f:
        m_tag = p_robot_tag.search(line)
        if m_tag:
            r_id = int(m_tag.group(1))
            t = int(m_tag.group(2))
            sim_duration = max(sim_duration, t)
            
            if p_start_travel.search(line):
                pending_travels[r_id] = t
            
            m_trav = p_travel_res.search(line)
            if m_trav:
                duration = int(m_trav.group(1))
                intervals.append((r_id, t - duration, t))
                if r_id in pending_travels: del pending_travels[r_id]
            
            m_work = p_work_res.search(line)
            if m_work:
                duration = int(m_work.group(1))
                intervals.append((r_id, t, t + duration))
                task_finish_times.append(t + duration)

for r_id, start_t in pending_travels.items():
    if start_t < sim_duration:
        intervals.append((r_id, start_t, sim_duration))

time_step = 100
time_axis = np.arange(0, sim_duration + time_step, time_step)
activity_grid = np.zeros((5, len(time_axis)))

for (r, start, end) in intervals:
    s_idx = max(0, int(start/time_step))
    e_idx = min(len(time_axis)-1, int(end/time_step))
    if s_idx <= e_idx:
        activity_grid[r, s_idx:e_idx+1] = 1

n_active = np.sum(activity_grid, axis=0)

n_completed = np.zeros(len(time_axis))
task_finish_times.sort()
curr = 0
for i, t in enumerate(time_axis):
    while curr < len(task_finish_times) and task_finish_times[curr] <= t:
        curr += 1
    n_completed[i] = curr

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

ax1.plot(time_axis/1000, n_active, label='Webots Active', color='blue', linewidth=2)
ax1.axhline(y=5, color='red', linestyle='--', label='Max Robots', alpha=0.5)
ax1.fill_between(time_axis/1000, n_active, color='blue', alpha=0.1)
ax1.set_ylabel('Active Robots')
ax1.set_title('Metric 1: Active Robots over Time')
ax1.legend(loc='lower right')
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 5.5)

ax2.plot(time_axis/1000, n_completed, label='Completed Tasks', color='green', linewidth=2)
ax2.fill_between(time_axis/1000, n_completed, color='green', alpha=0.1)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Total Completed Tasks')
ax2.set_title('Metric 2: Cumulative Tasks Completed')
ax2.legend(loc='lower right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('webots_simulation_plot_styled.png')