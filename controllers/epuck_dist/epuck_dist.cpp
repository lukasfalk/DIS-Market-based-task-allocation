/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_dist.cpp
 * description: Distributed controller.
 *              Self-allocates tasks and negotiates with neighbors.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/radio.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <algorithm>
#include <array>
#include <vector>

#include "../common/communication.hpp"
#include "../common/constants.hpp"
#include "../common/geometry.hpp"
#include "../common/logging.hpp"
#include "../common/pathfinding.hpp"
#include "../common/utils.hpp"

WbDeviceTag left_motor;   // handler for left wheel of the robot
WbDeviceTag right_motor;  // handler for the right wheel of the robot

// Logging macro, configured with prefix "[Robot {ID} @ t={TIME}ms] "
#define LOG(fmt, ...)                                                                  \
    do {                                                                               \
        char prefix[64];                                                               \
        snprintf(prefix, sizeof(prefix), "[Robot %d @ t=%dms] ", robot_id, sim_clock); \
        logMsg(prefix, fmt, ##__VA_ARGS__);                                            \
    } while (0)

enum class RobotState {
    STAY = 1,
    GO_TO_GOAL = 2,  // Initial state aliases
    OBSTACLE_AVOID = 3,
    RANDOM_WALK = 4,
    DEAD = 5,  // Robot is out of battery
};

// The state variables
constexpr RobotState DEFAULT_STATE = RobotState::STAY;
RobotState state;                // State of the robot
int sim_clock;                   // Simulation clock in milliseconds
uint16_t robot_id;               // Unique robot ID
RobotSpec robot_specialization;  // Specialization type of this robot wrt task types (A or B)
double my_pos[3];                // X, Z, Theta of this robot

// Represents the robot's local belief about a single task
struct LocalTaskInfo {
    uint16_t id;
    Point2d pos;
    TaskType type;

    bool isBeingCompleted;  // Do I believe someone is working on this task?
    bool isCompleted;       // Do I believe this task is done?
    int assignedRobotId;    // Who do I think is doing this? (-1 if free)
    double bestBidSeen;     // The best bid I've heard for this task (from assignedRobotId)

    int lastUpdateTimestamp;  // For cleaning up stale assignments (optional but good for robustness)
};
std::vector<LocalTaskInfo> world_state;  // Current known tasks
int current_task_id = -1;                // ID of the task I am currently trying to do
double current_bid_val = 0.0;            // My cost for this task
int last_completed_task_id = -1;         // ID of the last task I completed

int lmsg, rmsg;  // Communication variables
int indx;        // Event index to be sent to the supervisor

std::vector<float> buff;  // Buffer for physics plugin

// Pathfinding and waypoint following
PathPlanner planner;                 // Global path planner instance
std::vector<Point2d> waypoint_path;  // Computed path waypoints
int current_waypoint_idx = 0;        // Index of next waypoint to reach
bool target_valid = false;           // boolean; whether we are actively pursuing a target

double stat_max_velocity;

// Pause control: when clock < pause_until the robot stays paused
int pause_until = 0;
int pause_active = 0;

int battery_time_used = 0;  // Time spent traveling and at tasks (ms)
int travel_start_time = 0;  // Timestamp when this travel phase started (ms)

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
constexpr std::array<int, 16> Interconn = {17,  29,  34,  10, 8,  -38, -56, -76,
                                           -72, -58, -36, 8,  10, 36,  28,  18};

// Helper to find task index in local vector
int get_task_index(uint16_t task_id) {
    for (size_t i = 0; i < world_state.size(); ++i) {
        if (world_state[i].id == task_id) return i;
    }
    return -1;
}

// Return pause duration (ms) based on robot specialization and task type (customize as needed)
static int get_pause_duration_ms(RobotSpec robot_specialization, TaskType task_type) {
    switch (task_type) {
        case TASK_TYPE_A:
            return (robot_specialization == ROBOT_SPEC_A) ? 3000 : 9000;
        case TASK_TYPE_B:
            return (robot_specialization == ROBOT_SPEC_A) ? 5000 : 1000;
        default:
            return 0;  // Fallback but should never happen
    }
}

void allocate_task() {
    // 1. If I am already assigned a valid, incomplete task, check if I should keep it.
    //    (Conflict resolution happens in receive_updates, so if I'm here, I'm good).
    if (current_task_id != -1) {
        int idx = get_task_index(current_task_id);
        if (idx != -1 && !world_state[idx].isCompleted && world_state[idx].assignedRobotId == robot_id) {
            // I own this task and it's not done. Keep going.
            return;
        }
        // If task is done or stolen, reset:
        current_task_id = -1;
        target_valid = false;
        waypoint_path.clear();
        state = RobotState::STAY;
    }

    // 2. Find the best unassigned task
    double best_cost = std::numeric_limits<double>::infinity();
    int best_task_idx = -1;

    for (size_t i = 0; i < world_state.size(); ++i) {
        // Skip completed or being-completed tasks
        if (world_state[i].isCompleted || world_state[i].isBeingCompleted) continue;

        // Skip tasks assigned to OTHERS (respect their bid)
        if (world_state[i].assignedRobotId != -1 && world_state[i].assignedRobotId != robot_id) continue;

        // Calculate Cost (Distance/Path + Battery + Specialization)
        // Note: Re-use your pathfinding/cost logic here!
        Point2d start = {my_pos[0], my_pos[1]};
        Path p = planner.findPath(start, world_state[i].pos);

        // If path failed, skip
        if (p.waypoints.empty() && utils::dist(start, world_state[i].pos) > 0.05) continue;

        double dist = p.waypoints.empty() ? utils::dist(start, world_state[i].pos) : p.totalDistance;

        // Cost Function (Same as centralized for fairness)
        int time_travel = (dist / 0.5) * 1000 + 1000;
        int time_at_task = get_pause_duration_ms(robot_specialization, world_state[i].type);
        int battery_left = MAX_BATTERY_LIFETIME - battery_time_used;

        // Battery check
        if (time_travel + time_at_task > battery_left) continue;

        double cost = 1.5 * time_travel + time_at_task - 0.01 * battery_left;

        if (cost < best_cost) {
            best_cost = cost;
            best_task_idx = i;
        }
    }

    // 3. Assign myself
    if (best_task_idx != -1) {
        current_task_id = world_state[best_task_idx].id;
        current_bid_val = best_cost;

        // Update local belief
        world_state[best_task_idx].assignedRobotId = robot_id;
        world_state[best_task_idx].bestBidSeen = current_bid_val;

        // Trigger movement
        target_valid = true;
        // Logic to start moving will happen in run() -> compute_path_to_target()
        LOG("Self-allocated Task %d with cost %.2f\n", current_task_id, current_bid_val);
    }
}

// Check if we received a message and extract information
static void receive_updates() {
    // 1. Process all incoming messages
    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const void* data = wb_receiver_get_data(receiver_tag);
        size_t size = wb_receiver_get_data_size(receiver_tag);

        // Determine message type by size (hacky but standard in simple Webots sims)
        // OR rely on message structure.

        // CASE A: Supervisor Message (Global State / GPS)
        if (size == sizeof(MessageT)) {
            MessageT msg;
            memcpy(&msg, data, sizeof(MessageT));

            // Only accept GPS if addressed to me
            if (msg.msgType == MSG_GPS_ONLY && msg.robotId == robot_id) {
                my_pos[0] = msg.robotX;
                my_pos[1] = msg.robotY;
                my_pos[2] = msg.heading;
            } else if (msg.msgType == MSG_QUIT) {
                // Simulation is done!
                // Output remaining battery life, clean up and exit
                LOG("END OF SIM! Battery used: %dms (%.2f %% of total)\n", battery_time_used,
                    (battery_time_used * 100.0) / MAX_BATTERY_LIFETIME);

                // Stop motors
                wb_motor_set_velocity(left_motor, 0);
                wb_motor_set_velocity(right_motor, 0);
                wb_robot_step(TIME_STEP);

                wb_robot_cleanup();
                exit(0);
            }
            // CASE A.2: Supervisor announcing active tasks (Global Knowledge)
            else if (msg.msgType == MSG_EVENT_NEW) {
                // Check if we already know this task
                int idx = get_task_index(msg.eventId);
                if (idx == -1) {
                    // New task discovered! Add to world state.
                    LocalTaskInfo info;
                    info.id = msg.eventId;
                    info.pos = {msg.eventX, msg.eventY};
                    info.type = msg.taskType;
                    info.isBeingCompleted = false;
                    info.isCompleted = false;
                    info.assignedRobotId = -1;
                    info.bestBidSeen = 99999999.0;
                    world_state.push_back(info);
                } else {
                    // Task exists. If supervisor says it exists, it is NOT completed.
                    // This handles re-spawned tasks or tasks we thought were done but weren't.
                    if (world_state[idx].isCompleted) {
                        // Edge case: Task ID reuse? Assuming unique IDs increment,
                        // this shouldn't happen unless we missed a delete.
                        LOG("Warning: Task %d @(%f, %f) re-appeared after being marked done. Possible ID "
                            "reuse.\n",
                            msg.eventId, world_state[idx].pos.x, world_state[idx].pos.y);
                    }
                }
            }
        }  // CASE B: Neighbor Robot Message (RobotStateMsg)
        else if (size == sizeof(RobotStateMsg)) {
            RobotStateMsg msg;
            memcpy(&msg, data, sizeof(RobotStateMsg));

            // This is where Distributed Conflict Resolution happens
            int idx = get_task_index(msg.currentTaskId);
            if (idx != -1) {
                // 1. Update completion status
                if (msg.isTaskComplete) {
                    // Task is fully complete
                    world_state[idx].isCompleted = true;
                    world_state[idx].isBeingCompleted = false;
                    if (current_task_id == msg.currentTaskId) {
                        // Someone finished my task! Stop.
                        current_task_id = -1;
                        target_valid = false;
                    }
                } else if (msg.isTaskBeingCompleted) {
                    // Task is being worked on (robot arrived, working on it)
                    world_state[idx].isBeingCompleted = true;
                    if (current_task_id == msg.currentTaskId) {
                        // Someone is handling my task! Stop.
                        current_task_id = -1;
                        target_valid = false;
                    }
                }
                // 2. Conflict Resolution (only if not being worked on or completed)
                else if (!world_state[idx].isCompleted && !world_state[idx].isBeingCompleted) {
                    // If neighbor claims this task with a BETTER bid
                    if (msg.currentBid < world_state[idx].bestBidSeen) {
                        world_state[idx].assignedRobotId = msg.robotId;
                        world_state[idx].bestBidSeen = msg.currentBid;

                        // If *I* was targeting this, I must yield
                        if (current_task_id == msg.currentTaskId) {
                            LOG("Yielding Task %d @(%f, %f) to Robot %d (Bid %.2f vs %.2f)\n",
                                msg.currentTaskId, world_state[idx].pos.x, world_state[idx].pos.y,
                                msg.robotId, msg.currentBid, current_bid_val);
                            current_task_id = -1;
                            target_valid = false;
                            // allocate_task() will run next loop and pick a new one
                        }
                    }
                }
            }
        }
        wb_receiver_next_packet(receiver_tag);
    }
    // 2. Broadcast My State (Heartbeat)
    // We broadcast every time step. In a real system, we might do it less,
    // but in simulation, it ensures rapid conflict resolution.
    RobotStateMsg my_msg;
    my_msg.robotId = robot_id;

    if (pause_active && last_completed_task_id != -1) {
        // I am currently "treating the victim". Tell neighbors the task is being worked on.
        my_msg.currentTaskId = last_completed_task_id;
        my_msg.currentBid = 0.0;
        my_msg.isTaskBeingCompleted = true;
        my_msg.isTaskComplete = false;  // Not yet complete, still working
        wb_emitter_set_channel(emitter_tag, WB_CHANNEL_BROADCAST);
        wb_emitter_send(emitter_tag, &my_msg, sizeof(my_msg));
    } else if (current_task_id != -1) {
        // I am moving toward a task. Tell neighbors my bid so they can yield.
        my_msg.currentTaskId = current_task_id;
        my_msg.currentBid = current_bid_val;
        my_msg.isTaskBeingCompleted = false;
        my_msg.isTaskComplete = false;
        wb_emitter_set_channel(emitter_tag, WB_CHANNEL_BROADCAST);
        wb_emitter_send(emitter_tag, &my_msg, sizeof(my_msg));
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RESET and INIT (combined in function reset()) */
void reset(void) {
    wb_robot_init();
    int i;

    // get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    char s[4] = "ps0";
    for (i = 0; i < NB_SENSORS; i++) {
        // the device name is specified in the world file
        ds[i] = wb_robot_get_device(s);
        s[2]++;  // increases the device number
        wb_distance_sensor_enable(ds[i], 64);
    }

    sim_clock = 0;
    indx = 0;
    pause_until = 0;
    pause_active = 0;
    battery_time_used = 0;
    travel_start_time = 0;

    current_task_id = -1;
    current_bid_val = 0.0;
    last_completed_task_id = -1;
    world_state.clear();
    waypoint_path.clear();

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name;
    robot_name = (char*)wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck%d", &tmp_id)) {
        robot_id = (uint16_t)tmp_id;
        LOG("Initialized. My name is %s\n", robot_name);
    } else {
        fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name);
        exit(1);
    }

    // Am I used in this simulation?
    if (robot_id >= NUM_ROBOTS) {
        fprintf(stderr, "Robot %d is not needed. exiting ...\n", robot_id);
        wb_robot_cleanup();
        exit(0);
    }

    // Set robot specialization based on ID (first two robots are ROBOT_SPEC_A, rest are ROBOT_SPEC_B)
    robot_specialization = (robot_id <= 1) ? ROBOT_SPEC_A : ROBOT_SPEC_B;

    /* vvv  OBSOLETE (CENTRALIZED RADIO)  vvv
    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id + 1);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD);  // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id + 1);
    */

    // IMPORTANT: Distributed communication uses the broadcast channel
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, WB_CHANNEL_BROADCAST);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, TIME_STEP);
    wb_receiver_set_channel(receiver_tag, WB_CHANNEL_BROADCAST);

    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}

void update_state(int _sum_distances) {
    if (_sum_distances > STATECHANGE_DIST && state != RobotState::STAY) {
        state = RobotState::OBSTACLE_AVOID;
    } else if (current_task_id != -1) {
        state = RobotState::GO_TO_GOAL;
    } else {
        state = DEFAULT_STATE;
    }
}

/* vvv  NOT USED ANYMORE  vvv
// Compute optimal path to next target using visibility graph pathfinding
// Returns true if a path was found, false otherwise
int compute_path_to_target() {
    if (target_valid && waypoint_path.empty() && !targets.empty()) {
        // Convert current position to Point2d
        Point2d start = {my_pos[0], my_pos[1]};
        Point2d goal = {targets[0].x, targets[0].y};

        LOG("Called compute_path_to_target(): start is my pos (%.2f, %.2f), goal is first task "
            "in list (%.2f, %.2f)\n",
            my_pos[0], my_pos[1], targets[0].x, targets[0].y);

        // Call pathfinding to get the waypoint path
        Path path = planner.findPath(start, goal);

        if (!path.waypoints.empty()) {
            waypoint_path = path.waypoints;
            current_waypoint_idx = 0;
            LOG("  > New path with %d waypoints, distance %.3f\n", (int)waypoint_path.size(),
                path.totalDistance);
            return 1;
        } else {
            LOG("  > Failed to compute path to target (%.3f, %.3f)\n", goal.x, goal.y);
            waypoint_path.clear();
            return 0;
        }
    }
    return 0;
}*/

void update_self_motion(int msl, int msr) {
    double theta = my_pos[2];

    // Compute deltas of the robot
    double dr = (double)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = (double)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl) / 2.0;
    double dtheta = (dr - dl) / AXLE_LENGTH;

    // Compute deltas in the environment
    double dx = du * cosf(theta);
    double dy = du * sinf(theta);

    // Update position
    my_pos[0] += dx;
    my_pos[1] -= dy;
    my_pos[2] -= dtheta;

    // Keep orientation within 0, 2pi
    if (my_pos[2] > 2 * M_PI) {
        my_pos[2] -= 2.0 * M_PI;
    }
    if (my_pos[2] < 0) {
        my_pos[2] += 2.0 * M_PI;
    }

    // Keep track of highest velocity for modelling
    double velocity = du * 1000.0 / (double)TIME_STEP;
    if (state == RobotState::GO_TO_GOAL && velocity > stat_max_velocity) {
        stat_max_velocity = velocity;
    }
}

// Compute wheel speed to avoid obstacles
void compute_avoid_obstacle(int* msl, int* msr, int distances[]) {
    int d1 = 0, d2 = 0;  // Motor speed 1 and 2
    int sensor_nb;       // FOR-loop counters
    // const int sensor_sensitivity = 50;  // Not sure what this does

    for (sensor_nb = 0; sensor_nb < NB_SENSORS; sensor_nb++) {
        // d1 += (distances[sensor_nb] - sensor_sensitivity) * Interconn[sensor_nb] * 5;
        // d2 += (distances[sensor_nb] - sensor_sensitivity) * Interconn[sensor_nb + NB_SENSORS] * 5;
        d1 += (distances[sensor_nb]) * Interconn[sensor_nb] * 1;
        d2 += (distances[sensor_nb]) * Interconn[sensor_nb + NB_SENSORS] * 1;
    }

    // Normalizing speeds
    d1 /= 20;
    d2 /= 20;

    *msr = d1 + BIAS_SPEED;
    *msl = d2 + BIAS_SPEED;
    utils::limit(msl, MAX_SPEED);
    utils::limit(msr, MAX_SPEED);
}

// Computes wheel speed to move towards a goal point using proportional control
// This is a generalized version that can be used for any goal, not just the first target
void compute_go_to_point(int* msl, int* msr, double goal_x, double goal_y) {
    // Compute vector to goal
    float a = goal_x - my_pos[0];
    float b = goal_y - my_pos[1];
    // Compute wanted position from goal position and current location in robot coordinates
    float x = a * cosf(my_pos[2]) - b * sinf(my_pos[2]);  // x in robot coordinates
    float y = a * sinf(my_pos[2]) + b * cosf(my_pos[2]);  // y in robot coordinates

    float Ku = 0.2;               // Forward control coefficient
    float Kw = 20.0;              // Rotational control coefficient
    float range = 1;              // sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);  // Orientation of the wanted position

    // Compute forward control
    float u = Ku * range * cosf(bearing);
    u = std::max(0.0f, u);  // No backward movement
    // Compute rotational control
    float w = Kw * range * sinf(bearing);

    // Convert to wheel speeds!
    *msl = 50 * (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    *msr = 50 * (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    utils::limit(msl, MAX_SPEED);
    utils::limit(msr, MAX_SPEED);
}

// RUN e-puck
void run(int ms) {
    // --- 1. SENSORS & BATTERY ---
    int msl = 0, msr = 0;
    int distances[NB_SENSORS];
    int sum_distances = 0;

    for (int i = 0; i < NB_SENSORS; i++) {
        distances[i] = wb_distance_sensor_get_value(ds[i]);
        sum_distances += distances[i];
    }

    // Check battery
    if (battery_time_used >= MAX_BATTERY_LIFETIME) {
        state = RobotState::DEAD;
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        // We stop everything here.
        sim_clock += ms;
        return;
    }

    // --- 2. COMMUNICATE (RECEIVE) ---
    // Read messages from Supervisor (GPS/New Tasks) and Neighbors (Conflict Resolution)
    receive_updates();

    // --- 3. DECISION (ALLOCATE) ---
    // If not currently working (paused) and not dead, try to pick a task.
    // If we already have a task, this function checks if we should keep it.
    if (!pause_active && state != RobotState::DEAD) {
        allocate_task();
    }

    // --- 4. BROADCAST HEARTBEAT ---
    // We must tell neighbors what we are doing every step (or every N steps).
    // This allows neighbors to know if they should yield to us.
    if (current_task_id != -1) {
        RobotStateMsg msg;
        msg.robotId = robot_id;
        msg.currentTaskId = current_task_id;
        msg.currentBid = current_bid_val;
        msg.isTaskBeingCompleted = false;
        msg.isTaskComplete = false;

        // Broadcast to everyone (Channel -1 is standard broadcast in Webots if not set otherwise,
        // but here we use WB_CHANNEL_BROADCAST constant if defined, or -1)
        wb_emitter_set_channel(emitter_tag, WB_CHANNEL_BROADCAST);
        wb_emitter_send(emitter_tag, &msg, sizeof(msg));
    }

    // --- 5. PAUSE LOGIC (WORKING ON TASK) ---
    if (pause_active) {
        if (sim_clock < pause_until) {
            msl = 0;
            msr = 0;  // Stay still
        } else {
            // Task is NOW actually complete - broadcast completion to supervisor
            if (last_completed_task_id != -1) {
                RobotStateMsg done_msg;
                done_msg.robotId = robot_id;
                done_msg.currentTaskId = last_completed_task_id;
                done_msg.currentBid = 0;
                done_msg.isTaskBeingCompleted = false;
                done_msg.isTaskComplete = true;  // NOW it's truly complete
                wb_emitter_set_channel(emitter_tag, WB_CHANNEL_BROADCAST);
                wb_emitter_send(emitter_tag, &done_msg, sizeof(done_msg));
                LOG("Task %d fully completed after working pause.\n", last_completed_task_id);
                last_completed_task_id = -1;  // Reset after broadcasting
            }
            pause_active = 0;          // Wake up
            state = RobotState::STAY;  // Ready to move again
        }
    }
    // --- 6. MOVEMENT STATE MACHINE ---
    else {
        // Update state based on sensors (Obstacle Avoidance override)
        update_state(sum_distances);

        switch (state) {
            case RobotState::STAY:
                msl = 0;
                msr = 0;
                // If we have a target, start moving!
                if (current_task_id != -1) state = RobotState::GO_TO_GOAL;
                break;

            case RobotState::GO_TO_GOAL:
                // A. Path Planning (Lazy evaluation)
                // If we have a target ID but no path, calculate it now.
                if (waypoint_path.empty() && current_task_id != -1) {
                    int idx = get_task_index(current_task_id);
                    if (idx != -1) {
                        Point2d start = {my_pos[0], my_pos[1]};
                        Point2d goal = world_state[idx].pos;

                        Path p = planner.findPath(start, goal);
                        if (!p.waypoints.empty()) {
                            waypoint_path = p.waypoints;
                            current_waypoint_idx = 0;
                            travel_start_time = sim_clock;
                            LOG("Computed %zu-step path to Task %d @(%f, %f), from my position @(%f, %f)\n",
                                waypoint_path.size(), current_task_id, goal.x, goal.y, my_pos[0],
                                my_pos[1]);
                        } else {
                            // Path generation failed (unreachable? bug?). Drop task.
                            LOG("Pathfinding failed for Task %d. Dropping.\n", current_task_id);
                            current_task_id = -1;
                            state = RobotState::STAY;
                        }
                    }
                }

                // B. Path Following
                if (!waypoint_path.empty()) {
                    Point2d target = waypoint_path[current_waypoint_idx];
                    double dist = utils::dist(my_pos[0], my_pos[1], target.x, target.y);

                    // Reached Waypoint?
                    if (dist < WAYPOINT_ARRIVAL_THRESHOLD) {
                        current_waypoint_idx++;

                        // Reached FINAL Waypoint (Task Location)?
                        if (current_waypoint_idx >= (int)waypoint_path.size()) {
                            // 1. Stop Motors
                            msl = 0;
                            msr = 0;

                            // 2. Calculate Battery Usage
                            int travel_time = sim_clock - travel_start_time;
                            battery_time_used += travel_time;

                            // 3. Update Local World State
                            int idx = get_task_index(current_task_id);
                            if (idx != -1) {
                                world_state[idx].isBeingCompleted = true;  // Not fully complete yet!

                                // 4. Start "Working" Pause
                                int task_duration =
                                    get_pause_duration_ms(robot_specialization, world_state[idx].type);
                                pause_until = sim_clock + task_duration;
                                pause_active = 1;
                                battery_time_used += task_duration;  // Add task cost

                                LOG("Arrived at Task %d @(%f, %f). Will be completed in %dms.\n",
                                    current_task_id, world_state[idx].pos.x, world_state[idx].pos.y,
                                    task_duration);
                            }

                            // 5. Broadcast "BEING COMPUTED" Message
                            // Tell neighbors this task is being worked on so they stop bidding.
                            // Note: We do NOT send isTaskComplete=true yet - that comes after pause.
                            RobotStateMsg working_msg;
                            working_msg.robotId = robot_id;
                            working_msg.currentTaskId = current_task_id;
                            working_msg.currentBid = 0;  // Bid irrelevant now
                            working_msg.isTaskBeingCompleted = true;
                            working_msg.isTaskComplete = false;  // Not complete yet!
                            wb_emitter_set_channel(emitter_tag, WB_CHANNEL_BROADCAST);
                            wb_emitter_send(emitter_tag, &working_msg, sizeof(working_msg));

                            // 6. Store task ID for completion broadcast after pause
                            last_completed_task_id = current_task_id;
                            current_task_id = -1;
                            waypoint_path.clear();
                            state = RobotState::STAY;
                        }
                    } else {
                        // Move towards current waypoint
                        compute_go_to_point(&msl, &msr, target.x, target.y);
                    }
                }
                break;

            case RobotState::OBSTACLE_AVOID:
                compute_avoid_obstacle(&msl, &msr, distances);
                break;

            case RobotState::RANDOM_WALK:  // Fallback if stuck
                msl = 400;
                msr = 400;
                break;

            default:
                break;
        }
    }

    // --- 7. ACTUATE & UPDATE ---
    wb_motor_set_velocity(left_motor, msl * MAX_SPEED_WEB / 1000.0);
    wb_motor_set_velocity(right_motor, msr * MAX_SPEED_WEB / 1000.0);

    update_self_motion(msl, msr);
    sim_clock += ms;
}

// MAIN
int main() {
    reset();

    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {
        run(TIME_STEP);
    }
    wb_robot_cleanup();

    return 0;
}
