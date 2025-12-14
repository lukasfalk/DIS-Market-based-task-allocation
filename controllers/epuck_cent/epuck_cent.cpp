/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_cent.cpp
 * description: Centralized controller.
 *              Receives bids from robots and assigns tasks.
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
#include <limits>
#include <vector>

#include "../common/communication.hpp"
#include "../common/constants.hpp"
#include "../common/geometry.hpp"
#include "../common/logging.hpp"
#include "../common/pathfinding.hpp"
#include "../common/utils.hpp"

WbDeviceTag left_motor;   // handler for left wheel of the robot
WbDeviceTag right_motor;  // handler for the right wheel of the robot

// Enable or disable logging
#define ENABLE_LOGGING 0  // Set to 1 to enable logging, 0 to disable

#if ENABLE_LOGGING
// Logging macro, configured with prefix "[Robot {ID} @ t={TIME}ms] "
#define LOG(fmt, ...)                                                                  \
    do {                                                                               \
        char prefix[64];                                                               \
        snprintf(prefix, sizeof(prefix), "[Robot %d @ t=%dms] ", robot_id, sim_clock); \
        logMsg(prefix, fmt, ##__VA_ARGS__);                                            \
    } while (0)
#else
#define LOG(fmt, ...) \
    do {              \
    } while (0)
#endif

enum class RobotState {
    STAY = 1,
    GO_TO_GOAL = 2,  // Initial state aliases
    OBSTACLE_AVOID = 3,
    RANDOM_WALK = 4,
    DEAD = 5,  // Robot is out of battery
};

constexpr RobotState DEFAULT_STATE = RobotState::STAY;

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
constexpr std::array<int, 16> Interconn = {17,  29,  34,  10, 8,  -38, -56, -76,
                                           -72, -58, -36, 8,  10, 36,  28,  18};

// The state variables
int sim_clock;                   // Simulation clock in milliseconds
uint16_t robot_id;               // Unique robot ID
RobotSpec robot_specialization;  // Specialization type of this robot wrt task types (A or B)
RobotState state;                // State of the robot
double my_pos[3];                // X, Z, Theta of this robot
bool target_valid = false;       // boolean; whether we are supposed to go to the target

struct LocalTaskInfo {
    double x;
    double y;
    int id;
    TaskType type;  // Task type for pause duration calculation
};
std::vector<LocalTaskInfo> targets;

int last_completed_task_id = -1;  // ID of the last task we completed (for sending completion msg)

std::vector<float> buff;  // Buffer for physics plugin

// Pathfinding and waypoint following
PathPlanner planner;                 // Global path planner instance
std::vector<Point2d> waypoint_path;  // Computed path waypoints
int current_waypoint_idx = 0;        // Index of next waypoint to reach

double stat_max_velocity;

// Pause control: when clock < pause_until the robot stays paused
int pause_until = 0;
int pause_active = 0;

// Battery tracking (in milliseconds)
int battery_time_used = 0;  // Time spent traveling and at tasks (in ms)
int travel_start_time = 0;  // Timestamp when this travel phase started (ms)

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

// Calculate the insertion cost for adding a task at a specific position in the targets list
// Returns the MARGINAL cost (extra distance) of inserting at this position
static double calculate_insertion_cost(const Point2d& new_task_pos, int insert_idx) {
    Point2d before_pos, after_pos;
    bool has_after = false;

    // Determine the "before" position
    if (insert_idx == 0) {
        // Inserting at start: before = my current position
        before_pos = {my_pos[0], my_pos[1]};
    } else {
        // Inserting in middle/end: before = previous task in list
        before_pos = {targets[insert_idx - 1].x, targets[insert_idx - 1].y};
    }

    // Determine the "after" position (if any)
    if (insert_idx < (int)targets.size()) {
        after_pos = {targets[insert_idx].x, targets[insert_idx].y};
        has_after = true;
    }

    // Calculate distances using pathfinding
    Path p_before_to_new = planner.findPath(before_pos, new_task_pos);
    double d_before_to_new = p_before_to_new.waypoints.empty() ? utils::dist(before_pos, new_task_pos)
                                                               : p_before_to_new.totalDistance;

    if (!has_after) {
        // Appending to end: cost is simply distance from last task to new task
        return d_before_to_new;
    }

    // Inserting in middle: cost = (before->new) + (new->after) - (before->after)
    Path p_new_to_after = planner.findPath(new_task_pos, after_pos);
    double d_new_to_after = p_new_to_after.waypoints.empty() ? utils::dist(new_task_pos, after_pos)
                                                             : p_new_to_after.totalDistance;

    Path p_before_to_after = planner.findPath(before_pos, after_pos);
    double d_before_to_after = p_before_to_after.waypoints.empty() ? utils::dist(before_pos, after_pos)
                                                                   : p_before_to_after.totalDistance;

    return d_before_to_new + d_new_to_after - d_before_to_after;
}

// Find the best insertion position for a task and return {best_index, marginal_distance}
static std::pair<int, double> find_best_insertion(const Point2d& task_pos) {
    double best_cost = std::numeric_limits<double>::infinity();
    int best_idx = 0;

    if (targets.empty()) {
        // List is empty: cost is simply path from me to the task
        Path p = planner.findPath({my_pos[0], my_pos[1]}, task_pos);
        double dist = p.waypoints.empty() ? utils::dist(my_pos[0], my_pos[1], task_pos.x, task_pos.y)
                                          : p.totalDistance;
        return std::make_pair(0, dist);
    }

    // Try inserting at every position (including end)
    for (size_t i = 0; i <= targets.size(); ++i) {
        double insertion_cost = calculate_insertion_cost(task_pos, i);
        if (insertion_cost < best_cost) {
            best_cost = insertion_cost;
            best_idx = i;
        }
    }

    return std::make_pair(best_idx, best_cost);
}

// Calculate the full bid value including time and battery considerations
static double calculate_full_bid(double marginal_distance, TaskType task_type) {
    int time_at_task = get_pause_duration_ms(robot_specialization, task_type);
    int time_to_travel = (marginal_distance / 0.5) * 1000 + 1000;  // Assuming 0.5 m/s + 1s overhead
    int battery_left = MAX_BATTERY_LIFETIME - battery_time_used;

    // Check if we have enough battery
    if (time_to_travel + time_at_task > battery_left) {
        return -1.0;  // Can't accept this task
    }

    // Same formula: prioritize travel time, account for task time and battery
    return 1.5 * time_to_travel + time_at_task - 0.01 * battery_left;
}

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Check if we received a message and extract information
static void receive_updates() {
    MessageT msg;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const MessageT* pmsg = (const MessageT*)wb_receiver_get_data(receiver_tag);

        // Save a copy, because wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(MessageT));
        wb_receiver_next_packet(receiver_tag);

        // Double check this message is for me
        // Communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if (msg.robotId != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d doesn't match receiver %d\n", msg.robotId,
                    robot_id);
            exit(1);
        }

        if (targets.empty()) target_valid = false;

        // Event state machine
        if (msg.msgType == MSG_GPS_ONLY) {
            my_pos[0] = msg.robotX;
            my_pos[1] = msg.robotY;
            my_pos[2] = msg.heading;
            continue;
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
        } else if (msg.msgType == MSG_EVENT_DONE) {
            // NOTE: In the new logic, the robot detects its own arrival.
            // This handler is kept for backward compatibility but should not be triggered.
            // The supervisor no longer sends MSG_EVENT_DONE based on proximity.
            LOG("Warning: Received MSG_EVENT_DONE from supervisor (legacy behavior)\n");
        } else if (msg.msgType == MSG_EVENT_WON) {
            // Insert event at index
            LOG("Won bid for task %d, adding to list of tasks at index %d\n", msg.eventId, msg.eventIndex);

            LocalTaskInfo new_target = {msg.eventX, msg.eventY, msg.eventId, msg.taskType};
            if (msg.eventIndex >= 0 && msg.eventIndex <= (int)targets.size()) {
                targets.insert(targets.begin() + msg.eventIndex, new_target);
            } else {
                targets.push_back(new_target);
            }

            target_valid = true;
        }
        // check if new event is being auctioned
        else if (msg.msgType == MSG_EVENT_NEW) {
            // Use helper functions to find best insertion position and calculate bid
            Point2d task_pos = {msg.eventX, msg.eventY};

            std::pair<int, double> insertion_result = find_best_insertion(task_pos);
            int best_idx = insertion_result.first;
            double marginal_dist = insertion_result.second;

            // Calculate the full bid value
            double final_bid = calculate_full_bid(marginal_dist, msg.taskType);

            if (final_bid < 0) {
                // This task would cause us to run out of battery - decline
                int time_at_task = get_pause_duration_ms(robot_specialization, msg.taskType);
                int time_to_travel = (marginal_dist / 0.5) * 1000 + 1000;
                int battery_left = MAX_BATTERY_LIFETIME - battery_time_used;
                LOG("declining task %d: need %dms but only %dms battery left\n", msg.eventId,
                    time_to_travel + time_at_task, battery_left);
            } else {
                // Send my bid to the supervisor
                BidT my_bid;
                my_bid.msgType = ROBOT_MSG_BID;
                my_bid.robotId = robot_id;
                my_bid.eventId = msg.eventId;
                my_bid.bidValue = final_bid;
                my_bid.eventIndex = best_idx;
                LOG("sending bid for event %d with value %.2f at index %d\n", msg.eventId, final_bid,
                    best_idx);
                wb_emitter_set_channel(emitter_tag, robot_id + 1);
                wb_emitter_send(emitter_tag, &my_bid, sizeof(BidT));
            }
        }
    }

    // Communication with physics plugin (channel 0)
    if (!targets.empty()) {
        // Line from my position to first target
        wb_emitter_set_channel(emitter_tag, 0);

        buff.clear();
        buff.push_back(BREAK);
        buff.push_back(my_pos[0]);
        buff.push_back(my_pos[1]);
        buff.push_back(targets[0].x);
        buff.push_back(targets[0].y);

        // Lines between targets
        for (size_t k = 1; k < targets.size(); ++k) {
            buff.push_back(BREAK);
            buff.push_back(targets[k - 1].x);
            buff.push_back(targets[k - 1].y);
            buff.push_back(targets[k].x);
            buff.push_back(targets[k].y);
        }

        wb_emitter_send(emitter_tag, buff.data(), buff.size() * sizeof(float));
        wb_emitter_set_channel(emitter_tag, robot_id + 1);
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
    pause_until = 0;
    pause_active = 0;
    battery_time_used = 0;
    travel_start_time = 0;
    last_completed_task_id = -1;

    // Init target positions
    targets.clear();
    waypoint_path.clear();

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name;
    robot_name = (char*)wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck%d", &tmp_id)) {
        robot_id = (uint16_t)tmp_id;
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

    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id + 1);
    double range = wb_emitter_get_range(emitter_tag);
    LOG("Emitter range set to %.2f meters.\n", range);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD);  // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id + 1);

    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}

void update_state(int _sum_distances) {
    if (_sum_distances > STATECHANGE_DIST && state == RobotState::GO_TO_GOAL) {
        state = RobotState::OBSTACLE_AVOID;
    } else if (target_valid) {
        state = RobotState::GO_TO_GOAL;
    } else {
        state = DEFAULT_STATE;
    }
}

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
}

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
    float msl_w, msr_w;
    // Motor speed and sensor variables
    int msl = 0, msr = 0;       // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    int sum_distances = 0;      // sum of all distance sensor inputs, used as threshold for state change.

    // Other variables
    int sensor_nb;

    // Add the weighted sensors values
    for (sensor_nb = 0; sensor_nb < NB_SENSORS; sensor_nb++) {
        distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
        sum_distances += distances[sensor_nb];
    }

    // Get info from supervisor
    receive_updates();

    if (state == RobotState::DEAD) {
        // Only step the robot clock forward and return
        sim_clock += ms;
        return;
    };

    // Check if battery has been depleted to switch to DEAD state
    if (battery_time_used >= MAX_BATTERY_LIFETIME) {
        state = RobotState::DEAD;
        LOG("BATTERY DEPLETED! (used %dms / %dms max)\n", battery_time_used, MAX_BATTERY_LIFETIME);

        // Motors off, stay immobile
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        update_self_motion(0, 0);
        return;
    }

    // If we are paused, keep stopped until deadline
    if (pause_active) {
        if (sim_clock < pause_until) {
            // enforce stop (motors set later from msl/msr)
            msl = 0;
            msr = 0;
        } else {
            // Task is NOW actually complete - send completion message to supervisor
            if (last_completed_task_id != -1) {
                RobotStateMsg done_msg;
                done_msg.msgType = ROBOT_MSG_STATE;
                done_msg.robotId = robot_id;
                done_msg.currentTaskId = last_completed_task_id;
                done_msg.currentBid = 0;
                done_msg.isTaskBeingCompleted = false;
                done_msg.isTaskComplete = true;  // NOW it's truly complete
                wb_emitter_send(emitter_tag, &done_msg, sizeof(done_msg));
                LOG("Task %d fully completed after working pause.\n", last_completed_task_id);
                last_completed_task_id = -1;  // Reset after sending
            }
            // pause expired -> resume normal behavior
            pause_active = 0;
        }
    }

    // Set wheel speeds depending on state
    if (!pause_active) {
        // State may change because of obstacles
        update_state(sum_distances);

        switch (state) {
            case RobotState::STAY:
                msl = 0;
                msr = 0;
                break;

            case RobotState::GO_TO_GOAL:

                // Compute path to the first target if we haven't already
                if (waypoint_path.empty()) {
                    compute_path_to_target();
                    travel_start_time = sim_clock;  // Mark when travel started
                    LOG("Started travelling to first of %d tasks (%d waypoints)\n", (int)targets.size(),
                        (int)waypoint_path.size());
                }

                // If we have waypoints, navigate through them
                if (!waypoint_path.empty()) {
                    Point2d current_waypoint = waypoint_path[current_waypoint_idx];

                    // Check if we've reached the current waypoint
                    double dist_to_waypoint =
                        utils::dist(my_pos[0], my_pos[1], current_waypoint.x, current_waypoint.y);
                    if (dist_to_waypoint < WAYPOINT_ARRIVAL_THRESHOLD) {
                        LOG("Waypoint %d/%d (%.2f, %.2f) reached\n", current_waypoint_idx + 1,
                            (int)waypoint_path.size(), current_waypoint.x, current_waypoint.y);
                        current_waypoint_idx++;
                        if (current_waypoint_idx >= (int)waypoint_path.size()) {
                            // Reached final destination - we have arrived at the task!
                            msl = 0;
                            msr = 0;

                            // 1. Track battery consumed during travel
                            int travel_time_ms = sim_clock - travel_start_time;
                            battery_time_used += travel_time_ms;
                            LOG("  > Arrived at task - traveled for %dms; total battery used: %dms / %dms "
                                "(%.2f %%)\n",
                                travel_time_ms, battery_time_used, MAX_BATTERY_LIFETIME,
                                (battery_time_used * 100.0) / MAX_BATTERY_LIFETIME);

                            // 2. Start "Working" Pause
                            if (!targets.empty()) {
                                int task_time_ms =
                                    get_pause_duration_ms(robot_specialization, targets[0].type);
                                pause_until = sim_clock + task_time_ms;
                                pause_active = 1;
                                battery_time_used += task_time_ms;  // Add task cost
                                LOG("  > Working on task %d for %dms\n", targets[0].id, task_time_ms);

                                // 3. Broadcast "BEING COMPLETED" Message to supervisor
                                RobotStateMsg working_msg;
                                working_msg.msgType = ROBOT_MSG_STATE;
                                working_msg.robotId = robot_id;
                                working_msg.currentTaskId = targets[0].id;
                                working_msg.currentBid = 0;
                                working_msg.isTaskBeingCompleted = true;
                                working_msg.isTaskComplete = false;  // Not complete yet!
                                wb_emitter_send(emitter_tag, &working_msg, sizeof(working_msg));

                                // 4. Store task ID for completion broadcast after pause, then remove from
                                // list
                                last_completed_task_id = targets[0].id;
                                targets.erase(targets.begin());
                            }

                            if (targets.empty()) target_valid = false;
                            state = RobotState::STAY;

                            // 5. Reset for next target
                            waypoint_path.clear();
                            current_waypoint_idx = 0;
                            travel_start_time = 0;
                        } else {
                            // Move to next waypoint
                            compute_go_to_point(&msl, &msr, waypoint_path[current_waypoint_idx].x,
                                                waypoint_path[current_waypoint_idx].y);
                            LOG("  > Moving to waypoint %d/%d (%.2f, %.2f)\n", current_waypoint_idx + 1,
                                (int)waypoint_path.size(), waypoint_path[current_waypoint_idx].x,
                                waypoint_path[current_waypoint_idx].y);
                        }
                    } else {
                        // Move towards current waypoint
                        compute_go_to_point(&msl, &msr, current_waypoint.x, current_waypoint.y);
                    }
                } else {
                    // No path computed yet or pathfinding failed - stop
                    msl = 0;
                    msr = 0;
                }
                break;

            case RobotState::OBSTACLE_AVOID:
                compute_avoid_obstacle(&msl, &msr, distances);
                break;

            case RobotState::RANDOM_WALK:
                msl = 400;
                msr = 400;
                break;

            case RobotState::DEAD:
                break;

            default:
                LOG("Invalid state: robot_id %d \n", robot_id);
        }
    }
    // Set the speed
    msl_w = msl * MAX_SPEED_WEB / 1000;
    msr_w = msr * MAX_SPEED_WEB / 1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    update_self_motion(msl, msr);

    // Update simulation clock
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
