/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_crown.c
 * author:
 * description: E-puck file for market-based task allocations (DIS lab05)
 *
 * $Revision$	february 2016 by Florian Maushart
 * $Date$
 * $Author$      Last update 2024 by Wanting Jin
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

#include "../auct_super/message.h"
#include "pathfinding.hpp"

#define MAX_SPEED_WEB 6.28  // Maximum speed webots
WbDeviceTag left_motor;     // handler for left wheel of the robot
WbDeviceTag right_motor;    // handler for the right wheel of the robot

#define DEBUG 1
#define DBG(x) printf x

#define TIME_STEP 64  // Timestep (ms)
#define RX_PERIOD 2   // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH 0.052         // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628   // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205       // Wheel radius (meters)
#define DELTA_T TIME_STEP / 1000  // Timestep (seconds)
#define MAX_SPEED 800             // Maximum speed

#define INVALID -999
#define BREAK -999  // for physics plugin

#define NUM_ROBOTS 5  // Change this also in the supervisor!
#define MAX_RUNTIME (3 * 60 * 1000)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 10  // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY = 1,
    GO_TO_GOAL = 2,  // Initial state aliases
    OBSTACLE_AVOID = 3,
    RANDOM_WALK = 4,
    DEAD = 5,  // Robot is out of battery
} robot_state_t;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS 8
#define BIAS_SPEED 400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17, 29, 34, 10, 8, -38, -56, -76, -72, -58, -36, 8, 10, 36, 28, 18};

// The state variables
int sim_clock;                      // Simulation clock in milliseconds
uint16_t robot_id;                  // Unique robot ID
robot_spec_t robot_specialization;  // Specialization type of this robot wrt task types (A or B)
robot_state_t state;                // State of the robot
double my_pos[3];                   // X, Z, Theta of this robot
char target_valid;                  // boolean; whether we are supposed to go to the target
double target[99][3];               // x and z coordinates of target position (max 99 targets)
int lmsg, rmsg;                     // Communication variables
int indx;                           // Event index to be sent to the supervisor

float buff[99];  // Buffer for physics plugin

// Pathfinding and waypoint following
#define MAX_PATH_LENGTH 50
Point2d waypoint_path[MAX_PATH_LENGTH];          // Computed path waypoints
int waypoint_count = 0;                          // Number of waypoints in current path
int current_waypoint_idx = 0;                    // Index of next waypoint to reach
const double WAYPOINT_ARRIVAL_THRESHOLD = 0.01;  // Distance in meters to consider waypoint reached

double stat_max_velocity;

// Pause control: when clock < pause_until the robot stays paused
int pause_until = 0;
int pause_active = 0;

// Battery tracking (in milliseconds)
#define MAX_BATTERY_LIFETIME (2 * 60 * 1000)  // 2 minutes of battery life in ms (matches supervisor)
int battery_time_used = 0;                    // Time spent traveling and at tasks (in ms)
int travel_start_time = 0;                    // Timestamp when this travel phase started (ms)

// Return pause duration (ms) based on robot specialization and task type (customize as needed)
static int get_pause_duration_ms(robot_spec_t robot_specialization, task_type_t task_type) {
    switch (task_type) {
        case TASK_TYPE_A:
            return (robot_specialization == ROBOT_SPEC_A) ? 3000 : 9000;
        case TASK_TYPE_B:
            return (robot_specialization == ROBOT_SPEC_A) ? 5000 : 1000;
        default:
            return 0;  // Fallback but should never happen
    }
}

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
    return ((double)rand()) / ((double)RAND_MAX);
}

void limit(int* number, int limit) {
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

// Check if we received a message and extract information
static void receive_updates() {
    message_t msg;
    int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const message_t* pmsg = (const message_t*)wb_receiver_get_data(receiver_tag);

        // Save a copy, because wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(message_t));
        wb_receiver_next_packet(receiver_tag);

        // Double check this message is for me
        // Communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if (msg.robot_id != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d doesn't match receiver %d\n", msg.robot_id, robot_id);
            exit(1);
        }

        // Find target list length
        target_list_length = 0;
        while (target[target_list_length][2] != INVALID) {
            target_list_length++;
        }

        if (target_list_length == 0)
            target_valid = 0;

        // Event state machine
        if (msg.event_state == MSG_EVENT_GPS_ONLY) {
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
            continue;
        } else if (msg.event_state == MSG_QUIT) {
            // Simulation is done!
            // Output remaining battery life, clean up and exit
            DBG(("[Robot %d] (t=%dms) END OF SIM! Battery used: %dms (%.2f %% of total)\n",
                 robot_id, sim_clock, battery_time_used, (battery_time_used * 100.0) / MAX_BATTERY_LIFETIME));

            // Stop motors
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            wb_robot_step(TIME_STEP);

            wb_robot_cleanup();
            exit(0);
        } else if (msg.event_state == MSG_EVENT_DONE) {
            // If event is done, delete it from array
            for (i = 0; i <= target_list_length; i++) {
                if ((int)target[i][2] == msg.event_id) {    // Look for correct id (in case wrong event was done first)
                    for (; i <= target_list_length; i++) {  // Push list to the left from event index
                        target[i][0] = target[i + 1][0];
                        target[i][1] = target[i + 1][1];
                        target[i][2] = target[i + 1][2];
                    }
                }
            }
            // Adjust target list length
            if (target_list_length - 1 == 0)
                target_valid = 0;  // Used in general state machine
            target_list_length = target_list_length - 1;

            // Stop and pause based on robot type and event type
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            // set pause deadline (sim_clock is in ms)
            int task_time_ms = get_pause_duration_ms(robot_specialization, msg.task_type);
            pause_until = sim_clock + task_time_ms;
            pause_active = 1;
            state = STAY;

            // Update battery consumption for this task
            // 1. Consumption during travel
            if (travel_start_time > 0) {
                int travel_time_ms = sim_clock - travel_start_time;
                battery_time_used += travel_time_ms;
                DBG(("[Robot %d] (t=%dms) used %dms traveling to task (battery used so far: %dms / %dms)\n",
                     robot_id, sim_clock, travel_time_ms, battery_time_used, MAX_BATTERY_LIFETIME));
                travel_start_time = 0;  // reset
            }

            // 2. Consumption at task
            battery_time_used += task_time_ms;
            DBG(("[Robot %d] (t=%dms) used %dms at task (battery used so far: %dms / %dms)\n",
                 robot_id, sim_clock, task_time_ms, battery_time_used, MAX_BATTERY_LIFETIME));

            // Reset waypoint path for next target
            waypoint_count = 0;
            current_waypoint_idx = 0;
        } else if (msg.event_state == MSG_EVENT_WON) {
            // Insert event at index
            DBG(("[Robot %d] (t=%dms) Won bid for task %d, adding to list of tasks at index %d\n",
                 robot_id, sim_clock, msg.event_id, msg.event_index));
            for (i = target_list_length; i >= msg.event_index; i--) {
                target[i + 1][0] = target[i][0];
                target[i + 1][1] = target[i][1];
                target[i + 1][2] = target[i][2];
            }
            target[msg.event_index][0] = msg.event_x;
            target[msg.event_index][1] = msg.event_y;
            target[msg.event_index][2] = msg.event_id;
            target_valid = 1;  // used in general state machine
            target_list_length = target_list_length + 1;

            // DBG(("  > and resetting waypoint path (currently at %d/%d waypoints, for task 0/%d)\n", current_waypoint_idx + 1, waypoint_count, target_list_length));
            // // Reset waypoint path - we'll compute a new path to this target
            // waypoint_count = 0;
            // current_waypoint_idx = 0;
        }
        // check if new event is being auctioned
else if (msg.event_state == MSG_EVENT_NEW) {
    indx = 0;
    double d;
    int whereInsert=-1;

    // --- INITIALIZATION FIX ---
    if (target_list_length == 0) {
        // Case 0: List is empty. Cost is simply path from Me -> Event
        Point2d start = {my_pos[0], my_pos[1]};
        Point2d goal = {msg.event_x, msg.event_y};
        Point2d temp_waypoint_buffer[MAX_PATH_LENGTH];
        
        d = get_path(start, goal, temp_waypoint_buffer, MAX_PATH_LENGTH);
        if (d < 0) d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
    
    } else {
        // List has items. Initialize d to Infinity so we find the true lowest insertion cost.
        d = 100000000.0; 
        
        Point2d temp_waypoint_buffer[MAX_PATH_LENGTH]; // Buffer for A*

        // Iterate through every existing task to find the best insertion slot
        for(int i = 0; i < target_list_length; i++){
            
            double current_insertion_cost = 0;
            
            // --- CASE 1: Insertion at the START ---
            if(i == 0){
                // Cost: (Me->New) + (New->Task[0]) - (Me->Task[0])
                double dbeforetogoal = get_path((Point2d){my_pos[0], my_pos[1]}, (Point2d){msg.event_x, msg.event_y}, temp_waypoint_buffer, MAX_PATH_LENGTH); 
                double daftertogoal = get_path((Point2d){target[i][0], target[i][1]}, (Point2d){msg.event_x, msg.event_y}, temp_waypoint_buffer, MAX_PATH_LENGTH);
                double dbeforetodafter = get_path((Point2d){my_pos[0], my_pos[1]}, (Point2d){target[i][0], target[i][1]}, temp_waypoint_buffer, MAX_PATH_LENGTH);
                whereInsert = 0;
                current_insertion_cost = dbeforetogoal + daftertogoal - dbeforetodafter;
            }
            // --- CASE 2: Insertion in the MIDDLE --- 
            else {
                // Cost: (Prev->New) + (New->Curr) - (Prev->Curr)
                double dbeforetogoal = get_path((Point2d){target[i-1][0], target[i-1][1]}, (Point2d){msg.event_x, msg.event_y}, temp_waypoint_buffer, MAX_PATH_LENGTH);
                double daftertogoal = get_path((Point2d){target[i][0], target[i][1]}, (Point2d){msg.event_x, msg.event_y}, temp_waypoint_buffer, MAX_PATH_LENGTH);
                double dbeforetodafter = get_path((Point2d){target[i-1][0], target[i-1][1]}, (Point2d){target[i][0], target[i][1]}, temp_waypoint_buffer, MAX_PATH_LENGTH);
                whereInsert = 1;
                current_insertion_cost = dbeforetogoal + daftertogoal - dbeforetodafter;
            }

            // CHECK: Is this insertion better than what we found so far?
            if(current_insertion_cost < d){
                d = current_insertion_cost;
                indx = i;
            }
            // --- CASE 3: Appending to the END ---
            // --- NESTING FIX: This check happens for EVERY loop, but only triggers on the last item ---
            if(i == target_list_length - 1){
                // The cost to append is simply distance from Last Task -> New Event
                double d_append = get_path((Point2d){target[i][0], target[i][1]}, (Point2d){msg.event_x, msg.event_y}, temp_waypoint_buffer, MAX_PATH_LENGTH);

                if(d_append < d) {
                    d = d_append;
                    indx = i + 1; // Insert AFTER the last element
                    whereInsert = 2;
                }
            }
        }
    }

    // --- BIDDING LOGIC (Battery & Time) ---
    // Use 'd' (which is now the optimized marginal path distance)
    int time_at_task = get_pause_duration_ms(robot_specialization, msg.task_type); 
    int time_to_travel = (d / 0.5) * 1000 + 1000; 

    int battery_time_left = MAX_BATTERY_LIFETIME - battery_time_used;

    if (time_to_travel + time_at_task > battery_time_left) {
        // Battery Refusal
        DBG(("[Robot %d] declining task\n", robot_id));
    } else {
        double final_bid = 1.5 * time_to_travel + time_at_task - 0.01 * battery_time_left;
        const bid_t my_bid = {robot_id, msg.event_id, final_bid, indx};
        DBG(("Robot inserted task at %d \n", whereInsert));
        wb_emitter_set_channel(emitter_tag, robot_id + 1);
        wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));
    }
}
    }

    // Communication with physics plugin (channel 0)
    i = 0;
    k = 1;

    // Find target list length
    target_list_length = 0;
    while (target[target_list_length][2] != INVALID) {
        target_list_length++;
    }

    if (target_list_length > 0) {
        // Line from my position to first target
        wb_emitter_set_channel(emitter_tag, 0);
        buff[0] = BREAK;  // draw new line
        buff[1] = my_pos[0];
        buff[2] = my_pos[1];
        buff[3] = target[0][0];
        buff[4] = target[0][1];
        // Lines between targets
        for (i = 5; i < 5 * target_list_length - 1; i = i + 5) {
            buff[i] = BREAK;
            buff[i + 1] = buff[i - 2];
            buff[i + 2] = buff[i - 1];
            buff[i + 3] = target[k][0];
            buff[i + 4] = target[k][1];
            k++;
        }
        // send, reset channel
        if (target[0][2] == INVALID) {
            buff[0] = my_pos[0];
            buff[1] = my_pos[1];
        }
        wb_emitter_send(emitter_tag, &buff, (5 * target_list_length) * sizeof(float));
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
    indx = 0;
    pause_until = 0;
    pause_active = 0;
    battery_time_used = 0;
    travel_start_time = 0;

    // Init target positions to "INVALID"
    for (i = 0; i < 99; i++) {
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID;
    }

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

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD);  // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id + 1);

    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}

void update_state(int _sum_distances) {
    if (_sum_distances > STATECHANGE_DIST && state == GO_TO_GOAL) {
        state = OBSTACLE_AVOID;
    } else if (target_valid) {
        state = GO_TO_GOAL;
    } else {
        state = DEFAULT_STATE;
    }
}

// Compute optimal path to next target using visibility graph pathfinding
// Returns true if a path was found, false otherwise
int compute_path_to_target() {
    if (target_valid && waypoint_count == 0) {
        // Convert current position to Point2d
        Point2d start = {my_pos[0], my_pos[1]};
        Point2d goal = {target[0][0], target[0][1]};

        DBG(("[Robot %d] called compute_path_to_target(): start is my pos (%.2f, %.2f), goal is first task in list (%.2f, %.2f)\n",
             robot_id, my_pos[0], my_pos[1], target[0][0], target[0][1]));

        // Call pathfinding to get the waypoint path
        // get_path returns the path distance (sum of segment lengths), or -1 if no path
        // It fills waypoint_path array with the waypoints
        double path_distance = get_path(start, goal, waypoint_path, MAX_PATH_LENGTH);

        // Count the actual waypoints by iterating until we hit an invalid point
        // (The path starts at start, ends at goal, and includes intermediate waypoints)
        int count = 0;
        while (count < MAX_PATH_LENGTH && (waypoint_path[count].x != 0 || waypoint_path[count].y != 0)) {
            count++;
        }

        if (path_distance > 0 && count > 0) {
            waypoint_count = count;
            current_waypoint_idx = 0;
            DBG(("  > new path with %d waypoints, distance %.3f\n",
                 waypoint_count, path_distance));
            return 1;
        } else {
            DBG(("  > failed to compute path to target (%.3f, %.3f)\n",
                 goal.x, goal.y));
            waypoint_count = 0;
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
    if (state == GO_TO_GOAL && velocity > stat_max_velocity) {
        stat_max_velocity = velocity;
    }
}

// Compute wheel speed to avoid obstacles
void compute_avoid_obstacle(int* msl, int* msr, int distances[]) {
    int d1 = 0, d2 = 0;                 // Motor speed 1 and 2
    int sensor_nb;                      // FOR-loop counters
    const int sensor_sensitivity = 50;  // Not sure what this does

    for (sensor_nb = 0; sensor_nb < NB_SENSORS; sensor_nb++) {
        d1 += (distances[sensor_nb] - sensor_sensitivity) * Interconn[sensor_nb] * 5;
        d2 += (distances[sensor_nb] - sensor_sensitivity) * Interconn[sensor_nb + NB_SENSORS] * 5;
    }
    d1 /= 80;
    d2 /= 80;  // Normalizing speeds

    *msr = d1 + BIAS_SPEED;
    *msl = d2 + BIAS_SPEED;
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
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
    float Kw = 10.0;              // Rotational control coefficient
    float range = 1;              // sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);  // Orientation of the wanted position

    // Compute forward control
    float u = Ku * range * cosf(bearing);
    // Compute rotational control
    float w = Kw * range * sinf(bearing);

    // Convert to wheel speeds!
    *msl = 50 * (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    *msr = 50 * (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
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

    if (state == DEAD) {
        // Only step the robot clock forward and return
        sim_clock += ms;
        return;
    };

    // Check if battery has been depleted to switch to DEAD state
    if (battery_time_used >= MAX_BATTERY_LIFETIME) {
        state = DEAD;
        DBG(("[Robot %d] (t=%dms) BATTERY DEPLETED! (used %dms / %dms max)\n",
             robot_id, sim_clock, battery_time_used, MAX_BATTERY_LIFETIME));

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
            // pause expired -> resume normal behavior
            pause_active = 0;
        }
    }

    // Set wheel speeds depending on state
    if (!pause_active) {
        // State may change because of obstacles
        update_state(sum_distances);

        switch (state) {
            case STAY:
                msl = 0;
                msr = 0;
                break;

            case GO_TO_GOAL:

                // Compute path to the first target if we haven't already
                if (waypoint_count == 0) {
                    // Find target list length
                    int target_list_length = 0;
                    while (target[target_list_length][2] != INVALID) {
                        target_list_length++;
                    }

                    compute_path_to_target();
                    travel_start_time = sim_clock;  // Mark when travel started
                    DBG(("[Robot %d] (t=%dms) started travelling to first of %d tasks (%d waypoints)\n",
                         robot_id, sim_clock, target_list_length, waypoint_count));
                }

                // If we have waypoints, navigate through them
                if (waypoint_count > 0) {
                    Point2d current_waypoint = waypoint_path[current_waypoint_idx];
                    // DBG(("[Robot %d] (t=%dms) current waypoint: %d/%d (%.2f, %.2f)\n", robot_id, sim_clock, current_waypoint_idx + 1, waypoint_count, current_waypoint.x, current_waypoint.y));

                    // Check if we've reached the current waypoint
                    double dist_to_waypoint = dist(my_pos[0], my_pos[1], current_waypoint.x, current_waypoint.y);
                    if (dist_to_waypoint < WAYPOINT_ARRIVAL_THRESHOLD) {
                        DBG(("[Robot %d] (t=%dms) waypoint %d/%d (%.2f, %.2f) reached",
                             robot_id, sim_clock, current_waypoint_idx + 1, waypoint_count, current_waypoint.x, current_waypoint.y));
                        current_waypoint_idx++;
                        if (current_waypoint_idx >= waypoint_count) {
                            // Reached final destination - track battery consumed during travel
                            msl = 0;
                            msr = 0;
                            int travel_time_ms = sim_clock - travel_start_time;
                            battery_time_used += travel_time_ms;
                            DBG(("; arrived at task - traveled for %dms; total battery used: %dms / %dms (%.2f %%)\n",
                                 travel_time_ms, battery_time_used, MAX_BATTERY_LIFETIME, (battery_time_used * 100.0) / MAX_BATTERY_LIFETIME));
                            DBG(("    > resetting waypoints and travel time for next task\n"));
                            waypoint_count = 0;  // Reset path for next target
                            current_waypoint_idx = 0;
                            travel_start_time = 0;
                        } else {
                            // Move to next waypoint
                            compute_go_to_point(&msl, &msr, waypoint_path[current_waypoint_idx].x, waypoint_path[current_waypoint_idx].y);
                            DBG(("; moving to waypoint %d/%d (%.2f, %.2f)\n",
                                 current_waypoint_idx + 1, waypoint_count, waypoint_path[current_waypoint_idx].x, waypoint_path[current_waypoint_idx].y));
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

            case OBSTACLE_AVOID:
                compute_avoid_obstacle(&msl, &msr, distances);
                break;

            case RANDOM_WALK:
                msl = 400;
                msr = 400;
                break;

            case DEAD:
                break;

            default:
                printf("Invalid state: robot_id %d \n", robot_id);
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
int main(int argc, char** argv) {
    reset();

    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {
        run(TIME_STEP);
    }
    wb_robot_cleanup();

    return 0;
}
