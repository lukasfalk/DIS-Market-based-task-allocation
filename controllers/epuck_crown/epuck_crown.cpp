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

#include "../common/communication.hpp"
#include "../common/geometry.hpp"
#include "../common/logging.hpp"
#include "../common/pathfinding.hpp"

#define MAX_SPEED_WEB 6.28  // Maximum speed webots
WbDeviceTag left_motor;     // handler for left wheel of the robot
WbDeviceTag right_motor;    // handler for the right wheel of the robot

#define LOG(fmt, ...)                                                                  \
    do {                                                                               \
        char prefix[64];                                                               \
        snprintf(prefix, sizeof(prefix), "[Robot %d @ t=%dms] ", robot_id, sim_clock); \
        log_msg(prefix, fmt, ##__VA_ARGS__);                                           \
    } while (0)

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

#define COMMUNICATION_CHANNEL 1

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST \
    10  // minimum value of all sensor inputs combined to change to obstacle avoidance mode

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
int sim_clock;                   // Simulation clock in milliseconds
uint16_t robot_id;               // Unique robot ID
RobotSpec robot_specialization;  // Specialization type of this robot wrt task types (A or B)
robot_state_t state;             // State of the robot
double my_pos[3];                // X, Z, Theta of this robot
char target_valid;               // boolean; whether we are supposed to go to the target
double target[99][3];            // x and z coordinates of target position (max 99 targets)
int lmsg, rmsg;                  // Communication variables
int indx;                        // Event index to be sent to the supervisor

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

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) { return ((double)rand()) / ((double)RAND_MAX); }

void limit(int* number, int limit) {
    if (*number > limit) *number = limit;
    if (*number < -limit) *number = -limit;
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

// Send beacon message to other robots, to link up neighborhood
static void beacon() {
    // Send beacon message to other robots
    BeaconMessage packet = {MSG_BEACON, robot_id, sim_clock};
    wb_emitter_set_channel(emitter_tag, COMMUNICATION_CHANNEL);
    wb_emitter_send(emitter_tag, &packet, sizeof(BeaconMessage));
}

// Check if we received a message and extract information
static void receive_updates() {
    MessageT msg;
    int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const MessageT* pmsg = (const MessageT*)wb_receiver_get_data(receiver_tag);

        // Save a copy, because wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(MessageT));
        wb_receiver_next_packet(receiver_tag);

        // Double check this message is for me
        // Communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if (msg.robot_id != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d doesn't match receiver %d\n", msg.robot_id,
                    robot_id);
            exit(1);
        }

        // Find target list length
        target_list_length = 0;
        while (target[target_list_length][2] != INVALID) {
            target_list_length++;
        }

        if (target_list_length == 0) target_valid = 0;

        // Event state machine
        if (msg.msg_type == MSG_GPS_ONLY) {
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
            continue;
        } else if (msg.msg_type == MSG_QUIT) {
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
        } else if (msg.msg_type == MSG_EVENT_DONE) {
            // If event is done, delete it from array
            for (i = 0; i <= target_list_length; i++) {
                if ((int)target[i][2] ==
                    msg.event_id) {  // Look for correct id (in case wrong event was done first)
                    for (; i <= target_list_length; i++) {  // Push list to the left from event index
                        target[i][0] = target[i + 1][0];
                        target[i][1] = target[i + 1][1];
                        target[i][2] = target[i + 1][2];
                    }
                }
            }
            // Adjust target list length
            if (target_list_length - 1 == 0) target_valid = 0;  // Used in general state machine
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
                LOG("used %dms traveling to task (battery used so far: %dms / %dms)\n", travel_time_ms,
                    battery_time_used, MAX_BATTERY_LIFETIME);
                travel_start_time = 0;  // reset
            }

            // 2. Consumption at task
            battery_time_used += task_time_ms;
            LOG("used %dms at task (battery used so far: %dms / %dms)\n", task_time_ms, battery_time_used,
                MAX_BATTERY_LIFETIME);

            // Reset waypoint path for next target
            waypoint_count = 0;
            current_waypoint_idx = 0;
        } else if (msg.msg_type == MSG_EVENT_WON) {
            // Insert event at index
            LOG("Won bid for task %d, adding to list of tasks at index %d\n", msg.event_id,
                msg.event_index);
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

            // DBG(("  > and resetting waypoint path (currently at %d/%d waypoints, for task 0/%d)\n",
            // current_waypoint_idx + 1, waypoint_count, target_list_length));
            // // Reset waypoint path - we'll compute a new path to this target
            // waypoint_count = 0;
            // current_waypoint_idx = 0;
        }
        // check if new event is being auctioned
        else if (msg.msg_type == MSG_EVENT_NEW) {
            //########################## Distributed bidding #####################
            indx = target_list_length;

            // Calculate actual path distance using pathfinding (accounts for walls/obstacles)
            Point2d start = {my_pos[0], my_pos[1]};
            Point2d goal = {msg.event_x, msg.event_y};
            Point2d temp_waypoint_buffer[MAX_PATH_LENGTH];
            double d = get_path(start, goal, temp_waypoint_buffer, MAX_PATH_LENGTH);

            // If pathfinding failed, use Euclidean distance as fallback
            if (d < 0) {
                d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
            }

            int time_at_task = get_pause_duration_ms(robot_specialization, msg.task_type);
            int time_to_travel =
                (d / 0.5) * 1000 + 1000;  // Assuming average speed of 0.5 m/s (converted to ms) + 1s
                                          // overhead for collisions and other delays

            // Check battery: if accepting this task would exceed battery life, add penalty
            int battery_time_left = MAX_BATTERY_LIFETIME - battery_time_used;

            if (time_to_travel + time_at_task > battery_time_left) {
                // This task would cause us to run out of battery - bid very high to refuse it
                LOG("declining task %d: need %dms but only %dms battery left\n", msg.event_id,
                    time_to_travel + time_at_task, battery_time_left);
            } else {
                // We have enough battery - use normal bidding (time to complete task)
                double final_bid = 1.5 * time_to_travel + time_at_task - 0.01 * battery_time_left;
                // 100% battery:  1.5 * 11200 + 5000 - 0.01 * 120000 = 16800 + 5000 - 1200 = 20600
                // 50% battery:   1.5 * 11200 + 5000 - 0.01 * 60000  = 16800 + 5000 - 600  = 21200
                // 10% battery:   1.5 * 11200 + 5000 - 0.01 * 12000  = 16800 + 5000 - 120  = 21680
                // Thus, more battery left results in a slightly cheaper/stronger bid (more attractive)

                // Send my bid to the supervisor
                const BidT my_bid = {robot_id, msg.MSG_FLOODING, final_bid, indx};
                LOG("sending bid for event %d with value %.2f at index %d\n", msg.event_id, final_bid,
                    indx);
                
                wb_emitter_set_channel(emitter_tag, COMMUNICATION_CHANNEL);
                wb_emitter_send(emitter_tag, &my_bid, sizeof(BidT));
            }            
            /*
            //########################## Centralized bidding #####################
            indx = target_list_length;

            // Calculate actual path distance using pathfinding (accounts for walls/obstacles)
            Point2d start = {my_pos[0], my_pos[1]};
            Point2d goal = {msg.event_x, msg.event_y};
            Point2d temp_waypoint_buffer[MAX_PATH_LENGTH];
            double d = get_path(start, goal, temp_waypoint_buffer, MAX_PATH_LENGTH);

            // If pathfinding failed, use Euclidean distance as fallback
            if (d < 0) {
                d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
            }

            int time_at_task = get_pause_duration_ms(robot_specialization, msg.task_type);
            int time_to_travel =
                (d / 0.5) * 1000 + 1000;  // Assuming average speed of 0.5 m/s (converted to ms) + 1s
                                          // overhead for collisions and other delays

            // Check battery: if accepting this task would exceed battery life, add penalty
            int battery_time_left = MAX_BATTERY_LIFETIME - battery_time_used;

            if (time_to_travel + time_at_task > battery_time_left) {
                // This task would cause us to run out of battery - bid very high to refuse it
                LOG("declining task %d: need %dms but only %dms battery left\n", msg.event_id,
                    time_to_travel + time_at_task, battery_time_left);
            } else {
                // We have enough battery - use normal bidding (time to complete task)
                double final_bid = 1.5 * time_to_travel + time_at_task - 0.01 * battery_time_left;
                // 100% battery:  1.5 * 11200 + 5000 - 0.01 * 120000 = 16800 + 5000 - 1200 = 20600
                // 50% battery:   1.5 * 11200 + 5000 - 0.01 * 60000  = 16800 + 5000 - 600  = 21200
                // 10% battery:   1.5 * 11200 + 5000 - 0.01 * 12000  = 16800 + 5000 - 120  = 21680
                // Thus, more battery left results in a slightly cheaper/stronger bid (more attractive)

                // Send my bid to the supervisor
                const BidT my_bid = {robot_id, msg.event_id, final_bid, indx};
                LOG("sending bid for event %d with value %.2f at index %d\n", msg.event_id, final_bid,
                    indx);
                wb_emitter_set_channel(emitter_tag, robot_id + 1);
                wb_emitter_send(emitter_tag, &my_bid, sizeof(BidT));
            }
            */
        }
        else if (msg.msg_type == MSG_BEACON) {
            /*
            BeaconMessage packet = {MSG_BEACON, robot_id, sim_clock};
            wb_emitter_set_channel(emitter_tag, COMMUNICATION_CHANNEL);
            wb_emitter_send(emitter_tag, &packet, sizeof(BeaconMessage));
            */

            const double *data = wb_receiver_get_data(receiver_tag);;
            const BeaconMessage *received_packet = (const BeaconMessage*)data;

            LOG("Received beacon from robot %d at time %dms\n", received_packet->robot_id, received_packet->timestamp);
            // Add received beacon info to neighborhood structure
            for (i = 0; i < NUM_ROBOTS; i++) {
                if (neighborhood.neighbors[i] == received_packet->robot_id) {
                    neighborhood.sim_clocks[i] = received_packet->sim_clock;
                    break;  // Exit loop once the robot is found
                }
            }
        }
        else if (msg.msg_type == MSG_FLOODING) {
            // Flooding. Emit all known bids to neighbors, including this one
            const BidsT bids_packet;
            bool new_info = false;
            for (int i = 0; i < NUM_ROBOTS; i++) {
                new_info = neighborhood.neighbors[i] == bids_packet.robot_id[i] ? true : false;   // or whatever your robot IDs actually are
                new_info = neighborhood.bids[i] == bids_packet.bid_values[i] ? true : false; 
            }
            if (new_info) {
                for (int i = 0; i < NUM_ROBOTS; i++) {
                    bids_packet.robot_id[i] = neighborhood.neighbors[i];
                    bids_packet.bid_values[i] = neighborhood.bids[i];
                }
                bids_packet.event_id = msg.event_id;
                wb_emitter_set_channel(emitter_tag, COMMUNICATION_CHANNEL);
                wb_emitter_send(emitter_tag, &bids_packet, sizeof(BidsT));
            } else{
                LOG("No new info to flood\n");
                // Determine if robot won the auction based on bids
                uint16_t lowest_bid_robot = -1;
                double lowest_bid_value = std::numeric_limits<double>::max();
                for (int i = 0; i < NUM_ROBOTS; i++) {
                    if (BidsT.bid_values[i] < lowest_bid_value) {
                        lowest_bid_value = BidsT.bid_values[i];
                        lowest_bid_robot = BidsT.robot_id[i];
                    }
                }
                if (lowest_bid_robot == robot_id) {
                    LOG("I won the auction for event %d\n", bids_packet.event_id);
                    for (int i = 0; i < NUM_ROBOTS; i++) {
                        auction_wins.robot_id[i] = neighborrhood.neighbors[i];
                    }
                    auction_wins.event_id = bids_packet.event_id;
                    // Perform actions for winning the auction
                }
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
        wb_emitter_set_channel(emitter_tag, COMMUNICATION_CHANNEL);
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

    NeighborhoodT neighborhood;
    AuctionWinsT auction_wins;

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
    wb_emitter_set_channel(emitter_tag, COMMUNICATION_CHANNEL);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD);  // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, COMMUNICATION_CHANNEL);

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

        LOG("Called compute_path_to_target(): start is my pos (%.2f, %.2f), goal is first task "
            "in list (%.2f, %.2f)\n",
            my_pos[0], my_pos[1], target[0][0], target[0][1]);

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
            LOG("  > New path with %d waypoints, distance %.3f\n", waypoint_count, path_distance);
            return 1;
        } else {
            LOG("  > Failed to compute path to target (%.3f, %.3f)\n", goal.x, goal.y);
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
    // [TODO]: receive_updates from other robots (radio). receive_updates() should flood with bids.
    // Then determine if bid winner and go to the actions section.
    // Remove robots out of range from neighborhood structure.
    for (int i = 0; i < NUM_ROBOTS; i++) {
        if (neighborhood.neighbors[i] != -1 &&
            (sim_clock - neighborhood.sim_clocks[i] > 5000)) {  // 5 seconds timeout
            LOG("Robot %d is out of range, removing from neighborhood\n", neighborhood.neighbors[i]);
            neighborhood.neighbors[i] = -1;
            neighborhood.bids[i] = 0.0;
            neighborhood.sim_clocks[i] = 0;
        }
    }

    beacon();


    if (state == DEAD) {
        // Only step the robot clock forward and return
        sim_clock += ms;
        return;
    };

    // Check if battery has been depleted to switch to DEAD state
    if (battery_time_used >= MAX_BATTERY_LIFETIME) {
        state = DEAD;
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
                    LOG("Started travelling to first of %d tasks (%d waypoints)\n", target_list_length,
                        waypoint_count);
                }

                // If we have waypoints, navigate through them
                if (waypoint_count > 0) {
                    Point2d current_waypoint = waypoint_path[current_waypoint_idx];
                    // LOG("Current waypoint: %d/%d (%.2f, %.2f)\n", current_waypoint_idx + 1,
                    // waypoint_count,
                    //     current_waypoint.x, current_waypoint.y);

                    // Check if we've reached the current waypoint
                    double dist_to_waypoint =
                        dist(my_pos[0], my_pos[1], current_waypoint.x, current_waypoint.y);
                    if (dist_to_waypoint < WAYPOINT_ARRIVAL_THRESHOLD) {
                        LOG("Waypoint %d/%d (%.2f, %.2f) reached\n", current_waypoint_idx + 1,
                            waypoint_count, current_waypoint.x, current_waypoint.y);
                        current_waypoint_idx++;
                        if (current_waypoint_idx >= waypoint_count) {
                            // Reached final destination - track battery consumed during travel
                            msl = 0;
                            msr = 0;
                            int travel_time_ms = sim_clock - travel_start_time;
                            battery_time_used += travel_time_ms;
                            LOG("  > Arrived at task - traveled for %dms; total battery used: %dms / %dms "
                                "(%.2f %%)\n",
                                travel_time_ms, battery_time_used, MAX_BATTERY_LIFETIME,
                                (battery_time_used * 100.0) / MAX_BATTERY_LIFETIME);
                            LOG("    > Resetting waypoints and travel time for next task\n");
                            waypoint_count = 0;  // Reset path for next target
                            current_waypoint_idx = 0;
                            travel_start_time = 0;
                        } else {
                            // Move to next waypoint
                            compute_go_to_point(&msl, &msr, waypoint_path[current_waypoint_idx].x,
                                                waypoint_path[current_waypoint_idx].y);
                            LOG("  > Moving to waypoint %d/%d (%.2f, %.2f)\n", current_waypoint_idx + 1,
                                waypoint_count, waypoint_path[current_waypoint_idx].x,
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
int main(int argc, char** argv) {
    reset();

    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {
        run(TIME_STEP);
    }
    wb_robot_cleanup();

    return 0;
}
