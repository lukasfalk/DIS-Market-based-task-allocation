#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// --- Simulation ---
#define TIME_STEP 64
#define DELTA_T (TIME_STEP / 1000.0)  // Timestep (seconds)

#define MAX_ROBOTS 9
#define NUM_ROBOTS 5
#define MAX_EVENTS 10
#define MAX_RUNTIME (3 * 60 * 1000)

#define BREAK -999  // for physics plugin

// --- Robot ---
#define MAX_BATTERY_LIFETIME (2 * 60 * 1000)

#define AXLE_LENGTH 0.052
#define WHEEL_RADIUS 0.0205

#define MAX_SPEED 800
#define MAX_SPEED_WEB 6.28
#define SPEED_UNIT_RADS 0.00628  // Conversion factor from speed unit to radian per second

#define NB_SENSORS 8
#define BIAS_SPEED 400

// --- Control & Navigation ---
#define STATECHANGE_DIST 580  // Min value of all sensor inputs combined to enter obstacle avoidance mode
#define MAX_PATH_LENGTH 50
#define WAYPOINT_ARRIVAL_THRESHOLD 0.01  // Distance in meters to consider waypoint reached

// --- Communication ---
#define RX_PERIOD 2

// --- Other ---
#define INVALID -999

#endif  // CONSTANTS_HPP
