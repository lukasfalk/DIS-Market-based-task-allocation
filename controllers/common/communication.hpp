#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <cstdint>

// Channel definitions
constexpr int CHANNEL_BROADCAST = -1;  // Standard Webots broadcast (if used)
constexpr int CHANNEL_PHYSICS = 0;     // Reserved for drawing lines in simulation

// Event/Task Types
enum TaskType : int { TASK_TYPE_INVALID = 0, TASK_TYPE_A = 'A', TASK_TYPE_B = 'B' };

// Robot Specialization Types
enum RobotSpec : int {
    ROBOT_SPEC_A = 'A',  // Specialized for A
    ROBOT_SPEC_B = 'B'   // Specialized for B
};

// Message State Types
enum MessageType : int {
    MSG_EVENT_INVALID = 0,
    MSG_GPS_ONLY,
    MSG_EVENT_NEW,
    MSG_EVENT_WON,
    MSG_EVENT_DONE,
    MSG_QUIT,
    MSG_BEACON,
    MSG_FLOODING
};

// Message sent from Supervisor -> Robot
struct MessageT {
    uint16_t robot_id;  // Target ID
    double robot_x;
    double robot_y;
    double heading;

    MessageType msg_type;

    // Event details (valid if type is related to an event)
    uint16_t event_id;
    TaskType task_type;
    double event_x;
    double event_y;

    // Index suggestion for task list
    int event_index;
};

// Message sent from Robot -> Supervisor
struct BidT {
    uint16_t robot_id;
    uint16_t event_id;
    double bid_value;  // Estimated cost/time
    int event_index;   // Where the robot plans to insert the task
};

// Flooding
struct BidsT {
    uint16_t robot_id[NUM_ROBOTS];
    double bid_values[NUM_ROBOTS];  // Estimated cost/time
    uint16_t event_id;
}

struct NeighborhoodT {
    uint16_t neighbors[NUM_ROBOTS];  // IDs of neighboring robots
    double bids[NUM_ROBOTS];       // Bids received from neighbors
    int sim_clocks[NUM_ROBOTS];   // Simulation time when the bid was received
};

struct BeaconMessage {
    int msg_type;       // It's good practice to include the type inside the packet
    uint16_t sender_id; // ID of the robot sending this message
    int sim_clock;      // The timestamp
};

struct AuctionWinsT {
    uint16_t robot_id[NUM_ROBOTS];
    uint16_t event_id;
    int event_index;
};

#endif  // COMMUNICATION_HPP