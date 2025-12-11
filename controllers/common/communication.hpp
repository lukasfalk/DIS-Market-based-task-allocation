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
    MSG_QUIT
};

// Message sent from Supervisor -> Robot
struct MessageT {
    uint16_t robotId;  // Target ID
    double robotX;
    double robotY;
    double heading;

    MessageType msgType;

    // Event details (valid if type is related to an event)
    uint16_t eventId;
    TaskType taskType;
    double eventX;
    double eventY;

    // Index suggestion for task list
    int eventIndex;
};

// Message sent from Robot -> Supervisor
struct BidT {
    uint16_t robotId;
    uint16_t eventId;
    double bidValue;  // Estimated cost/time
    int eventIndex;   // Where the robot plans to insert the task
};

#endif  // COMMUNICATION_HPP