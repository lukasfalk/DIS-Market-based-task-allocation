/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        supervisor_dist.cpp
 * description: Supervisor for Part 2 (Distributed).
 *              Acts as Environment Manager and Stats Collector only.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <assert.h>
#include <time.h>

#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>

using namespace std;

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../common/communication.hpp"
#include "../common/constants.hpp"
#include "../common/geometry.hpp"
#include "../common/logging.hpp"
#include "../common/map_config.hpp"
#include "../common/utils.hpp"

uint64_t g_simTime = 0;

// Logging macro, configured with prefix "[Supervisor @ t={TIME}ms] "
#define LOG(fmt, ...)                                                                                \
    do {                                                                                             \
        char prefix[64];                                                                             \
        snprintf(prefix, sizeof(prefix), "[Supervisor @ t=%llums] ", (unsigned long long)g_simTime); \
        logMsg(prefix, fmt, ##__VA_ARGS__);                                                          \
    } while (0)

#define AUCTION_TIMEOUT 1000  // number of steps after which an auction stops

#define EVENT_RANGE (0.05)  // distance within which a robot must come to do event
// #define EVENT_TIMEOUT (10000)          // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000)  // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)
#define PROXIMITY_THRESHOLD (0.20)  // distance below which robots are considered in collision

// Parameters that can be changed
#define NUM_ACTIVE_EVENTS 10        // number of active events
#define TOTAL_EVENTS_TO_HANDLE 999  // Events after which simulation stops or...

#define MAX_WALLS 2

WbNodeRef g_eventNodes[MAX_EVENTS];
vector<WbNodeRef> g_eventNodesFree;

Point2d randCoord() {
    // Sample uniformly within the arena bounds defined by the pathfinding graph
    // Visibilty graph x in [-0.58, 0.58], y in [-0.58, 0.58]
    // Use a padding of 0.02 to avoid spawning exactly on walls
    double padding = 0.02;
    const double ARENA_MIN = -0.58 + padding;
    const double ARENA_MAX = 0.58 - padding;
    // => Spawn bounds: x in [-0.56, 0.56], y in [-0.56, 0.56]
    // ^ for outer walls - interior walls are handled by rejection sampling below, with same padding

    Point2d candidate;
    int attempts = 0;
    do {
        candidate.x = ARENA_MIN + (ARENA_MAX - ARENA_MIN) * utils::random_01();
        candidate.y = ARENA_MIN + (ARENA_MAX - ARENA_MIN) * utils::random_01();
        attempts++;
    } while (MapConfig::isPointInInteriorWall(candidate, padding));

    return candidate;
}

double expovariate(double mu) {
    double uniform = utils::random_01();
    while (uniform < 1e-7) uniform = utils::random_01();
    return -log(uniform) * mu;
}

// Event class
class Event {
    // Public variables
   public:
    uint16_t id_;     // event id
    Point2d pos_;     // event pos
    WbNodeRef node_;  // event node ref
    TaskType type_;   // type of event (TASK_TYPE_A or TASK_TYPE_B)

    // Task data
    uint16_t completedBy_;  // id of the robot that completed this event
    uint64_t timeDone_;     // time at which the assigned robot reached the event

    // Public functions
   public:
    // Event creation
    Event(uint16_t id)
        : id_(id),
          pos_(randCoord()),
          // vvv  1/3 of the time task type A, 2/3 of the time type B  vvv
          type_(utils::random_01() < (1.0f / 3.0f) ? TASK_TYPE_A : TASK_TYPE_B),
          completedBy_(-1),
          timeDone_(-1) {
        node_ = g_eventNodesFree.back();  // Place node
        g_eventNodesFree.pop_back();      // remove the "free" node from the free list

        double eventNodePos[3] = {pos_.x, pos_.y, 0.01};  // Place event in arena
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), eventNodePos);

        bool colorSet = false;  // Give the new event a color depending on its type
        double red[3] = {1.0, 0.0, 0.0};
        double blue[3] = {0.0, 0.0, 1.0};
        WbFieldRef f_children = wb_supervisor_node_get_field(node_, "children");
        if (f_children && wb_supervisor_field_get_count(f_children) > 0) {
            WbNodeRef shape = wb_supervisor_field_get_mf_node(f_children, 0);
            if (shape) {
                WbFieldRef f_appearance = wb_supervisor_node_get_field(shape, "appearance");
                if (f_appearance) {
                    WbNodeRef appearance = wb_supervisor_field_get_sf_node(f_appearance);
                    if (appearance) {
                        WbFieldRef f_material = wb_supervisor_node_get_field(appearance, "material");
                        if (f_material) {
                            WbNodeRef material = wb_supervisor_field_get_sf_node(f_material);
                            if (material) {
                                // Most Material nodes expose 'diffuseColor' (SFColor)
                                WbFieldRef f_color = wb_supervisor_node_get_field(material, "diffuseColor");
                                if (f_color) {
                                    // If event's task type is A, color red
                                    // If event's task type is B, color blue
                                    wb_supervisor_field_set_sf_color(f_color,
                                                                     type_ == TASK_TYPE_A ? red : blue);
                                    colorSet = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!colorSet) {
            // Fallback: try a direct 'color' field (if some PROTO used it) or just warn
            WbFieldRef f_colorDirect = wb_supervisor_node_get_field(node_, "color");
            if (f_colorDirect)
                wb_supervisor_field_set_sf_color(f_colorDirect, red);
            else
                LOG("Warning: couldn't set event node color (no material/color field)\n");
        }
    }

    bool isDone() const { return timeDone_ != (uint64_t)-1; }

    // Mark event as done by moving it out of the arena
    void markDone(uint64_t clk, uint16_t robotId) {
        timeDone_ = clk;
        completedBy_ = robotId;
        double eventNodePos[3] = {-5, -5, 0.1};
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), eventNodePos);
        g_eventNodesFree.push_back(node_);
    }
};

// Supervisor class
class Supervisor {
   private:
    uint64_t clock_;

    uint16_t nextEventId_;
    vector<unique_ptr<Event>> events_;
    uint16_t numActiveEvents_;
    uint64_t timeOfNextEvent;
    uint64_t timeOfNextGPSTick;

    // Statistics
    uint16_t numEventsHandled;  // total number of events handled
    double statTotalDistance;   // total distance traveled
    Point2d statRobotPrevPos[NUM_ROBOTS];
    uint32_t robotBatteryUsed[NUM_ROBOTS];  // time (ms) spent active (moving + at tasks)

    WbNodeRef robots_[NUM_ROBOTS];
    // We keep receivers handles just in case, but likely won't use them
    // unless we implement a "I am finished" confirmation message.
    WbDeviceTag emitter_;
    WbDeviceTag receivers_[NUM_ROBOTS];

    // track proximity times and current proximity state between robots
    double proximityTime_[NUM_ROBOTS][NUM_ROBOTS];
    bool proximityState_[NUM_ROBOTS][NUM_ROBOTS];

    typedef vector<pair<Event*, MessageType>> EventQueueT;

   private:
    struct Wall {
        Point2d a;         // one endpoint
        Point2d b;         // other endpoint
        double thickness;  // full thickness (m)
    };
    Wall walls_[MAX_WALLS];
    int numWalls_ = 0;

    // track time spent near walls per robot
    double proximityWallTime_[NUM_ROBOTS];  // seconds
    double proximityAnyTime_ = 0.0;  // Any time in sim where there is a proximity either near wall or
                                     // another robot

    // robot geometry
    const double robotRadius_ = 0.05;

   private:
    void addEvent() {
        events_.push_back(unique_ptr<Event>(new Event{nextEventId_++}));  // add to list
        assert(numActiveEvents_ < NUM_ACTIVE_EVENTS);  // check max. active events not reached
        numActiveEvents_++;
        timeOfNextEvent = clock_ + expovariate(EVENT_GENERATION_DELAY);
    }

    // Init robot and get robot_ids and receivers
    void linkRobot(uint16_t id) {
        const char kRobotNameFormat[] = "e-puck%d";
        char nodeName[16];

        // Get the robot node's handle
        sprintf(nodeName, kRobotNameFormat, id);
        robots_[id] = wb_supervisor_node_get_from_def(nodeName);
        if (!robots_[id]) {
            LOG("Missing node for robot #%d\n", id);
            exit(1);
        }

        // (Getting receiver no longer needed)
    }

    Point2d getRobotPos(uint16_t robotId) {
        WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robotId], "translation");
        const double* pos = wb_supervisor_field_get_sf_vec3f(f_pos);
        return Point2d(pos[0], pos[1]);
    }

    void setRobotPos(uint16_t robotId, const Point2d& pos) {
        WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robotId], "translation");
        double p[3] = {pos.x, pos.y, 0.01};
        return wb_supervisor_field_set_sf_vec3f(f_pos, p);
    }

    // Assemble a new message to be sent to robots (added type of event)
    void buildMessage(uint16_t robotId, const Event* event, MessageType msgType, MessageT* msg) {
        WbFieldRef f_rot = wb_supervisor_node_get_field(robots_[robotId], "rotation");
        Point2d pos = getRobotPos(robotId);
        const double* rot = wb_supervisor_field_get_sf_rotation(f_rot);

        msg->robotId = robotId;
        msg->robotX = pos.x;                // no gps noise used here
        msg->robotY = pos.y;                // no gps noise used here
        double heading = -rot[2] * rot[3];  // no gps noise used here
        msg->heading = heading > 2 * M_PI ? heading - 2 * M_PI : heading;
        msg->msgType = msgType;
        msg->eventId = -1;
        msg->taskType = TASK_TYPE_INVALID;

        if (event) {
            assert(msgType != MSG_EVENT_INVALID && msgType != MSG_GPS_ONLY);
            msg->eventId = event->id_;
            msg->eventX = event->pos_.x;
            msg->eventY = event->pos_.y;
            msg->taskType = event->type_;
            // NOTE: eventIndex not used/irrelevant for "distributed" supervisor
            // (robots handle their own task lists/indexing)
            msg->eventIndex = -1;
        }
    }

    // Track which robots are "working" on tasks (near task for required duration)
    struct RobotWorkState {
        int taskId = -1;           // Task robot is working on (-1 = none)
        uint64_t arrivalTime = 0;  // When robot arrived at task
        int requiredDuration = 0;  // How long robot needs to stay
    };
    RobotWorkState robotWorkStates_[NUM_ROBOTS];

    void markEventsDone() {
        // METHOD 1: Process incoming messages from robots (if they're in range)
        while (wb_receiver_get_queue_length(receivers_[0]) > 0) {
            const void* data = wb_receiver_get_data(receivers_[0]);
            size_t size = wb_receiver_get_data_size(receivers_[0]);

            // Use msgType discriminator instead of size
            if (size >= sizeof(RobotMsgType)) {
                RobotMsgType msgType;
                memcpy(&msgType, data, sizeof(RobotMsgType));

                if (msgType == ROBOT_MSG_STATE && size >= sizeof(RobotStateMsg)) {
                    RobotStateMsg msg;
                    memcpy(&msg, data, sizeof(RobotStateMsg));

                    if (msg.isTaskComplete) {
                        // Find the event
                        for (auto& event : events_) {
                            if (event->id_ == msg.currentTaskId && !event->isDone()) {
                                LOG("Robot %d reported completion of Task %d.\n", msg.robotId,
                                    msg.currentTaskId);

                                // Calculate battery usage for time spent at the task completing it
                                int timeAtTask = 0;
                                if (msg.robotId <= 1) {  // A-specialist (robots 0, 1)
                                    timeAtTask = (event->type_ == TASK_TYPE_A) ? 3000 : 5000;
                                } else {  // B-specialist (robots 2, 3, 4)
                                    timeAtTask = (event->type_ == TASK_TYPE_A) ? 9000 : 1000;
                                }
                                robotBatteryUsed[msg.robotId] += timeAtTask;

                                numEventsHandled++;
                                event->markDone(clock_, msg.robotId);
                                numActiveEvents_--;
                                break;
                            }
                        }
                    }
                }
            }
            wb_receiver_next_packet(receivers_[0]);
        }

        // METHOD 2: Proximity-based completion detection (backup for limited comm range)
        // Check if any robot is near a task long enough to complete it
        for (int robotId = 0; robotId < NUM_ROBOTS; ++robotId) {
            Point2d robotPos = getRobotPos(robotId);
            bool robotSpecA = (robotId <= 1);  // Robots 0,1 are A-specialists

            for (auto& event : events_) {
                if (event->isDone()) continue;

                double dist = robotPos.distanceTo(event->pos_);

                if (dist < EVENT_RANGE) {
                    // Robot is at the task location
                    if (robotWorkStates_[robotId].taskId != event->id_) {
                        // Just arrived at this task
                        robotWorkStates_[robotId].taskId = event->id_;
                        robotWorkStates_[robotId].arrivalTime = clock_;
                        // Calculate required duration based on specialization
                        if (event->type_ == TASK_TYPE_A) {
                            robotWorkStates_[robotId].requiredDuration = robotSpecA ? 3000 : 9000;
                        } else {
                            robotWorkStates_[robotId].requiredDuration = robotSpecA ? 5000 : 1000;
                        }
                    } else {
                        // Still at the same task - check if duration met
                        uint64_t timeAtTask = clock_ - robotWorkStates_[robotId].arrivalTime;
                        if (timeAtTask >= (uint64_t)robotWorkStates_[robotId].requiredDuration) {
                            // Task completed via proximity detection!
                            LOG("Robot %d completed Task %d via proximity detection (at task for %llums)\n",
                                robotId, event->id_, (unsigned long long)timeAtTask);

                            robotBatteryUsed[robotId] += robotWorkStates_[robotId].requiredDuration;
                            numEventsHandled++;
                            event->markDone(clock_, robotId);
                            numActiveEvents_--;
                            robotWorkStates_[robotId].taskId = -1;  // Reset
                        }
                    }
                } else {
                    // Robot moved away from task
                    if (robotWorkStates_[robotId].taskId == event->id_) {
                        robotWorkStates_[robotId].taskId = -1;
                    }
                }
            }
        }
    }

    void collisionDetection(uint64_t stepSize) {
        // Accumulate time (s) robots spend within PROXIMITY_THRESHOLD
        double dtInSeconds = (double)stepSize / 1000.0;
        bool anyProximityThisStep = false;

        // Check all robot pairs for proximity
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            Point2d robotIPos = getRobotPos(i);
            for (int j = i + 1; j < NUM_ROBOTS; ++j) {
                Point2d robotJPos = getRobotPos(j);
                double distance = robotIPos.distanceTo(robotJPos);
                bool within = distance < PROXIMITY_THRESHOLD;
                if (within) {
                    proximityTime_[i][j] += dtInSeconds;
                    proximityTime_[j][i] += dtInSeconds;
                    anyProximityThisStep = true;
                }
                proximityState_[i][j] = within;
                proximityState_[j][i] = within;
            }

            // Check proximity to walls (no avoidance)
            for (int w = 0; w < numWalls_; ++w) {
                double dist = robotIPos.distanceToSegment(walls_[w].a, walls_[w].b);
                // Optional safety margin
                double threshold = (walls_[w].thickness / 2.0) + robotRadius_ + 0.0;
                if (dist < threshold) {
                    proximityWallTime_[i] += dtInSeconds;
                    anyProximityThisStep = true;
                }
            }
        }

        // Increment the final parameter
        if (anyProximityThisStep) proximityAnyTime_ += dtInSeconds;
    }

    // Calculate total distance travelled by robots
    void calcStatTotalDistance(uint64_t stepSize) {
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            Point2d robotPos = getRobotPos(i);
            auto dist = robotPos.distanceTo(statRobotPrevPos[i]);
            if (dist > 1e-6) {
                robotBatteryUsed[i] += stepSize;
                statTotalDistance += dist;
            }
            statRobotPrevPos[i] = robotPos;
        }
    }

   public:
    Supervisor() : events_(MAX_EVENTS) {}

    // Reset robots & events
    void reset() {
        clock_ = 0;
        g_simTime = 0;

        // Initialize & link events
        nextEventId_ = 0;
        events_.clear();
        numActiveEvents_ = 0;
        timeOfNextEvent = 0;  // invalid state
        timeOfNextGPSTick = 0;

        numEventsHandled = 0;
        statTotalDistance = 0.0;

        // For wall proximity code
        numWalls_ = 0;
        walls_[numWalls_++] = Wall{Point2d(-0.6375, 0.0), Point2d(-0.2625, 0.0), 0.01};  // Lefthand wall
        walls_[numWalls_++] = Wall{Point2d(0.125, 0.65), Point2d(0.125, -0.2), 0.01};    // Top wall

        // Init wall proximity times
        for (int i = 0; i < NUM_ROBOTS; ++i) proximityWallTime_[i] = 0.0;

        // Init robot work states (for proximity-based completion detection)
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            robotWorkStates_[i].taskId = -1;
            robotWorkStates_[i].arrivalTime = 0;
            robotWorkStates_[i].requiredDuration = 0;
        }
        proximityAnyTime_ = 0.0;

        // Add the first few events
        for (int i = 0; i < NUM_ACTIVE_EVENTS; ++i) addEvent();

        // Link & initialize robots
        for (int i = 0; i < NUM_ROBOTS; i++) {
            linkRobot(i);

            Point2d pos = randCoord();
            setRobotPos(i, pos);
            statRobotPrevPos[i] = pos;
            robotBatteryUsed[i] = 0;
        }

        // Initialize the emitter
        emitter_ = wb_robot_get_device("sup_emitter");
        if (!emitter_) {
            LOG("Missing supervisor emitter!\n");
            exit(1);
        }

        // Initialize receiver to listen to robot broadcasts
        receivers_[0] = wb_robot_get_device("rec0");
        if (receivers_[0]) {
            wb_receiver_enable(receivers_[0], TIME_STEP);
            wb_receiver_set_channel(receivers_[0], WB_CHANNEL_BROADCAST);
        } else {
            LOG("Missing supervisor receiver rec0!\n");
        }
    }

    // Do a step
    bool step(uint64_t stepSize) {
        clock_ += stepSize;
        g_simTime = clock_;

        // 1. Check for physical completion of tasks
        markEventsDone();

        // 2. Add a random new event, if the time has come
        assert(timeOfNextEvent > 0);
        if (clock_ >= timeOfNextEvent && numActiveEvents_ < NUM_ACTIVE_EVENTS) {
            addEvent();
        }

        /* vvv  NO MORE AUCTIONS IN DISTRIBUTED VERSION  vvv
        // Send and receive messages
        BidT* pbid;  // inbound
        for (int i = 0; i < NUM_ROBOTS; i++) {
            // Check if we're receiving data
            if (wb_receiver_get_queue_length(receivers_[i]) > 0) {
                assert(wb_receiver_get_queue_length(receivers_[i]) > 0);
                assert(wb_receiver_get_data_size(receivers_[i]) == sizeof(BidT));

                pbid = (BidT*)wb_receiver_get_data(receivers_[i]);
                assert(pbid->robotId == i);

                Event* event = events_.at(pbid->eventId).get();
                event->updateAuction(pbid->robotId, pbid->bidValue, pbid->eventIndex);
                // TODO: Refactor this (same code above in handleAuctionEvents)
                if (event->isAssigned()) {
                    eventQueue.emplace_back(event, MSG_EVENT_WON);
                    auction = NULL;
                    LOG("Robot %d won event %d\n", event->assignedTo_, event->id_);
                }

                wb_receiver_next_packet(receivers_[i]);
            }
        }
        */

        // 3. Broadcast World State (GPS + All Active Tasks)
        MessageT msg;

        // Only send GPS updates at fixed intervals
        bool sendGPSThisStep = (clock_ >= timeOfNextGPSTick);
        if (sendGPSThisStep) timeOfNextGPSTick = clock_ + GPS_INTERVAL;

        for (int i = 0; i < NUM_ROBOTS; i++) {
            // Send updates to the robot
            // Set emitter channel to specific robot
            while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);

            if (sendGPSThisStep) {
                buildMessage(i, NULL, MSG_GPS_ONLY, &msg);
                // while (wb_emitter_get_channel(emitter_) != i+1) wb_emitter_set_channel(emitter_, i+1);
                // ^ Already done above?
                wb_emitter_send(emitter_, &msg, sizeof(MessageT));
            }

            // Send ALL active events to this robot
            // This replaces the auction logic. The robot receives the list and decides.
            for (const auto& event : events_) {
                if (!event->isDone()) {
                    // Send as MSG_EVENT_NEW so the robot knows it exists.
                    // The robot is responsible for checking if it already knows about this ID.
                    buildMessage(i, event.get(), MSG_EVENT_NEW, &msg);
                    wb_emitter_send(emitter_, &msg, sizeof(MessageT));
                }
            }
        }

        // 4. Stats
        collisionDetection(stepSize);     // Keep track of our proximity/colission paramter
        calcStatTotalDistance(stepSize);  // Keep track of distance travelled by all robots

        // 5. Time to end the experiment?
        if (numEventsHandled >= TOTAL_EVENTS_TO_HANDLE || (MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
            // Send QUIT message to all robots
            for (int i = 0; i < NUM_ROBOTS; i++) {
                LOG("Sending MSG_QUIT message to robot %d\n", i);
                buildMessage(i, NULL, MSG_QUIT, &msg);
                wb_emitter_set_channel(emitter_, i + 1);
                wb_emitter_send(emitter_, &msg, sizeof(MessageT));
            }

            double clock_s = ((double)clock_) / 1000.0;
            double ehr = ((double)numEventsHandled) / clock_s;
            double perf = ((double)numEventsHandled) / statTotalDistance;

            // Print stats
            LOG("Handled %d events in %d seconds, events handled per second = %.2f\n", numEventsHandled,
                (int)clock_ / 1000, ehr);

            // print proximity matrix (seconds)
            LOG("*********PROXIMTY TO OTHER ROBOTS*********\n\n");
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                // Print proximity times for robot i to all other robots j in one line
                stringstream ss;
                ss << "Robot " << i << ": ";
                for (int j = 0; j < NUM_ROBOTS; ++j) {
                    ss << j << ": " << fixed << std::setprecision(2) << proximityTime_[i][j];
                    if (j < NUM_ROBOTS - 1) ss << ", ";
                }
                LOG("%s\n", ss.str().c_str());
            }
            // print wall proximity times
            LOG("*********PROXIMITY TO WALL*********\n");
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                LOG("Robot %d was near a wall for %.2f seconds.\n", i, proximityWallTime_[i]);
            }

            // final metric for any proximity
            LOG("*********ANY PROXIMITY METRIC*********\n");
            LOG("Total time any robot was near another robot or a wall: %.2f seconds\n", proximityAnyTime_);

            // battery used
            LOG("*********BATTERY USED METRIC*********\n");
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                double batteryAsPercentage = (robotBatteryUsed[i] * 100.0) / MAX_BATTERY_LIFETIME;
                LOG("Battery usage for robot %d: %dms - corresponds to %.2f %% of total battery life\n", i,
                    robotBatteryUsed[i], batteryAsPercentage);
            }

            LOG("Performance: %f\n", perf);
            return false;
        } else {
            return true;
        }  // continue
    }  // << step() <<
};

// Links up all the nodes we are interested in.
// Gets called by webots at robot_live(reset)
void link_event_nodes() {
    const char kEventNameFormat[] = "e%d";
    char nodeName[16];

    for (int i = 0; i < MAX_EVENTS; ++i) {
        sprintf(nodeName, kEventNameFormat, i);
        g_eventNodes[i] = wb_supervisor_node_get_from_def(nodeName);
        g_eventNodesFree.push_back(g_eventNodes[i]);
    }
}

// MAIN LOOP (does steps)
int main(void) {
    Supervisor supervisor{};

    // Initialization
    wb_robot_init();
    link_event_nodes();
    wb_robot_step(TIME_STEP);

    srand(time(NULL));
    supervisor.reset();

    // Start the sim loop
    LOG("Starting main loop...\n");
    while (wb_robot_step(TIME_STEP) != -1) {
        if (!supervisor.step(TIME_STEP)) break;  // break at return = false
    }
    wb_supervisor_simulation_reset_physics();
    wb_robot_cleanup();
    exit(0);
    return 0;
}
