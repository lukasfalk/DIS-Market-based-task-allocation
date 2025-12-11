/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        auct_super.cc
 * author:
 * description: Supervisor for market-based task allocation (DIS lab05)
 *
 * $Revision$	February 2016 by Florian Maushart
 * $Date$
 * $Author$   Last update 2024 by Wanting Jin
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <assert.h>
#include <time.h> /* time_t, struct tm, difftime, time, mktime */

#include <bitset>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

using namespace std;

// Define M_PI if not already defined (needed for some compilers)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../common/communication.hpp"
#include "../common/constants.hpp"
#include "../common/geometry.hpp"
#include "../common/logging.hpp"
#include "../common/map_config.hpp"
#include "../common/pathfinding.hpp"
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

#define EVENT_RANGE (0.05)             // distance within which a robot must come to do event
#define EVENT_TIMEOUT (10000)          // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000)  // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)
#define PROXIMITY_THRESHOLD (0.20)  // distance below which robots are considered in collision

// Parameters that can be changed
#define NUM_ACTIVE_EVENTS 10        // number of active events
#define TOTAL_EVENTS_TO_HANDLE 999  // Events after which simulation stops or...

#define MAX_WALLS 2

WbNodeRef g_eventNodes[MAX_EVENTS];
vector<WbNodeRef> g_eventNodesFree;

// Test function to verify interior wall detection
void testInteriorWallDetection() {
    printf("\n=== Testing Interior Wall Detection ===\n");

    // Test points that should be INSIDE walls
    Point2d inside_bijc = {0.125, 0.0};   // Middle of vertical wall BIJC
    Point2d inside_efhg = {-0.395, 0.0};  // Middle of horizontal wall EFHG

    // Test points that should be OUTSIDE walls
    Point2d outside_1 = {0.3, 0.3};    // Top right
    Point2d outside_2 = {-0.4, -0.4};  // Bottom left
    Point2d outside_3 = {0.0, 0.0};    // Center

    printf("Testing INSIDE points (should be rejected):\n");
    printf("  (0.125, 0.0) in wall: %s\n", MapConfig::isPointInInteriorWall(inside_bijc) ? "YES" : "NO");
    printf("  (-0.395, 0.0) in wall: %s\n", MapConfig::isPointInInteriorWall(inside_efhg) ? "YES" : "NO");

    printf("Testing OUTSIDE points (should be accepted):\n");
    printf("  (0.3, 0.3) in wall: %s\n", MapConfig::isPointInInteriorWall(outside_1) ? "YES" : "NO");
    printf("  (-0.4, -0.4) in wall: %s\n", MapConfig::isPointInInteriorWall(outside_2) ? "YES" : "NO");
    printf("  (0.0, 0.0) in wall: %s\n", MapConfig::isPointInInteriorWall(outside_3) ? "YES" : "NO");

    printf("=== Interior Wall Test Complete ===\n\n");
}

double gauss(void) {
    double x1, x2, w;
    do {
        x1 = 2.0 * utils::random_01() - 1.0;
        x2 = 2.0 * utils::random_01() - 1.0;
        w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w)) / w);
    return (x1 * w);
}

Point2d randCoord() {
    // Sample uniformly within the arena bounds defined by the pathfinding graph
    // Arena bounds: x in [-0.575, 0.575], y in [-0.575, 0.575]
    // Excludes points inside interior walls
    const double ARENA_MIN = -0.575;
    const double ARENA_MAX = 0.575;

    Point2d candidate;
    int attempts = 0;
    do {
        candidate.x = ARENA_MIN + (ARENA_MAX - ARENA_MIN) * utils::random_01();
        candidate.y = ARENA_MIN + (ARENA_MAX - ARENA_MIN) * utils::random_01();
        attempts++;
    } while (MapConfig::isPointInInteriorWall(candidate));

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
    uint16_t id_;          // event id
    Point2d pos_;          // event pos
    WbNodeRef node_;       // event node ref
    TaskType type_;        // type of event (TASK_TYPE_A or TASK_TYPE_B)
    uint16_t assignedTo_;  // id of the robot that will handle this event

    // Auction data
    uint64_t timeAnnounced_;  // time at which event was announced to robots
    bitset<NUM_ROBOTS> bidsIn_;
    uint16_t bestBidder_;  // id of the robot that had the best bid so far
    double bestBid_;       // value of the best bid (lower is better)
    uint64_t timeDone_;    // time at which the assigned robot reached the event
    int bidderIndex;       // index at which the bidder will put event in tasklist

    // Public functions
   public:
    // Event creation
    Event(uint16_t id)
        : id_(id),
          pos_(randCoord()),
          type_(utils::random_01() < (1.0f / 3.0f) ? TASK_TYPE_A : TASK_TYPE_B),  // 1/3 of the time task
                                                                                  // type A, 2/3 of the time
                                                                                  // type B
          assignedTo_(-1),
          timeAnnounced_(-1),
          bestBidder_(-1),
          bestBid_(0.0),
          timeDone_(-1) {
        node_ = g_eventNodesFree.back();  // Place node
        g_eventNodesFree.pop_back();      // remove the "free" node from the free list

        double eventNodePos[3];  // Place event in arena
        eventNodePos[0] = pos_.x;
        eventNodePos[1] = pos_.y;
        eventNodePos[2] = .01;
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
                                    if (type_ == TASK_TYPE_A)  // If event's task type is A, color red
                                        wb_supervisor_field_set_sf_color(f_color, red);
                                    else  // If event's task type is B, color blue
                                        wb_supervisor_field_set_sf_color(f_color, blue);
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

    bool isAssigned() const { return assignedTo_ != (uint16_t)-1; }
    bool wasAnnounced() const { return timeAnnounced_ != (uint64_t)-1; }
    bool hasBids() const { return bestBidder_ != (uint16_t)-1; }
    bool isDone() const { return timeDone_ != (uint64_t)-1; }

    // Check if event can be assigned
    void updateAuction(uint16_t bidder, double bid, int index) {
        if (bid >= 0.0 && (!hasBids() || bid < bestBid_)) {
            bestBidder_ = bidder;
            bestBid_ = bid;
            bidderIndex = index;
        }
        bidsIn_.set(bidder);
        if (bidsIn_.all()) assignedTo_ = bestBidder_;
    }

    void restartAuction() {
        assignedTo_ = -1;
        timeAnnounced_ = -1;
        bidsIn_.reset();
        bestBidder_ = -1;
        bestBid_ = 0.0;
        timeDone_ = -1;
    }

    // Mark event as done by moving it out of the arena
    void markDone(uint64_t clk) {
        timeDone_ = clk;
        double eventNodePos[3] = {-5, -5, 0.1};
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), eventNodePos);
        g_eventNodesFree.push_back(node_);
    }
};

// Supervisor class
class Supervisor {
    // Private variables
   private:
    uint64_t clock_;

    uint16_t nextEventId_;
    vector<unique_ptr<Event>> events_;
    uint16_t numActiveEvents_;
    uint64_t timeOfNextEvent;
    Point2d pos_;    // supervisor pos
    Event* auction;  // the event currently being auctioned
    uint64_t timeOfNextGPSTick;

    uint16_t numEventsHandled;  // total number of events handled
    double statTotalDistance;   // total distance traveled
    Point2d statRobotPrevPos[NUM_ROBOTS];
    uint32_t robotBatteryUsed[NUM_ROBOTS];  // time spent moving //I think int and unsigned int are too
                                            // small, a larger non-floating datatype can be used (but i
                                            // cant remember them and i have no internet LMAO)

    WbNodeRef robots_[NUM_ROBOTS];
    WbDeviceTag emitter_;
    WbDeviceTag receivers_[NUM_ROBOTS];

    // track proximity times and current proximity state between robots
    double proximityTime_[NUM_ROBOTS][NUM_ROBOTS];
    bool proximityState_[NUM_ROBOTS][NUM_ROBOTS];

    typedef vector<pair<Event*, MessageType>> EventQueueT;

   private:
    // wall struct
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
    // Private functions
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
        const char kReceiverNameFormat[] = "rec%d";
        char nodeName[16];

        // Get the robot node's handle
        sprintf(nodeName, kRobotNameFormat, id);
        robots_[id] = wb_supervisor_node_get_from_def(nodeName);
        if (!robots_[id]) {
            LOG("Missing node for robot #%d\n", id);
            exit(1);
        }

        // Get the respective receiver
        sprintf(nodeName, kReceiverNameFormat, id);
        receivers_[id] = wb_robot_get_device(nodeName);
        if (!receivers_[id]) {
            LOG("Missing receiver for robot #%d\n", id);
            exit(1);
        }
        wb_receiver_enable(receivers_[id], 2);  // 32
        wb_receiver_set_channel(receivers_[id], id + 1);
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
        msg->eventX = 0.0;
        msg->eventY = 0.0;
        msg->taskType = TASK_TYPE_INVALID;

        if (event) {
            assert(msgType != MSG_EVENT_INVALID && msgType != MSG_GPS_ONLY);
            msg->eventId = event->id_;
            msg->eventX = event->pos_.x;
            msg->eventY = event->pos_.y;
            msg->eventIndex = event->bidderIndex;
            msg->taskType = event->type_;
        }
    }

    Point2d getRobotPos(uint16_t robotId) {
        WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robotId], "translation");
        const double* pos = wb_supervisor_field_get_sf_vec3f(f_pos);
        return Point2d(pos[0], pos[1]);
    }

    void setRobotPos(uint16_t robotId, const Point2d& pos) { setRobotPos(robotId, pos.x, pos.y); }

    void setRobotPos(uint16_t robotId, double x, double y) {
        WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robotId], "translation");
        double pos[3] = {x, y, 0.01};
        return wb_supervisor_field_set_sf_vec3f(f_pos, pos);
    }

    // Marks one event as done, if one of the robots is within the range
    void markEventsDone(EventQueueT& eventQueue) {
        for (auto& event : events_) {
            if (!event->isAssigned() || event->isDone()) continue;

            Point2d currentRobotPos = getRobotPos(event->assignedTo_);
            double dist = event->pos_.distanceTo(currentRobotPos);

            if (dist <= EVENT_RANGE) {
                LOG("Robot %d reached event %d\n", event->assignedTo_, event->id_);

                // Calculate battery usage for time spent at the task completing it
                int timeAtTask = 0;
                if (event->assignedTo_ <= 1) {  // A-specialist (robots 0, 1)
                    timeAtTask = (event->type_ == TASK_TYPE_A) ? 3000 : 5000;
                } else {  // B-specialist (robots 2, 3, 4)
                    timeAtTask = (event->type_ == TASK_TYPE_A) ? 9000 : 1000;
                }
                robotBatteryUsed[event->assignedTo_] += timeAtTask;

                numEventsHandled++;
                event->markDone(clock_);
                numActiveEvents_--;
                eventQueue.emplace_back(event.get(), MSG_EVENT_DONE);
            }
        }
    }

    void collisionDetection(uint64_t stepSize) {
        // accumulate time (in seconds) robots spend within PROXIMITY_THRESHOLD
        double dtInSeconds = (double)stepSize / 1000.0;
        bool anyProximityThisStep = false;

        // check all robot pairs for proximity
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
        }

        // Check proximity to walls (no avoidance)
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            Point2d robotIPos = getRobotPos(i);
            for (int w = 0; w < numWalls_; ++w) {
                double dist = robotIPos.distanceToSegment(walls_[w].a, walls_[w].b);
                double threshold = (walls_[w].thickness / 2.0) + robotRadius_ + 0.0;  // optional safety
                                                                                      // margin
                if (dist < threshold) {
                    proximityWallTime_[i] += dtInSeconds;
                    anyProximityThisStep = true;
                }
            }
        }

        if (anyProximityThisStep) {
            proximityAnyTime_ += dtInSeconds;  // increment the final parameter
        }
    }
    void handleAuctionEvents(EventQueueT& eventQueue) {
        // For each unassigned event
        for (auto& event : events_) {
            if (event->isAssigned()) continue;

            // Send announce, if new
            // IMPL DETAIL: Only allow one auction at a time.
            if (!event->wasAnnounced() && !auction) {
                event->timeAnnounced_ = clock_;
                eventQueue.emplace_back(event.get(), MSG_EVENT_NEW);
                auction = event.get();
                LOG("Event %d announced\n", event->id_);

                // End early or restart, if timed out
            } else if (clock_ - event->timeAnnounced_ > EVENT_TIMEOUT) {
                // End early if we have any bids at all
                if (event->hasBids()) {
                    // IMPLEMENTATION DETAIL: If about to time out, assign to
                    // the highest bidder or restart the auction if there is none.
                    event->assignedTo_ = event->bestBidder_;
                    eventQueue.emplace_back(event.get(), MSG_EVENT_WON);  // FIXME?
                    auction = NULL;
                    LOG("Robot %d won event %d\n", event->assignedTo_, event->id_);

                    // Restart (incl. announce) if no bids
                } else {
                    // (reannounced in next iteration)
                    event->restartAuction();
                    if (auction == event.get()) auction = NULL;
                }
            }
        }
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

    // Public fucntions
   public:
    Supervisor() : events_(MAX_EVENTS) {}

    // Reset robots & events
    void reset() {
        clock_ = 0;
        g_simTime = 0;

        // initialize & link events
        nextEventId_ = 0;
        events_.clear();
        numActiveEvents_ = 0;
        timeOfNextEvent = 0;  // invalid state
        auction = NULL;
        timeOfNextGPSTick = 0;

        numEventsHandled = 0;
        statTotalDistance = 0.0;

        // for wall proximity code
        numWalls_ = 0;
        walls_[numWalls_++] = Wall{Point2d(-0.6375, 0.0), Point2d(-0.2625, 0.0), 0.01};  // Lefthand wall
        walls_[numWalls_++] = Wall{Point2d(0.125, 0.65), Point2d(0.125, -0.2), 0.01};    // Top wall

        // init wall proximity times
        for (int i = 0; i < NUM_ROBOTS; ++i) proximityWallTime_[i] = 0.0;

        // add the first few events
        for (int i = 0; i < NUM_ACTIVE_EVENTS; ++i) {
            addEvent();
        }

        // link & initialize robots
        for (int i = 0; i < NUM_ROBOTS; i++) {
            linkRobot(i);

            Point2d pos = randCoord();
            setRobotPos(i, pos);
            statRobotPrevPos[i] = pos;
            robotBatteryUsed[i] = 0;
        }

        // initialize the emitter
        emitter_ = wb_robot_get_device("sup_emitter");
        if (!emitter_) {
            LOG("Missing supervisor emitter!\n");
            exit(1);
        }
    }

    // Do a step
    bool step(uint64_t stepSize) {
        clock_ += stepSize;
        g_simTime = clock_;

        // Events that will be announced next or that have just been assigned/done
        EventQueueT eventQueue;

        markEventsDone(eventQueue);

        // ** Add a random new event, if the time has come
        assert(timeOfNextEvent > 0);
        if (clock_ >= timeOfNextEvent && numActiveEvents_ < NUM_ACTIVE_EVENTS) {
            addEvent();
        }

        handleAuctionEvents(eventQueue);

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

        // outbound
        MessageT msg;
        bool tickGPSThisStep = false;

        if (clock_ >= timeOfNextGPSTick) {
            tickGPSThisStep = true;
            timeOfNextGPSTick = clock_ + GPS_INTERVAL;
        }

        for (int i = 0; i < NUM_ROBOTS; i++) {
            // Send updates to the robot
            while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);

            if (tickGPSThisStep) {
                buildMessage(i, NULL, MSG_GPS_ONLY, &msg);
                //        printf("sending message %d , %d \n",msg.event_id,msg.robot_id);
                while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);
                wb_emitter_send(emitter_, &msg, sizeof(MessageT));
            }

            for (const auto& eventAndMessageTypeTuple : eventQueue) {
                const Event* event = eventAndMessageTypeTuple.first;
                const MessageType msg_type = eventAndMessageTypeTuple.second;
                if (event->isAssigned() && event->assignedTo_ != i) continue;

                buildMessage(i, event, msg_type, &msg);
                while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);
                //        printf("> Sent message to robot %d // msg_type=%d\n", i, msg_type);
                //        printf("sending message event %d , robot %d , emitter %d, channel
                //        %d\n",msg.event_id,msg.robot_id,emitter_,      wb_emitter_get_channel(emitter_));

                wb_emitter_send(emitter_, &msg, sizeof(MessageT));
            }
        }

        // Keep track of our proximity/colission paramter
        collisionDetection(stepSize);

        // Keep track of distance travelled by all robots
        calcStatTotalDistance(stepSize);

        // Time to end the experiment?
        if (numEventsHandled >= TOTAL_EVENTS_TO_HANDLE || (MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
            for (int i = 0; i < NUM_ROBOTS; i++) {
                // LOG("Sending MSG_QUIT message to robot %d\n", i);
                buildMessage(i, NULL, MSG_QUIT, &msg);
                wb_emitter_set_channel(emitter_, i + 1);
                wb_emitter_send(emitter_, &msg, sizeof(MessageT));
            }
            double clock_s = ((double)clock_) / 1000.0;
            double ehr = ((double)numEventsHandled) / clock_s;
            double perf = ((double)numEventsHandled) / statTotalDistance;

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

    // initialization
    wb_robot_init();
    link_event_nodes();
    wb_robot_step(TIME_STEP);

    srand(time(NULL));
    supervisor.reset();

    // start the controller
    LOG("Starting main loop...\n");
    while (wb_robot_step(TIME_STEP) != -1) {
        if (!supervisor.step(TIME_STEP)) break;  // break at return = false
    }
    wb_supervisor_simulation_reset_physics();
    wb_robot_cleanup();
    exit(0);
    return 0;
}
