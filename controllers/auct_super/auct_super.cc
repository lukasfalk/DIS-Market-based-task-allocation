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
#include <memory>
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

#include "Point2d.h"
#include "message.h"

#define DBG(x) printf x
#define RAND ((float)rand() / RAND_MAX)

#define MAX_ROBOTS 9
#define MAX_EVENTS 10

#define STEP_SIZE 64          // simulation step size
#define AUCTION_TIMEOUT 1000  // number of steps after which an auction stops

#define EVENT_RANGE (0.1)              // distance within which a robot must come to do event
#define EVENT_TIMEOUT (10000)          // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000)  // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)
#define PROXIMITY_THRESHOLD (0.20)  // distance below which robots are considered in collision

// Parameters that can be changed
#define NUM_ROBOTS 5                          // Change this also in the epuck_crown.c!
#define NUM_ACTIVE_EVENTS 10                  // number of active events
#define TOTAL_EVENTS_TO_HANDLE 999            // Events after which simulation stops or...
#define MAX_RUNTIME (3 * 60 * 1000)           // ...total runtime after which simulation stops
#define MAX_BATTERY_LIFETIME (2 * 60 * 1000)  // 2 minutes of battery life in ms

#define MAX_WALLS 2

WbNodeRef g_event_nodes[MAX_EVENTS];
vector<WbNodeRef> g_event_nodes_free;

double gauss(void) {
    double x1, x2, w;
    do {
        x1 = 2.0 * RAND - 1.0;
        x2 = 2.0 * RAND - 1.0;
        w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w)) / w);
    return (x1 * w);
}

double rand_coord() {
    // return -1.0 + 2.0*RAND;
    return -0.55 + 0.95 * RAND;  // Full arena of final project
}

double expovariate(double mu) {
    double uniform = RAND;
    while (uniform < 1e-7) uniform = RAND;
    return -log(uniform) * mu;
}

// Event class
class Event {
    // Public variables
   public:
    uint16_t id_;            // event id
    Point2d pos_;            // event pos
    WbNodeRef node_;         // event node ref
    task_type_t task_type_;  // type of event (TASK_TYPE_A or TASK_TYPE_B)
    uint16_t assigned_to_;   // id of the robot that will handle this event

    // Auction data
    uint64_t t_announced_;  // time at which event was announced to robots
    bitset<NUM_ROBOTS> bids_in_;
    uint16_t best_bidder_;  // id of the robot that had the best bid so far
    double best_bid_;       // value of the best bid (lower is better)
    uint64_t t_done_;       // time at which the assigned robot reached the event
    int bidder_index;       // index at which the bidder will put event in tasklist

    // Public functions
   public:
    // Event creation
    Event(uint16_t id)
        : id_(id),
          pos_(rand_coord(), rand_coord()),
          task_type_(RAND < (1.0f / 3.0f) ? TASK_TYPE_A : TASK_TYPE_B),  // 1/3 of the time task type A, 2/3 of the time type B
          assigned_to_(-1),
          t_announced_(-1),
          best_bidder_(-1),
          best_bid_(0.0),
          t_done_(-1) {
        node_ = g_event_nodes_free.back();  // Place node
        g_event_nodes_free.pop_back();      // remove the "free" node from the free list

        double event_node_pos[3];  // Place event in arena
        event_node_pos[0] = pos_.x;
        event_node_pos[1] = pos_.y;
        event_node_pos[2] = .01;
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), event_node_pos);

        bool color_set = false;  // Give the new event a color depending on its type
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
                                    if (task_type_ == TASK_TYPE_A)  // If event's task type is A, color red
                                        wb_supervisor_field_set_sf_color(f_color, red);
                                    else  // If event's task type is B, color blue
                                        wb_supervisor_field_set_sf_color(f_color, blue);
                                    color_set = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!color_set) {
            // Fallback: try a direct 'color' field (if some PROTO used it) or just warn
            WbFieldRef f_color_direct = wb_supervisor_node_get_field(node_, "color");
            if (f_color_direct)
                wb_supervisor_field_set_sf_color(f_color_direct, red);
            else
                DBG(("Warning: couldn't set event node color (no material/color field)\n"));
        }
    }

    bool is_assigned() const { return assigned_to_ != (uint16_t)-1; }
    bool was_announced() const { return t_announced_ != (uint64_t)-1; }
    bool has_bids() const { return best_bidder_ != (uint16_t)-1; }
    bool is_done() const { return t_done_ != (uint64_t)-1; }

    // Check if event can be assigned
    void updateAuction(uint16_t bidder, double bid, int index) {
        if (bid >= 0.0 && (!has_bids() || bid < best_bid_)) {
            best_bidder_ = bidder;
            best_bid_ = bid;
            bidder_index = index;
        }
        bids_in_.set(bidder);
        if (bids_in_.all()) assigned_to_ = best_bidder_;
    }

    void restartAuction() {
        assigned_to_ = -1;
        t_announced_ = -1;
        bids_in_.reset();
        best_bidder_ = -1;
        best_bid_ = 0.0;
        t_done_ = -1;
    }

    void markDone(uint64_t clk) {
        t_done_ = clk;
        double event_node_pos[3] = {-5, -5, 0.1};
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_, "translation"), event_node_pos);
        g_event_nodes_free.push_back(node_);
    }
};

// Supervisor class
class Supervisor {
    // Private variables
   private:
    uint64_t clock_;

    uint16_t next_event_id_;
    vector<unique_ptr<Event>> events_;
    uint16_t num_active_events_;
    uint64_t t_next_event_;
    Point2d pos_;    // supervisor pos
    Event* auction;  // the event currently being auctioned
    uint64_t t_next_gps_tick_;

    uint16_t num_events_handled_;  // total number of events handled
    double stat_total_distance_;   // total distance traveled
    double stat_robot_prev_pos_[NUM_ROBOTS][2];
    uint32_t robot_battery_used[NUM_ROBOTS];  // time spent moving //I think int and unsigned int are too small, a larger non-floating datatype can be used (but i cant remember them and i have no internet LMAO)

    WbNodeRef robots_[NUM_ROBOTS];
    WbDeviceTag emitter_;
    WbDeviceTag receivers_[NUM_ROBOTS];

    // track proximity times and current proximity state between robots
    double proximity_time_[NUM_ROBOTS][NUM_ROBOTS];
    bool proximity_state_[NUM_ROBOTS][NUM_ROBOTS];

    typedef vector<pair<Event*, message_event_state_t>> event_queue_t;

   private:
    // wall struct
    struct Wall {
        Point2d a;         // one endpoint
        Point2d b;         // other endpoint
        double thickness;  // full thickness (m)
    };
    Wall walls_[MAX_WALLS];
    int num_walls_ = 0;

    // track time spent near walls per robot
    double proximity_wall_time_[NUM_ROBOTS];  // seconds
    double proximity_any_time_ = 0.0;         // any time in simulation where there is a proximity either near wall or another robot

    // robot geometry
    const double robot_radius_ = 0.05;
    // Private functions
   private:
    void addEvent() {
        events_.push_back(unique_ptr<Event>(new Event{next_event_id_++}));  // add to list
        assert(num_active_events_ < NUM_ACTIVE_EVENTS);                     // check max. active events not reached
        num_active_events_++;
        t_next_event_ = clock_ + expovariate(EVENT_GENERATION_DELAY);
    }

    // Init robot and get robot_ids and receivers
    void linkRobot(uint16_t id) {
        const char kRobotNameFormat[] = "e-puck%d";
        const char kReceiverNameFormat[] = "rec%d";
        char node_name[16];

        // Get the robot node's handle
        sprintf(node_name, kRobotNameFormat, id);
        robots_[id] = wb_supervisor_node_get_from_def(node_name);
        if (!robots_[id]) {
            DBG(("Missing node for robot #%d\n", id));
            exit(1);
        }

        // Get the respective receiver
        sprintf(node_name, kReceiverNameFormat, id);
        receivers_[id] = wb_robot_get_device(node_name);
        if (!receivers_[id]) {
            DBG(("Missing receiver for robot #%d\n", id));
            exit(1);
        }
        wb_receiver_enable(receivers_[id], 2);  // 32
        wb_receiver_set_channel(receivers_[id], id + 1);
    }

    // Assemble a new message to be sent to robots (added type of event)
    void buildMessage(uint16_t robot_id, const Event* event, message_event_state_t event_state, message_t* msg) {
        WbFieldRef f_rot = wb_supervisor_node_get_field(robots_[robot_id], "rotation");
        const double* pos = getRobotPos(robot_id);
        const double* rot = wb_supervisor_field_get_sf_rotation(f_rot);

        msg->robot_id = robot_id;
        msg->robot_x = pos[0];              // no gps noise used here
        msg->robot_y = pos[1];              // no gps noise used here
        double heading = -rot[2] * rot[3];  // no gps noise used here
        msg->heading = heading > 2 * M_PI ? heading - 2 * M_PI : heading;
        msg->event_state = event_state;
        msg->event_id = -1;
        msg->event_x = 0.0;
        msg->event_y = 0.0;
        msg->task_type = TASK_TYPE_INVALID;

        if (event) {
            assert(event_state != MSG_EVENT_INVALID && event_state != MSG_EVENT_GPS_ONLY);
            msg->event_id = event->id_;
            msg->event_x = event->pos_.x;
            msg->event_y = event->pos_.y;
            msg->event_index = event->bidder_index;
            msg->task_type = event->task_type_;
        }
    }

    const double* getRobotPos(uint16_t robot_id) {
        WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id], "translation");
        return wb_supervisor_field_get_sf_vec3f(f_pos);
    }

    void setRobotPos(uint16_t robot_id, double x, double y) {
        WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id], "translation");
        double pos[3] = {x, y, 0.01};
        return wb_supervisor_field_set_sf_vec3f(f_pos, pos);
    }

    // Marks one event as done, if one of the robots is within the range
    void markEventsDone(event_queue_t& event_queue) {
        for (auto& event : events_) {
            if (!event->is_assigned() || event->is_done()) continue;

            const double* robot_pos = getRobotPos(event->assigned_to_);
            Point2d robot_pos_pt(robot_pos[0], robot_pos[1]);
            double dist = event->pos_.Distance(robot_pos_pt);

            if (dist <= EVENT_RANGE) {
                printf("D robot %d reached event %d\n", event->assigned_to_, event->id_);
                // calculate battery usage for waiting after reaching the task
                if (event->assigned_to_ == 0 || event->assigned_to_ == 1) {  // Robot A
                    if (event->task_type_ == TASK_TYPE_A) {
                        robot_battery_used[event->assigned_to_] += 3000;  // Robot A task A
                    } else {
                        robot_battery_used[event->assigned_to_] += 5000;  // Robot A task B
                    }
                } else {  // Robot B
                    if (event->task_type_ == TASK_TYPE_A) {
                        robot_battery_used[event->assigned_to_] += 9000;  // Robot B task A
                    } else {
                        robot_battery_used[event->assigned_to_] += 1000;  // Robot B task B
                    }
                }
                num_events_handled_++;
                event->markDone(clock_);
                num_active_events_--;
                event_queue.emplace_back(event.get(), MSG_EVENT_DONE);
            }
        }
    }

    void collisionDetection(uint64_t step_size) {
        // accumulate time (in seconds) robots spend within PROXIMITY_THRESHOLD
        double dt_s = (double)step_size / 1000.0;
        bool any_proximity_this_step = false;

        // check all robot pairs for proximity
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            const double* pos_i = getRobotPos(i);
            Point2d robot_i_pos(pos_i[0], pos_i[1]);
            for (int j = i + 1; j < NUM_ROBOTS; ++j) {
                const double* pos_j = getRobotPos(j);
                Point2d robot_j_pos(pos_j[0], pos_j[1]);
                double distance = robot_i_pos.Distance(robot_j_pos);
                bool within = distance < PROXIMITY_THRESHOLD;
                if (within) {
                    proximity_time_[i][j] += dt_s;
                    proximity_time_[j][i] += dt_s;
                    any_proximity_this_step = true;
                }
                proximity_state_[i][j] = within;
                proximity_state_[j][i] = within;
            }
        }

        // Check proximity to walls (no avoidance)
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            const double* pos_i = getRobotPos(i);
            Point2d robot_i_pos(pos_i[0], pos_i[1]);
            for (int w = 0; w < num_walls_; ++w) {
                double dist = pos_.DistanceToSegment(robot_i_pos, walls_[w].a, walls_[w].b);
                double threshold = (walls_[w].thickness / 2.0) + robot_radius_ + 0.0;  // optional safety margin
                if (dist < threshold) {
                    proximity_wall_time_[i] += dt_s;
                    any_proximity_this_step = true;
                }
            }
        }

        if (any_proximity_this_step) {
            proximity_any_time_ += dt_s;  // increment the final parameter
        }
    }
    void handleAuctionEvents(event_queue_t& event_queue) {
        // For each unassigned event
        for (auto& event : events_) {
            if (event->is_assigned()) continue;

            // Send announce, if new
            // IMPL DETAIL: Only allow one auction at a time.
            if (!event->was_announced() && !auction) {
                event->t_announced_ = clock_;
                event_queue.emplace_back(event.get(), MSG_EVENT_NEW);
                auction = event.get();
                printf("A event %d announced\n", event->id_);

                // End early or restart, if timed out
            } else if (clock_ - event->t_announced_ > EVENT_TIMEOUT) {
                // End early if we have any bids at all
                if (event->has_bids()) {
                    // IMPLEMENTATION DETAIL: If about to time out, assign to
                    // the highest bidder or restart the auction if there is none.
                    event->assigned_to_ = event->best_bidder_;
                    event_queue.emplace_back(event.get(), MSG_EVENT_WON);  // FIXME?
                    auction = NULL;
                    printf("W robot %d won event %d\n", event->assigned_to_, event->id_);

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
    void statTotalDistance(uint64_t step_size) {
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            const double* robot_pos = getRobotPos(i);
            double delta[2] = {robot_pos[0] - stat_robot_prev_pos_[i][0], robot_pos[1] - stat_robot_prev_pos_[i][1]};
            if ((delta[0] + delta[1]) > 0.0) {
                robot_battery_used[i] += step_size;
            }
            stat_total_distance_ += sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
            stat_robot_prev_pos_[i][0] = robot_pos[0];
            stat_robot_prev_pos_[i][1] = robot_pos[1];
        }
    }

    // Public fucntions
   public:
    Supervisor() : events_(MAX_EVENTS) {}

    // Reset robots & events
    void reset() {
        clock_ = 0;

        // initialize & link events
        next_event_id_ = 0;
        events_.clear();
        num_active_events_ = 0;
        t_next_event_ = 0;  // invalid state
        auction = NULL;
        t_next_gps_tick_ = 0;

        num_events_handled_ = 0;
        stat_total_distance_ = 0.0;

        // for wall proximity code
        num_walls_ = 0;
        walls_[num_walls_++] = Wall{Point2d(-0.6375, 0.0), Point2d(-0.2625, 0.0), 0.01};  // Lefthand wall
        walls_[num_walls_++] = Wall{Point2d(0.125, 0.65), Point2d(0.125, -0.2), 0.01};    // Top wall

        // init wall proximity times
        for (int i = 0; i < NUM_ROBOTS; ++i) proximity_wall_time_[i] = 0.0;

        // add the first few events
        for (int i = 0; i < NUM_ACTIVE_EVENTS; ++i) {
            addEvent();
        }

        // link & initialize robots
        for (int i = 0; i < NUM_ROBOTS; i++) {
            linkRobot(i);

            double pos[2] = {rand_coord(), rand_coord()};
            setRobotPos(i, pos[0], pos[1]);
            stat_robot_prev_pos_[i][0] = pos[0];
            stat_robot_prev_pos_[i][1] = pos[1];
            robot_battery_used[i] = 0;
        }

        // initialize the emitter
        emitter_ = wb_robot_get_device("sup_emitter");
        if (!emitter_) {
            DBG(("Missing supervisor emitter!\n"));
            exit(1);
        }
    }

    // Do a step
    bool step(uint64_t step_size) {
        clock_ += step_size;

        // Events that will be announced next or that have just been assigned/done
        event_queue_t event_queue;

        markEventsDone(event_queue);

        // ** Add a random new event, if the time has come
        assert(t_next_event_ > 0);
        if (clock_ >= t_next_event_ && num_active_events_ < NUM_ACTIVE_EVENTS) {
            addEvent();
        }

        handleAuctionEvents(event_queue);

        // Send and receive messages
        bid_t* pbid;  // inbound
        for (int i = 0; i < NUM_ROBOTS; i++) {
            // Check if we're receiving data
            if (wb_receiver_get_queue_length(receivers_[i]) > 0) {
                assert(wb_receiver_get_queue_length(receivers_[i]) > 0);
                assert(wb_receiver_get_data_size(receivers_[i]) == sizeof(bid_t));

                pbid = (bid_t*)wb_receiver_get_data(receivers_[i]);
                assert(pbid->robot_id == i);

                Event* event = events_.at(pbid->event_id).get();
                event->updateAuction(pbid->robot_id, pbid->value, pbid->event_index);
                // TODO: Refactor this (same code above in handleAuctionEvents)
                if (event->is_assigned()) {
                    event_queue.emplace_back(event, MSG_EVENT_WON);
                    auction = NULL;
                    printf("W robot %d won event %d\n", event->assigned_to_, event->id_);
                }

                wb_receiver_next_packet(receivers_[i]);
            }
        }

        // outbound
        message_t msg;
        bool is_gps_tick = false;

        if (clock_ >= t_next_gps_tick_) {
            is_gps_tick = true;
            t_next_gps_tick_ = clock_ + GPS_INTERVAL;
        }

        for (int i = 0; i < NUM_ROBOTS; i++) {
            // Send updates to the robot
            while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);

            if (is_gps_tick) {
                buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
                //        printf("sending message %d , %d \n",msg.event_id,msg.robot_id);
                while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);
                wb_emitter_send(emitter_, &msg, sizeof(message_t));
            }

            for (const auto& e_es_tuple : event_queue) {
                const Event* event = e_es_tuple.first;
                const message_event_state_t event_state = e_es_tuple.second;
                if (event->is_assigned() && event->assigned_to_ != i) continue;

                buildMessage(i, event, event_state, &msg);
                while (wb_emitter_get_channel(emitter_) != i + 1) wb_emitter_set_channel(emitter_, i + 1);
                //        printf("> Sent message to robot %d // event_state=%d\n", i, event_state);
                //        printf("sending message event %d , robot %d , emitter %d, channel
                //        %d\n",msg.event_id,msg.robot_id,emitter_,      wb_emitter_get_channel(emitter_));

                wb_emitter_send(emitter_, &msg, sizeof(message_t));
            }
        }

        // Keep track of our proximity/colission paramter
        collisionDetection(step_size);

        // Keep track of distance travelled by all robots
        statTotalDistance(step_size);

        // Time to end the experiment?
        if (num_events_handled_ >= TOTAL_EVENTS_TO_HANDLE || (MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
            for (int i = 0; i < NUM_ROBOTS; i++) {
                buildMessage(i, NULL, MSG_QUIT, &msg);
                wb_emitter_set_channel(emitter_, i + 1);
                wb_emitter_send(emitter_, &msg, sizeof(message_t));
            }
            double clock_s = ((double)clock_) / 1000.0;
            double ehr = ((double)num_events_handled_) / clock_s;
            double perf = ((double)num_events_handled_) / stat_total_distance_;

            printf("Handled %d events in %d seconds, events handled per second = %.2f\n", num_events_handled_,
                   (int)clock_ / 1000, ehr);

            // print proximity matrix (seconds)
            printf("*********PROXIMTY TO OTHER ROBOTS*********\n\n");
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                printf("Robot %d:", i);
                for (int j = 0; j < NUM_ROBOTS; ++j) {
                    printf(" %.2f", proximity_time_[i][j]);
                }
                printf("\n");
            }
            // print wall proximity times
            printf("*********PROXIMTY TO WALL*********\n");
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                printf("Robot %d was near a wall for %.2f seconds:", i, proximity_wall_time_[i]);
                printf("\n");
            }

            // final metric for any proximity
            printf("*********ANY PROXIMITY METRIC*********\n");
            printf("Total time any robot was near another robot or a wall: %.2f seconds\n", proximity_any_time_);

            // battery used
            printf("*********BATTERY USED METRIC*********\n");
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                printf("Battery usage for robot %d: %.2f, which corresponds to %d %% of total battery life\n", i, robot_battery_used[i] / 1000.0, (robot_battery_used[i]) / (MAX_BATTERY_LIFETIME) * 100);
            }

            printf("Performance: %f\n", perf);
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
    char node_name[16];

    for (int i = 0; i < MAX_EVENTS; ++i) {
        sprintf(node_name, kEventNameFormat, i);
        g_event_nodes[i] = wb_supervisor_node_get_from_def(node_name);
        g_event_nodes_free.push_back(g_event_nodes[i]);
    }
}

// MAIN LOOP (does steps)
// int main(void) {
//     Supervisor supervisor{};

//     // initialization
//     wb_robot_init();
//     link_event_nodes();
//     wb_robot_step(STEP_SIZE);

//     srand(time(NULL));
//     supervisor.reset();

//     // start the controller
//     printf("Starting main loop...\n");
//     while (wb_robot_step(STEP_SIZE) != -1) {
//         if (!supervisor.step(STEP_SIZE)) break;  // break at return = false
//     }
//     wb_supervisor_simulation_reset_physics();
//     wb_robot_cleanup();
//     exit(0);
//     return 0;
// }

// --- TIME TO TEST THE PATHFINDING STUFF ---

#include "../epuck_crown/pathfinding.hpp"

void visualize_graph(PathPlanner* planner) {
    // Note: Webots drawing API requires access to the supervisor node root.
    // The drawing functions in Webots are basic and limited.
    // For now, we'll skip direct visualization as it requires complex Webots API usage.
    // Instead, you can verify the pathfinding visually by:
    // 1. Plotting the waypoints in Rviz or similar
    // 2. Adding debug output to the PathPlanner
    // 3. Using the e-puck's motion to trace the path

    if (!planner) return;

    const auto& nodes = planner->getNodes();
    const auto& adj_matrix = planner->getAdjMatrix();

    printf("=== Visibility Graph Visualization ===\n");
    printf("Number of nodes: %zu\n", nodes.size());
    printf("Nodes:\n");
    for (size_t i = 0; i < nodes.size(); ++i) {
        printf("  Node %zu ('%c'): (%.3f, %.3f)\n", i, nodes[i].id, nodes[i].pos.x, nodes[i].pos.y);
    }

    printf("\nEdges (adjacency matrix):\n");
    int edge_count = 0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            if (adj_matrix[i][j] > 0) {
                printf("  Edge %d: Node %zu ('%c') <-> Node %zu ('%c'), distance: %.3f\n",
                       edge_count++, i, nodes[i].id, j, nodes[j].id, adj_matrix[i][j]);
            } else {
                printf("  No edge between Node %zu and Node %zu (value: %.3f)\n", i, j, adj_matrix[i][j]);
            }
        }
    }
    printf("Total edges: %d\n", edge_count);
}

void visualize_path(const std::vector<Point2d>& path) {
    if (path.size() < 2) {
        printf("Path is empty or contains only 1 point.\n");
        return;
    }

    printf("=== Path Visualization ===\n");
    printf("Path length: %zu waypoints\n", path.size());
    double total_distance = 0.0;

    for (size_t i = 0; i < path.size(); ++i) {
        printf("  Waypoint %zu: (%.3f, %.3f)\n", i, path[i].x, path[i].y);

        if (i > 0) {
            double segment_dist = path[i].Distance(path[i - 1]);
            total_distance += segment_dist;
            printf("    -> distance from previous: %.3f\n", segment_dist);
        }
    }
    printf("Total path distance: %.3f\n", total_distance);
}

int main(void) {
    Supervisor supervisor{};
    PathPlanner planner;  // Create an instance of your planner

    // initialization
    wb_robot_init();
    link_event_nodes();
    wb_robot_step(STEP_SIZE);

    srand(time(NULL));
    supervisor.reset();

    // Visualize the visibility graph
    printf("\n");
    visualize_graph(&planner);
    printf("\n");

    // Test pathfinding with a few test cases
    printf("=== Running Pathfinding Test Cases ===\n\n");

    // --- TEST CASE 1: Simple, unobstructed path ---
    printf("[Test Case 1] Unobstructed path\n");
    Point2d start1 = {0.48, 0.2};   // in top right quadrant
    Point2d goal1 = {0.16, -0.53};  // in bottom right quadrant under the vertical wall
    printf(">> start: (%.3f, %.3f), goal: (%.3f, %.3f)\n", start1.x, start1.y, goal1.x, goal1.y);
    std::vector<Point2d> path1 = planner.findPath(start1, goal1);
    visualize_path(path1);
    printf("\n");

    // --- TEST CASE 2: Path around one wall short ---
    printf("[Test Case 2] Path around one wall (short)\n");
    Point2d goal2 = {-0.3, -0.4};  // in bottom left quadrant under+left of the vertical wall
    printf(">> start: (%.3f, %.3f), goal: (%.3f, %.3f)\n", start1.x, start1.y, goal2.x, goal2.y);
    std::vector<Point2d> path2 = planner.findPath(start1, goal2);
    visualize_path(path2);
    printf("\n");

    // --- TEST CASE 3: Path around one wall longer ---
    printf("[Test Case 3] Path around one wall (longer)\n");
    Point2d goal3 = {-0.38, -0.15};  // in bottom left quadrant left of the vertical wall & under the horizontal wall
    printf(">> start: (%.3f, %.3f), goal: (%.3f, %.3f)\n", start1.x, start1.y, goal3.x, goal3.y);
    std::vector<Point2d> path3 = planner.findPath(start1, goal3);
    visualize_path(path3);
    printf("\n");

    // --- TEST CASE 4: Path around two walls ---
    printf("[Test Case 4] Path around two walls\n");
    Point2d goal4 = {-0.4, 0.16};  // in top left quadrant left of vertical wall & above horizontal wall
    printf(">> start: (%.3f, %.3f), goal: (%.3f, %.3f)\n", start1.x, start1.y, goal4.x, goal4.y);
    std::vector<Point2d> path4 = planner.findPath(start1, goal4);
    visualize_path(path4);
    printf("\n");

    // --- TEST CASE 5: Edge case - start and goal are the same ---
    printf("[Test Case 5] Start equals goal\n");
    Point2d goal5 = {start1.x, start1.y};  // same as start
    printf(">> start: (%.3f, %.3f), goal: (%.3f, %.3f)\n", start1.x, start1.y, goal5.x, goal5.y);
    std::vector<Point2d> path5 = planner.findPath(start1, goal5);
    visualize_path(path5);
    printf("\n");

    printf("=== Pathfinding Tests Complete ===\n");

    wb_robot_cleanup();
    exit(0);
    return 0;
}