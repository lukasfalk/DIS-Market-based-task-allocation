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
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cmath>

#include <vector>
#include <memory>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

using namespace std;

#include "Point2d.h"
#include "message.h"

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#define DBG(x) printf x
#define RAND ((float) rand()/RAND_MAX)

#define MAX_ROBOTS 9
#define MAX_EVENTS 10

#define STEP_SIZE 64            // simulation step size
#define AUCTION_TIMEOUT 1000    // number of steps after which an auction stops

#define EVENT_RANGE (0.1)      // distance within which a robot must come to do event
#define EVENT_TIMEOUT (10000)   // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000) // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)

// Parameters that can be changed
#define NUM_ROBOTS 5                 // Change this also in the epuck_crown.c!
#define NUM_ACTIVE_EVENTS 10          // number of active events
#define TOTAL_EVENTS_TO_HANDLE  999   // Events after which simulation stops or...
#define MAX_RUNTIME (3*60*1000)      // 3 minutes runtime
//

WbNodeRef g_event_nodes[MAX_EVENTS];
vector<WbNodeRef> g_event_nodes_free;

double gauss(void) 
{
  double x1, x2, w;
  do {
      x1 = 2.0 * RAND - 1.0;
      x2 = 2.0 * RAND - 1.0;
      w = x1*x1 + x2*x2;
  } while (w >= 1.0);

  w = sqrt((-2.0 * log(w))/w);
  return(x1*w);
}

double rand_coord() {
  // return -1.0 + 2.0*RAND;
  return -0.55 + 1.1*RAND;
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
  uint16_t id_;          //event id
  Point2d pos_;          //event pos
  WbNodeRef node_;       //event node ref
  uint16_t assigned_to_; //id of the robot that will handle this event

  // Auction data
  uint64_t t_announced_;        //time at which event was announced to robots
  bitset<NUM_ROBOTS> bids_in_;
  uint16_t best_bidder_;        //id of the robot that had the best bid so far
  double best_bid_;             //value of the best bid (lower is better)
  uint64_t t_done_;             //time at which the assigned robot reached the event
  int bidder_index;             //index at which the bidder will put event in tasklist

// Public functions
public:
  //Event creation
  Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()),
    assigned_to_(-1), t_announced_(-1), best_bidder_(-1), best_bid_(0.0), t_done_(-1)
  {
    node_ = g_event_nodes_free.back();  // Place node
    g_event_nodes_free.pop_back();
    
    double event_node_pos[3];           // Place event in arena
    event_node_pos[0] = pos_.x;
    event_node_pos[1] = pos_.y;
    event_node_pos[2] = .01;
    wb_supervisor_field_set_sf_vec3f(
      wb_supervisor_node_get_field(node_,"translation"),
      event_node_pos);
  }

  bool is_assigned() const { return assigned_to_ != (uint16_t) -1; }
  bool was_announced() const { return t_announced_ != (uint64_t) -1; }
  bool has_bids() const { return best_bidder_ != (uint16_t) -1; }
  bool is_done() const { return t_done_ != (uint64_t) -1; }

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
    double event_node_pos[3] = {-5,-5,0.1};
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),
                                     event_node_pos);
    g_event_nodes_free.push_back(node_);
  }
};

// Supervisor class
class Supervisor {

//Private variables
private:
  uint64_t clock_;

  uint16_t next_event_id_;
  vector<unique_ptr<Event> > events_;
  uint16_t num_active_events_;
  uint64_t t_next_event_;
  Event* auction; // the event currently being auctioned
  uint64_t t_next_gps_tick_;

  uint16_t num_events_handled_; // total number of events handled
  double stat_total_distance_;  // total distance traveled
  double stat_robot_prev_pos_[NUM_ROBOTS][2];

  WbNodeRef robots_[NUM_ROBOTS];
  WbDeviceTag emitter_;
  WbDeviceTag receivers_[NUM_ROBOTS];

  typedef vector<pair<Event*, message_event_state_t>> event_queue_t;

// Private functions
private:
  void addEvent() {
    events_.push_back(unique_ptr<Event>(new Event{next_event_id_++})); // add to list
    assert(num_active_events_ < NUM_ACTIVE_EVENTS); // check max. active events not reached
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
    wb_receiver_enable(receivers_[id], 2); //32
    wb_receiver_set_channel(receivers_[id], id+1);
  }

  // Assemble a new message to be sent to robots
  void buildMessage(uint16_t robot_id, const Event* event,
      message_event_state_t event_state, message_t* msg) {
    WbFieldRef f_rot = wb_supervisor_node_get_field(robots_[robot_id],
                                                    "rotation");
    const double *pos = getRobotPos(robot_id);
    const double *rot = wb_supervisor_field_get_sf_rotation(f_rot);

    msg->robot_id = robot_id;
    msg->robot_x = pos[0]; // no gps noise used here
    msg->robot_y = pos[1]; // no gps noise used here
    double heading = -rot[2] *rot[3]; // no gps noise used here
    msg->heading = heading > 2*M_PI ? heading - 2*M_PI : heading;
    msg->event_state = event_state;
    msg->event_id = -1;
    msg->event_x = 0.0;
    msg->event_y = 0.0;

    if (event) {
      assert(event_state != MSG_EVENT_INVALID && 
             event_state != MSG_EVENT_GPS_ONLY);
      msg->event_id = event->id_;
      msg->event_x = event->pos_.x;
      msg->event_y = event->pos_.y;
      msg->event_index = event->bidder_index;
    }
  }

  const double* getRobotPos(uint16_t robot_id) {
    WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id],
      "translation");
    return wb_supervisor_field_get_sf_vec3f(f_pos);
  }

  void setRobotPos(uint16_t robot_id, double x, double y) {
    WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id],
      "translation");
    double pos[3] = {x, y, 0.001};
    return wb_supervisor_field_set_sf_vec3f(f_pos, pos);
  }

  // Marks one event as done, if one of the robots is within the range
  void markEventsDone(event_queue_t& event_queue) {
    for (auto& event : events_) {
      if (!event->is_assigned() || event->is_done())
        continue;
      
      const double *robot_pos = getRobotPos(event->assigned_to_);
      Point2d robot_pos_pt(robot_pos[0], robot_pos[1]);
      double dist = event->pos_.Distance(robot_pos_pt);

      if (dist <= EVENT_RANGE) {
        printf("D robot %d reached event %d\n", event->assigned_to_,
          event->id_);
        num_events_handled_++;
        event->markDone(clock_);
        num_active_events_--;
        event_queue.emplace_back(event.get(), MSG_EVENT_DONE);
      }
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
          event_queue.emplace_back(event.get(), MSG_EVENT_WON); // FIXME?
          auction = NULL;
          printf("W robot %d won event %d\n", event->assigned_to_, event->id_);

        // Restart (incl. announce) if no bids
        } else {
          // (reannounced in next iteration)
          event->restartAuction();
          if (auction == event.get())
            auction = NULL;
        }
      }
    }
  }

  // Calculate total distance travelled by robots
  void statTotalDistance() {
    for (int i=0; i<NUM_ROBOTS; ++i) {
      const double *robot_pos = getRobotPos(i);
      double delta[2] = {
        robot_pos[0] - stat_robot_prev_pos_[i][0],
        robot_pos[1] - stat_robot_prev_pos_[i][1]
      };
      stat_total_distance_ += sqrt(delta[0]*delta[0] + delta[1]*delta[1]);
      stat_robot_prev_pos_[i][0] = robot_pos[0];
      stat_robot_prev_pos_[i][1] = robot_pos[1];
    }
  }

// Public fucntions
public:
  Supervisor() : events_(MAX_EVENTS){}
  
  // Reset robots & events
  void reset() {
    clock_ = 0;

    // initialize & link events
    next_event_id_ = 0;
    events_.clear();
    num_active_events_ = 0;
    t_next_event_ = 0; // invalid state
    auction = NULL;
    t_next_gps_tick_ = 0;

    num_events_handled_ = 0;
    stat_total_distance_ = 0.0;

    // add the first few events
    for (int i=0; i<NUM_ACTIVE_EVENTS; ++i) {
      addEvent();
    }

    // link & initialize robots
    for (int i=0;i<NUM_ROBOTS;i++) {
      linkRobot(i);

      //double pos[2] = {rand_coord(), rand_coord()};
      const double* robot_pos = getRobotPos(i);
      DBG(("Robot %d pos: [%f,%f]\n", i, robot_pos[0], robot_pos[1]));
      //setRobotPos(i, robot_pos[0], robot_pos[1]);
      stat_robot_prev_pos_[i][0] = robot_pos[0];
      stat_robot_prev_pos_[i][1] = robot_pos[1];
    }

    // initialize the emitter
    //emitter_ = wb_robot_get_device("sup_emitter"); //THIS IS ORIGINAL NAME FROM LAB05
    emitter_ = wb_robot_get_device("emitter"); //Our version of emitter name
    if (!emitter_) {
      DBG(("Missing supervisor emitter!\n"));
      exit(1);
    }
  }

  //Do a step
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
    bid_t* pbid; // inbound
    for (int i=0;i<NUM_ROBOTS;i++) {
      // Check if we're receiving data
      if (wb_receiver_get_queue_length(receivers_[i]) > 0) {
        printf("Supervisor detected message in receiver for robot %d\n", i);
        assert(wb_receiver_get_queue_length(receivers_[i]) > 0);
        assert(wb_receiver_get_data_size(receivers_[i]) == sizeof(bid_t));
        pbid = (bid_t*) wb_receiver_get_data(receivers_[i]); 
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

    for (int i=0;i<NUM_ROBOTS;i++) {
      // Send updates to the robot
      while (wb_emitter_get_channel(emitter_) != i+1)
      wb_emitter_set_channel(emitter_, i+1);
      
      if (is_gps_tick) {
        buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
//        printf("sending message %d , %d \n",msg.event_id,msg.robot_id);
        while (wb_emitter_get_channel(emitter_) != i+1)
            wb_emitter_set_channel(emitter_, i+1);        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }

      for (const auto& e_es_tuple : event_queue) {
        const Event* event = e_es_tuple.first;
        const message_event_state_t event_state = e_es_tuple.second;
        if (event->is_assigned() && event->assigned_to_ != i) continue;

        buildMessage(i, event, event_state, &msg);
        while (wb_emitter_get_channel(emitter_) != i+1)
              wb_emitter_set_channel(emitter_, i+1);        
//        printf("> Sent message to robot %d // event_state=%d\n", i, event_state);
//        printf("sending message event %d , robot %d , emitter %d, channel %d\n",msg.event_id,msg.robot_id,emitter_,      wb_emitter_get_channel(emitter_));
        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }
    }

    // Keep track of distance travelled by all robots
    statTotalDistance();

    // Time to end the experiment?
    if (num_events_handled_ >= TOTAL_EVENTS_TO_HANDLE ||(MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
      for(int i=0;i<NUM_ROBOTS;i++){
          buildMessage(i, NULL, MSG_QUIT, &msg);
          wb_emitter_set_channel(emitter_, i+1);
          wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }
      double clock_s = ((double) clock_) / 1000.0;
      double ehr = ((double) num_events_handled_) / clock_s;
      double perf = ((double) num_events_handled_) / stat_total_distance_;
      
      printf("Handled %d events in %d seconds, events handled per second = %.2f\n",
             num_events_handled_, (int) clock_ / 1000, ehr);
      printf("Performance: %f\n", perf);
      return false;
    } 
    else { return true;} //continue
  } // << step() <<
};

//Links up all the nodes we are interested in.
//Gets called by webots at robot_live(reset)
void link_event_nodes() {
  const char kEventNameFormat[] = "e%d";
  char node_name[16];
  
  for (int i=0; i<MAX_EVENTS; ++i) {
    sprintf(node_name, kEventNameFormat, i);
    g_event_nodes[i] = wb_supervisor_node_get_from_def(node_name);
    g_event_nodes_free.push_back(g_event_nodes[i]);
  }
}

// MAIN LOOP (does steps)
int main(void) 
{
  Supervisor supervisor{};

  // initialization
  wb_robot_init();
  link_event_nodes();
  wb_robot_step(STEP_SIZE);

  srand(time(NULL));
  supervisor.reset();

  // start the controller
  printf("Starting main loop...\n");
  while (wb_robot_step(STEP_SIZE) != -1)
  {
    if (!supervisor.step(STEP_SIZE)) break; //break at return = false
  }
  wb_supervisor_simulation_reset_physics();
  wb_robot_cleanup();
  exit(0);
  return 0;

}