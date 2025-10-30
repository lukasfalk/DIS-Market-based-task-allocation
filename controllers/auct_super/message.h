#include <stdint.h>

// A message sent from the supervisor to the robot
typedef enum {
  MSG_EVENT_INVALID = 0, 
  MSG_EVENT_GPS_ONLY,
  MSG_EVENT_NEW,
  MSG_EVENT_WON,
  MSG_EVENT_DONE,
  MSG_QUIT
} message_event_state_t;

typedef struct {
  uint16_t robot_id; // id of the intended receiver
  double robot_x;
  double robot_y;
  double heading; // robot heading
  message_event_state_t event_state;
  // [[ valid if event_state > 0: 
  uint16_t event_id;
  double event_x;
  double event_y;
  // ]]
  int event_index;
} message_t;

// A message sent from a robot to the supervisor
typedef struct {
  uint16_t robot_id; // id of the sender
  uint16_t event_id; // id of the event the bid is for
  double value; // value of the bid (estimated distance)
  int event_index;
} bid_t;
