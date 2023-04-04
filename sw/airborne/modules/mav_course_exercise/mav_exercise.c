/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "modules/core/abi.h"
#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"
#include "autopilot_static.h"
#include <stdio.h>
#include <time.h>
#include <math.h> // not sure if including this library would cause errors
// #include "modules/computer_vision/opticflow_module.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string, __FUNCTION__, ##__VA_ARGS__)

/**
 * Increments the heading of the drone
 * @param[in] incrementDegrees: the amount by which the heading is increased (in degrees)
 */
uint8_t increase_nav_heading(float incrementDegrees);

/**
 * Moves given waypoint forward by certain distance
 * @param[in] waypoint: the waypoint name/ID
 * @param[in] distanceMeters: distance to move forward (in m)
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);

/**
 * Moves given waypoint to a certain coordinate
 * @param[in] waypoint: the waypoint name/ID
 * @param[in] new_coor: address pointer to the struct that contains x,y,z coordinate
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

/**
 * Increases drone heading by random amount
 */
uint8_t chooseRandomIncrementAvoidance(void);

float RotateCenterArena(void);

/* *
 * Enum created for all the navigation states
 * the drone can enter.
 */
enum navigation_state_t
{
  SAFE,
  COLOR_FOUND,
  OBSTACLE_FOUND,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
  HOLD,
  SEARCH_FOR_SAFE_HEADING,
  COLOR_SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

/* *
 * Enum created for the colors drone detects
 */
enum color_t
{
  ORANGE,
  WHITE
};

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE; // Navigation State of the drone

float oa_color_count_frac = 0.20f;           // fraction of image for orange obstacle threshold
float wa_color_count_frac = 0.50f;           // fraction of image for orange obstacle threshold
int32_t color_count1 = 0;                    // orange color count from color filter for obstacle detection
int32_t color_count2 = 0;                    // white color count from color filter for obstacle detection
int16_t o_obs_free_confidence = 0;           // a measure of how certain we are that the way ahead is safe wrt orange
int16_t w_obs_free_confidence = 0;           // a measure of how certain we are that the way ahead is safe wrt white
int16_t color_free_confidence = 0;           // a measure of how certain we are that the way ahead is safe.
enum color_t color = ORANGE;                 // variable to store the color detected
int16_t orange_y = 0;                        // y pixel coordinate of centre of orange obstacle
int16_t white_y = 0;                         // y pixel coordinate of centre of white obstacle
float moveDistance = 0.7;                    // waypoint displacement [m]
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

float out_of_bounds_dheading = 40.0;   // heading increment when the drone goes out of bounds [deg]
float heading_increment = 30.0f;       // heading angle increment if out of bounds [deg]
float orange_heading_increment = 10.f; // heading increment when orange obstacle is detected [deg]
float white_heading_increment = 20.f;  // heading increment when white obstacle is detected [deg]
float color_heading_increment = 10.f;  // variable to store heading increment based on color detected [deg]
int counter = 0;                       // counter for deciding navigation state change
int counter_hold;                      // counter for hold state
int counter_threshold = 20;            // counter threshold for test mode (going back and forth)
int test = 0;                          // if the test mode is enabled
float turn_left = 0;                   // the confidence variable for turning left (turn variable)
float turn_right = 0;                  // the confidence variable for turning right (turn variable)
float rotate_90 = 0;                   // the confidence variable for turning 90 degrees (turn variable)
float stay_center = 0;                 // the confidence variable for staying in the center
float turn = 0;                        // stores the maximum  value out of any turn variable
int turn_decision = 10;                // confidence variable value at which the drone should take a turn
int turn_cap = 16;                     // maximum value a turn confidence variable can have4

float x_init = 0; // stores the x coordinate of the drone at initialization
float y_init = 0; // stores the y coordinate of the drone at initialization

float psuedo_div = 0;             // stores the optical flow of the central section of the image / psuedo divergence
float flow_diff;                  // stores the optical flow difference between left and right side of the image
float flow_left_mav;              // stores the optical flow of the left section of the image
float flow_center_mav;            // stores the optical flow of the central section of the image
float flow_right_mav;             // stores the optical flow of the right section of the image
float total_thresh = 150;         // threshold value for psuedo divergence where the drone decides to turn around
float diff_thresh = 100;          // theshold value for the optical flow difference between left and right sections
float flow_noise_threshold = 300; // threshold for flow noise
// needed to receive output from a separate module running on a parallel process
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x1, int16_t pixel_y1,
                               int16_t __attribute__((unused)) pixel_x2,
                               int16_t pixel_y2,
                               int32_t quality1, int32_t quality2)
{
  color_count1 = quality1; // get the pixel count for orange color
  color_count2 = quality2; // get the pixel count for white color

  orange_y = pixel_y1; // get the y value for orange obstacle
  white_y = pixel_y2;  // get the y value for white obstacle
}

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                            uint32_t __attribute__((unused)) stamp,
                            int32_t __attribute__((unused)) flow_x,
                            int32_t __attribute__((unused)) flow_y,
                            int32_t __attribute__((unused)) flow_der_x,
                            int32_t __attribute__((unused)) flow_der_y,
                            float __attribute__((unused)) quality,
                            float __attribute__((unused)) size_divergence,
                            float flow_left,
                            float flow_center,
                            float flow_right)
{
  // get optical flow value for each section of image
  flow_left_mav = flow_left;
  flow_center_mav = flow_center;
  flow_right_mav = flow_right;
}

void mav_exercise_init(void)
{
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);

  x_init = stateGetPositionEnu_i()->x;
  y_init = stateGetPositionEnu_i()->y;
}

/**
 * Update the turn confidence variables according to the optical flow values
 * @param[in] mid_flow: optic flow value in the middle of the picture
 * @param[in] flow_difference: difference in magnitude of flow valules between left and right quadrants of the image
 * @param[in] full_flow: total flow (absolute) value
 * @param[in] left_is_smallest:  boolean to indicate if the left quadrant has the smallest flow value
 * @param[in] right_is_smallest:  boolean to indicate if the right quadrant has the smallest flow value
 */
void momentum_calc(float mid_flow, float flow_difference, float full_flow, bool left_is_smallest, bool right_is_smallest)
{
  // if the flow in the center is above a threshold and the difference between flow of
  // left and right side of the image is below a certain threshold means there might be something right in front of
  // the drone with not much scope to turn to any side. Then,
  // the drone should turn around --> increase confidence of turn around
  if ((mid_flow > total_thresh) && (flow_difference < diff_thresh))
  {
    rotate_90 += 2;
    turn_right -= 1;
    turn_left -= 1;
    stay_center -= 1;
    // PRINT("DETECTED Turn around\n\n");
  }
  // else if the flow in the left is smallest but still above some threshold along with the flow difference
  // being substantial, the drone should turn left --> increase confidence in turning left
  else if (left_is_smallest && (flow_difference > diff_thresh) && fabs(flow_left_mav) > 10)
  {
    turn_left += 2;
    turn_right -= 1;
    stay_center -= 1;
    rotate_90 -= 1;
    // PRINT("DETECTED Left\n\n");
  }
  // else if the flow in the right is smallest but still above some threshold along with the flow difference
  // being substantial, the drone should turn right  --> increase confidence in turning right
  else if (right_is_smallest && (flow_difference > diff_thresh) && fabs(flow_right_mav) > 10)
  {
    turn_right += 2;
    turn_left -= 1;
    stay_center -= 1;
    rotate_90 -= 1;
    // PRINT("DETECTED Right\n\n");
  }
  // in this case, the flow in center is smallest and there is no obstacle ahead
  // so the drone can go ahead in the same direction without turning
  // increase confidence in staying at the center
  else
  {
    stay_center += 2;
    rotate_90 -= 1;
    turn_right -= 1;
    turn_left -= 1;
    // PRINT("DETECTED Center\n\n");
  }

  // constrain the values of the turn confidence variables
  Bound(turn_left, 0, turn_cap);
  Bound(turn_right, 0, turn_cap);
  Bound(stay_center, 0, turn_cap);
  Bound(rotate_90, 0, turn_cap);
}

/*
 * Calculates/Updates the color free confidence variables for way ahead
 */
void calc_color_free_conf(void)
{
  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  int32_t orange_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t white_count_threshold = wa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  // update our safe confidence using color threshold for orange
  if (color_count1 < orange_count_threshold)
  {
    o_obs_free_confidence++;
  }
  else
  {
    o_obs_free_confidence -= 2;
    // be more cautious with positive obstacle detections
  }

  // bound o_obs_free_confidence
  Bound(o_obs_free_confidence, 0, max_trajectory_confidence);

  // update our safe confidence using color threshold for white
  if (color_count2 < white_count_threshold)
  {
    w_obs_free_confidence++;
  }
  else
  {
    w_obs_free_confidence -= 2;
    // be more cautious with positive obstacle detections
  }

  // bound w_obs_free_confidence
  Bound(w_obs_free_confidence, 0, max_trajectory_confidence);

  // decide on the color free confidence based on the color detected
  if (o_obs_free_confidence < w_obs_free_confidence)
  {
    color_free_confidence = o_obs_free_confidence;
    color = ORANGE;
  }
  else
  {
    color_free_confidence = w_obs_free_confidence;
    color = WHITE;
  }

  // PRINT("Orange -- Color_count: %d  threshold: %d \n\n", color_count1, orange_count_threshold);
  // PRINT("White  -- Color_count: %d  threshold: %d \n\n", color_count2, white_count_threshold);
}

/*
 * Fuction that is called periodically at 10 Hz for operation of the drone
 * Contains confidence calculactions and the state machine.
 */
void mav_exercise_periodic(void)
{
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight())
  {
    return;
  }

  // calculate the value of flow variables
  float psuedo_div = fabs(flow_center_mav);
  float flow_difference = fabs(fabs(flow_left_mav) - fabs(flow_right_mav));
  float full_flow = (fabs(flow_center_mav) + fabs(flow_left_mav) + fabs(flow_right_mav));

  bool right_is_smallest = (fabs(flow_right_mav) < fabs(flow_center_mav)) && (fabs(flow_right_mav) < fabs(flow_left_mav));
  bool left_is_smallest = (fabs(flow_left_mav) < fabs(flow_center_mav)) && (fabs(flow_left_mav) < fabs(flow_right_mav));

  // PRINT("Flow left =  %f, center =  %f, right =  %f\n", flow_left_mav, flow_center_mav, flow_right_mav);
  // PRINT("Total Flow =  %f, Flow diff =  %f, low_total =  %f\n", psuedo_div, flow_difference, full_flow);

  // Updating the color free confidence variables
  calc_color_free_conf();

  // STATE MACHINE STARTS
  switch (navigation_state)
  {
  case SAFE:
    // test mode for developing algorithms
    if (test)
    {
      PRINT("orange y: %d\n\n", orange_y);
      counter++;
      if (counter > counter_threshold)
      {
        // PRINT("\n\n\n\n\n\n\n\n")
        // PRINT("         ***period change***\n");
        moveDistance = -moveDistance;
        counter = 0;
        turn_left -= 100;
        turn_right -= 100;
        stay_center -= 100;
        rotate_90 -= 100;
      }
      if (counter < 4)
      {
        turn_left -= 100;
        turn_right -= 100;
        stay_center -= 100;
        rotate_90 -= 100;
      }

      moveWaypointForward(WP_GOAL, 2.0f * moveDistance);

      momentum_calc(psuedo_div, flow_difference, full_flow, left_is_smallest, right_is_smallest);

      Bound(turn_left, 0, turn_cap);
      Bound(turn_right, 0, turn_cap);
      Bound(stay_center, 0, turn_cap);
      Bound(rotate_90, 0, turn_cap);

      // choose the maximum between the turns

      turn = fmaxf(turn_left, turn_right);
      turn = fmaxf(turn, rotate_90);

      // PRINT("rotate90, turn_right, stay_center, turn_left: %f, %f, %f, %f \n", rotate_90, turn_right, stay_center, turn_left);
      if (turn_left > turn_right)
      {
        // PRINT("turning left\n");
      }
      else if (turn_right > turn_left)
      {
        // PRINT("turning right\n");
      }
      else if (rotate_90 >= turn)
      {
        // PRINT("rotating 90\n");
      }
      else
      {
        // PRINT("staying center\n");
      }

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY)))
      {
        // PRINT("Out of Bounds\n\n");
      }
      else if (turn >= stay_center && turn >= turn_decision)
      {
        // PRINT("Obstacle Found\n\n");
      }
      else
      {
        //("SAFE\n\n");
      }

      // PRINT("       ------------------------------------------------\n\n\n");
      break;
    }
    // ------------------------------------------------------
    // ------------------------------------------------------
    // TEST ENDS HERE
    // ------------------------------------------------------
    // ------------------------------------------------------
    else
    {
      PRINT("STATE: SAFE\n\n\n");
      // Calculate turn confidence variables based on optical flow
      momentum_calc(psuedo_div, flow_difference, full_flow, left_is_smallest, right_is_smallest);

      // get the max value of turn variable
      turn = fmaxf(turn_left, turn_right);
      turn = fmaxf(turn, rotate_90);

      // move TRAJECTORY waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 0.8f * moveDistance);

      // if color_free_confidence is zero -- colored obstacle ahead
      if (color_free_confidence == 0)
      {
        // switch to COLOR_FOUND state
        navigation_state = COLOR_FOUND;
      }
      // if turn variable exceeds the turn decison and stay center variable
      // there's an obstacle ahead and the drone needs to turn
      else if (turn >= stay_center && turn >= turn_decision)
      {
        // switch to OBSTACLE_FOUND state
        navigation_state = OBSTACLE_FOUND;
      }
      // if the TRAJECTORY waypoint is out of bounds
      else if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY)))
      {
        // switch to OUT_OF_BOUNDS state
        navigation_state = OUT_OF_BOUNDS;
      }
      // Otherwise it is safe to fly ahead
      else
      {
        // move GOAL waypoint forward to move the drone forward
        moveWaypointForward(WP_GOAL, moveDistance);
        // TODO: Keeps the navigation at SAFE ------------ REDUNDANT
        navigation_state = SAFE;
      }
    }
    break;

  case COLOR_FOUND:
    PRINT("STATE: COLOR_FOUND-- %d (0=orange)\n\n", color);
    // stop the drone
    waypoint_move_here_2d(WP_GOAL);
    waypoint_move_here_2d(WP_TRAJECTORY);

    // randomly select new search direction
    chooseRandomIncrementAvoidance();

    navigation_state = COLOR_SEARCH_FOR_SAFE_HEADING;
    break;

  case OBSTACLE_FOUND:
    PRINT("STATE: OBSTACLE_FOUND\n\n");

    // STOP the drone
    moveWaypointForward(WP_GOAL, 0.0f);
    moveWaypointForward(WP_TRAJECTORY, 0.0f);

    // if turn confidence value equals turn_left
    if (turn == turn_left)
    {
      // switch to TURN_LEFT state
      navigation_state = TURN_LEFT;
    }
    // if turn confidence value equals turn_right
    else if (turn == turn_right)
    {
      // switch to TURN_RIGHT state
      navigation_state = TURN_RIGHT;
    }
    // else the drone should turn around
    else
    {
      // switch to TURN_AROUND state
      navigation_state = TURN_AROUND;
    }

    // Reset turn confidence variables to restart the cycle
    turn_left = 0;
    turn_right = 0;
    stay_center = 0;
    rotate_90 = 0;
    turn = 0;

    // Reset counter to run a state for a certain number
    counter = 0;
    break;

  case TURN_LEFT:
    PRINT("STATE: TURN_LEFT\n\n");
    // decrease heading every time the state is
    // called for turning towards the left
    increase_nav_heading(-1 * heading_increment);

    // if the drone is in this state for certain count
    // switch to HOLD state
    if (counter >= 8)
    {
      navigation_state = HOLD;
      // reset the counter for HOLD state hit
      counter_hold = 0;
    }
    // Increment counter for hits of this state
    counter++;
    break;

  case TURN_RIGHT:
    PRINT("STATE: TURN_RIGHT\n\n");
    // increase heading every time the state is
    // called for turning towards the right
    increase_nav_heading(1 * heading_increment);

    // if the drone is in this state for certain count
    // switch to HOLD state
    if (counter >= 8)
    {
      navigation_state = HOLD;
      // reset the counter for HOLD state hit
      counter_hold = 0;
    }
    // Increment counter for hits of this state
    counter++;
    break;

  case TURN_AROUND:
    PRINT("STATE: TURN_AROUND\n\n");

    // for the first few times
    if (counter < 4)
    {
      // move back the GOAL, and thus the drone
      moveWaypointForward(WP_GOAL, -1.0f);
      // PRINT("GOAL MOVED BACK\n\n");
    }
    // then
    else if (counter == 4)
    {
      // stop the drione and increase the heading by 90 degrees
      moveWaypointForward(WP_GOAL, 0.0f);
      increase_nav_heading(90);
      // PRINT("GOAL STILL+TURN\n\n");
    }
    // after certain hits of the state
    // switch to HOLD state
    else
    {
      navigation_state = HOLD;
      // reset the counter for HOLD state hit
      counter_hold = 0;
    }
    // Increment counter for hits of this state
    counter++;
    break;

  case HOLD:
    PRINT("STATE: HOLD\n\n\n");
    // stay in this state for certain count
    // then switch to SAFE state for moving
    if (counter_hold >= 3)
    {
      navigation_state = SAFE;
    }
    counter_hold++;
    break;

  case SEARCH_FOR_SAFE_HEADING:
    PRINT("STATE: SEARCH_FOR_SAFE_HEADING\n\n");
    // STOP the drone
    moveWaypointForward(WP_GOAL, 0.0f);

    // wait for cerain hits
    // then switch to SAFE state
    if (counter >= 1)
    {
      navigation_state = SAFE;
    }
    counter++;
    break;
  case COLOR_SEARCH_FOR_SAFE_HEADING:
    PRINT("STATE: COLOR_SEARCH_FOR_SAFE_HEADING\n\n");

    // STOP the drone
    moveWaypointForward(WP_GOAL, 0.0f);

    // initialize variable for storing colored obstacle's central pixel's y value
    int16_t color_y;
    // if the detected color is orange
    if (color == ORANGE)
    {
      // store the value for orange pixel's y value
      color_y = orange_y;
      // select heading increment for orange obstacles
      color_heading_increment = orange_heading_increment;
    }
    // or the detected color is white
    else
    {
      // store the value for orange pixel's y value
      color_y = white_y;
      // select heading increment for white obstacles
      color_heading_increment = white_heading_increment;
    }

    // if the pixel's y value is positive -- the obstacle is towards the left
    if (color_y > 0)
    {
      // increase heading to turn right
      increase_nav_heading(color_heading_increment);
    }
    else // if the pixel's y value is negative or zero -- the obstacle is towards the right
    {
      // decrease heading to turn right
      increase_nav_heading(-color_heading_increment);
    }
    // make sure we have a couple of good orange color readings before declaring the way safe
    if (color_free_confidence >= 2)
    {
      navigation_state = HOLD;
    }
    break;

  case OUT_OF_BOUNDS:
    PRINT("STATE: OUT_OF_BOUNDS\n\n");
    // Increase the heading to rotate
    increase_nav_heading(out_of_bounds_dheading);
    // STOP the drone
    moveWaypointForward(WP_GOAL, 0.0f);
    // Move the TRAJECTORY waypoint forward to check for out of bounds
    moveWaypointForward(WP_TRAJECTORY, 0.8f);

    // If the TRAJECTORY waypoint has moved inside the obstacle zone
    if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY)))
    {
      // add offset to head back into arena
      // reset turn confidence variables
      turn_left = 0;
      turn_right = 0;
      stay_center = 0;
      rotate_90 = 0;
      turn = 0;
      // reset the counter
      counter = 0;
      // ensure direction is safe before continuing
      navigation_state = SEARCH_FOR_SAFE_HEADING;
    }
    break;

  default:
    break;
  }

  // end of state machine and periodic function
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0)
  {
    heading_increment = 5.f;
  }
  else
  {
    heading_increment = -5.f;
  }
  return false;
}