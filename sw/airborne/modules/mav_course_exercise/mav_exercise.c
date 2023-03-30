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
//#include "modules/computer_vision/opticflow_module.h"
     
#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t chooseRandomIncrementAvoidance(void);
float RotateCenterArena(void);

enum navigation_state_t {
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

enum color_t {
  ORANGE,
  WHITE
};
int varr = 2;

// define and initialise global variables
float oa_color_count_frac = 0.20f;
float wa_color_count_frac = 0.50f;
enum navigation_state_t navigation_state = SAFE;
int32_t color_count1 = 0;               // orange color count from color filter for obstacle detection
int32_t color_count2 = 0;               // white color count from color filter for obstacle detection
int16_t o_obs_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
int16_t w_obs_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
int16_t color_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
enum color_t color = ORANGE;
int16_t orange_y = 0;
int16_t white_y = 0;
float moveDistance = 1.0;               // waypoint displacement [m]
float oob_haeding_increment = 5.f;      // heading angle increment if out of bounds [deg]
float obstacle_heading_increment = 15.f;
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
float heading_increment = 30.0f;
float orange_heading_increment = 10.f;
float white_heading_increment = 20.f;
float color_heading_increment = 10.f;
float turn_around_increment = 20.f;
// float divergence_threshold = 0.3f;
float size_div = 0;
int counter = 0;
int counter_hold;
int counter_threshold = 20;
float total_flow = 0;
float flow_diff;
  
int test = 0;
int rotate = 0;
float thresh_1 = 50.0;
float thresh_2 = 50.0;
float thresh_3 = 50.0;

float turn_left = 0;
float turn_right = 0;
float stay_center = 0;
float flow_left_mav;
float flow_center_mav;
float flow_right_mav;
float flow_noise_threshold = 300;
float min_move_dist = 0.5;
int turn_decision = 14;
int turn_cap = 20;
float out_of_bounds_dheading = 40.0;

float rotate_90 = 0;
float turn = 0;
float x_init = 0;
float y_init = 0;
int nav_heading_int;

// // expoenentially weighted moving average params
float total_thresh = 150;
float diff_thresh = 100;


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
                               int32_t quality1, int32_t quality2) {
  color_count1 = quality1;
  color_count2 = quality2;

  orange_y = pixel_y1;
  white_y = pixel_y2;
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
  flow_left_mav = flow_left;
  flow_center_mav = flow_center;
  flow_right_mav = flow_right;
}


void mav_exercise_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);

  x_init = stateGetPositionEnu_i()->x;
  y_init = stateGetPositionEnu_i()->y;
  //PRINT("out_of_bounds_dheading: %f turn_cap: %d turn_decision: %d flow_noise_threshold: %f \n", out_of_bounds_dheading, turn_cap, turn_decision, flow_noise_threshold);
}


/**
 * Run the optical flow with fast9 and lukaskanade on a new image frame
 * @param[in] mid_flow: optic flow value in the middle of the picture
 * @param[in] flow_difference: difference in magnitude of flow valules between left and right quadrants of the image
 * @param[in] full_flow: total flow (absolute) value
 * @param[in] left_is_smallest:  boolean to indicate if the left quadrant has the smallest flow value
 * @param[in] right_is_smallest:  boolean to indicate if the right quadrant has the smallest flow value
 */
void momentum_calc(float mid_flow, float flow_difference, float full_flow, bool left_is_smallest, bool right_is_smallest) {

  if (((mid_flow>total_thresh) && (flow_difference<diff_thresh)) )//|| (full_flow < 50))//|| (mid_flow < 5))
  {
    rotate_90 +=2;
    turn_right -= 1;
    turn_left -=1;
    stay_center -=1;
    //PRINT("DETECTED Turn around\n\n");
  }
  else if (left_is_smallest && (flow_difference>diff_thresh) && fabs(flow_left_mav) > 10)
  {
    turn_left += 2;
    turn_right -=1;
    stay_center -=1;
    rotate_90 -=1;
    //PRINT("DETECTED Left\n\n");
  }
  else if (right_is_smallest && (flow_difference>diff_thresh) && fabs(flow_right_mav) > 10)
  {
    turn_right += 2;
    turn_left -=1;
    stay_center -=1;
    rotate_90 -=1;
    //PRINT("DETECTED Right\n\n");
  }
  else {
    stay_center +=2;
    rotate_90 -=1;
    turn_right -= 1;
    turn_left -=1;
    ///PRINT("DETECTED Center\n\n");
  } 
  

  Bound(turn_left, 0, turn_cap);
  Bound(turn_right, 0, turn_cap);
  Bound(stay_center, 0, turn_cap);
  Bound(rotate_90, 0, turn_cap);   
  
}

void calc_color_free_conf(void)
{
  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  int32_t orange_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t white_count_threshold = wa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  // update our safe confidence using color threshold for orange
  if(color_count1 < orange_count_threshold){
    o_obs_free_confidence++;
  } else {
    o_obs_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }
    
  // bound o_obs_free_confidence
  Bound(o_obs_free_confidence, 0, max_trajectory_confidence);
  
  // update our safe confidence using color threshold for white
  if(color_count2 < white_count_threshold){
    w_obs_free_confidence++;
  } else {
    w_obs_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }
    
  // bound o_obs_free_confidence
  Bound(w_obs_free_confidence, 0, max_trajectory_confidence);
  if(o_obs_free_confidence < w_obs_free_confidence){
      color_free_confidence = o_obs_free_confidence;
      color = ORANGE;
  }
  else{
    color_free_confidence = w_obs_free_confidence;
    color = WHITE;
  }
  PRINT("Orange -- Color_count: %d  threshold: %d \n\n", color_count1, orange_count_threshold);
  PRINT("White  -- Color_count: %d  threshold: %d \n\n", color_count2, white_count_threshold);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    return;
  }
  
  //PRINT("Color_count: %d  threshold: %d state: %d \n\n", color_count, color_count_threshold, navigation_state);

  float total_flow = fabs(flow_center_mav);
  float flow_difference = fabs(fabs(flow_left_mav) - fabs(flow_right_mav));
  float full_flow = (fabs(flow_center_mav) + fabs(flow_left_mav) + fabs(flow_right_mav));
  
  bool right_is_smallest = (fabs(flow_right_mav) < fabs(flow_center_mav)) && (fabs(flow_right_mav) < fabs(flow_left_mav));
  bool left_is_smallest = (fabs(flow_left_mav) < fabs(flow_center_mav)) && (fabs(flow_left_mav) < fabs(flow_right_mav));


  // PRINT("Flow left =  %f, center =  %f, right =  %f\n", flow_left_mav, flow_center_mav, flow_right_mav);
  // PRINT("Total Flow =  %f, Flow diff =  %f, low_total =  %f, factor = %f\n", total_flow, flow_difference,full_flow, full_flow_factor);
  calc_color_free_conf();


  switch (navigation_state){
    case SAFE:
      if (test){
        //calc_color_free_conf();
        PRINT("orange y: %d\n\n", orange_y);
        counter++;
        if (counter>counter_threshold){
          // PRINT("\n\n\n\n\n\n\n\n")
          //PRINT("         ***period change***\n");
          moveDistance = -moveDistance;
          counter = 0;
          turn_left -= 100; turn_right -= 100; stay_center -= 100; rotate_90 -= 100;            
        }
        if (counter < 4)
        {turn_left -= 100; turn_right -= 100; stay_center -= 100; rotate_90 -= 100;}

        moveWaypointForward(WP_GOAL, 2.0f * moveDistance);
        
        momentum_calc(total_flow, flow_difference, full_flow, left_is_smallest, right_is_smallest);


        Bound(turn_left, 0, turn_cap);
        Bound(turn_right, 0, turn_cap);
        Bound(stay_center, 0, turn_cap);
        Bound(rotate_90, 0, turn_cap);

        // choose the maximum between the turns

        turn = fmaxf(turn_left, turn_right);
        turn = fmaxf(turn, rotate_90);

        //PRINT("rotate90, turn_right, stay_center, turn_left: %f, %f, %f, %f \n", rotate_90, turn_right, stay_center, turn_left);
        if (turn_left > turn_right){
          //PRINT("turning left\n");
        } else if (turn_right > turn_left){
          //PRINT("turning right\n");
        } else if (rotate_90 >= turn){
          //PRINT("rotating 90\n");
        } else {
          //PRINT("staying center\n");
        }

        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          //PRINT("Out of Bounds\n\n");
        } else if (turn >= stay_center && turn >= turn_decision){
          //PRINT("Obstacle Found\n\n");
        } else {
          //("SAFE\n\n");
        }


        //PRINT("       ------------------------------------------------\n\n\n");
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
        momentum_calc(total_flow, flow_difference, full_flow, left_is_smallest, right_is_smallest);

        turn = fmaxf(turn_left, turn_right);
        turn = fmaxf(turn, rotate_90);
        // PRINT("rotate90 =  %f, turn_right =  %f, stay_center =  %f, turn_left =  %f\n", rotate_90, turn_right, stay_center, turn_left);
        // if (turn_left > turn_right){
        //   PRINT("SELECTED left\n");
        // } else if (turn_right > turn_left){
        //   PRINT("SELECTED right\n");
        // } else if (rotate_90 >= turn){
        //   PRINT("SELECTED rotating\n");
        // } else {
        //   PRINT("SELECTED staying center\n");
        // }
        // Move waypoint forward
        moveWaypointForward(WP_TRAJECTORY, 0.8f * moveDistance);
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          navigation_state = OUT_OF_BOUNDS;
          //PRINT("Out of Bounds\n\n");
        } else if (color_free_confidence == 0){
          navigation_state = COLOR_FOUND;
        } else if (turn >= stay_center && turn >= turn_decision){
          navigation_state = OBSTACLE_FOUND;
          //PRINT("Obstacle Found\n\n");
        } else {
          moveWaypointForward(WP_GOAL, moveDistance);
          navigation_state = SAFE;
          //PRINT("staying center\n");
        }
      }
      break;
    case COLOR_FOUND:
      PRINT("STATE: COLOR_FOUND-- %d (0=orange)\n\n",color);
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = COLOR_SEARCH_FOR_SAFE_HEADING;

      break;
    case OBSTACLE_FOUND:
      // stop
      PRINT("STATE: OBSTACLE_FOUND\n\n");
      // waypoint_move_here_2d(WP_GOAL);
      // waypoint_move_here_2d(WP_TRAJECTORY);

      moveWaypointForward(WP_GOAL, 0.0f);
      moveWaypointForward(WP_TRAJECTORY, 0.0f);

      if (turn == turn_left) {
          //PRINT("SELECTED Turn Left\n\n");
          navigation_state = TURN_LEFT;
        }
        else if (turn == turn_right){
          //PRINT("SELECTED Turn Right\n\n");
          navigation_state = TURN_RIGHT;
        }
        else {
          //PRINT("SELECTED Rotate 90 degrees\n\n");
          navigation_state = TURN_AROUND;
        }
      
      turn_left = 0;
      turn_right =0;
      stay_center =0;
      rotate_90 = 0;
      turn = 0;
      counter = 0;
      break;
    case TURN_LEFT:
      PRINT("STATE: TURN_LEFT\n\n");
      increase_nav_heading(-1*heading_increment);
      counter_hold = 0;
      
      if (counter >= 4){
      navigation_state = HOLD;
      }
      counter++;
      break;
    
    case TURN_RIGHT:
      PRINT("STATE: TURN_RIGHT\n\n");
      increase_nav_heading(1*heading_increment);
      counter_hold = 0;

      if (counter >= 4){
      navigation_state = HOLD;
      }
      counter++;
      break;

    case TURN_AROUND:
      PRINT("STATE: TURN_AROUND\n\n");
      if(counter<varr){
        // move back
        moveWaypointForward(WP_GOAL, -1.0f);
        PRINT("GOAL MOVED BACK\n\n");
      } 
      else if(counter == varr) {
        moveWaypointForward(WP_GOAL, 0.0f);
        // if (left_is_smallest){increase_nav_heading(-90);}
        // else {increase_nav_heading(90);}
        increase_nav_heading(90);
        // moveWaypointForward(WP_GOAL, 1.0f);
        PRINT("GOAL STILL+TURN\n\n");
      }
      
      else {
        //moveWaypointForward(WP_TRAJECTORY, 0.8f);
        counter_hold = 0;
        navigation_state = HOLD;
      }
      counter++;
      break;

    case HOLD:
      PRINT("STATE: HOLD\n\n\n");
      
      if (counter_hold >= 3){
      navigation_state = SAFE;
      }
      counter_hold++;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      PRINT("STATE: SEARCH_FOR_SAFE_HEADING\n\n");
      moveWaypointForward(WP_GOAL, 0.0f);

      if (counter >= 1){
        navigation_state = SAFE;
      }
      counter ++;
      break;
    case COLOR_SEARCH_FOR_SAFE_HEADING:
      PRINT("STATE: COLOR_SEARCH_FOR_SAFE_HEADING\n\n");
      moveWaypointForward(WP_GOAL, 0.0f);
      int16_t color_y;
      if(color==ORANGE){
        color_y = orange_y;
        color_heading_increment = orange_heading_increment;
      }
      else{
        color_y = white_y;
        color_heading_increment = white_heading_increment;
      }
      
      if(color_y > 0) {
        increase_nav_heading(color_heading_increment);
      }
      else{
        increase_nav_heading(-color_heading_increment);
      }
      // make sure we have a couple of good orange color readings before declaring the way safe
      if (color_free_confidence >= 2){
        navigation_state = HOLD;
      }
      break;
    case OUT_OF_BOUNDS:
      PRINT("STATE: OUT_OF_BOUNDS\n\n");
      increase_nav_heading(out_of_bounds_dheading);
      moveWaypointForward(WP_GOAL, 0.0f);
      moveWaypointForward(WP_TRAJECTORY, 0.8f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        // reset safe counter
        turn_left = 0;
        turn_right = 0;
        stay_center = 0;
        rotate_90 = 0;
        turn = 0;
        counter = 0;
        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  
  PRINT("       ------------------------------------------------\n\n\n");
  return;
}


/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees) {
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
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
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
  if (rand() % 2 == 0) {
    heading_increment = 5.f;

  } else {
    heading_increment = -5.f;

  }
  return false;
}