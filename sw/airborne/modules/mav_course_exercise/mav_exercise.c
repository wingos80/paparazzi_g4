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
uint8_t choose20degreesIncrementAvoidance(void);
float RotateCenterArena(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
  HOLD,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// define and initialise global variables
float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SAFE;
enum navigation_state_t this_turn = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float moveDistance = 1.0;               // waypoint displacement [m]
float oob_haeding_increment = 5.f;      // heading angle increment if out of bounds [deg]
float obstacle_heading_increment = 15.f;
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
float heading_increment = 10.f;
float turn_around_increment = 40.f;
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
int min_momentum = 0;
int turn_decision = 6;
int turn_cap = 7;
float out_of_bounds_dheading = 40.0;
int count_out_of_bound = 0;
float rotate_90 = 0;
float turn = 0;
float x_init = 0;
float y_init = 0;
int nav_heading_int;

// // expoenentially weighted moving average params
float total_thresh = 150.0;
float diff_thresh = 35.3;
float total_low_thresh = 1.5;

// needed to receive output from a separate module running on a parallel process
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra) {
  color_count = quality;
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
  PRINT("out_of_bounds_dheading: %f turn_cap: %d turn_decision: %d flow_noise_threshold: %f \n", out_of_bounds_dheading, turn_cap, turn_decision, flow_noise_threshold);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    return;
  }
  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  // int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  // float flow;
  // // exponentially weighted moving average to smooth flow values
  // for (int j=0;j<3;j++){
  //   if(j==0){flow = flow_left_mav;}
  //   else if (j==1){flow = flow_center_mav;}
  //   else if (j==2){flow = flow_right_mav;}

  //   s_cur[j] = rho * s_prev[j] + (1.0-rho) * flow;
  //   s_cur_bc[j] = s_cur[j]/(1.0-pow(rho,(i+1)));

  //   s_prev[j] = s_cur[j];
  // }
  // s_cur[0] = rho * s_prev[0] + (1.0-rho) * flow_left_mav;
  // s_cur_bc[0] = s_cur[0]/(1.0-pow(rho,(i+1)));

  // s_prev[0] = s_cur[0];

  float total_flow = fabs(flow_center_mav);
  float flow_difference = fabs(fabs(flow_left_mav) - fabs(flow_right_mav));
  float full_flow = (fabs(flow_center_mav) + fabs(flow_left_mav) + fabs(flow_right_mav));
  float full_flow_factor = (fabs(flow_center_mav) + fabs(flow_left_mav) + fabs(flow_right_mav))/fabs(flow_center_mav);

  bool right_is_smallest = (fabs(flow_right_mav) < fabs(flow_center_mav)) && (fabs(flow_right_mav) < fabs(flow_left_mav));
  bool left_is_smallest = (fabs(flow_left_mav) < fabs(flow_center_mav)) && (fabs(flow_left_mav) < fabs(flow_right_mav));

  PRINT("Flow left =  %f, center =  %f, right =  %f\n", flow_left_mav, flow_center_mav, flow_right_mav);
  PRINT("Total Flow =  %f, Flow diff =  %f, low_total =  %f\n", total_flow, flow_difference,full_flow);

  if (((total_flow>total_thresh) && (flow_difference<diff_thresh)) || (full_flow_factor <= total_low_thresh)
          || (full_flow < 50)){
    rotate_90 +=2;
    turn_right -= 1;
    turn_left -=1;
    stay_center -=1;
    PRINT("DETECTED Turn around\n\n");
  }
  else if (left_is_smallest && (flow_difference>diff_thresh) && fabs(flow_left_mav) > 10) {
    turn_left += 2;
    turn_right -=1;
    stay_center -=1;
    rotate_90 -=1;
    PRINT("DETECTED Left\n");
  }
  else if (right_is_smallest && (flow_difference>diff_thresh) && fabs(flow_right_mav) > 10) {
    turn_right += 2;
    turn_left -=1;
    stay_center -=1;
    rotate_90 -=1;
    PRINT("DETECTED Right\n\n");
  }
  else {
    stay_center +=2;
    rotate_90 -=1;
    turn_right -= 1;
    turn_left -=1;
    PRINT("DETECTED Center\n\n");
  }
  PRINT("rotate90 =  %f, turn_right =  %f, stay_center =  %f, turn_left =  %f\n", rotate_90, turn_right, stay_center, turn_left);

  switch (navigation_state){
    case SAFE:
      if (test){
        counter++;
        if (counter>counter_threshold){
          // PRINT("\n\n\n\n\n\n\n\n")
          PRINT("         ***period change***\n");
          moveDistance = -moveDistance;
          counter = 0;
          turn_left -= 100; turn_right -= 100; stay_center -= 100; rotate_90 -= 100;            
        }
        moveWaypointForward(WP_GOAL, 2.0f * moveDistance);

        if ((total_flow>total_thresh) && (flow_difference<diff_thresh)){
          rotate_90 +=2;
          turn_right -= 1;
          turn_left -=1;
          stay_center -=1;
          // PRINT("Turn around\n\n");
        }
        else if (left_is_smallest && (flow_difference>diff_thresh)) {
          turn_left += 2;
          turn_right -=1;
          stay_center -=1;
          rotate_90 -=1;
          // PRINT("Left\n\n");
        }
        else if (right_is_smallest && (flow_difference>diff_thresh)) {
          turn_right += 2;
          turn_left -=1;
          stay_center -=1;
          rotate_90 -=1;
          // PRINT("Right\n\n");
        }
        else {
          stay_center +=2;
          rotate_90 -=1;
          turn_right -= 1;
          turn_left -=1;
          // PRINT("Center\n\n");
        }    

        Bound(turn_left, 0, turn_cap);
        Bound(turn_right, 0, turn_cap);
        Bound(stay_center, 0, turn_cap);
        Bound(rotate_90, 0, turn_cap);

        // choose the maximum between the turns

        turn = fmaxf(turn_left, turn_right);
        turn = fmaxf(turn, rotate_90);

        PRINT("rotate90, turn_right, stay_center, turn_left: %f, %f, %f, %f \n", rotate_90, turn_right, stay_center, turn_left);
        if (turn_left > turn_right){
          PRINT("turning left\n");
        } else if (turn_right > turn_left){
          PRINT("turning right\n");
        } else if (rotate_90 >= turn){
          PRINT("rotating 90\n");
        } else {
          PRINT("staying center\n");
        }

        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          PRINT("Out of Bounds\n\n");
        } else if (turn >= stay_center && turn >= turn_decision){
          PRINT("Obstacle Found\n\n");
        } else {
          PRINT("SAFE\n\n");
        }


        PRINT("       ------------------------------------------------\n\n\n");
        break;
      }
      // ------------------------------------------------------
      // ------------------------------------------------------
      // TEST ENDS HERE
      // ------------------------------------------------------
      // ------------------------------------------------------

      else{
        PRINT("STATE: SAFE\n\n\n");

        Bound(turn_left, 0, turn_cap);
        Bound(turn_right, 0, turn_cap);
        Bound(stay_center, 0, turn_cap);
        Bound(rotate_90, 0, turn_cap);
        turn = fmaxf(turn_left, turn_right);
        turn = fmaxf(turn, rotate_90);

        if (turn_left > turn_right){
          this_turn = TURN_LEFT;
          PRINT("SELECTED left\n");
        } else if (turn_right > turn_left){
          this_turn = TURN_RIGHT;
          PRINT("SELECTED right\n");
        } else if (rotate_90 >= turn){
          this_turn = TURN_AROUND;
          PRINT("SELECTED rotating\n");
        } else {
          PRINT("SELECTED staying center\n");
        }
        // Move waypoint forward
        moveWaypointForward(WP_TRAJECTORY, 1.2f * moveDistance);
        
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          navigation_state = OUT_OF_BOUNDS;
          PRINT("SAFE ---> OUT_OF_BOUNDS\n\n");
        } else if (turn >= stay_center && turn >= turn_decision){
          navigation_state = OBSTACLE_FOUND;
          PRINT("SAFE ---> OBSTACLE_FOUND\n\n");
        } else {
          moveWaypointForward(WP_GOAL, moveDistance);
          navigation_state = SAFE;
          PRINT("SAFE!\n");
        }

      }
      break;
    case OBSTACLE_FOUND:
      // stop
      PRINT("STATE: OBSTACLE_FOUND\n\n");
      // waypoint_move_here_2d(WP_GOAL);
      // waypoint_move_here_2d(WP_TRAJECTORY);

      moveWaypointForward(WP_GOAL, 0.0f);
      moveWaypointForward(WP_TRAJECTORY, 0.0f);

      if (this_turn == TURN_LEFT) {
          PRINT("OBSTACLE_FOUND --> LEFT");
          navigation_state = TURN_LEFT;
        }
        else if (this_turn == TURN_RIGHT){
          PRINT("OBSTACLE_FOUND --> RIGHT");
          navigation_state = TURN_RIGHT;
        }
        else {
          PRINT("OBSTACLE_FOUND --> AROUND");
          navigation_state = TURN_AROUND;
        }
      counter =0;
      turn_left = 0;
      turn_right =0;
      stay_center =0;
      rotate_90 = 0;
      turn = 0;
      break;
    case TURN_LEFT:
      PRINT("STATE: TURN_LEFT\n\n");
      increase_nav_heading(-1*heading_increment);
      counter_hold = 0;
      // make sure we have a couple of good readings before declaring the way safe
      if (counter >= 2){
      navigation_state = HOLD;
      }
      counter++;
      break;
    
    case TURN_RIGHT:
      PRINT("STATE: TURN_RIGHT\n\n");
      increase_nav_heading(1*heading_increment);
      counter_hold = 0;
      // make sure we have a couple of good readings before declaring the way safe
      if (counter >= 2){
      navigation_state = HOLD;
      }
      counter++;
      break;

    case TURN_AROUND:
      PRINT("STATE: TURN_AROUND\n\n");
      increase_nav_heading(turn_around_increment);      
      counter_hold = 0;

      // make sure we have a couple of good readings before declaring the way safe
      if(counter <= 1) { 
        moveWaypointForward(WP_GOAL, -1.5*moveDistance);
      }
      else if (counter >= 3){
        navigation_state = HOLD;
      }
      else {
        moveWaypointForward(WP_GOAL, 0.0f);
      }
      counter++;
      break;

    case HOLD:
      PRINT("STATE: HOLD\n\n\n");
      // make sure we have a couple of good readings before declaring the way safe
      if (counter_hold >= 2){
      navigation_state = SAFE;
      }
      counter_hold++;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      PRINT("STATE: SEARCH_FOR_SAFE_HEADING\n\n");
      //increase_nav_heading(40);

      // make sure we have a couple of good readings before declaring the way safe
      if (counter >= 1){
        navigation_state = SAFE;
      }
      counter ++;
      break;
    case OUT_OF_BOUNDS:
      PRINT("STATE: OUT_OF_BOUNDS\n\n");
      increase_nav_heading(out_of_bounds_dheading);
      moveWaypointForward(WP_GOAL, 0.0f);
      moveWaypointForward(WP_TRAJECTORY, 0.8f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        //increase_nav_heading(120);
        // reset safe counter
        turn_left = min_momentum;
        turn_right = min_momentum;
        stay_center = 0;
        rotate_90 = min_momentum;
        turn = 0;
        counter = 0;
        count_out_of_bound++;
        // ensure direction is safe before continuing
        if(count_out_of_bound==3){
          navigation_state = SEARCH_FOR_SAFE_HEADING;
          count_out_of_bound = 0;
        }
      }
      break;
    default:
      break;
  }
  PRINT("------------------------------------------------\n\n\n");

  // switch (navigation_state){
  //   case SAFE:
  //       PRINT("\n\n\nSAFE\n\n\n");
  //       if ((fabs(flow_left_mav) < fabs(flow_center_mav)) && (fabs(flow_left_mav) < fabs(flow_right_mav))) {
  //         turn_left += 2;
  //         turn_right -=1;
  //         stay_center -=1;
  //         rotate_90 -=1;
  //         //PRINT("Decison: Turn Left");
  //       }
  //       else if ((fabs(flow_right_mav) < fabs(flow_left_mav)) && (fabs(flow_right_mav) < fabs(flow_center_mav))) {
  //         turn_right += 2;
  //         turn_left -=1;
  //         stay_center -=1;
  //         rotate_90 -=1;
  //         //PRINT("Decison: Turn Right");
  //       }
  //       else {
  //         if (fabs(flow_center_mav) > 80){
  //           rotate_90 +=3;
  //           turn_right -= 1;
  //           turn_left -=1;
  //           stay_center -=1;
  //           //PRINT("Decison: Rotate 90");
  //         }
  //         else {
  //           stay_center +=2;
  //           rotate_90 -=1;
  //           turn_right -= 1;
  //           turn_left -=1;
  //           //PRINT("Decison: Stay Center");
  //         }
  //       }

  //       Bound(turn_left, 0, 14);
  //       Bound(turn_right, 0, 14);
  //       Bound(stay_center, 0, 14);
  //       Bound(rotate_90, 0, 14);

  //       turn = fmaxf(turn_left, turn_right);
  //       turn = fmaxf(turn, rotate_90);

  //       // if (turn >= stay_center){
  //       //   if (turn == turn_left) {
  //       //     PRINT("Turn Left");
  //       //   }
  //       //   else if (turn == turn_right){
  //       //     PRINT("Turn Right");
  //       //   }
  //       //   else {
  //       //     PRINT("Rotate 90 degrees");
  //       //   }
  //       // }
  //       // else{
  //       //   PRINT("Stay Center");
  //       // }

  //       moveWaypointForward(WP_TRAJECTORY, 1.5f * 0.5);
  //       if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
  //         navigation_state = OUT_OF_BOUNDS;
  //       }
  //       else {
  //         if (turn >= stay_center && turn >= 7){
  //           navigation_state = WAIT1;
  //       }
  //         else {
  //           guidance_h_set_vel(speed_sp, 0); 
            
  //           moveWaypointForward(WP_GOAL, moveDistance);
  //       }  
  //     }
  //     break;
  //   case WAIT1:
  //     moveDistance = 0;
  //     moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
  //     waypoint_move_here_2d(WP_GOAL);
  //     waypoint_move_here_2d(WP_TRAJECTORY);
  //     PRINT("\n\n\nWAIT1\n\n\n");
  //     counter ++;
  //       if (counter >= 20) {
  //         navigation_state = OBSTACLE_FOUND;
  //         counter = 0;
  //       }
  //       else {
  //         navigation_state = WAIT1;
  //       }
  //     break; 

  //   case OBSTACLE_FOUND:
  //     // stop
  //     PRINT("\n\n\nOBSTACLE FOUND\n\n\n");
  //     waypoint_move_here_2d(WP_GOAL);
  //     waypoint_move_here_2d(WP_TRAJECTORY);


  //     guidance_h_set_vel(0, 0);
  //     if (turn == turn_left) {
  //       increase_nav_heading(-1*obstacle_heading_increment);
  //       counter ++;
  //     }
  //     else if (turn == turn_right){
  //       //nav_heading_int = ANGLE_BFP_OF_REAL(0.35);
  //       increase_nav_heading(obstacle_heading_increment);
  //       counter ++;
  //     }
  //     else {
  //       increase_nav_heading(90);
  //       counter ++;
  //     }
  //     turn_left =0;
  //     turn_right = 0;
  //     stay_center = 0;
  //     rotate_90 = 0;
  //     // select new search direction based on optic flow divergence - to be implemented
  //     //chooseAvoidanceDirection();
  //     if (counter >= 2) {
  //       counter = 0;
  //       navigation_state = WAIT2;
  //     }else{
  //       navigation_state = OBSTACLE_FOUND;
  //     }
  //     break;
  //   case WAIT2:
  //   guidance_h_set_vel(0, 0);
  //   PRINT("\n\n\nWAIT2\n\n\n");
  //   counter ++;
  //     if (counter >= 30) {
  //       navigation_state = SAFE;
  //       counter = 0;
  //     }
  //     else {
  //       navigation_state = WAIT2;
  //     }
  //    break;
  //   case OUT_OF_BOUNDS:
  //     // stop
  //     //guidance_h_set_body_vel(0.05, 0.1*RotateCenterArena());

  //     // start turn back into arena
  //     //guidance_h_set_heading_rate(RotateCenterArena() * RadOfDeg(incrementDegreesRate));
  //     //PRINT("!!!!!!!!!!!!!!OUT OF BOUNDS!!!!!!!!!!!!!!!!!!!!!! \n");

  //     navigation_state = SAFE;

  //     break;
    // case REENTER_ARENA:
    //   // force floor center to opposite side of turn to head back into arena
    //   if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
    //     // return to heading mode
    //     float test_heading = stateGetNedToBodyEulers_f()->psi + RotateCenterArena()*(3.14/4);
    //     guidance_h_set_heading(test_heading);
    //     // reset safe counter
    //     obstacle_free_confidence = 0;

    //     // ensure direction is safe before continuing
    //     navigation_state = SAFE;
    //   }
      //break;
  //   default:
  //     break;
  // }
  
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


uint8_t choose20degreesIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = obstacle_heading_increment;
  } else {
    heading_increment = obstacle_heading_increment;
  }
  return false;
}


// float RotateCenterArena(void){

//   // alpha - angle between reference heading and current position regarding starting point
  

//   float delta_x = (stateGetPositionEnu_i()->x - x_init);
//   float delta_y = (stateGetPositionEnu_i()->y - y_init);
//   float avoidance_heading_direction;

//   float alpha = atan(delta_x/delta_y);

//   if (delta_y > 0){
//     if (delta_x >0){
//       if (stateGetNedToBodyEulers_f()->psi > alpha){
//         avoidance_heading_direction = 1.f;
//       } else{
//         avoidance_heading_direction = -1.f;
//       }
//     } else{
//         if (stateGetNedToBodyEulers_f()->psi < alpha){
//           avoidance_heading_direction = -1.f;
//         }
//         else {
//           avoidance_heading_direction = 1.f;
//         }
//     }
//   } else {
//     if (delta_x < 0){
//       if ((3.14159 + stateGetNedToBodyEulers_f()->psi) > alpha){
//         avoidance_heading_direction = 1.f;
//       } else{
//         avoidance_heading_direction = -1.f;
//       }
//     } else{
//       if ((3.14159 + stateGetNedToBodyEulers_f()->psi) > -alpha){
//         avoidance_heading_direction = -1.f;
//       }
//       else{
//         avoidance_heading_direction = 1.f;
//       }
//     }
//   }

// return avoidance_heading_direction;
// }
