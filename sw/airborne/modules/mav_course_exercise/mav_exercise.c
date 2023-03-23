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
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "autopilot_static.h"
#include <stdio.h>
#include <time.h>


/*
#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
*/

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t choose20degreesIncrementAvoidance(void);


float RotateCenterArena(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};


// define settings
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]


// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 2;   // a measure of how certain we are that the way ahead is safe.
float moveDistance = 2;                 // waypoint displacement [m]
float obstacle_heading_increment = 20.f;
//float heading_increment = 20.f;
uint32_t now_ts;
float DIVERGENCE_TRESHHOLD = 0.3f;
float optical_flow_count = 0;
int16_t mode_direction = 1;
float incrementDegreesRate = 45.f;
float RadToCenter = 0.f;
float x_init = 0.f;
float y_init = 0.f;

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// needed to receive output from a separate module running on a parallel process
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID ABI_BROADCAST
#endif


#ifndef FLOOR_VISUAL_DETECTION_ID
#define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
//#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
//#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra) {
  color_count = quality;
}



static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}




static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                            uint32_t __attribute__((unused)) stamp,
                            int32_t __attribute__((unused)) flow_x,
                            int32_t __attribute__((unused)) flow_y, 
                            int32_t __attribute__((unused)) flow_der_x,
                            int32_t __attribute__((unused)) flow_der_y,
                            float __attribute__((unused)) quality,
                            float size_divergence) {
  optical_flow_count = size_divergence;
}



void mav_exercise_init(void) {
  
  // Initialise random values
  srand(time(NULL));
  //chooseAvoidanceDirection();
  x_init = stateGetPositionEnu_i()->x;
  y_init = stateGetPositionEnu_i()->y;

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying



  PRINT("Confidence: %f \n", obstacle_free_confidence);

  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    PRINT("Confidence_if: %f \n", obstacle_free_confidence);
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  } 
  
  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  PRINT("Floor centroid: %f\n", floor_centroid_frac);
  PRINT("Divergence_count: %f  Divergence_threshold: %f \n", optical_flow_count, DIVERGENCE_TRESHHOLD);
  PRINT("Confidence: %f \n", obstacle_free_confidence);

  // update our safe confidence using color threshold
 /*  if (optical_flow_count < DIVERGENCE_TRESHHOLD ) {
    obstacle_free_confidence++; */
  if (color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }


  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  float speed_sp = 2;//fminf(oag_max_speed, 0.6f*obstacle_free_confidence);

  switch (navigation_state) {
    case SAFE:
      PRINT("In SAFE mode");
        if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        PRINT("Should fly");
        guidance_h_set_body_vel(speed_sp, 0);
        PRINT("Should fly");
        guidance_h_set_body_vel(speed_sp, 0);
      }
      break;

    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);
      // select new search direction based on optic flow divergence - to be implemented
      //chooseAvoidanceDirection();
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_heading_rate(RotateCenterArena() * RadOfDeg(incrementDegreesRate));
      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){ 
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        //guidance_h_set_heading(new_heading);
        navigation_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0.05, 0.1*RotateCenterArena());

      // start turn back into arena
      guidance_h_set_heading_rate(RotateCenterArena() * RadOfDeg(incrementDegreesRate));
      PRINT("!!!!!!!!!!!!!!OUT OF BOUNDS!!!!!!!!!!!!!!!!!!!!!! \n");
      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}


/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
//float RotateCenterArena(float *new_heading, float *incrementDegreesRate){
float RotateCenterArena(void){

  // alpha - angle between reference heading and current position regarding starting point
  

  float delta_x = (stateGetPositionEnu_i()->x - x_init);
  float delta_y = (stateGetPositionEnu_i()->y - y_init);

  float alpha = atan(delta_x/delta_y);

  if (delta_y > 0){
    if (delta_x >0){
      if (stateGetNedToBodyEulers_f()->psi > alpha){
        avoidance_heading_direction = 1.f;
      } else{
        avoidance_heading_direction = -1.f;
      }
    } else{
        if (stateGetNedToBodyEulers_f()->psi < alpha){
          avoidance_heading_direction = -1.f;
        }
        else {
          avoidance_heading_direction = 1.f;
        }
    }
  } else {
    if (delta_x < 0){
      if ((3.14159 + stateGetNedToBodyEulers_f()->psi) > alpha){
        avoidance_heading_direction = 1.f;
      } else{
        avoidance_heading_direction = -1.f;
      }
    } else{
      if ((3.14159 + stateGetNedToBodyEulers_f()->psi) > -alpha){
        avoidance_heading_direction = -1.f;
      }
      else{
        avoidance_heading_direction = 1.f;
      }
    }
  }

return avoidance_heading_direction;
}


/*
 // Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}


 // Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}


 // Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}


 // Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


 // Sets the variable 'heading_increment' randomly positive/negative
 


uint8_t choose20degreesIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = obstacle_heading_increment;
  } else {
    heading_increment = -obstacle_heading_increment;
  }
  return false;
}
*/