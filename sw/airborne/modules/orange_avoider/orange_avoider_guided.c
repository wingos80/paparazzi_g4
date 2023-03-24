/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>



#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


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
float oag_floor_count_frac = 0.01f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
int16_t mode_direction = 1;
float incrementDegreesRate = 30.f;
float RadToCenter = 0.f;
float x_init = 0.f;
float y_init = 0.f;
float turn_left = 0;
float turn_right = 0;
float stay_center = 0;
float flow_left_mav;
float flow_center_mav;
float flow_right_mav;
float rotate_90 = 0;
float turn = 0;

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID COLOR_OBJECT_DETECTION1_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif


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



static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID COLOR_OBJECT_DETECTION2_ID
//#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
//#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  //chooseAvoidanceDirection();
  x_init = stateGetPositionEnu_i()->x;
  y_init = stateGetPositionEnu_i()->y;

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  // VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  // VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);


  if ((abs(flow_left_mav) < abs(flow_center_mav)) && (abs(flow_left_mav) < abs(flow_right_mav))) {
    turn_left += 2;
    turn_right -=1;
    stay_center -=1;
    rotate_90 -=1;
    //VERBOSE_PRINT("Decison: Turn Left");
  }
  else if ((abs(flow_right_mav) < abs(flow_left_mav)) && (abs(flow_right_mav) < abs(flow_center_mav))) {
    turn_right += 2;
    turn_left -=1;
    stay_center -=1;
    rotate_90 -=1;
    //VERBOSE_PRINT("Decison: Turn Right");
  }
  else {
    if (abs(flow_center_mav) > 80){
      rotate_90 +=3;
      turn_right -= 1;
      turn_left -=1;
      stay_center -=1;
      //VERBOSE_PRINT("Decison: Rotate 90");
    }
    else {
      stay_center +=2;
      rotate_90 -=1;
      turn_right -= 1;
      turn_left -=1;
      //VERBOSE_PRINT("Decison: Stay Center");
    }
  }

  if(color_count < color_count_threshold){
    turn_left++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }


  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  Bound(turn_left, 0, 14);
  Bound(turn_right, 0, 14);
  Bound(stay_center, 0, 14);
  Bound(rotate_90, 0, 14);

  if (turn >= stay_center){
    if (turn == turn_left) {
      VERBOSE_PRINT("Turn Left");
    }
    else if (turn == turn_right){
      VERBOSE_PRINT("Turn Right");
    }
    else {
      VERBOSE_PRINT("Rotate 90 degrees");
    }
  }
  else{
    VERBOSE_PRINT("Stay Center");
  }

  turn = fmaxf(turn_left, turn_right);
  turn = fmaxf(turn, rotate_90);

  //float speed_sp = fminf(oag_max_speed, 0.3f * stay_center);  // change velocity here
  float speed_sp = 0.35;

  switch (navigation_state){
    case SAFE:
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.33){ // change upper bound of fabsf(floor_centroid_frac) for the drone to get closer to the edge before turning (higher bound = goes closer to the edge)
        navigation_state = OUT_OF_BOUNDS;
      } else if (turn >= stay_center){
        navigation_state = OBSTACLE_FOUND;
      } else {
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
      navigation_state = SAFE;
      // make sure we have a couple of good readings before declaring the way safe
      // if (obstacle_free_confidence >= 2){ 
      //   guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
      //   //guidance_h_set_heading(new_heading);
      //   navigation_state = SAFE;
      // }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0.05, 0.1*RotateCenterArena());

      // start turn back into arena
      guidance_h_set_heading_rate(RotateCenterArena() * RadOfDeg(incrementDegreesRate));
      VERBOSE_PRINT("!!!!!!!!!!!!!!OUT OF BOUNDS!!!!!!!!!!!!!!!!!!!!!! \n");
      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        float test_heading = stateGetNedToBodyEulers_f()->psi + RotateCenterArena()*(3.14/4);
        guidance_h_set_heading(test_heading);
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







