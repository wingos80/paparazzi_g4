/*
 * Copyright (C) 2014 Hann Woei Ho
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

/**
 * @file modules/computer_vision/opticflow_module.c
 * @brief Optical-flow estimation module
 *
 */

#include "opticflow_module.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "modules/core/abi.h"
#include "modules/pose_history/pose_history.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "lib/vision/image.h"
#include "errno.h"

#include "cv.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string, __FUNCTION__, ##__VA_ARGS__)

/* ABI messages sender ID */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)

#ifndef OPTICFLOW_FPS
#define OPTICFLOW_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

#ifndef OPTICFLOW_FPS_CAMERA2
#define OPTICFLOW_FPS_CAMERA2 0 ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FPS)
PRINT_CONFIG_VAR(OPTICFLOW_FPS_CAMERA2)

#ifdef OPTICFLOW_CAMERA2
#define ACTIVE_CAMERAS 2
#else
#define ACTIVE_CAMERAS 1
#endif

// #ifndef NUM_VER_SEC
// #define NUM_VER_SEC 1     ///< Number of vertical sections on image to calculate optical flow
// #endif

#ifndef NUM_HOR_SEC
#define NUM_HOR_SEC 3 ///< Number of horizontal sections on image to calculate optical flow
#endif

#ifndef NUM_SEC
#define NUM_SEC NUM_HOR_SEC + NUM_HOR_SEC ///< Number of horizontal sections on image to calculate optical flow
#endif

#ifndef LEFT
#define LEFT 0 ///< Number of horizontal sections on image to calculate optical flow
#endif

#ifndef CENTER
#define CENTER 0.5 ///< Number of horizontal sections on image to calculate optical flow
#endif

#ifndef RIGHT
#define RIGHT 1 ///< Number of horizontal sections on image to calculate optical flow
#endif

#ifndef ROTATE90
#define ROTATE90 0.75 ///< Number of horizontal sections on image to calculate optical flow
#endif

#ifndef CENTER_FACTOR
#define CENTER_FACTOR 2
#endif

/* The main opticflow variables */
struct opticflow_t opticflow[ACTIVE_CAMERAS]; ///< Opticflow calculations
struct opticflow_t opticflow_mav[NUM_SEC][ACTIVE_CAMERAS];
struct opticflow_t opticflow_l[ACTIVE_CAMERAS]; ///< Opticflow calculations
struct opticflow_t opticflow_c[ACTIVE_CAMERAS]; ///< Opticflow calculations
struct opticflow_t opticflow_r[ACTIVE_CAMERAS]; ///< Opticflow calculations

static struct opticflow_result_t opticflow_result[ACTIVE_CAMERAS]; ///< The opticflow result
static float flow_y_test[NUM_SEC];

static bool opticflow_got_result[ACTIVE_CAMERAS]; ///< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;           ///< Mutex lock fo thread safety

/* Static functions */
struct image_t *opticflow_module_calc(struct image_t *img, uint8_t camera_id); ///< The main optical flow calculation thread

struct image_t cropped_img;
struct image_t cropped_img_gray;
struct image_t sections_img_p[NUM_HOR_SEC];
struct image_t bigger_sections_img_p[NUM_HOR_SEC - 1];
struct image_t final_img;

float ttc = 1110.0;
float constt = 0.0;
float gradi = 2.0;
float divergencee;
float dive_sizee;
float flow_x_test;
float flow_y_center;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  for (int idx_camera = 0; idx_camera < ACTIVE_CAMERAS; idx_camera++)
  {
    if (opticflow_result[idx_camera].noise_measurement < 0.8)
    {
      pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
                                   &opticflow_result[idx_camera].fps, &opticflow_result[idx_camera].corner_cnt,
                                   &opticflow_result[idx_camera].tracked_cnt, &opticflow_result[idx_camera].flow_x,
                                   &opticflow_result[idx_camera].flow_y, &opticflow_result[idx_camera].flow_der_x,
                                   &opticflow_result[idx_camera].flow_der_y, &opticflow_result[idx_camera].vel_body.x,
                                   &opticflow_result[idx_camera].vel_body.y, &opticflow_result[idx_camera].vel_body.z,
                                   &opticflow_result[idx_camera].div_size,
                                   &opticflow_result[idx_camera].surface_roughness,
                                   &opticflow_result[idx_camera].divergence,
                                   &opticflow_result[idx_camera].camera_id); // TODO: no noise measurement here...
      // pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
      //                              &opticflow_result[idx_camera].fps, &opticflow_result[idx_camera].corner_cnt,
      //                              &opticflow_result[idx_camera].tracked_cnt, &opticflow_result[idx_camera].flow_x,
      //                              &opticflow_result[idx_camera].flow_y, &opticflow_result[idx_camera].flow_der_x,
      //                              &opticflow_result[idx_camera].flow_der_y, &opticflow_result[idx_camera].vel_body.x,
      //                              &opticflow_result[idx_camera].vel_body.y, &opticflow_result[idx_camera].vel_body.z,
      //                              &flow_y_center,
      //                              &flow_x_test,
      //                              &divergencee,
      //                              &opticflow_result[idx_camera].camera_id); // TODO: no noise measurement here...
    }
  }
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif

/**
 * Initialize the optical flow module for the bottom camera
 */
void opticflow_module_init(void)
{
  // Initialize the opticflow calculation
  for (int idx_camera = 0; idx_camera < ACTIVE_CAMERAS; idx_camera++)
  {
    // for (int i = 0; i< NUM_VER_SEC; i++) {
    // for (int j = 0; j< NUM_HOR_SEC; j++) {
    opticflow_got_result[idx_camera] = false;
    //}
    //}
  }
  opticflow_calc_init(opticflow);
  for (int index = 0; index < NUM_SEC; index++)
  {
    opticflow_calc_init_mav(opticflow_mav[index], index);
    // opticflow_calc_init(opticflow_r);
  }

  cv_add_to_device(&OPTICFLOW_CAMERA, opticflow_module_calc, OPTICFLOW_FPS, 0);
#ifdef OPTICFLOW_CAMERA2
  cv_add_to_device(&OPTICFLOW_CAMERA2, opticflow_module_calc, OPTICFLOW_FPS_CAMERA2, 1);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTIC_FLOW_EST, opticflow_telem_send);
#endif
}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void opticflow_module_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  // Update the stabilization loops on the current calculation
  for (int idx_camera = 0; idx_camera < ACTIVE_CAMERAS; idx_camera++)
  {
    if (opticflow_got_result[idx_camera])
    {
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID + idx_camera, now_ts,
                             opticflow_result[idx_camera].flow_x,
                             opticflow_result[idx_camera].flow_y,
                             opticflow_result[idx_camera].flow_der_x,
                             opticflow_result[idx_camera].flow_der_y,
                             opticflow_result[idx_camera].noise_measurement,
                             opticflow_result[idx_camera].div_size,
                             flow_y_test[0],
                             flow_y_test[1],
                             flow_y_test[2]);
      // TODO Find an appropriate quality measure for the noise model in the state filter, for now it is tracked_cnt
      if (opticflow_result[idx_camera].noise_measurement < 0.8)
      {
        AbiSendMsgVELOCITY_ESTIMATE(VEL_OPTICFLOW_ID + idx_camera, now_ts,
                                    opticflow_result[idx_camera].vel_body.x,
                                    opticflow_result[idx_camera].vel_body.y,
                                    0.0f, // opticflow_result.vel_body.z,
                                    opticflow_result[idx_camera].noise_measurement,
                                    opticflow_result[idx_camera].noise_measurement,
                                    -1.0f // opticflow_result.noise_measurement // negative value disables filter updates with OF-based vertical velocity.
        );
      }
      opticflow_got_result[idx_camera] = false;
    };
  }
  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator
 * @param[in] *img The image_t structure of the captured image
 * @param[in] camera_id The camera index id
 * @return *img The processed image structure
 */
struct image_t *opticflow_module_calc(struct image_t *img, uint8_t camera_id)
{
  // crop the image first - take away 120 pixels from the width and 170 pixels from the heigh
  int w_change = 160;
  int h_change = 200;
  uint16_t new_w = img->w - w_change;
  uint16_t new_h = img->h - h_change;
  int section_h = new_h / NUM_HOR_SEC;
  int section_w = new_w;
  int index;
  bool ret_val = false;
  float temp_div_size = 0.0001;
  float time_threshold = 0.5;

  static struct opticflow_result_t temp_result[ACTIVE_CAMERAS]; // static so that the number of corners is kept between frames

  image_create(&cropped_img, new_w, new_h, IMAGE_YUV422);
  // image_create(&cropped_img_gray, new_w, new_h, IMAGE_YUV422);
  image_create(&final_img, section_w, section_h * NUM_HOR_SEC, IMAGE_YUV422);

  crop_img(img, &cropped_img, w_change, h_change);

  struct opticflow_t *opticflow_pp;

  // optic flow calculations for first layer sections
  for (index = 0; index < NUM_HOR_SEC; index++)
  {

    image_create(&sections_img_p[index], section_w, section_h, IMAGE_YUV422);
    divide_img(&cropped_img, &sections_img_p[index], section_w, section_h, index);

    opticflow_pp = &opticflow_mav[index][camera_id];

    ret_val = opticflow_calc_frame(opticflow_pp, &sections_img_p[index], &temp_result[camera_id], index);
    if (ret_val)
    {
      // Copy the result if finished
      pthread_mutex_lock(&opticflow_mutex);
      opticflow_result[camera_id] = temp_result[camera_id];
      opticflow_got_result[camera_id] = true;
      flow_y_test[index] = opticflow_result[camera_id].flow_y;
      if (index == 1)
      {
        flow_x_test = temp_result[camera_id].flow_x;
        // flow_y_center = flow_y_test[index];
        // divergencee = temp_result[camera_id].divergence;
        // dive_sizee = temp_result[camera_id].div_size;
        // ttc = 1.0/(gradi*abs(divergencee)+ constt);
      }
      PRINT("Section: %d, flow_y: %f\n", index, flow_y_test[index]);

      pthread_mutex_unlock(&opticflow_mutex);
    }
    else if (opticflow_pp->got_first_img == true)
    {
      if (index == 1)
      {
        flow_y_test[1] = flow_y_test[1] / CENTER_FACTOR;
      }
      PRINT("Failed Calculation(Same Image)/n/n/n Section: %d, flow_y: %f\n", index, flow_y_test[index]);
    }
    else
    {
      flow_y_test[index] = 0.0;
      PRINT("Failed Calculation /n/n/n Section: %d\n", index);
    }
  }

  // optic flow calculations for second layer sections
  // for (index=0;index<(NUM_HOR_SEC-1);index++){
  //   image_create(&bigger_sections_img_p[index], 2*section_w, 2*section_h, IMAGE_YUV422);
  //   glue_img(&sections_img_p[index], &bigger_sections_img_p[index], section_w, section_h, index);
  //   glue_img(&sections_img_p[index+1], &bigger_sections_img_p[index], section_w, section_h, index);

  //   struct opticflow_t *opticflow_pp = &opticflow_mav[index+NUM_HOR_SEC][camera_id];
  //   bool ret_val = false;

  //   ret_val = opticflow_calc_frame(opticflow_pp, &bigger_sections_img_p[index], &temp_result[camera_id], NUM_HOR_SEC + index);
  //   if(ret_val){
  //     // Copy the result if finished
  //     pthread_mutex_lock(&opticflow_mutex);
  //     opticflow_result[camera_id] = temp_result[camera_id];
  //     opticflow_got_result[camera_id] = true;

  //     flow_y_test[NUM_HOR_SEC+index] = opticflow_result[camera_id].flow_y;
  //     PRINT("Section: %d, flow_y: %f\n", NUM_HOR_SEC + index, flow_y_test[NUM_HOR_SEC+index]);

  //     pthread_mutex_unlock(&opticflow_mutex);
  //   }
  //   else{
  //     flow_y_test[index+NUM_HOR_SEC] = 0;
  //     PRINT("Failed Calculation. Section: %d\n", NUM_HOR_SEC + index);
  //   }
  // }

  // flow_y_test[0] = flow_y_test[0] + flow_y_test[0 + NUM_HOR_SEC] - flow_y_test[1];
  // flow_y_test[2] = flow_y_test[2] + flow_y_test[1 + NUM_HOR_SEC] - flow_y_test[1];
  // flow_y_test[1] = flow_y_test[1] + flow_y_test[0 + NUM_HOR_SEC] + flow_y_test[1 + NUM_HOR_SEC] - flow_y_test[0] - flow_y_test[2];
  flow_y_test[1] = CENTER_FACTOR * flow_y_test[1];

  float turn;

  // // compute time to contact
  // PRINT("time to contact: %f\n", ttc);
  // // time to contact colouring
  // if (0.0 < ttc < time_threshold){
  //   if ((abs(flow_y_test[2]) < abs(flow_y_test[0]))){
  //   turn = RIGHT;
  // }
  //   else {
  //     turn = LEFT;
  //   }
  // } else{
  //   turn = CENTER;
  // }

  // leo's
  if ((fabs(flow_y_test[2]) < fabs(flow_y_test[1])) && (fabs(flow_y_test[2]) < fabs(flow_y_test[0])))
  {
    turn = RIGHT;
    PRINT("Decision: Turn Right\n");
  }
  else if ((fabs(flow_y_test[0]) < fabs(flow_y_test[1])) && (fabs(flow_y_test[0]) < fabs(flow_y_test[2])))
  {
    turn = LEFT;
    PRINT("Decision: Turn Left\n");
  }
  else
  {
    if (fabs(flow_y_test[1]) > 80)
    {
      turn = 1.5;
      PRINT("Decision: Rotate 90\n");
    }
    else
    {
      turn = CENTER;
      PRINT("Decision: Stay Center\n");
    }
  }
  // if (turn = CENTER) {
  //   if (abs(flow_y_test[1]) > 100){
  //     turn = 1.5;
  //   }
  // }
  // // if ((abs(flow_y_test[2]) > abs(flow_y_test[1])) && (abs(flow_y_test[0]) > abs(flow_y_test[1]))) {
  // //   turn = CENTER;
  // // }
  // // else if ((abs(flow_y_test[2]) > abs(flow_y_test[0])) && (abs(flow_y_test[2]) > abs(flow_y_test[1]))) {
  // //   turn = LEFT;
  // // }
  // // else if ((abs(flow_y_test[0]) > abs(flow_y_test[1])) && (abs(flow_y_test[0]) > abs(flow_y_test[2]))) {
  // //   turn = RIGHT;
  // // }
  // // else {
  // //   turn = CENTER;
  // //}

  for (int index = 0; index < NUM_HOR_SEC; index++)
  {
    div_coloring(&sections_img_p[index], turn);
    glue_img(&sections_img_p[index], &final_img, section_w, section_h, index);
  }
  PRINT("--------------------------------------------------------------------------------------\n\n");
  
  // img = &cropped_img;
  img = &final_img;
  return img;
}

//  * @param img height = 520, width = 240 (front cam normally)
//  * @return struct image_t*
//  */
// struct image_t *crop_img(struct image_t *img)
// {
//   // take away 120 pixels from the width and 170 pixels from the heigh
//   int w_change = 120;
//   int h_change = 170;
//   uint16_t new_w = img->w - 120;
//   uint16_t new_h = img->h - 170;