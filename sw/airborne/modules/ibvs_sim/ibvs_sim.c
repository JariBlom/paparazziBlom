/*
 * Copyright (C) Jari Blom
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ibvs_sim/ibvs_sim.c"
 * @author Jari Blom
 * Simulate Image Based Visual Servoing
 */

/**
 * Camera reference frame as well as virtual camera reference frame
 * is defined from 0 to 1
 * World Frame is ENU
 */

#include "std.h"
#include <stdio.h>

#include "modules/ibvs_sim/ibvs_sim.h"
#include "firmwares/rotorcraft/guidance.h"
#include "state.h"
#include <math.h>
#include "cv.h"

#ifndef IBVS_FPS
#define IBVS_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef IBVS_FX
#define IBVS_FX 343.1211       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef IBVS_FY
#define IBVS_FY 348.5053       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef IBVS_X_GAIN
#define IBVS_X_GAIN 0.005		  // Control gain for servoing in the x-direction
#endif

#ifndef IBVS_Y_GAIN
#define IBVS_Y_GAIN -0.005       // Control gain for servoing in the y-direction
#endif

#ifndef IBVS_Z_GAIN
#define IBVS_Z_GAIN 0.001        // Control gain for servoing in the z-direction
#endif

#ifndef IBVS_YAW_GAIN
#define IBVS_YAW_GAIN 0.001        // Control gain for servoing around the z-axis
#endif

#ifndef IBVS_ON
#define IBVS_ON FALSE        // Setting for taking over flight plan by ibvs algorithm
#endif

#ifndef IBVS_SET_GUIDANCE
#define IBVS_SET_GUIDANCE FALSE        // Setting for taking over flight plan by ibvs algorithm
#endif





// Define variables and functions only used in this file
// Goal values for features
uint16_t x_gstar = 0;
uint16_t y_gstar = 0;
float s3_star = 1;
float alpha_star = 0.1; // Orientation in mrad
uint16_t mu_2002star = 500; //
float v_xstar, v_ystar, v_zstar, r_star;

// Camera parameters
const float l1 = IBVS_FX;
const float l2 = IBVS_FY;
uint32_t x_center = 120;
uint32_t y_center = 120;
int i,j,k,l;
struct FloatEulers *euler_angles;
float res1[3][1];
float res2[2][1];
float LX[3][1];
float pv[2][1];
struct Tracked_object object_to_track;
float item;
float x_gain = -1*IBVS_X_GAIN;
float y_gain = -1*IBVS_Y_GAIN;
float z_gain = IBVS_Z_GAIN;
float yaw_gain = IBVS_YAW_GAIN;


static void multiply3x33x1(float mat1[3][3],float mat2[3][1],float res[3][1]);
static void multiply2x33x1(float mat1[2][3],float mat2[3][1],float res[2][1]);
static struct image_t *calc_ibvs_control(struct image_t *img);
static void calc_frame_ibvs_control(struct opticflow_t *opticflow,struct Tracked_object *object_to_track);

// Gains for velocity commands

// Maximum velocity command values

void ibvs_sim_init()
{
	printf("Initializing ibvs_sim \n");
	// Initialize the opticflow calculation
	// There should also be a check if we got a result and if that result is usefull
    // ibvs_got_result = false;

  cv_add_to_device(&IBVS_CAMERA, calc_ibvs_control, IBVS_FPS);

    /*we should probably have something similar to this for IBVS
	#if PERIODIC_TELEMETRY
	  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTIC_FLOW_EST, opticflow_telem_send);
	#endif
    */

}

void ibvs_sim_periodic()
{

}

// This function multiplies mat1 and mat2
// Only works for 3x3 * 3x3, because of the function definition
void multiply3x33x1(float mat1[3][3],float mat2[3][1], float res[3][1])
{
	for(int j1=0;j1<3;j1++){
		for(int i1=0;i1<1;i1++){
			item = 0.0;
			for(int k1=0;k1<3;k1++){
				item += mat1[j1][k1]*mat2[k1][i1];
			}
			res[j1][i1] = item;
		}
	}
}

// This function multiplies mat1 and mat2
// Only works for 2x3 * 3x3, because of the function definition
void multiply2x33x1(float mat1[2][3],float mat2[3][1],float res[2][1])
{
	for(int j1=0;j1<2;j1++){
		for(int i1=0;i1<1;i1++){
			item = 0.0;
			for(int k1=0;k1<3;k1++){
				item += mat1[j1][k1]*mat2[k1][i1];
			}
			res[j1][i1] = item;
		}
	}
}

struct image_t *calc_ibvs_control(struct image_t *img){

	calc_frame_ibvs_control(&opticflow,&object_to_track);

	return img;
}

// Actual IBVS
void calc_frame_ibvs_control(struct opticflow_t *opticflow,struct Tracked_object *object_to_track)
{
	if(opticflow->object_tracking){
		// Init object_tracking
		if(opticflow->ibvs_init){
			// Initialize L_matrix
			for(i=0;i<2;i++){
				for(j=0;j<3;j++){
					object_to_track->L_matrix[i][j] = 0;
				}
			}
			object_to_track->L_matrix[0][0] = l1;
			object_to_track->L_matrix[1][1] = l2;
			opticflow->ibvs_init = false;
			object_to_track->ibvs_go = IBVS_ON;
			object_to_track->set_guidance = IBVS_SET_GUIDANCE;
		}
		object_to_track->corner_loc = calloc(opticflow->nr_of_corners_detected, sizeof(struct Coor_camera));
		// Define corners in the camera frame, with (0,0) at the center of the frame and y positive upwards
		for(i=0;i<opticflow->nr_of_corners_detected;i++){
			object_to_track->corner_loc[i].x = opticflow->fast9_ret_corners[i].x-x_center;
			object_to_track->corner_loc[i].y = -1*(opticflow->fast9_ret_corners[i].y-y_center);
		}
		//printf("%d\n",sizeof(object_to_track->corner_loc));
		// Rotate to virtual camera frame
		euler_angles = stateGetNedToBodyEulers_f();
		float R[3][3] = {
				{cos( euler_angles->phi),0,sin( euler_angles->phi)},
				{sin( euler_angles->theta)*sin( euler_angles->phi), cos( euler_angles->theta),-sin( euler_angles->theta)*cos( euler_angles->phi)},
				{-sin( euler_angles->phi)*cos( euler_angles->theta), sin( euler_angles->theta), cos( euler_angles->theta)*cos( euler_angles->phi)}
		};

		for(k=0;k<opticflow->nr_of_corners_detected;k++){
			float beta = l1*l2*cos( euler_angles->phi)*cos( euler_angles->theta)-
					object_to_track->corner_loc[k].x*l2*sin( euler_angles->theta) +
					object_to_track->corner_loc[k].y*l1*sin( euler_angles->phi)*cos( euler_angles->theta);
			// Fill LX
			LX[0][0] = l2*(int)object_to_track->corner_loc[k].x;
			LX[1][0] = l1*(int)object_to_track->corner_loc[k].y;
			LX[2][0] = l1*l2;

			multiply3x33x1(R,LX,res1);
			multiply2x33x1(object_to_track->L_matrix,res1,res2);
			pv[0][0] = res2[0][0]/beta;
			pv[1][0] = res2[1][0]/beta;
			object_to_track->corner_loc[k].xv = (uint32_t) pv[0][0];
			object_to_track->corner_loc[k].yv = (uint32_t) pv[1][0];
		}

		// Calculate features
		// Average x,y
		int x_sum = 0;
		int y_sum = 0;
		for(i=0;i<opticflow->nr_of_corners_detected;i++){
			x_sum += (int)object_to_track->corner_loc[i].xv;
			y_sum += (int)object_to_track->corner_loc[i].yv;
		}
		int x_g = x_sum/opticflow->nr_of_corners_detected;
		int y_g = y_sum/opticflow->nr_of_corners_detected;
		// Surface area and corners
		float mu_20 = 0;
		float mu_02 = 0;
		float mu_11 = 0;
		for(i=0;i<opticflow->nr_of_corners_detected;i++){
			mu_20 += (object_to_track->corner_loc[i].xv-x_g)*(object_to_track->corner_loc[i].xv-x_g);
			mu_02 += (object_to_track->corner_loc[i].yv-y_g)*(object_to_track->corner_loc[i].yv-y_g);
			mu_11 += (object_to_track->corner_loc[i].xv-x_g)*(object_to_track->corner_loc[i].yv-y_g);
		}
		free(object_to_track->corner_loc);
		float s3 = sqrtf((mu_2002star)/(mu_20 + mu_02));
		float s4 = 0.5 * atanf(2*mu_11/((l2/l1)*mu_20-(l1/l2)*mu_02));

		// Getting velocity commands
		v_xstar = x_gain * (x_g-x_gstar);
		v_ystar = y_gain * (y_g-y_gstar);
		v_zstar = z_gain * (s3-s3_star);
		r_star = yaw_gain * (s4-alpha_star);
		printf("Commands to be given: (vx,vy) = (%f,%f)\n",v_xstar,v_ystar);
		// Sending velocity commands
		if(object_to_track->ibvs_go){
      printf("In if statement set guidance\n");
      guidance_h_mode_changed(10);
      // guidance_v_mode_changed(8);
			// Check what are x and y in the body frame by sending only one of the commands
			// Probably x is forward right?
			// guidance_v_set_guided_vz(v_zstar);
			guidance_h_set_guided_vel(v_xstar,v_ystar);
			// guidance_h_set_guided_heading_rate(r_star);
		}
	}
}


