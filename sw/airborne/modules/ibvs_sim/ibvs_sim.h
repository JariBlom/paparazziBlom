/*
 * Copyright (C) Jari Blom
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ibvs_sim/ibvs_sim.h"
 * @author Jari Blom
 * Simulate Image Based Visual Servoing
 */

#ifndef IBVS_SIM_H
#define IBVS_SIM_H

#include "math/pprz_geodetic_double.h"

extern void ibvs_sim_init();
extern void ibvs_sim_periodic();

// Define Camera coordinate system
struct Coor_camera{
	float x; // when both are between 0 and 1 they are in view
	float y;
};

#endif

