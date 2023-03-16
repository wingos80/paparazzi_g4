/*
 * Copyright (C) 2023 Avani Patidar <av.patidar1998@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/obstacle_detect/obstacle_detect.h"
 * @author Avani Patidar <av.patidar1998@gmail.com>
 * Module for detecting the obstacles during flight time
 */

#ifndef OBSTACLE_DETECT_H
#define OBSTACLE_DETECT_H

extern void obst_detect_init(void);
extern void obst_detect_1Hz(void);
extern void obst_detect_15Hz(void);

#endif  // OBSTACLE_DETECT_H
