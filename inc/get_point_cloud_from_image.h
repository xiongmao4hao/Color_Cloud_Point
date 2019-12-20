#pragma once
#ifndef GET_POINT_CLOUD_FROM_IMAGE_H
#define GET_POINT_CLOUD_FROM_IMAGE_H

#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.h>
#include <k4arecord/types.h>
#include <k4arecord/playback.h>
#include <k4abt.h>


void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table);
void generate_point_cloud(const k4a_calibration_t* calibration, const k4a_image_t depth_image, const k4a_image_t xy_table, k4a_image_t point_cloud, int* point_count, const char* file_path, int rotation_flag);


#endif
