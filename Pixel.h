#pragma once
#ifndef PIXEL_H
#define PIXEL_H

#include <k4a/k4a.hpp>
#include <iostream>
#include <array>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

struct Pixel
{
	uint8_t Blue;
	uint8_t Green;
	uint8_t Red;
	uint8_t Alpha;
};

struct CloudPoint
{
	float x;
	float y;
	float z;
};
using DepthPixel = uint16_t;

#endif