#include <k4a/k4a.hpp>
#include <iostream>
#include <array>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "DepthPixelColorizer.h"

inline std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode) {
	switch (depthMode)
	{
	case K4A_DEPTH_MODE_NFOV_2X2BINNED:
		return { (uint16_t)500, (uint16_t)5800 };
	case K4A_DEPTH_MODE_NFOV_UNBINNED:
		return { (uint16_t)500, (uint16_t)4000 };
	case K4A_DEPTH_MODE_WFOV_2X2BINNED:
		return { (uint16_t)250, (uint16_t)3000 };
	case K4A_DEPTH_MODE_WFOV_UNBINNED:
		return { (uint16_t)250, (uint16_t)2500 };
	case K4A_DEPTH_MODE_PASSIVE_IR:
	default:
		throw std::logic_error("Invalid depth mode!");
	}
}

inline void ColorConvertHSVtoRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b) {
	if (s == 0.0f) {
		//gray
		out_r = out_g = out_b = v;
		return;
	}

	h = fmodf(h, 1.0f) / (60.0f / 360.0f);
	int i = (int)h;
	float f = h - (float)i;
	float p = v * (1.0f - s);
	float q = v * (1.0f - s * f);
	float t = v * (1.0f - s * (1.0f - f));

	switch (i)
	{
	case 0: out_r = v; out_g = t; out_b = p; break;
	case 1: out_r = q; out_g = v; out_b = p; break;
	case 2: out_r = p; out_g = v; out_b = t; break;
	case 3: out_r = p; out_g = q; out_b = v; break;
	case 4: out_r = t; out_g = p; out_b = v; break;
	case 5: default: out_r = v; out_g = p; out_b = q; break;
	}
}

Pixel DepthPixelColorizer::ColorizeBlueToRed(const DepthPixel &depthPixel, const DepthPixel &min, const DepthPixel &max) {
	constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();

	Pixel result = { uint8_t(0), uint8_t(0), uint8_t(0), PixelMax };

	if (depthPixel == 0)
	{
		return result;
	}

	uint16_t clampedValue = depthPixel;
	clampedValue = std::min(clampedValue, max);
	clampedValue = std::max(clampedValue, min);

	float hue = (clampedValue - min) / static_cast<float>(max - min);

	constexpr float range = 2.f / 3.f;
	hue *= range;
	hue = range - hue;

	float fRed = 0.f;
	float fGreen = 0.f;
	float fBlue = 0.f;
	ColorConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);

	result.Red = static_cast<uint8_t>(fRed * PixelMax);
	result.Green = static_cast<uint8_t>(fGreen * PixelMax);
	result.Blue = static_cast<uint8_t>(fBlue * PixelMax);

	return result;
}

void ColorizeDepthImage(const k4a::image &depthImage,
	DepthPixelColorizer visualizationFn,
	std::pair<uint16_t, uint16_t> expectedValueRange,
	std::vector<Pixel> *buffer)
{
	const k4a_image_format_t imageFormat = depthImage.get_format();
	if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
	{
		throw std::logic_error("Attempted to colorize a non-depth image!");
	}

	const int width = depthImage.get_width_pixels();
	const int height = depthImage.get_height_pixels();

	buffer->resize(static_cast<size_t>(width * height));

	const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			const size_t currentPixel = static_cast<size_t>(h * width + w);
			(*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],
				expectedValueRange.first,
				expectedValueRange.second);
		}
	}

}