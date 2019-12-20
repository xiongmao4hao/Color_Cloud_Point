#pragma once
#ifndef PARAMETERS_H
#define PARAMETERS_H

float rotation_matrix_1t0_inv[3][3] = { 0.24131225f, -0.02231422f,  0.97019095f,
										0.05137034f,  0.99862767f,  0.01019109f,
									   -0.96908693f,  0.04737981f,  0.24212738f };

float rotation_matrix_0_to_world_inv[3][3] = { -0.06316f, -0.9950f, -0.0765f,
												0.7416f,   0.0045f, -0.6710f,
												0.6681f,  -0.0991f,  0.7376f };

float translation_matrix_1t0[3] = { -587.5420f, 11.3095f, 72.2416f };

float translation_matrix_0_to_world[3] = { 1293.1f, 1780.8f, -2297.5f };

#endif

