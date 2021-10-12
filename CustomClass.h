#include <iostream>
#include <string>

#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/standard_main_function.h>
#include <barrett/math/matrix.h> 
#include <math.h>
#include "movingBurt.h"

// Required to run logger
#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()
#include <boost/tuple/tuple.hpp>
#include <barrett/log.h>

// required to get time
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define BILLION  1000000000L;

typedef typename ::barrett::math::Matrix<4,4> Matrix_4x4; //self-def matrix type
typedef typename ::barrett::math::Matrix<4,1> Matrix_4x1; //self-def matrix type
typedef typename ::barrett::math::Matrix<2,1> Matrix_2x1; //self-def matrix type
typedef typename ::barrett::math::Matrix<3,4> Matrix_3x4; //self-def matrix type
typedef typename ::barrett::math::Matrix<3,3> Matrix_3x3; //self-def matrix type
typedef typename ::barrett::math::Matrix<6,3, void> Matrix_6x3xv; //self-def matrix type
typedef typename ::barrett::math::Matrix<3,1> Matrix_3x1; //self-def matrix type
typedef typename ::barrett::math::Matrix<6,4> Matrix_6x4; //self-def matrix type

using namespace barrett;
using detail::waitForEnter;

#ifndef CONTROLLERWARPER_STUFF
#define CONTROLLERWARPER_STUFF

extern std::string fname_rtma;
extern bool fname_init; 

// squreWave
double Pert_arr0[500] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// sigma = 0.04;
double Pert_arr1[500] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0001000, 0.0001000, 0.0001000, 0.0001000, 0.0001000, 0.0002000, 0.0002000, 0.0003000, 0.0004000, 0.0005000, 0.0006000, 0.0007000, 0.0009000, 0.001200, 0.001400, 0.001800, 0.002200, 0.002700, 0.003300, 0.004100, 0.005000, 0.006100, 0.007400, 0.009000, 0.01090, 0.01320, 0.01590, 0.01910, 0.02290, 0.02740, 0.03270, 0.03890, 0.04620, 0.05470, 0.06460, 0.07610, 0.08940, 0.1048, 0.1225, 0.1429, 0.1662, 0.1928, 0.2232, 0.2577, 0.2968, 0.3410, 0.3908, 0.4467, 0.5094, 0.5794, 0.6573, 0.7439, 0.8398, 0.9457, 1.0623, 1.1902, 1.3303, 1.4831, 1.6494, 1.8297, 2.0247, 2.2348, 2.4606, 2.7024, 2.9606, 3.2354, 3.5268, 3.8349, 4.1595, 4.5003, 4.8569, 5.2286, 5.6148, 6.0144, 6.4263, 6.8493, 7.282, 7.7226, 8.1695, 8.6206, 9.0739, 9.5272, 9.9782, 10.4244, 10.8634, 11.2927, 11.7095, 12.1115, 12.4959, 12.8604, 13.2024, 13.5198, 13.8101, 14.0715, 14.302, 14.5001, 14.6641, 14.793, 14.8857, 14.9416, 14.9603, 14.9416, 14.8857, 14.793, 14.6641, 14.5001, 14.302, 14.0715, 13.8101, 13.5198, 13.2024, 12.8604, 12.4959, 12.1115, 11.7095, 11.2927, 10.8634, 10.4244, 9.9782, 9.5272, 9.0739, 8.6206, 8.1695, 7.7226, 7.282, 6.8493, 6.4263, 6.0144, 5.6148, 5.2286, 4.8569, 4.5003, 4.1595, 3.8349, 3.5268, 3.2354, 2.9606, 2.7024, 2.4606, 2.2348, 2.0247, 1.8297, 1.6494, 1.4831, 1.3303, 1.1902, 1.0623, 0.9457, 0.8398, 0.7439, 0.6573, 0.5794, 0.5094, 0.4467, 0.3908, 0.3410, 0.2968, 0.2577, 0.2232, 0.1928, 0.1662, 0.1429, 0.1225, 0.1048, 0.08940, 0.07610, 0.06460, 0.05470, 0.04620, 0.03890, 0.03270, 0.02740, 0.02290, 0.01910, 0.01590, 0.01320, 0.01090, 0.009000, 0.007400, 0.006100, 0.005000, 0.004100, 0.003300, 0.002700, 0.002200, 0.001800, 0.001400, 0.001200, 0.0009000, 0.0007000, 0.0006000, 0.0005000, 0.0004000, 0.0003000, 0.0002000, 0.0002000, 0.0001000, 0.0001000, 0.0001000, 0.0001000, 0.0001000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// sigma = 0.08;
double Pert_arr2[500] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000200, 0.000200, 0.000200, 0.000200, 0.000200, 0.000300, 0.000300, 0.000300, 0.000400, 0.000400, 0.000500, 0.000500, 0.000600, 0.000600, 0.000700, 0.000800, 0.000900, 0.00100, 0.00110, 0.00120, 0.00140, 0.00150, 0.00170, 0.00190, 0.00210, 0.00230, 0.00250, 0.00280, 0.00310, 0.00340, 0.00370, 0.00410, 0.00450, 0.00500, 0.00550, 0.00600, 0.00660, 0.00730, 0.00800, 0.00870, 0.00960, 0.0105, 0.0115, 0.0125, 0.0137, 0.0150, 0.0164, 0.0179, 0.0195, 0.0212, 0.0231, 0.0251, 0.0274, 0.0297, 0.0323, 0.0351, 0.0380, 0.0413, 0.0447, 0.0484, 0.0524, 0.0567, 0.0613, 0.0662, 0.0714, 0.0771, 0.0831, 0.0895, 0.0964, 0.1038, 0.1116, 0.120, 0.1289, 0.1383, 0.1484, 0.1591, 0.1705, 0.1826, 0.1954, 0.209, 0.2234, 0.2386, 0.2547, 0.2717, 0.2897, 0.3086, 0.3287, 0.3497, 0.372, 0.3953, 0.4199, 0.4457, 0.4728, 0.5013, 0.5311, 0.5624, 0.5951, 0.6294, 0.6651, 0.7025, 0.7416, 0.7823, 0.8247, 0.8689, 0.9149, 0.9627, 1.0123, 1.0639, 1.1174, 1.1729, 1.2303, 1.2897, 1.3512, 1.4147, 1.4803, 1.548, 1.6177, 1.6895, 1.7634, 1.8394, 1.9175, 1.9976, 2.0798, 2.164, 2.2502, 2.3383, 2.4285, 2.5205, 2.6143, 2.71, 2.8074, 2.9065, 3.0072, 3.1094, 3.2132, 3.3183, 3.4247, 3.5323, 3.641, 3.7507, 3.8613, 3.9727, 4.0847, 4.1973, 4.3103, 4.4236, 4.537, 4.6504, 4.7636, 4.8766, 4.9891, 5.101, 5.2122, 5.3225, 5.4317, 5.5397, 5.6463, 5.7514, 5.8548, 5.9563, 6.0557, 6.153, 6.248, 6.3404, 6.4302, 6.5172, 6.6012, 6.6822, 6.7599, 6.8342, 6.9051, 6.9723, 7.0358, 7.0954, 7.151, 7.2026, 7.25, 7.2932, 7.3321, 7.3665, 7.3965, 7.422, 7.4429, 7.4592, 7.4708, 7.4778, 7.4802, 7.4778, 7.4708, 7.4592, 7.4429, 7.422, 7.3965, 7.3665, 7.3321, 7.2932, 7.25, 7.2026, 7.151, 7.0954, 7.0358, 6.9723, 6.9051, 6.8342, 6.7599, 6.6822, 6.6012, 6.5172, 6.4302, 6.3404, 6.248, 6.153, 6.0557, 5.9563, 5.8548, 5.7514, 5.6463, 5.5397, 5.4317, 5.3225, 5.2122, 5.101, 4.9891, 4.8766, 4.7636, 4.6504, 4.537, 4.4236, 4.3103, 4.1973, 4.0847, 3.9727, 3.8613, 3.7507, 3.641, 3.5323, 3.4247, 3.3183, 3.2132, 3.1094, 3.0072, 2.9065, 2.8074, 2.71, 2.6143, 2.5205, 2.4285, 2.3383, 2.2502, 2.164, 2.0798, 1.9976, 1.9175, 1.8394, 1.7634, 1.6895, 1.6177, 1.548, 1.4803, 1.4147, 1.3512, 1.2897, 1.2303, 1.1729, 1.1174, 1.0639, 1.0123, 0.9627, 0.9149, 0.8689, 0.8247, 0.7823, 0.7416, 0.7025, 0.6651, 0.6294, 0.5951, 0.5624, 0.5311, 0.5013, 0.4728, 0.4457, 0.4199, 0.3953, 0.372, 0.3497, 0.3287, 0.3086, 0.2897, 0.2717, 0.2547, 0.2386, 0.2234, 0.209, 0.1954, 0.1826, 0.1705, 0.1591, 0.1484, 0.1383, 0.1289, 0.120, 0.1116, 0.1038, 0.0964, 0.0895, 0.0831, 0.0771, 0.0714, 0.0662, 0.0613, 0.0567, 0.0524, 0.0484, 0.0447, 0.0413, 0.0380, 0.0351, 0.0323, 0.0297, 0.0274, 0.0251, 0.0231, 0.0212, 0.0195, 0.0179, 0.0164, 0.0150, 0.0137, 0.0125, 0.0115, 0.0105, 0.00960, 0.00870, 0.00800, 0.00730, 0.00660, 0.00600, 0.00550, 0.00500, 0.00450, 0.00410, 0.00370, 0.00340, 0.00310, 0.00280, 0.00250, 0.00230, 0.00210, 0.00190, 0.00170, 0.00150, 0.00140, 0.00120, 0.00110, 0.00100, 0.000900, 0.000800, 0.000700, 0.000600, 0.000600, 0.000500, 0.000500, 0.000400, 0.000400, 0.000300, 0.000300, 0.000300, 0.000200, 0.000200, 0.000200, 0.000200, 0.000200, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0.000100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// sigma = 0.16;
double Pert_arr3[500] = {0.0283, 0.0295, 0.0306, 0.0318, 0.0331, 0.0344, 0.0357, 0.0371, 0.0385, 0.0400, 0.0415, 0.0431, 0.0448, 0.0465, 0.0482, 0.0500, 0.0519, 0.0538, 0.0558, 0.0579, 0.0600, 0.0622, 0.0644, 0.0668, 0.0692, 0.0716, 0.0742, 0.0768, 0.0796, 0.0824, 0.0853, 0.0882, 0.0913, 0.0944, 0.0977, 0.101, 0.1045, 0.108, 0.1117, 0.1154, 0.1193, 0.1233, 0.1273, 0.1315, 0.1358, 0.1403, 0.1448, 0.1495, 0.1543, 0.1593, 0.1643, 0.1695, 0.1749, 0.1804, 0.186, 0.1917, 0.1977, 0.2037, 0.2099, 0.2163, 0.2229, 0.2296, 0.2364, 0.2434, 0.2506, 0.258, 0.2656, 0.2733, 0.2812, 0.2893, 0.2976, 0.306, 0.3147, 0.3235, 0.3326, 0.3418, 0.3513, 0.3609, 0.3708, 0.3809, 0.3911, 0.4016, 0.4123, 0.4233, 0.4344, 0.4458, 0.4574, 0.4693, 0.4813, 0.4936, 0.5062, 0.5189, 0.532, 0.5452, 0.5587, 0.5724, 0.5864, 0.6007, 0.6151, 0.6299, 0.6449, 0.6601, 0.6756, 0.6914, 0.7074, 0.7236, 0.7402, 0.7569, 0.774, 0.7913, 0.8088, 0.8267, 0.8448, 0.8631, 0.8817, 0.9006, 0.9197, 0.9391, 0.9587, 0.9786, 0.9988, 1.0192, 1.0399, 1.0608, 1.082, 1.1034, 1.1251, 1.147, 1.1692, 1.1916, 1.2142, 1.2371, 1.2602, 1.2836, 1.3072, 1.331, 1.355, 1.3792, 1.4037, 1.4284, 1.4532, 1.4783, 1.5036, 1.5291, 1.5547, 1.5806, 1.6066, 1.6328, 1.6591, 1.6857, 1.7123, 1.7392, 1.7661, 1.7933, 1.8205, 1.8479, 1.8754, 1.9029, 1.9307, 1.9585, 1.9863, 2.0143, 2.0424, 2.0705, 2.0987, 2.1269, 2.1551, 2.1834, 2.2118, 2.2401, 2.2685, 2.2968, 2.3252, 2.3535, 2.3818, 2.4101, 2.4383, 2.4665, 2.4945, 2.5226, 2.5505, 2.5784, 2.6061, 2.6337, 2.6612, 2.6886, 2.7159, 2.7429, 2.7699, 2.7966, 2.8232, 2.8495, 2.8757, 2.9017, 2.9274, 2.9529, 2.9781, 3.0031, 3.0279, 3.0523, 3.0765, 3.1004, 3.124, 3.1473, 3.1702, 3.1928, 3.2151, 3.237, 3.2586, 3.2798, 3.3006, 3.321, 3.3411, 3.3607, 3.3799, 3.3987, 3.4171, 3.435, 3.4525, 3.4696, 3.4861, 3.5022, 3.5179, 3.533, 3.5477, 3.5618, 3.5755, 3.5887, 3.6013, 3.6134, 3.625, 3.6361, 3.6466, 3.6566, 3.666, 3.6749, 3.6833, 3.691, 3.6982, 3.7049, 3.711, 3.7165, 3.7214, 3.7258, 3.7296, 3.7328, 3.7354, 3.7375, 3.7389, 3.7398, 3.7401, 3.7398, 3.7389, 3.7375, 3.7354, 3.7328, 3.7296, 3.7258, 3.7214, 3.7165, 3.711, 3.7049, 3.6982, 3.691, 3.6833, 3.6749, 3.666, 3.6566, 3.6466, 3.6361, 3.625, 3.6134, 3.6013, 3.5887, 3.5755, 3.5618, 3.5477, 3.533, 3.5179, 3.5022, 3.4861, 3.4696, 3.4525, 3.435, 3.4171, 3.3987, 3.3799, 3.3607, 3.3411, 3.321, 3.3006, 3.2798, 3.2586, 3.237, 3.2151, 3.1928, 3.1702, 3.1473, 3.124, 3.1004, 3.0765, 3.0523, 3.0279, 3.0031, 2.9781, 2.9529, 2.9274, 2.9017, 2.8757, 2.8495, 2.8232, 2.7966, 2.7699, 2.7429, 2.7159, 2.6886, 2.6612, 2.6337, 2.6061, 2.5784, 2.5505, 2.5226, 2.4945, 2.4665, 2.4383, 2.4101, 2.3818, 2.3535, 2.3252, 2.2968, 2.2685, 2.2401, 2.2118, 2.1834, 2.1551, 2.1269, 2.0987, 2.0705, 2.0424, 2.0143, 1.9863, 1.9585, 1.9307, 1.9029, 1.8754, 1.8479, 1.8205, 1.7933, 1.7661, 1.7392, 1.7123, 1.6857, 1.6591, 1.6328, 1.6066, 1.5806, 1.5547, 1.5291, 1.5036, 1.4783, 1.4532, 1.4284, 1.4037, 1.3792, 1.355, 1.331, 1.3072, 1.2836, 1.2602, 1.2371, 1.2142, 1.1916, 1.1692, 1.147, 1.1251, 1.1034, 1.082, 1.0608, 1.0399, 1.0192, 0.9988, 0.9786, 0.9587, 0.9391, 0.9197, 0.9006, 0.8817, 0.8631, 0.8448, 0.8267, 0.8088, 0.7913, 0.774, 0.7569, 0.7402, 0.7236, 0.7074, 0.6914, 0.6756, 0.6601, 0.6449, 0.6299, 0.6151, 0.6007, 0.5864, 0.5724, 0.5587, 0.5452, 0.532, 0.5189, 0.5062, 0.4936, 0.4813, 0.4693, 0.4574, 0.4458, 0.4344, 0.4233, 0.4123, 0.4016, 0.3911, 0.3809, 0.3708, 0.3609, 0.3513, 0.3418, 0.3326, 0.3235, 0.3147, 0.306, 0.2976, 0.2893, 0.2812, 0.2733, 0.2656, 0.258, 0.2506, 0.2434, 0.2364, 0.2296, 0.2229, 0.2163, 0.2099, 0.2037, 0.1977, 0.1917, 0.186, 0.1804, 0.1749, 0.1695, 0.1643, 0.1593, 0.1543, 0.1495, 0.1448, 0.1403, 0.1358, 0.1315, 0.1273, 0.1233, 0.1193, 0.1154, 0.1117, 0.108, 0.1045, 0.101, 0.0977, 0.0944, 0.0913, 0.0882, 0.0853, 0.0824, 0.0796, 0.0768, 0.0742, 0.0716, 0.0692, 0.0668, 0.0644, 0.0622, 0.0600, 0.0579, 0.0558, 0.0538, 0.0519, 0.0500, 0.0482, 0.0465, 0.0448, 0.0431, 0.0415, 0.0400, 0.0385, 0.0371, 0.0357, 0.0344, 0.0331, 0.0318, 0.0306, 0.0295};

int day; // really need this??? doubt it! 

// double getTime(){
// 	struct timespec t;
// 	if( clock_gettime( CLOCK_REALTIME, &t) == -1 ) {
//       perror( "clock gettime" );
//       exit( EXIT_FAILURE );
//     } 
// 	double t_val, tnsec; 
// 	t_val = t.tv_sec + t.tv_nsec/BILLION;
// 	return t_val;
// }

double GetAbsTime(void)
{
	//WIN: returns a seconds timestamp for a system counter
	#ifdef _UNIX_C
	
	struct timeval tim;
	if (gettimeofday(&tim, NULL) == 0)
	{
		double t = double(tim.tv_sec-(24*3600*day)) + (double(tim.tv_usec) / 1000000.0);
	return t;
	}
	else{
		return 0.0;
	}
	#else
		LONGLONG current_time;
		QueryPerformanceCounter((LARGE_INTEGER *)&current_time);
		return (double)current_time / win_counter_freq;
		cout<<"windows time: "<<(current_time/win_counter_freq)<<endl;
	#endif
}


// int getTime(double * sec, long * nsec ){
// 	struct timespec t;
// 	if( clock_gettime( CLOCK_REALTIME, &t) == -1 ) {
//       perror( "clock gettime" );
//       exit( EXIT_FAILURE );
//     } 
	
// 	*sec = t.tv_sec;
// 	*nsec = t.tv_nsec;
// 	return 1;
// }

template<size_t DOF>
class JointControlClass : public systems::System{ 
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<double> timeInput;
	Input<jp_type> wamJPInput;
	Input<jv_type> wamJVInput;
	Input<cp_type> wamCPInput;
	Input<cv_type> wamCVInput;
	Output<jt_type> wamJTOutput;
	Output<cf_type> wamCFPretOutput;
	Output<int> wamIterationOutput;
    Output<int> wamRDTOutput; // read-time to synchronize stand-alone data and wam data
	jp_type input_q_0;
	cp_type input_x_0;
	bool    setx0flag;
	int     x0iterator; 			// from 0 to 512
	cp_type input_x0_stt;
	cp_type input_x0_edn;
	systems::Ramp time;
	Output<int>	wamTaskState;

protected:
	typename Output<jt_type>::Value* outputValue1; 
	typename Output<int>::Value* outputValue2;
	typename Output<cf_type>::Value* outputValue3; 	// should be CFPretOutput 
	typename Output<int>::Value* outputValue4; 		// should be IterationOutput
	typename Output<int>::Value* outputValue5; 		// should be holeOutput
	
	systems::Wam<DOF>& wam;
	ProductManager& pm;
	
	
public:
	explicit JointControlClass(ProductManager& pm, Matrix_4x4 K_q, Matrix_4x4 B_q, Matrix_3x3 K_x, Matrix_3x3 B_x,
	 Matrix_4x4 K_q1, Matrix_4x4 B_q1,
	 jp_type input_q_0, cp_type input_x_0, 
	 systems::Wam<DOF>& wam, const std::string& sysName = "JointControlClass") :
		pm(pm), systems::System(sysName), wam(wam), timeInput(this),
		wamJPInput(this), wamJVInput(this), wamCPInput(this), wamCVInput(this),
		time(pm.getExecutionManager(), 1.0), 
		wamJTOutput(this, &outputValue1), 
		wamRDTOutput(this, &outputValue2),
		wamCFPretOutput(this, &outputValue3), 
		wamIterationOutput(this, &outputValue4),
		wamTaskState(this, &outputValue5),

		K_q(K_q), B_q(B_q), K_q1(K_q1), B_q1(B_q1),
		input_q_0(input_q_0), K_x(K_x), B_x(B_x), input_x_0(input_x_0){
			loop_iteration = 0;
			loop_itMax = 500*0.1; 	// freq*s
      		rampTime = 2.5;
			rdt = 0;
		 	if_set_JImp = false; 	// 
			if_set_Imp = false;
			K_qQuantum = (K_q1 - K_q0)/double(loop_itMax);
			iteration_MAX = 4;
			iteration = 0;
			task_state = 0;
			setx0flag = false; 
			x0iterator = 0;
			if_Jacobbian_update = true;
      		//printf("K_qQ is: %.3f, %.3f, %.3f, %.3f\n", K_qQuantum(0,0), K_qQuantum(1,1), K_qQuantum(2,2), K_qQuantum(3,3));
			struct timeval tim0;
			if (gettimeofday(&tim0, NULL) == 0){
				day = int(tim0.tv_sec/(24*3600))-1;
			}
		}

	virtual ~JointControlClass() { this->mandatoryCleanUp(); }

	void setImpedance(Matrix_3x3 K_x1, Matrix_3x3 B_x1){ 
		printf("x0: %f, %f, %f", input_x_0[0], input_x_0[1], input_x_0[2]);
		K_x = K_x1;
		B_x = B_x1; 
	}

	void setImpedanceWait(Matrix_3x3 K_x1, Matrix_3x3 B_x1){ 
		K_x0 = K_x1;
		B_x0 = B_x1; 
	}

	void updateImpedanceWait(){ 
    //	printf("release now! \n");
		K_x = K_x0;
		B_x = B_x0; 
	}

	void setTaskState(int ts){
		task_state = ts;
	}

	void setJointImpedance(Matrix_4x4 K_q1, Matrix_3x3 B_q1){ //higher one, ST_HOLD
		// how to avoid continuous increasing?
		K_q = K_q1;
		B_q = B_q1;
	}

	void setImpedance_inc(Matrix_3x3 K_x1, Matrix_3x3 B_x1){ 
		K_xQuantum = (K_x1 - K_x)/loop_itMax;
		B_xQuantum = (B_x1 - B_x)/loop_itMax;
		if_set_Imp = true;
	}

	void setJointImpedance_inc(Matrix_4x4 K_q1, Matrix_3x3 B_q1){ //higher one, ST_HOLD
		// how to avoid continuous increasing?
		K_qQuantum = (K_q1 - K_q)/loop_itMax;
		B_qQuantum = (B_q1 - B_q)/loop_itMax;
		if_set_JImp = true;
	}

	void setx0(cp_type center_pos){
		printf("set center to: %f, %f, %f\n", center_pos[0], center_pos[1], center_pos[2]);
		input_x_0 = center_pos;
	}

	void setx0Gradual(cp_type center_pos){
		//input_x0_stt = input_x_0;
		input_x0_stt = wamCPInput.getValue();
		input_x0_edn = center_pos;
		x0iterator = 1;
		setx0flag = true; 
	}

	void setq0(jp_type center_pos){
		input_q_0 = center_pos;
	}

	int get_rdt(){
		return rdt;
	}

	int update_input_time0(){
		input_time0 = input_time;
		return 1;
	}

	int setpretAmp(){ // used in stochastic perturbation
		pretAmplitude_x = 4.0;
		pretAmplitude_y = 4.0;
	}

	int resetpretAmp(){ // used in stochastic perturbation
		pretAmplitude_x = 0.0;
		pretAmplitude_y = 0.0;
	}

	int resetpretFlip(bool flip){
		pert_flip = flip; // 0 for no pert, 1 for pert
	}

	int setFoffset(double Fy){
		f_offset[0] = 0;
		f_offset[1] = Fy;
		f_offset[2] = 0;
	}

	int enablePert(){
		pert_enable = true;
	}
	int disablePert(){
    if (~atpert){ // to insure the perturb is not going to disrupted
		pert_enable = false;
    }
	}
	bool getAtpert(){
		return atpert;
	}
	int enablePertCount(){
		pert_count_enable = true;
	}
	int disablePertCount(){
		pert_count_enable = false;
		if_pert_finish = false;
		iteration = 0;
	}
	int resetPertCount(){
		iteration = 0;
	}
	int setPertMag(double mag){
		pert_mag = mag;
		return 1;
	}

	int setPertPositionMag(double mag){
		pert_pos_mag = mag; 
		printf("\n position perturbation %f\n", mag);
		return 1;
	}

	int setPertTime(int time){
		pert_time = time;
		return 1;
	}

	bool getPertFinish(){
		return if_pert_finish;
	}

	int setUpdateJaccobian(bool ifUpdate){
		if_Jacobbian_update = ifUpdate;
		return 1;
	}

	int setPulsePert(bool ispulse){
		is_pulse_pert = ispulse; 
		return 1;
	}

	int gettime(double * t1, double * t2){
    //printf("Within gettime \n");
		*t1 = input_time;
		*t2 = ernie_time; 
    //printf("inputT: %f,      ernieT: %f   \n", input_time, ernie_time);
		return 1;
	}

protected:
	double	input_time;
	double 	ernie_time; 	 // the computer time
	// long 	ernie_time_nsec;
	double  input_time0;	 // give a time offset when increase
	double	input_iteration;
	double 	pretAmplitude_x;
	double 	pretAmplitude_y;
	double 	rampTime;
	int 	iteration;
	int 	iteration_MAX;
	int 	loop_iteration;  // these are my code different from James
	int		loop_itMax;
	int		task_state;
	cf_type input_prevPret;
	cf_type prevPret;
	jp_type input_q;
	jv_type input_q_dot;
	cp_type input_x;
	cv_type input_x_dot;
	jt_type torqueOutput;
	cf_type forceOutput;
	jt_type force2torqueOutput;
	cf_type f_pretOutput;
	double output_iteration;
    double pert_mag; //impulse-perturbation magnitude (Newton)
	double pert_pos_mag; //impulse-perturbation magnitude (m)
	int 	rdt; 
	bool 	if_set_JImp;
	bool 	if_set_Imp;
	bool    pert_flip; 
	bool 	pert_enable;
	bool    pert_count_enable;
    bool  	atpert; 
	bool  	if_pert_finish;
	bool    if_Jacobbian_update;
	bool 	is_pulse_pert;
	int 	pert_time; // randomize a time in the burtRTMA.h to cound down perturbation.

	// Initialize variables 
	Matrix_4x4 K_q;
	Matrix_4x4 B_q;
	Matrix_4x4 K_q0; 	//lower value of K_q, because K_q would be set to 0 when forceMet.
	Matrix_4x4 B_q0;
	Matrix_4x4 K_q1;	//higher value of K_q
	Matrix_4x4 B_q1;
	Matrix_4x4 K_qQuantum; // each small part
	Matrix_4x4 B_qQuantum;
	Matrix_3x3 K_xQuantum;
	Matrix_3x3 B_xQuantum;
	Matrix_3x3 K_x; 
	Matrix_3x3 B_x; 
	Matrix_3x3 K_x0;	// wait impedance, after enable release, it will become current impedance. 
	Matrix_3x3 B_x0; 
	Matrix_3x1 x_0; 	//Matrix_4x1 x_0;
	Matrix_3x1 x;   	//Matrix_4x1 x; 
	Matrix_3x1 x_dot;	//Matrix_4x1 x_dot;
	Matrix_4x1 q_0;
	Matrix_4x1 q; 
	Matrix_4x1 q_dot;
	Matrix_4x1 tau;
	Matrix_4x1 tau_q;
	Matrix_4x1 tau_x;
	Matrix_4x1 tau_pret;
	//Matrix_2x1 callRand;
	Matrix_3x1 f_pret;
	Matrix_6x4 J_tot;
	Matrix_3x4 J_x;
	Matrix_3x1 f_offset;

	virtual void operate() {
		
		input_time = timeInput.getValue();
		ernie_time = GetAbsTime();
		input_q = wamJPInput.getValue();
 		input_q_dot = wamJVInput.getValue();
		input_x = wamCPInput.getValue();
 		input_x_dot = wamCVInput.getValue();
 	
		// Custom torque calculations
		// Convert imputs to matrix type
		// Define q
		q[0] = input_q[0];
		q[1] = input_q[1];
		q[2] = input_q[2];
		q[3] = input_q[3];

		// Define q_0
		q_0[0] = input_q_0[0]; 
		q_0[1] = input_q_0[1]; 
		q_0[2] = input_q_0[2]; 
		q_0[3] = input_q_0[3]; 

		// Define q_dot
		q_dot[0] = input_q_dot[0];
		q_dot[1] = input_q_dot[1];
		q_dot[2] = input_q_dot[2];
		q_dot[3] = input_q_dot[3];

		// x_0 get value
		x_0[0] = input_x_0[0];
		x_0[1] = input_x_0[1];
		x_0[2] = input_x_0[2];

		// x get value
		x[0] = input_x[0];
		x[1] = input_x[1];
		x[2] = input_x[2];

		//x_dot get value
		x_dot[0] = input_x_dot[0];
		x_dot[1] = input_x_dot[1];
		x_dot[2] = input_x_dot[2];

		// Import Jacobian
		
		if(if_Jacobbian_update){
      		J_tot.block(0,0,6,4) = wam.getToolJacobian(); // Entire 6D Jacobian
			J_x.block(0,0,3,4) = J_tot.block(0,0,3,4); // 3D Translational Jacobian

		}

		// Joint impedance controller
		if ((input_time-input_time0) < rampTime ) {
			tau_q = ((input_time-input_time0)/rampTime)*K_q*(q_0 - q) - B_q*(q_dot);
		}
		else {
			tau_q = K_q*(q_0 - q) - B_q*(q_dot);
		}

		// End-effector impedance controller
		if ((input_time-input_time0) < rampTime ) {
			tau_x = J_x.transpose()*(((input_time-input_time0)/rampTime)*K_x*(x_0 - x) - B_x*(x_dot)); 	
		}
		else {
			tau_x = J_x.transpose()*(K_x*(x_0 - x) - B_x*(x_dot)); 
		}
		// Control Law Implamentation

		// iteration_MAX - stochastic perturbation
		if (is_pulse_pert) 
		{ // inpulse perturbation here

			if (pert_count_enable || atpert)
			{ 
				// if starting count, or already perturb the first pulse:
				iteration++;
			}
    //if ((iteration <= pert_time) || (iteration >= pert_time + 150))
	if ((iteration <= pert_time) || (iteration >= pert_time + 500))
    //if ((iteration <= pert_time) || (iteration >= pert_time + 40))
    //if ((iteration <= pert_time) || (iteration >= pert_time + 20)) // good short perturbation
			  { // no pulse --- perturbation duration
//      if ((iteration <= pert_time) || (iteration >= pert_time + 400)){ // no pulse
//      if ((iteration <= pert_time) || (iteration >= pert_time + 1000)){ // no pulse
       			f_pretOutput[0] = 0;
        		f_pretOutput[1] = 0;
       			f_pretOutput[2] = 0;
            	atpert = false;
      		}
			else 
			{ 	// halve pulse
				f_pretOutput[0] = 0;
				//f_pretOutput[1] = pert_mag;
				f_pretOutput[1] = Pert_arr1[iteration-pert_time-1];
				f_pretOutput[2] = 0; 

				x_0[0] = input_x_0[0];
				x_0[1] = input_x_0[1] + pert_pos_mag;
				x_0[2] = input_x_0[2];
        		atpert = true;
      		        
			}

			if (iteration >= pert_time + 500) 
			{
				if_pert_finish = true;
			}

			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2];
		}
		else{ // stochastic perturbation here
		if (iteration < iteration_MAX){
			iteration++;
			f_pretOutput[0] = prevPret[0];
			f_pretOutput[1] = prevPret[1];
			f_pretOutput[2] = prevPret[2];

			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2]; 
		}
		// More than iteration_MAX iterations since last update get new preturbaiton amplitude
		else if (iteration >= iteration_MAX) {
			// Reset the count
			iteration = 1;
			f_pretOutput.setRandom();
    		f_pretOutput[0] = f_pretOutput[0];
    		f_pretOutput[1] = f_pretOutput[1];
    		f_pretOutput[2] = f_pretOutput[2];
			f_pretOutput[2] = 0.0;

			// Make Preturbation unifore amplitude
			//pretAmplitude_x = 0.0;
			if (f_pretOutput[0] >= 0 ) {
				f_pretOutput[0] = pretAmplitude_x;
			} else if (f_pretOutput[0] < 0 ){
				f_pretOutput[0] = -pretAmplitude_x;
			}
			//pretAmplitude_y = 0.0;
			if (f_pretOutput[1] >= 0 ) {
				f_pretOutput[1] = pretAmplitude_y;
			} else if (f_pretOutput[1] < 0 ){
				f_pretOutput[1] = -pretAmplitude_y;
			}

			// only in x, y direction
			// f_pretOutput[0] = 0;
			// f_pretOutput[1] = 0;
		
			f_pret[0] = f_pretOutput[0];
			f_pret[1] = f_pretOutput[1];
			f_pret[2] = f_pretOutput[2]; 

		}
		}
		if (setx0flag) { // let the shift finished in 128 iterations
			input_x_0 = (input_x0_edn - input_x0_stt)/512*(x0iterator+1) + input_x0_stt;
			x0iterator++;
			if (x0iterator>511) {
				setx0flag = false;
				}
		}
		tau_pret = J_x.transpose()*(f_pret);

    
		// Sum torque commands
		tau = tau_q + tau_x + tau_pret;
    	//tau = tau_x + tau_pret;
		// Save outputs
		// Save outputs
		prevPret[0] = f_pretOutput[0];
		prevPret[1] = f_pretOutput[1];
		prevPret[2] = f_pretOutput[2]; 

		torqueOutput[0] = tau[0];
		torqueOutput[1] = tau[1];
		torqueOutput[2] = tau[2];
		torqueOutput[3] = tau[3];

		// update readtime variable
		rdt++;

		this->outputValue1->setData(&torqueOutput);
		this->outputValue2->setData(&rdt);
		this->outputValue3->setData(&f_pretOutput);
		this->outputValue4->setData(&iteration);
		this->outputValue5->setData(&task_state);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JointControlClass);
};


template<size_t DOF>
class ControllerWarper{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	private:
	// wam variables
	ProductManager& pm;
	systems::Wam<DOF>& wam;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::Summer<jt_type, 2> jtSum;
	
	// controller variables
	jp_type input_q_00;
	cp_type input_x_00;
	cp_type	center_pos; 	// keep this inorder to same with the old code.
	cp_type center_pos0; 	// the reference center [-0.448, 0.418, 0]
	Matrix_4x4 K_q0;		// free-moving stiffness and damping
	Matrix_4x4 B_q0; 
	Matrix_3x3 K_x0;			
	Matrix_3x3 B_x0;
	Matrix_4x4 K_q1;		// locked stiffness and damping
	Matrix_4x4 B_q1;
	Matrix_3x3 K_x1; 		
	Matrix_3x3 B_x1;
	bool forceMet;
	bool TrackRef;
	public:
	JointControlClass<DOF> jj;
	ControllerWarper(ProductManager& pm, systems::Wam<DOF>& wam, 
		Matrix_4x4 K_q0, Matrix_4x4 K_q1, Matrix_4x4 B_q0, Matrix_4x4 B_q1,
		Matrix_3x3 K_x0, Matrix_3x3 K_x1, Matrix_3x3 B_x0, Matrix_3x3 B_x1,
		jp_type input_q_00, cp_type input_x_00):
	pm(pm), wam(wam),
	K_q0(K_q0), B_q0(B_q0), K_q1(K_q1), B_q1(B_q1),
	K_x0(K_x0), B_x0(B_x0), K_x1(K_x1), B_x1(B_x1),
	input_q_00(input_q_00), input_x_00(input_x_00), center_pos(input_x_00), center_pos0(input_x_00),
	jj(pm, K_q0, B_q0, K_x0, B_x0, K_q1, B_q1, input_q_00, input_x_00, wam),
	forceMet(false), TrackRef(false){
	// after initilization, mvoeTo
	printf("Move to joint controller position, in CustomClass:: Controller Wrapper.");
	wam.moveTo(jj.input_q_0);
	barrett::btsleep(5.0);
  //pert_on = 0;
	}

	~ControllerWarper(){}

	bool init() {
		wam.gravityCompensate();
  		pm.getSafetyModule()->setVelocityLimit(2.5);  // Was 2.5 (Hongwei, 9/5/2019)
  		pm.getSafetyModule()->setTorqueLimit(4.5);    // Was 4.5 (Hongwei, 9/5/2019)

  		wam.moveTo(jj.input_q_0); //center_pos
		TrackRef = false;
  		barrett::btsleep(0.5);
		
  		wam.idle();
  		//printf("Begin idle \n");
  		return true;
	}

	void setCenter_endpoint(cp_type newCenter) {
  		//printf("Enter function: setCenter.");
		center_pos = newCenter; 
		jj.setx0(center_pos);
	}

	void setCenter_joint(double *jointCenter) {
		// copy the value of each element
		input_q_00[0] = jointCenter[0];
		input_q_00[1] = jointCenter[1];
		input_q_00[2] = jointCenter[2];
		input_q_00[3] = jointCenter[3]; 
		// set to jj
		jj.setq0(input_q_00);
	}

	void startController(){
		// connect 
	}

	int setK1(double K_input){
		K_x1(0,0) = K_input;
		K_x1(1,1) = K_input;
		K_x1(2,2) = K_input;
		//printf("cw: setKx to %f \n", K_input);
		return 1;
	}

	int setB1(double B_input){
		B_x1(0,0) = B_input;
		B_x1(1,1) = B_input;
		B_x1(2,2) = B_input;
		//printf("cw: setBx to %f \n", B_input);
		return 1;
	}

	void setForceMet(bool wasMet){
		forceMet = wasMet;
		
		if (wasMet){
			// change the K_q to a low value here
			jj.setImpedanceWait(K_x0, B_x0);
			printf("\nset impedance to: %.3f, %.3f, %.3f\n", K_x0(0,0), K_x0(1,1), K_x0(2,2));
		}
		else {
			// change the K_q to a high value here
			jj.update_input_time0();			// initializing ramp
			jj.setImpedance(K_x1, B_x1);
			printf("\nset impedance to: %.3f, %.3f, %.3f\n", K_x1(0,0), K_x1(1,1), K_x1(2,2));
			
		}
	}

	void moveToq0(void ){
		wam.moveTo(jj.input_q_0); //center_pos
		TrackRef = false;
  		barrett::btsleep(0.5);
	}

	void connectForces() {
    	printf("Enter function: connectForces.");
  		barrett::systems::modXYZ<cp_type> mod_axes;
		systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
		systems::connect(jj.time.output, jj.timeInput);
 		systems::connect(wam.jpOutput, jj.wamJPInput);
 		systems::connect(wam.jvOutput, jj.wamJVInput);
		systems::connect(wam.toolPosition.output, jj.wamCPInput);	
 		systems::connect(wam.toolVelocity.output, jj.wamCVInput);
		// track reference
		// wam.trackReferenceSignal(jtSum.output);
		wam.trackReferenceSignal(jj.wamJTOutput);
		TrackRef = true;
  		BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
	}

	void trackSignal(){ //enable the tracking signal out of CustomClass
		wam.trackReferenceSignal(jj.wamJTOutput);
		TrackRef = true;
  		BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
		printf("Track Ref! \n");
	}

	bool isTrackRef(){
		return TrackRef;
	}

};

template<size_t DOF>
class LoggerClass{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
  	ProductManager& pm;
	systems::Wam<DOF>& wam;
	//systems::Ramp time;
	systems::TupleGrouper<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type, int, int, int> tg;
	typedef boost::tuple<double, jp_type, jv_type, cp_type, cv_type, jt_type, cf_type, int, int, int> tuple_type;
	const size_t PERIOD_MULTIPLIER;
	char *tmpFile;
	const char* tmpFileName;
	systems::PeriodicDataLogger<tuple_type> logger;
  ControllerWarper<DOF> & controller1; 
	explicit LoggerClass(ProductManager& pm, systems::Wam<DOF>& wam, char *fname, char *tmpfname, ControllerWarper<DOF>& controller):
  				PERIOD_MULTIPLIER(1), 
				//time(pm.getExecutionManager(), 1.0),
				tmpFile(tmpfname),
  				logger(pm.getExecutionManager(), new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()), PERIOD_MULTIPLIER),
				pm(pm), wam(wam), 
  				controller1(controller),
				tmpFileName(fname){
    	printf("Entered the class Constructor!\n");
		printf("Fished the Constructor.");
	}
	
	virtual ~LoggerClass(){}

public:
	void datalogger_connect(){
		printf("Start connecting! \n");
		systems::connect(controller1.jj.time.output, tg.template getInput<0>());
		systems::connect(wam.jpOutput, tg.template getInput<1>());
		systems::connect(wam.jvOutput, tg.template getInput<2>());
		systems::connect(wam.toolPosition.output, tg.template getInput<3>());
		systems::connect(wam.toolVelocity.output, tg.template getInput<4>());
		systems::connect(controller1.jj.wamJTOutput, tg.template getInput<5>());
		systems::connect(controller1.jj.wamCFPretOutput, tg.template getInput<6>());
		systems::connect(controller1.jj.wamIterationOutput, tg.template getInput<7>());
		systems::connect(controller1.jj.wamRDTOutput, tg.template getInput<8>());
		systems::connect(controller1.jj.wamTaskState, tg.template getInput<9>());
	}

	void datalogger_start(){
		printf("Start timming! \n");
		controller1.jj.time.start();
		printf("Connect input! \n");
		connect(tg.output, logger.input);
		printf("Logging started.\n");
	}
	void datalogger_end(){
		logger.closeLog();
		printf("Logging stopped.\n");
		log::Reader<tuple_type> lr(tmpFile);
		if (fname_init){
			lr.exportCSV(fname_rtma.c_str());
			printf("save to: %s",fname_rtma.c_str());
		}
		else{
			lr.exportCSV(tmpFileName);
			printf("Output written to %s.\n", tmpFileName);
		}
		
		std::remove(tmpFile);
	}
};

#endif
