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
// sigma = 0.16
double Pert_arr1[500] = {0.007576,0.007877,0.008189,0.008512,0.008846,0.009192,0.009550,0.009920,0.010303,0.010699,0.011109,0.011533,0.011970,0.012423,0.012891,0.013374,0.013873,0.014388,0.014921,0.015470,0.016038,0.016623,0.017227,0.017851,0.018494,0.019157,0.019841,0.020546,0.021273,0.022022,0.022794,0.023590,0.024409,0.025253,0.026121,0.027016,0.027937,0.028884,0.029860,0.030863,0.031895,0.032956,0.034047,0.035169,0.036323,0.037508,0.038726,0.039977,0.041262,0.042582,0.043937,0.045328,0.046756,0.048221,0.049725,0.051267,0.052849,0.054471,0.056135,0.057840,0.059587,0.061378,0.063213,0.065092,0.067017,0.068988,0.071005,0.073071,0.075184,0.077347,0.079560,0.081822,0.084137,0.086503,0.088922,0.091394,0.093920,0.096501,0.099137,0.101830,0.104579,0.107386,0.110251,0.113174,0.116157,0.119200,0.122303,0.125468,0.128695,0.131984,0.135335,0.138750,0.142230,0.145773,0.149382,0.153056,0.156796,0.160602,0.164474,0.168414,0.172422,0.176497,0.180640,0.184851,0.189132,0.193481,0.197899,0.202386,0.206943,0.211569,0.216265,0.221031,0.225866,0.230771,0.235746,0.240790,0.245904,0.251088,0.256340,0.261662,0.267052,0.272511,0.278037,0.283632,0.289294,0.295023,0.300818,0.306679,0.312606,0.318597,0.324652,0.330771,0.336953,0.343196,0.349501,0.355865,0.362289,0.368771,0.375311,0.381907,0.388558,0.395263,0.402021,0.408831,0.415691,0.422600,0.429557,0.436561,0.443609,0.450700,0.457833,0.465007,0.472219,0.479468,0.486752,0.494070,0.501419,0.508799,0.516206,0.523639,0.531096,0.538575,0.546074,0.553592,0.561124,0.568671,0.576229,0.583796,0.591371,0.598949,0.606531,0.614112,0.621691,0.629265,0.636832,0.644389,0.651934,0.659464,0.666977,0.674470,0.681941,0.689387,0.696805,0.704193,0.711548,0.718868,0.726149,0.733390,0.740587,0.747738,0.754840,0.761890,0.768886,0.775825,0.782705,0.789522,0.796274,0.802958,0.809572,0.816112,0.822578,0.828964,0.835270,0.841493,0.847629,0.853676,0.859633,0.865495,0.871262,0.876930,0.882497,0.887960,0.893318,0.898568,0.903707,0.908734,0.913646,0.918441,0.923116,0.927671,0.932102,0.936409,0.940588,0.944638,0.948558,0.952345,0.955997,0.959514,0.962893,0.966134,0.969233,0.972191,0.975005,0.977675,0.980199,0.982575,0.984804,0.986884,0.988813,0.990591,0.992218,0.993692,0.995012,0.996179,0.997191,0.998049,0.998751,0.999297,0.999688,0.999922,1.000000,0.999922,0.999688,0.999297,0.998751,0.998049,0.997191,0.996179,0.995012,0.993692,0.992218,0.990591,0.988813,0.986884,0.984804,0.982575,0.980199,0.977675,0.975005,0.972191,0.969233,0.966134,0.962893,0.959514,0.955997,0.952345,0.948558,0.944638,0.940588,0.936409,0.932102,0.927671,0.923116,0.918441,0.913646,0.908734,0.903707,0.898568,0.893318,0.887960,0.882497,0.876930,0.871262,0.865495,0.859633,0.853676,0.847629,0.841493,0.835270,0.828964,0.822578,0.816112,0.809572,0.802958,0.796274,0.789522,0.782705,0.775825,0.768886,0.761890,0.754840,0.747738,0.740587,0.733390,0.726149,0.718868,0.711548,0.704193,0.696805,0.689387,0.681941,0.674470,0.666977,0.659464,0.651934,0.644389,0.636832,0.629265,0.621691,0.614112,0.606531,0.598949,0.591371,0.583796,0.576229,0.568671,0.561124,0.553592,0.546074,0.538575,0.531096,0.523639,0.516206,0.508799,0.501419,0.494070,0.486752,0.479468,0.472219,0.465007,0.457833,0.450700,0.443609,0.436561,0.429557,0.422600,0.415691,0.408831,0.402021,0.395263,0.388558,0.381907,0.375311,0.368771,0.362289,0.355865,0.349501,0.343196,0.336953,0.330771,0.324652,0.318597,0.312606,0.306679,0.300818,0.295023,0.289294,0.283632,0.278037,0.272511,0.267052,0.261662,0.256340,0.251088,0.245904,0.240790,0.235746,0.230771,0.225866,0.221031,0.216265,0.211569,0.206943,0.202386,0.197899,0.193481,0.189132,0.184851,0.180640,0.176497,0.172422,0.168414,0.164474,0.160602,0.156796,0.153056,0.149382,0.145773,0.142230,0.138750,0.135335,0.131984,0.128695,0.125468,0.122303,0.119200,0.116157,0.113174,0.110251,0.107386,0.104579,0.101830,0.099137,0.096501,0.093920,0.091394,0.088922,0.086503,0.084137,0.081822,0.079560,0.077347,0.075184,0.073071,0.071005,0.068988,0.067017,0.065092,0.063213,0.061378,0.059587,0.057840,0.056135,0.054471,0.052849,0.051267,0.049725,0.048221,0.046756,0.045328,0.043937,0.042582,0.041262,0.039977,0.038726,0.037508,0.036323,0.035169,0.034047,0.032956,0.031895,0.030863,0.029860,0.028884,0.027937,0.027016,0.026121,0.025253,0.024409,0.023590,0.022794,0.022022,0.021273,0.020546,0.019841,0.019157,0.018494,0.017851,0.017227,0.016623,0.016038,0.015470,0.014921,0.014388,0.013873,0.013374,0.012891,0.012423,0.011970,0.011533,0.011109,0.010699,0.010303,0.009920,0.009550,0.009192,0.008846,0.008512,0.008189,0.007877};
// sigma = 0.08;
double Pert_arr2[500] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000002,0.000002,0.000002,0.000002,0.000003,0.000003,0.000003,0.000004,0.000004,0.000005,0.000005,0.000006,0.000007,0.000008,0.000009,0.000010,0.000011,0.000013,0.000014,0.000016,0.000018,0.000020,0.000023,0.000025,0.000029,0.000032,0.000036,0.000040,0.000045,0.000050,0.000056,0.000063,0.000070,0.000078,0.000087,0.000097,0.000108,0.000120,0.000133,0.000148,0.000164,0.000182,0.000202,0.000224,0.000248,0.000274,0.000303,0.000335,0.000371,0.000409,0.000452,0.000498,0.000549,0.000604,0.000665,0.000732,0.000804,0.000884,0.000970,0.001065,0.001168,0.001280,0.001401,0.001534,0.001678,0.001834,0.002004,0.002187,0.002387,0.002603,0.002836,0.003089,0.003362,0.003656,0.003975,0.004318,0.004688,0.005086,0.005515,0.005976,0.006472,0.007004,0.007576,0.008189,0.008846,0.009550,0.010303,0.011109,0.011970,0.012891,0.013873,0.014921,0.016038,0.017227,0.018494,0.019841,0.021273,0.022794,0.024409,0.026121,0.027937,0.029860,0.031895,0.034047,0.036323,0.038726,0.041262,0.043937,0.046756,0.049725,0.052849,0.056135,0.059587,0.063213,0.067017,0.071005,0.075184,0.079560,0.084137,0.088922,0.093920,0.099137,0.104579,0.110251,0.116157,0.122303,0.128695,0.135335,0.142230,0.149382,0.156796,0.164474,0.172422,0.180640,0.189132,0.197899,0.206943,0.216265,0.225866,0.235746,0.245904,0.256340,0.267052,0.278037,0.289294,0.300818,0.312606,0.324652,0.336953,0.349501,0.362289,0.375311,0.388558,0.402021,0.415691,0.429557,0.443609,0.457833,0.472219,0.486752,0.501419,0.516206,0.531096,0.546074,0.561124,0.576229,0.591371,0.606531,0.621691,0.636832,0.651934,0.666977,0.681941,0.696805,0.711548,0.726149,0.740587,0.754840,0.768886,0.782705,0.796274,0.809572,0.822578,0.835270,0.847629,0.859633,0.871262,0.882497,0.893318,0.903707,0.913646,0.923116,0.932102,0.940588,0.948558,0.955997,0.962893,0.969233,0.975005,0.980199,0.984804,0.988813,0.992218,0.995012,0.997191,0.998751,0.999688,1.000000,0.999688,0.998751,0.997191,0.995012,0.992218,0.988813,0.984804,0.980199,0.975005,0.969233,0.962893,0.955997,0.948558,0.940588,0.932102,0.923116,0.913646,0.903707,0.893318,0.882497,0.871262,0.859633,0.847629,0.835270,0.822578,0.809572,0.796274,0.782705,0.768886,0.754840,0.740587,0.726149,0.711548,0.696805,0.681941,0.666977,0.651934,0.636832,0.621691,0.606531,0.591371,0.576229,0.561124,0.546074,0.531096,0.516206,0.501419,0.486752,0.472219,0.457833,0.443609,0.429557,0.415691,0.402021,0.388558,0.375311,0.362289,0.349501,0.336953,0.324652,0.312606,0.300818,0.289294,0.278037,0.267052,0.256340,0.245904,0.235746,0.225866,0.216265,0.206943,0.197899,0.189132,0.180640,0.172422,0.164474,0.156796,0.149382,0.142230,0.135335,0.128695,0.122303,0.116157,0.110251,0.104579,0.099137,0.093920,0.088922,0.084137,0.079560,0.075184,0.071005,0.067017,0.063213,0.059587,0.056135,0.052849,0.049725,0.046756,0.043937,0.041262,0.038726,0.036323,0.034047,0.031895,0.029860,0.027937,0.026121,0.024409,0.022794,0.021273,0.019841,0.018494,0.017227,0.016038,0.014921,0.013873,0.012891,0.011970,0.011109,0.010303,0.009550,0.008846,0.008189,0.007576,0.007004,0.006472,0.005976,0.005515,0.005086,0.004688,0.004318,0.003975,0.003656,0.003362,0.003089,0.002836,0.002603,0.002387,0.002187,0.002004,0.001834,0.001678,0.001534,0.001401,0.001280,0.001168,0.001065,0.000970,0.000884,0.000804,0.000732,0.000665,0.000604,0.000549,0.000498,0.000452,0.000409,0.000371,0.000335,0.000303,0.000274,0.000248,0.000224,0.000202,0.000182,0.000164,0.000148,0.000133,0.000120,0.000108,0.000097,0.000087,0.000078,0.000070,0.000063,0.000056,0.000050,0.000045,0.000040,0.000036,0.000032,0.000029,0.000025,0.000023,0.000020,0.000018,0.000016,0.000014,0.000013,0.000011,0.000010,0.000009,0.000008,0.000007,0.000006,0.000005,0.000005,0.000004,0.000004,0.000003,0.000003,0.000003,0.000002,0.000002,0.000002,0.000002,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// sigma = 0.04;
double Pert_arr3[500] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.000001,0.000001,0.000001,0.000001,0.000002,0.000002,0.000003,0.000004,0.000005,0.000006,0.000008,0.000010,0.000013,0.000016,0.000020,0.000025,0.000032,0.000040,0.000050,0.000063,0.000078,0.000097,0.000120,0.000148,0.000182,0.000224,0.000274,0.000335,0.000409,0.000498,0.000604,0.000732,0.000884,0.001065,0.001280,0.001534,0.001834,0.002187,0.002603,0.003089,0.003656,0.004318,0.005086,0.005976,0.007004,0.008189,0.009550,0.011109,0.012891,0.014921,0.017227,0.019841,0.022794,0.026121,0.029860,0.034047,0.038726,0.043937,0.049725,0.056135,0.063213,0.071005,0.079560,0.088922,0.099137,0.110251,0.122303,0.135335,0.149382,0.164474,0.180640,0.197899,0.216265,0.235746,0.256340,0.278037,0.300818,0.324652,0.349501,0.375311,0.402021,0.429557,0.457833,0.486752,0.516206,0.546074,0.576229,0.606531,0.636832,0.666977,0.696805,0.726149,0.754840,0.782705,0.809572,0.835270,0.859633,0.882497,0.903707,0.923116,0.940588,0.955997,0.969233,0.980199,0.988813,0.995012,0.998751,1.000000,0.998751,0.995012,0.988813,0.980199,0.969233,0.955997,0.940588,0.923116,0.903707,0.882497,0.859633,0.835270,0.809572,0.782705,0.754840,0.726149,0.696805,0.666977,0.636832,0.606531,0.576229,0.546074,0.516206,0.486752,0.457833,0.429557,0.402021,0.375311,0.349501,0.324652,0.300818,0.278037,0.256340,0.235746,0.216265,0.197899,0.180640,0.164474,0.149382,0.135335,0.122303,0.110251,0.099137,0.088922,0.079560,0.071005,0.063213,0.056135,0.049725,0.043937,0.038726,0.034047,0.029860,0.026121,0.022794,0.019841,0.017227,0.014921,0.012891,0.011109,0.009550,0.008189,0.007004,0.005976,0.005086,0.004318,0.003656,0.003089,0.002603,0.002187,0.001834,0.001534,0.001280,0.001065,0.000884,0.000732,0.000604,0.000498,0.000409,0.000335,0.000274,0.000224,0.000182,0.000148,0.000120,0.000097,0.000078,0.000063,0.000050,0.000040,0.000032,0.000025,0.000020,0.000016,0.000013,0.000010,0.000008,0.000006,0.000005,0.000004,0.000003,0.000002,0.000002,0.000001,0.000001,0.000001,0.000001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// sigma = 0.02
double Pert_arr4[500] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.000001,0.000001,0.000002,0.000004,0.000006,0.000010,0.000016,0.000025,0.000040,0.000063,0.000097,0.000148,0.000224,0.000335,0.000498,0.000732,0.001065,0.001534,0.002187,0.003089,0.004318,0.005976,0.008189,0.011109,0.014921,0.019841,0.026121,0.034047,0.043937,0.056135,0.071005,0.088922,0.110251,0.135335,0.164474,0.197899,0.235746,0.278037,0.324652,0.375311,0.429557,0.486752,0.546074,0.606531,0.666977,0.726149,0.782705,0.835270,0.882497,0.923116,0.955997,0.980199,0.995012,1.000000,0.995012,0.980199,0.955997,0.923116,0.882497,0.835270,0.782705,0.726149,0.666977,0.606531,0.546074,0.486752,0.429557,0.375311,0.324652,0.278037,0.235746,0.197899,0.164474,0.135335,0.110251,0.088922,0.071005,0.056135,0.043937,0.034047,0.026121,0.019841,0.014921,0.011109,0.008189,0.005976,0.004318,0.003089,0.002187,0.001534,0.001065,0.000732,0.000498,0.000335,0.000224,0.000148,0.000097,0.000063,0.000040,0.000025,0.000016,0.000010,0.000006,0.000004,0.000002,0.000001,0.000001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// sigma = 0.01
double Pert_arr5[500] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.000001,0.000004,0.000010,0.000025,0.000063,0.000148,0.000335,0.000732,0.001534,0.003089,0.005976,0.011109,0.019841,0.034047,0.056135,0.088922,0.135335,0.197899,0.278037,0.375311,0.486752,0.606531,0.726149,0.835270,0.923116,0.980199,1.000000,0.980199,0.923116,0.835270,0.726149,0.606531,0.486752,0.375311,0.278037,0.197899,0.135335,0.088922,0.056135,0.034047,0.019841,0.011109,0.005976,0.003089,0.001534,0.000732,0.000335,0.000148,0.000063,0.000025,0.000010,0.000004,0.000001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
				f_pretOutput[1] = pert_mag * Pert_arr3[iteration-pert_time-1]; // Andy like this! 
        //f_pretOutput[1] = pert_mag * Pert_arr1[iteration-pert_time-1];  // me try slower
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
