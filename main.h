#pragma once
#ifndef MAIN_H
#define MAIN_H

#ifndef F_PI
#define F_PI		((float)(M_PI))
#define F_2_PI		((float)(2.f*F_PI))
#define F_PI_2		((float)(F_PI/2.f))
#endif

//#define INIT_ALT 2000 // In km
//#define INIT_VEL 8000 // In m/s
//
//#define EARTH_TEX_NAME "earth.bmp"
//#define STARFIELD_TEX_NAME "stars.bmp"
//#define NAVBALL_TEX_NAME "navball.bmp"

#define G 6.67430e-11f
#define EARTH_MASS 5.972e24f
#define MU (G * EARTH_MASS)

#define CONVERSION_RATE 6371000.0f

#define U_TO_M(x) ((x) * (CONVERSION_RATE))
#define M_TO_U(x) ((x) / (CONVERSION_RATE))
#define U_TO_KM(x) ((x) * (CONVERSION_RATE / 1000))
#define KM_TO_U(x) ((x) / (CONVERSION_RATE / 1000))

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)

#define MS_TO_S(x) ((x) / 1000.0f)
#define S_TO_MS(x) ((x) * 1000.0f)

//#define MIN_WARP_LEVEL 0
//#define MAX_WARP_LEVEL 9
//
//#define MIN_THROTTLE_LEVEL 0
//#define MAX_THROTTLE_LEVEL 10
//
//#define ROTATION_RATE 0.05
//
//#define EARTH_RADIUS 1.0f
//#define SOI_LIMIT ( 10000000 )
//
//#define SUN_RADIUS 100.0f
//#define SUN_DISTANCE 5000.0f
//#define STAR_FIELD_RADIUS 5000.0f



#endif