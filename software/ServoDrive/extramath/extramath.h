#ifndef __EXTRAMATH_H_
#define __EXTRAMATH_H_

#define MIN(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a < _b ? _a : _b; })

#define MAX(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _a : _b; })

#define PI_F 3.1415926535897932f
#define PI_D 3.1415926535897932

static const float one_by_sqrt3 = 0.57735026919f;
static const float two_by_sqrt3 = 2.0f*0.57735026919f;
static const float sqrt3_by_2 = 0.86602540378f;

static inline float wrappmpi(float angle){
	while(angle > PI_F)
		angle -= (2.0f*PI_F);
	while(angle < -PI_F)
		angle += (2.0f*PI_F); 
	return angle;
}

//max error 0.1 rad
float fast_atan2(float y, float x);

//Argument reudction on theta is included in function. Max error 0.0002
void fast_cossin(float theta, float* cosout, float* sinout);

//Fast inverse square root
float Q_rsqrt( float number );

#endif //__EXTRAMATH_H_
