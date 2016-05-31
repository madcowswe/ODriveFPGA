#include <math.h>
#include "extramath.h"
//#include "Parameters.h"

#ifdef USING_FPOINT_HW
#include "altera_nios_custom_instr_floating_point_2_no_sqrt.h"
#endif


//Code originally from http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
//Which was in the public domain.
//It has since been modified.

//Max error 0.1 rad
float fast_atan2(float y, float x){
	float abs_y = fabs(y)+1e-10f;      // kludge to prevent 0/0 condition
	float angle;
	if (x>=0)
	{
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r*r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (PI_F/4.0f);
	}
	else
	{
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r*r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f*PI_F/4.0f);
	}
	if (y < 0)
	return(-angle);     // negate if in quad III or IV
	else
	return(angle);
}

extern const float cospi_table[64]; //first octant of cos(x)
extern const float sinpi_table[64]; //first octant of sin(x)

//Argument reudction on theta is included in function. Max error 0.0002
void fast_cossin(float theta, float* cosout, float* sinout){

	// theta = -pi to pi is x = -2^15 to 2^15-1
	int x = theta*(((float)(1<<15))/PI_F);

	//Argument reduction to the first octant.
	//s is sign bit, qo (quadrant octant) is 2 bits, y is 13 bits which is built up of (t is 6 bits, h is 7 bits)
	unsigned int s = ((unsigned int)x & (1u<<15));//>>15;
	unsigned int qo = ((unsigned int)x>>13) & 3u;
	unsigned int y = x & ((1u<<13)-1);
	if (qo & 1) //if octant is odd we need to reverse argument
		y = ((1u<<13)-1) - y;
	unsigned int t = y>>7;
	unsigned int h = y & ((1u<<7)-1);

	//Solve sin and cos in 1st octant.
	//Lookup
	float cos_pit = cospi_table[t];
	float sin_pit = sinpi_table[t];

	//1st order interpolation
	float hf = (PI_F/((float)(1<<15))) * (float)h;
	float sinxh = sin_pit + hf*cos_pit;
	float cosxh = cos_pit - hf*sin_pit;

	//Reconstruct actual sin and cos in full circle
	if (s)
		qo = (~qo) & 3u;
	float tempcosout;
	float tempsinout;
	switch (qo)
	{
		case 0:
			tempsinout = sinxh;
			tempcosout = cosxh;
			break;
		
		case 1:
			tempsinout = cosxh;
			tempcosout = sinxh;
			break;
		
		case 2:
			tempsinout = cosxh;
			tempcosout = -sinxh;
			break;
		
		case 3:
			tempsinout = sinxh;
			tempcosout = -cosxh;
			break;
	}
	if (s)
		tempsinout = -tempsinout;

	*cosout = tempcosout;
	*sinout = tempsinout;

}

//Fast inverse square root, from http://en.wikipedia.org/wiki/Fast_inverse_square_root#Overview_of_the_code
float Q_rsqrt( float number )
{
	int i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( int * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}

/*
void test_trig_funcs(){
	for (float theta = -4.0f; theta < 4.0f; theta += 0.01){
		float c,s;
		fast_cossin(theta, &c, &s);
		float cr = cosf(theta);
		float sr = sinf(theta);
		if(fabs(c-cr) > 0.0002 || fabs(s-sr) > 0.0002)
			printf("cos(%f) = r%f, a%f, d%f, sin(%f) = r%f, a%f, d%f\n", theta, cr, c, c-cr, theta, sr, s, s-sr);
	}

	for (float x = -100.0f; x < 100.0f; x+=1.0f)
	{
		for (float y = -100.0f; y < 100.0f; y+=1.0f)
		{
			float t = fast_atan2(y,x);
			float tr = atan2f(y,x);
			if (fabs(t-tr)>0.0102f)
			{
				printf("atan2(%f,%f) = r%f, a%f, d%f\n", y,x,t,tr,t-tr);
			}
		}
	}

	printf("test complete\n");
}
*/
