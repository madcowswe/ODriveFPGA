

#include <stdio.h>
#include "system.h"
#include "io.h"
#include "unistd.h"
#include <stdint.h>
#include <stdbool.h>
#include "extramath/extramath.h"


#define PWM_REG_MAXCTR 0x8
#define PWM_REG_EN 0x9
#define PWM_REG_UPDATEON_Z 0xA
#define PWM_REG_UPDATEON_MAX 0xB
#define PWM_REG_TRIGON_Z 0xC
#define PWM_REG_TRIGON_MAX 0xD
#define PWM_REG_UPDATE 0xF

#define TADCS_REG_EN 0
#define TADCS_REG_IRQFLAG 1
#define TADCS_REG_MAXSEQ 2
#define TADCS_OFFSET_CH_MAP 0x10
#define TADCS_OFFSET_SAMPSTORE 0x18

#define QEI_REG_COUNT 0x0
#define QEI_REG_clearOnZ 0x1
#define QEI_REG_revDir 0x2

// deadband timing. All units ns.
// tOn* = turn on delay
// tOff* = turn off delay
// tRise* = Rise time
// tFall* = Fall time

//static const float tOnFET = 20;
//static const float tRiseFET = 70;
//static const float tOffFET = 30;
static const float tFallFET = 40;
//static const float tOnDriver = 160;
//static const float toffDriver = 150;
//static const float tRiseDriver = 100;
static const float tFallDriver = 50;

static const float tDelayMatchDriver = 50;
static const float tSafety = 50;

//static const float deadtime_ns = 200;//tFallFET + tFallDriver + tDelayMatchDriver + tSafety;
#define deadtime_ns 200
static const int deadtimeint = (float)ALT_CPU_CPU_FREQ * deadtime_ns * 1e-9f;

#define PWMFrequency 8000
static const float dt = 1.0f/(float)PWMFrequency;
static const int PWMHalfPeriod = ALT_CPU_CPU_FREQ/(PWMFrequency*2);


//donkey
#define QudcountsPerRev 2400
static const float currentKp = 0.002f;
static const float currentKi = 5.0f;
static const float speedKp = 0.1f;
static const float speedKi = 0.0f;//0.4f;
static const float posKp = 35.0f;

//350kv motor
//#define QudcountsPerRev 2000
//static const float currentKp = 0.004f;
//static const float currentKi = 7.5f;
//static const float speedKp = 0.02f;//0.1f;
//static const float speedKi = 0.0f;//1.0f;

static const float currentlimit = 50.0f;
static const float lockinCurrent = 20.0f;
static const float speedLimit = 4000.0f;

static const float frictionCurrent = 8.0f;
static const float accelperA = 1150.0f; //rad/s2 / A
static const float Aperaccel = (1.0f/1150.0f);
static const float profileAccel = 10000.0f;//40000.0f; //rad/s2


static const float ADCtoAscalefactor = 3.3f/((float)(1<<12) * 50.0f * 0.0005f);
static const float ADCtoVbusSF = 3.3f * 11.0f / (float)(1<<12);
static const float encToPhasefactor = 2.0f*PI_F*7.0f/(float)QudcountsPerRev;
static const float omegaFilterConst = 0.95;

static const float BrakeResistorCurrent = 30.0f; //Amps
static const float BrakeResistorFactor = 1.0f/30.0f;

float fsign(float x){
	if(x > 0.0f) return 1.0f;
	if(x < 0.0f) return -1.0f;
	return 0.0f;
}

// Magnitude must not be larger than sqrt(3)/2, or 0.866
void SVM(float alpha, float beta, uint32_t* tAout, uint32_t* tBout, uint32_t* tCout){

	uint32_t Sextant;

	if (beta >= 0.0f)
	{
		if (alpha >= 0.0f)
		{
			//quadrant I
			if (one_by_sqrt3 * beta > alpha)
				Sextant = 2;
			else
				Sextant = 1;

		} else {
			//quadrant II
			if (-one_by_sqrt3 * beta > alpha)
				Sextant = 3;
			else
				Sextant = 2;
		}
	} else {
		if (alpha >= 0.0f)
		{
			//quadrant IV
			if (-one_by_sqrt3 * beta > alpha)
				Sextant = 5;
			else
				Sextant = 6;
		} else {
			//quadrant III
			if (one_by_sqrt3 * beta > alpha)
				Sextant = 4;
			else
				Sextant = 5;
		}
	}

	// PWM timings
	uint32_t tA, tB, tC;

	switch (Sextant) {

		// sextant 1-2
		case 1:
		{
			// Vector on-times
			uint32_t t1 = (alpha - one_by_sqrt3 * beta) * PWMHalfPeriod;
			uint32_t t2 = (two_by_sqrt3 * beta) * PWMHalfPeriod;

			// PWM timings
			tA = (PWMHalfPeriod - t1 - t2) / 2;
			tB = tA + t1;
			tC = tB + t2;

			break;
		}

		// sextant 2-3
		case 2:
		{
			// Vector on-times
			uint32_t t2 = (alpha + one_by_sqrt3 * beta) * PWMHalfPeriod;
			uint32_t t3 = (-alpha + one_by_sqrt3 * beta) * PWMHalfPeriod;

			// PWM timings
			tB = (PWMHalfPeriod - t2 - t3) / 2;
			tA = tB + t3;
			tC = tA + t2;

			break;
		}

		// sextant 3-4
		case 3:
		{
			// Vector on-times
			uint32_t t3 = (two_by_sqrt3 * beta) * PWMHalfPeriod;
			uint32_t t4 = (-alpha - one_by_sqrt3 * beta) * PWMHalfPeriod;

			// PWM timings
			tB = (PWMHalfPeriod - t3 - t4) / 2;
			tC = tB + t3;
			tA = tC + t4;

			break;
		}

		// sextant 4-5
		case 4:
		{
			// Vector on-times
			uint32_t t4 = (-alpha + one_by_sqrt3 * beta) * PWMHalfPeriod;
			uint32_t t5 = (-two_by_sqrt3 * beta) * PWMHalfPeriod;

			// PWM timings
			tC = (PWMHalfPeriod - t4 - t5) / 2;
			tB = tC + t5;
			tA = tB + t4;

			break;
		}

		// sextant 5-6
		case 5:
		{
			// Vector on-times
			uint32_t t5 = (-alpha - one_by_sqrt3 * beta) * PWMHalfPeriod;
			uint32_t t6 = (alpha - one_by_sqrt3 * beta) * PWMHalfPeriod;

			// PWM timings
			tC = (PWMHalfPeriod - t5 - t6) / 2;
			tA = tC + t5;
			tB = tA + t6;

			break;
		}

		// sextant 6-1
		case 6:
		{
			// Vector on-times
			uint32_t t6 = (-two_by_sqrt3 * beta) * PWMHalfPeriod;
			uint32_t t1 = (alpha + one_by_sqrt3 * beta) * PWMHalfPeriod;

			// PWM timings
			tA = (PWMHalfPeriod - t6 - t1) / 2;
			tC = tA + t1;
			tB = tC + t6;

			break;
		}

	} //switch

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;

}

struct axis_state_s {
	int id;
	void* pwm_base;
	void* qei_base;
	void* tas_base;
	int IsenseADC_AB[2];
	int IsenseOffset[2];
	bool  encrev;
	int encoffset;
	float IerrVstate_d;
	float IerrVstate_q;
	float omega;
	float oldenc;
	float Iqintstate;
};

struct axis_state_s axes[] = {
	{	//Axis 0
		.id = 0,
		.pwm_base = (void*)PWM_0_BASE,
		.qei_base = (void*)QEI_0_BASE,
		.tas_base = (void*)TRIGGERED_ADC_SEQUENCER_0_BASE,
		.IsenseADC_AB = {7, 6},
		.IsenseOffset = {0, 0},
		.encrev = 0,
		.encoffset = 0,
		.IerrVstate_d = 0.0f,
		.IerrVstate_q = 0.0f,
		.omega = 0.0f,
		.oldenc = 0.0f,
		.Iqintstate = 0.0f
	},
	{	//Axis 1
		.id = 1,
		.pwm_base = (void*)PWM_2_BASE,
		.qei_base = (void*)QEI_1_BASE,
		.tas_base = (void*)TRIGGERED_ADC_SEQUENCER_1_BASE,
		.IsenseADC_AB = {2, 1},
		.IsenseOffset = {0, 0},
		.encrev = 0,
		.encoffset = 0,
		.IerrVstate_d = 0.0f,
		.IerrVstate_q = 0.0f,
		.omega = 0.0f,
		.oldenc = 0.0f,
		.Iqintstate = 0.0f
	}
};

static const int numaxes = sizeof(axes)/sizeof(axes[0]);
static const void* brake_res_base = (void*)PWM_1_BASE;

void wait_for_ADC(struct axis_state_s* axis, float* Ia, float* Ib, float* Vbus){
	while(!IORD(axis->tas_base, TADCS_REG_IRQFLAG));
	IOWR(axis->tas_base, TADCS_REG_IRQFLAG, 0);

	*Ia = ADCtoAscalefactor * ((int)IORD(axis->tas_base, TADCS_OFFSET_SAMPSTORE + 0) - axis->IsenseOffset[0]);
	*Ib = ADCtoAscalefactor * ((int)IORD(axis->tas_base, TADCS_OFFSET_SAMPSTORE + 1) - axis->IsenseOffset[1]);

	*Vbus = ADCtoVbusSF * IORD(axis->tas_base, TADCS_OFFSET_SAMPSTORE + 2);
}

void control_current(struct axis_state_s* axis, float targetId, float targetIq, float Ia, float Ib, float phase, float* IbusEst){

	float Ialpha = Ia;
	float Ibeta = one_by_sqrt3 * Ia + two_by_sqrt3 * Ib;

	float c,s;
	fast_cossin(phase, &c, &s);
	float Id = c*Ialpha + s*Ibeta;
	float Iq = c*Ibeta  - s*Ialpha;

	float Ierr_d = targetId - Id;
	float Ierr_q = targetIq - Iq;

	float Vd = axis->IerrVstate_d + Ierr_d * currentKp;
	float Vq = axis->IerrVstate_q + Ierr_q * currentKp;

	float Vscalefactor = 0.98f * sqrt3_by_2 * Q_rsqrt(Vd*Vd + Vq*Vq);
	if (Vscalefactor < 1)
	{
		Vd *= Vscalefactor;
		Vq *= Vscalefactor;
	} else {
		axis->IerrVstate_d += Ierr_d * (currentKi * dt);
		axis->IerrVstate_q += Ierr_q * (currentKi * dt);
	}

	*IbusEst = Vd * Id + Vq * Iq; //Note V is in modulation, not voltage.

	float Valpha = c*Vd - s*Vq;
	float Vbeta  = c*Vq + s*Vd;

	uint32_t tABC[3];
	SVM(Valpha,Vbeta,&tABC[0],&tABC[1],&tABC[2]);

	IOWR(axis->pwm_base, PWM_REG_UPDATE, 0);
	for (int i = 0; i < 3; ++i)
	{
		IOWR(axis->pwm_base, 2*i, MAX((int)tABC[i] - deadtimeint/2, 0));
		IOWR(axis->pwm_base, 2*i + 1, tABC[i] + deadtimeint/2);
	}
	IOWR(axis->pwm_base, PWM_REG_UPDATE, 1);
}

void blocking_polar_control_current(struct axis_state_s* axis, float phase, float mag){
	float Ia, Ib, Vbus, Ibusest;
	wait_for_ADC(axis, &Ia, &Ib, &Vbus);
	control_current(axis, mag, 0.0f, Ia, Ib, phase, &Ibusest);
}

void setup_axis(struct axis_state_s* axis_state){

	IOWR(axis_state->qei_base, QEI_REG_revDir, axis_state->encrev);

	IOWR(axis_state->tas_base, TADCS_REG_MAXSEQ, 2);
	IOWR(axis_state->tas_base, TADCS_OFFSET_CH_MAP + 0, axis_state->IsenseADC_AB[0]); //Ia
	IOWR(axis_state->tas_base, TADCS_OFFSET_CH_MAP + 1, axis_state->IsenseADC_AB[1]); //Ib
	IOWR(axis_state->tas_base, TADCS_OFFSET_CH_MAP + 2, 0); //Vbus
	IOWR(axis_state->tas_base, TADCS_REG_IRQFLAG, 0);
	IOWR(axis_state->tas_base, TADCS_REG_EN, 1);

	IOWR(axis_state->pwm_base, PWM_REG_TRIGON_Z, 1);
	IOWR(axis_state->pwm_base, PWM_REG_UPDATEON_Z, 1);
	IOWR(axis_state->pwm_base, PWM_REG_MAXCTR, PWMHalfPeriod);
	IOWR(axis_state->pwm_base, PWM_REG_UPDATE, 0);
	for (int i = 0; i < 3; ++i)
	{
		IOWR(axis_state->pwm_base, 2*i, PWMHalfPeriod/2 - deadtimeint/2);//MAX((int)tABC[i] - deadtimeint/2, 0));
		IOWR(axis_state->pwm_base, 2*i + 1, PWMHalfPeriod/2 + deadtimeint/2);
	}
	IOWR(axis_state->pwm_base, PWM_REG_UPDATE, 1);
	IOWR(axis_state->pwm_base, PWM_REG_EN, 1);

}

void init_routine(){
	//sample current sense values during V0 PWM
	for (int i = -128; i < 128; ++i)
	{
		for(int ax = 0; ax < numaxes; ++ax){
			while(!IORD(axes[ax].tas_base, TADCS_REG_IRQFLAG));
			IOWR(axes[ax].tas_base, TADCS_REG_IRQFLAG, 0);
			int Ia = IORD(axes[ax].tas_base, TADCS_OFFSET_SAMPSTORE + 0);
			int Ib = IORD(axes[ax].tas_base, TADCS_OFFSET_SAMPSTORE + 1);
			if(i >= 0){
				axes[ax].IsenseOffset[0] += Ia;
				axes[ax].IsenseOffset[1] += Ib;
			}

			IOWR(axes[ax].pwm_base, PWM_REG_UPDATE, 0);
			for (int i = 0; i < 3; ++i)
			{
				IOWR(axes[ax].pwm_base, 2*i, PWMHalfPeriod/2 - deadtimeint/2);//MAX((int)tABC[i] - deadtimeint/2, 0));
				IOWR(axes[ax].pwm_base, 2*i + 1, PWMHalfPeriod/2 + deadtimeint/2);
			}
			IOWR(axes[ax].pwm_base, PWM_REG_UPDATE, 1);
		}
	}

	for(int ax = 0; ax < numaxes; ++ax){
		for (int i = 0; i < 2; ++i)
		{
			axes[ax].IsenseOffset[i] /= 128;
		}
	}




	//struct axis_state_s* test_ax = &axes[1];

#if 0
	//Fixed voltage test
	float finalIa, finalIb;
	while(0){
		for (int i = 0; i < 8000; ++i){
			float Vbus;
			wait_for_ADC(test_ax, &finalIa, &finalIb, &Vbus);

			uint32_t tABC[3];
			SVM(0.025f,0.0f,&tABC[0],&tABC[1],&tABC[2]);

			IOWR(test_ax->pwm_base, PWM_REG_UPDATE, 0);
			for (int i = 0; i < 3; ++i)
			{
				IOWR(test_ax->pwm_base, 2*i, MAX((int)tABC[i] - deadtimeint/2, 0));
				IOWR(test_ax->pwm_base, 2*i + 1, tABC[i] + deadtimeint/2);
			}
			IOWR(test_ax->pwm_base, PWM_REG_UPDATE, 1);
		}
		finalIa = finalIa;
	}

	//square wave current control test
	while(0){
		for (int i = 0; i < 100; ++i)
		{
			blocking_polar_control_current(test_ax, 0.0f, 10.0f);
		}
		for (int i = 0; i < 100; ++i)
		{
			blocking_polar_control_current(test_ax, 0.0f, 0.0f);
		}
	}

#endif


	//go to rotor zero phase to get ready to scan
	//TODO: maybe scan over a small angle range, to avoid the case when the rotor is stuck at exactly 180 degrees electrical.
	for (int i = 0; i < (1.0f*PWMFrequency); ++i)
	{
		for(int ax = 0; ax < numaxes; ++ax){
			blocking_polar_control_current(&axes[ax], -2*PI_F, lockinCurrent);
		}
	}

#ifdef ENC_IDX_PRESENT

	for(int ax = 0; ax < numaxes; ++ax){
		IOWR(axes[ax].qei_base, QEI_REG_clearOnZ, 1); //clear phase reg on next index pulse
	}

	//scan lockin until idx pulse
	for (float ph = 0; BROKEN METHOD FOR MULTI AXIS IORD(test_ax->pwm_base, QEI_REG_clearOnZ); ph += 0.0022f)
	{
		for(int ax = 0; ax < numaxes; ++ax){
			blocking_polar_control_current(ph, lockinCurrent);
		}
	}

#else
	for(int ax = 0; ax < numaxes; ++ax){
		IOWR(axes[ax].qei_base, QEI_REG_COUNT, 0);
	}
#endif

	//scan forwards
	int encvaluesum[numaxes];
	for(int ax = 0; ax < numaxes; ++ax){
		encvaluesum[ax] = 0.0f;
	}
	for (float ph = -2*PI_F; ph < 2*PI_F; ph += 4*PI_F/128.0f)
	{
		for (int i = 0; i < (0.02f*PWMFrequency); ++i)
		{
			for(int ax = 0; ax < numaxes; ++ax){
				blocking_polar_control_current(&axes[ax], ph, lockinCurrent);
			}
		}
		for(int ax = 0; ax < numaxes; ++ax){
			encvaluesum[ax] += IORD(axes[ax].qei_base, QEI_REG_COUNT);
		}
	}

	//check encoder direction
	for(int ax = 0; ax < numaxes; ++ax){
		int testencval = IORD(axes[ax].qei_base, QEI_REG_COUNT);
		if(testencval <= 0){ //encoder vs motor phases likely configured backwards, or disconnected
			while(1){
				//TODO flash red phase LED here
			}
		}
	}

	//scan backwards
	for (float ph = 2*PI_F; ph > -2*PI_F; ph -= 4*PI_F/128.0f)
	{
		for (int i = 0; i < (0.02f*PWMFrequency); ++i)
		{
			for(int ax = 0; ax < numaxes; ++ax){
				blocking_polar_control_current(&axes[ax], ph, lockinCurrent);
			}
		}
		for(int ax = 0; ax < numaxes; ++ax){
			encvaluesum[ax] += IORD(axes[ax].qei_base, QEI_REG_COUNT);
		}
	}

	for(int ax = 0; ax < numaxes; ++ax){
		axes[ax].encoffset = encvaluesum[ax] / 256;
	}
}

void setup_brake_resistor(){
	IOWR(brake_res_base, PWM_REG_TRIGON_Z, 1); //NOTE do not remove, watchdog depends on this.
	IOWR(brake_res_base, PWM_REG_UPDATEON_Z, 1);
	IOWR(brake_res_base, PWM_REG_MAXCTR, PWMHalfPeriod);
	IOWR(brake_res_base, PWM_REG_UPDATE, 0);

	IOWR(brake_res_base, 0, PWMHalfPeriod - deadtimeint); //A ready to inject some active modulation
	IOWR(brake_res_base, 1, PWMHalfPeriod);
	IOWR(brake_res_base, 2, 0); //B off
	IOWR(brake_res_base, 3, PWMHalfPeriod);
	IOWR(brake_res_base, 4, PWMHalfPeriod - deadtimeint); //C always grounded
	IOWR(brake_res_base, 5, PWMHalfPeriod);

	IOWR(brake_res_base, PWM_REG_UPDATE, 1);
	IOWR(brake_res_base, PWM_REG_EN, 1);
}

void setup_mag_pwm(){
	IOWR(MAGNET_PWM_BASE, PWM_REG_TRIGON_Z, 1); //NOTE do not remove, watchdog depends on this.
	IOWR(MAGNET_PWM_BASE, PWM_REG_UPDATEON_Z, 1);
	IOWR(MAGNET_PWM_BASE, PWM_REG_MAXCTR, PWMHalfPeriod);
	IOWR(MAGNET_PWM_BASE, PWM_REG_UPDATE, 0);

	IOWR(MAGNET_PWM_BASE, 0, 0); //A off
	IOWR(MAGNET_PWM_BASE, 1, PWMHalfPeriod);
	IOWR(MAGNET_PWM_BASE, 2, 0); //B off
	IOWR(MAGNET_PWM_BASE, 3, PWMHalfPeriod);
	IOWR(MAGNET_PWM_BASE, 4, 0); //C off
	IOWR(MAGNET_PWM_BASE, 5, PWMHalfPeriod);

	IOWR(MAGNET_PWM_BASE, PWM_REG_UPDATE, 1);
	IOWR(MAGNET_PWM_BASE, PWM_REG_EN, 1);
}

void blocking_control_motor(struct axis_state_s* axis, float possetpoint, float omega_ff, float Iq_ff, float* IbusEst){

	float Ia, Ib, Vbus;
	wait_for_ADC(axis, &Ia, &Ib, &Vbus);

	int enccount = IORD(axis->qei_base, QEI_REG_COUNT);
	float phase = ((enccount - axis->encoffset) % QudcountsPerRev) * encToPhasefactor;
	float dencbydt = encToPhasefactor*(enccount - axis->oldenc)*PWMFrequency;
	axis->oldenc = enccount;
	axis->omega = omegaFilterConst*axis->omega + (1-omegaFilterConst)*dencbydt;
	float pos = enccount * encToPhasefactor;

	float omegasetpoint = posKp * (possetpoint - pos) + omega_ff;
	if(omegasetpoint > speedLimit){
		omegasetpoint = speedLimit;
	}else{
		if(omegasetpoint < -speedLimit){
			omegasetpoint = -speedLimit;
		} else {
			//; //TODO use speed integral?
		}
	}
	float omegaerror = omegasetpoint - axis->omega;

	float targetId = 0.0f;
	float targetIq = speedKp * omegaerror + axis->Iqintstate + Iq_ff;
	if(targetIq > currentlimit){
		targetIq = currentlimit;
	}else{
		if(targetIq < -currentlimit){
			targetIq = -currentlimit;
		} else {
			axis->Iqintstate += omegaerror * (speedKi * dt);
		}
	}

	control_current(axis, targetId, targetIq, Ia, Ib, phase, IbusEst);

	//Logging only makes sense for 1 axis at a time
	if(axis->id == 0){
		static int logctr = 0;
		if(++logctr == 8){
			logctr = 0;
			IOWR(LOG_REG_0_BASE, 1, (int)(targetIq*500.0f));
			IOWR(LOG_REG_0_BASE, 0, (int)(axis->omega*10.0f));
			//IOWR(LOG_REG_0_BASE, 0, (int)(Icomp*1000.0f));
		}
	}

}

float dump_excess_current(float* regenCurrents, int numregen){
	float Icomp = 0.0f;
	for(int ax = 0; ax < numregen; ++ax){
		Icomp -= regenCurrents[ax];
	}
	if(Icomp < 0.0f)
		Icomp = 0.0f;

	int Icomp_compareval = PWMHalfPeriod - (int)((float)PWMHalfPeriod * BrakeResistorFactor * Icomp);

	IOWR(brake_res_base, PWM_REG_UPDATE, 0);
	IOWR(brake_res_base, 0, MAX(Icomp_compareval - deadtimeint, 0));
	IOWR(brake_res_base, 1, MAX(Icomp_compareval, deadtimeint));
	IOWR(brake_res_base, PWM_REG_UPDATE, 1);

	return Icomp;
}

struct point_s {
	float x;
	float y;
};

static const struct point_s star_waypoints[] = {
	 {0.0f, 0.0f}
	,{75,	150}
	,{65.6000074800000,	0.591397402499999}
	,{93.6517415400000,	147.643737082500}
	,{47.3906585475000,	5.26676355750000}
	,{111.131525557500,	140.723001000000}
	,{30.9161060775000,	14.3237254200000}
	,{126.341032942500,	129.672647055000}
	,{17.2115067900000,	27.1932007725000}
	,{138.324594412500,	115.187009625000}
	,{7.13797106250000,	43.0665531300000}
	,{146.329238722500,	98.1762745800000}
	,{1.32845619750000,	60.9464014050000}
	,{149.852004630000,	79.7092889625000}
	,{0.147995370000004,	79.7092889625000}
	,{148.671543802500,	60.9464014050000}
	,{3.67076127750000,	98.1762745800000}
	,{142.862028937500,	43.0665531300000}
	,{11.6754055875000,	115.187009625000}
	,{132.788493210000,	27.1932007725000}
	,{23.6589670575000,	129.672647055000}
	,{119.083893922500,	14.3237254200000}
	,{38.8684744425000,	140.723001000000}
	,{102.609341452500,	5.26676355750000}
	,{56.3482584600000,	147.643737082500}
	,{84.3999925200000,	0.591397402499999}
	,{75.0000000000003,	150}
	,{0.0f, 0.0f}
};
static const int num_star_wpts = sizeof(star_waypoints)/sizeof(star_waypoints[0]);

struct point_mag_s {
	float x;
	float y;
	float magStr;
	float duration_modifier;
};
static const struct point_mag_s magpickup_waypoints[] = {
	//{0.0f, 		0.0f, 	0.0f}
	//,{150.0f,	150.0f, 0.0f}
	//,{75.0f,	75.0f, 	0.4f}
	//,{0.0f,		0.0f, 	0.0f}
	//,{75.0f,	75.0f, 	0.0f}
	//,{0.0f,		150.0f, 0.4f}
	//,{0.0f,		0.0f, 	0.0f}
{ 000.0f, 000.0f, 0.0f, 0000.0f },
{ 032.6f, 032.6f, 0.0f, 0000.0f },
{ 032.6f, 032.6f, 0.4f, 0500.0f },
{ 053.5f, 019.0f, 0.4f, 0000.0f },
{ 053.5f, 019.0f, 0.0f, 1000.0f },
{ 078.1f, 015.1f, 0.0f, 0000.0f },
{ 078.1f, 015.1f, 0.4f, 0500.0f },
{ 102.2f, 021.5f, 0.4f, 0000.0f },
{ 102.2f, 021.5f, 0.0f, 1000.0f },
{ 121.6f, 037.2f, 0.0f, 0000.0f },
{ 121.6f, 037.2f, 0.4f, 0500.0f },
{ 133.0f, 059.5f, 0.4f, 0000.0f },
{ 133.0f, 059.5f, 0.0f, 1000.0f },
{ 134.3f, 084.4f, 0.0f, 0000.0f },
{ 134.3f, 084.4f, 0.4f, 0500.0f },
{ 125.3f, 107.7f, 0.4f, 0000.0f },
{ 125.3f, 107.7f, 0.0f, 1000.0f },
{ 107.7f, 125.3f, 0.0f, 0000.0f },
{ 107.7f, 125.3f, 0.4f, 0500.0f },
{ 084.4f, 134.3f, 0.4f, 0000.0f },
{ 084.4f, 134.3f, 0.0f, 1000.0f },
{ 059.5f, 133.0f, 0.0f, 0000.0f },
{ 059.5f, 133.0f, 0.4f, 0500.0f },
{ 037.3f, 121.6f, 0.4f, 0000.0f },
{ 037.3f, 121.6f, 0.0f, 1000.0f },
{ 021.5f, 102.3f, 0.0f, 0000.0f },
{ 021.5f, 102.3f, 0.4f, 0500.0f },
{ 015.1f, 078.2f, 0.4f, 0000.0f },
{ 015.1f, 078.2f, 0.0f, 1000.0f },
{ 019.0f, 053.5f, 0.0f, 0000.0f },
{ 019.0f, 053.5f, 0.4f, 0500.0f },
{ 032.6f, 032.6f, 0.4f, 0000.0f },
{ 032.6f, 032.6f, 0.0f, 1000.0f },
{ 053.5f, 019.0f, 0.0f, 0000.0f },
{ 053.5f, 019.0f, 0.4f, 0500.0f },
{ 078.1f, 015.1f, 0.4f, 0000.0f },
{ 078.1f, 015.1f, 0.0f, 1000.0f },
{ 102.2f, 021.5f, 0.0f, 0000.0f },
{ 102.2f, 021.5f, 0.4f, 0500.0f },
{ 121.6f, 037.2f, 0.4f, 0000.0f },
{ 121.6f, 037.2f, 0.0f, 1000.0f },
{ 133.0f, 059.5f, 0.0f, 0000.0f },
{ 133.0f, 059.5f, 0.4f, 0500.0f },
{ 134.3f, 084.4f, 0.4f, 0000.0f },
{ 134.3f, 084.4f, 0.0f, 1000.0f },
{ 125.3f, 107.7f, 0.0f, 0000.0f },
{ 125.3f, 107.7f, 0.4f, 0500.0f },
{ 107.7f, 125.3f, 0.4f, 0000.0f },
{ 107.7f, 125.3f, 0.0f, 1000.0f },
{ 084.4f, 134.3f, 0.0f, 0000.0f },
{ 084.4f, 134.3f, 0.4f, 0500.0f },
{ 059.5f, 133.0f, 0.4f, 0000.0f },
{ 059.5f, 133.0f, 0.0f, 1000.0f },
{ 037.3f, 121.6f, 0.0f, 0000.0f },
{ 037.3f, 121.6f, 0.4f, 0500.0f },
{ 021.5f, 102.3f, 0.4f, 0000.0f },
{ 021.5f, 102.3f, 0.0f, 1000.0f },
{ 015.1f, 078.2f, 0.0f, 0000.0f },
{ 015.1f, 078.2f, 0.4f, 0500.0f },
{ 019.0f, 053.5f, 0.4f, 0000.0f },
{ 019.0f, 053.5f, 0.0f, 1000.0f },
{ 000.0f, 000.0f, 0.0f, 0000.0f }
};
static const int num_mag_wpts = sizeof(magpickup_waypoints)/sizeof(magpickup_waypoints[0]);

void set_mag_power(float power){
	int magpowcompareval = (int)((float)PWMHalfPeriod * power);

	IOWR(MAGNET_PWM_BASE, PWM_REG_UPDATE, 0);
	IOWR(MAGNET_PWM_BASE, 0, magpowcompareval);
	IOWR(MAGNET_PWM_BASE, PWM_REG_UPDATE, 1);
}

int main()
{
	//printf("Hello from Nios II!\n");
	//while(1);

	setup_axis(&axes[0]);
	setup_axis(&axes[1]);
	init_routine();

	//Open loop spin
	//for (float ph = 0.0f; ; ph += 200.0f/(float)PWMFrequency)
	//	blocking_polar_control_current(ph, lockinCurrent);

	//while(1);

	setup_brake_resistor();
	setup_mag_pwm();

	for(int ax = 0; ax < numaxes; ++ax){
		axes[ax].oldenc = IORD(axes[ax].qei_base, QEI_REG_COUNT);
	}
	//TODO waypoints[0] = IORD(axis->qei_base, QEI_REG_COUNT) * encToPhasefactor

	while(1){
		for(int i = 0; i < 1; ++i)
		for(int wpt = 1; wpt < num_mag_wpts; ++wpt){

			float startpos[2] = {magpickup_waypoints[wpt-1].x, magpickup_waypoints[wpt-1].y};
			float endpos[2] = {magpickup_waypoints[wpt].x, magpickup_waypoints[wpt].y};
			float deltapos[2] = {endpos[0] - startpos[0], endpos[1] - startpos[1]};

			float param_accel = profileAccel * Q_rsqrt(deltapos[0]*deltapos[0] + deltapos[1]*deltapos[1] + 0.1f + magpickup_waypoints[wpt].duration_modifier);
			float I_param = Aperaccel * param_accel;

			float dir_sign[2];
			for(int dim = 0; dim < 2; ++dim){
				dir_sign[dim] = (endpos[dim] >= startpos[dim]) ? 1.0f : -1.0f;
			}

			float t = 0.0f;
			float px = 0.0f;

			float magpower = magpickup_waypoints[wpt].magStr;

			//accelerate
			do {

				px = 0.5f*param_accel*t*t;
				float param_omega = param_accel*t;

				float IbusEst[numaxes];
				for(int ax = 0; ax < numaxes; ++ax){

					float abs_pos = startpos[ax] + px*deltapos[ax];
					float setpoint_omega = param_omega*deltapos[ax];
					float I_ff = I_param*deltapos[ax] + frictionCurrent*dir_sign[ax];

					blocking_control_motor(&axes[ax], abs_pos, setpoint_omega, I_ff, &IbusEst[ax]);
				}
				dump_excess_current(IbusEst, numaxes);

				set_mag_power(magpower);

				t += dt;
			} while( px < 0.5f );

			t -= dt;

			//decelerate
			while(t > 0.0f){
				px = 0.5f*param_accel*t*t;
				float param_omega = param_accel*t;

				float IbusEst[numaxes];
				for(int ax = 0; ax < numaxes; ++ax){

					float abs_pos = endpos[ax] - px*deltapos[ax];
					float setpoint_omega = param_omega*deltapos[ax];
					float I_ff = -I_param*deltapos[ax] + frictionCurrent*dir_sign[ax];

					blocking_control_motor(&axes[ax], abs_pos, setpoint_omega, I_ff, &IbusEst[ax]);
				}
				dump_excess_current(IbusEst, numaxes);

				set_mag_power(magpower);

				t -= dt;
			}

		}

		//idle
		for(int i = 0; i < (15.0f*PWMFrequency); ++i){

			float IbusEst[numaxes];
			for(int ax = 0; ax < numaxes; ++ax){
				float abs_pos = 0.0f;
				float setpoint_omega = 0.0f;
				float I_ff = 0.0f;

				blocking_control_motor(&axes[ax], abs_pos, setpoint_omega, I_ff, &IbusEst[ax]);
			}
			dump_excess_current(IbusEst, numaxes);

			set_mag_power(0.0f);

		}


	}


}

////Use only for simulation!
//int alt_main(){
//	return main();
//}
