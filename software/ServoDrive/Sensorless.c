
// Algorithm from paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
// Retreived from: http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf

static const float sensorless_gamma = 50.0f/motor.L;
static const float sensorless_PLL_Kp = 40.0f;
static const float sensorless_PLL_Ki = 40000.0f;
static const float PLL_lock_var = 0.0075f;
static const float PLL_unlock_var = (2.0f * 2.0f) * PLL_lock_var;
static const float PLL_lock_magsqr =  0.2f;
static const float PLL_unlock_magsqr = (0.5f * 0.5f) * PLL_lock_magsqr;

static const float PLL_lockfilter_tau = 0.1f; //filter timeconstant, seconds
static const float PLL_lockfilter_c = 2.0f / (1.0f + 2.0f * (PLL_lockfilter_tau / dt)); //discrete time filter constant

//beware of inserting large angles!
float wrap_pm_pi(float theta) {
	while (theta >= M_PI) theta -= M_TWOPI;
	while (theta < -M_PI) theta += M_TWOPI;
	return theta;
}

//Valpha and Vbeta reffers to the voltage that was applied during the PWM period prior to the current measurement instance for Ialpha and Ibeta
//returns true if PLL is locked
bool sensorless_estimate(float Ialpha, float Ibeta, float Valpha, float Vbeta, float* theta_out, float* omega_out){

	static float x[2] = {0.0f, 0.0f};
	static float PLL_theta = 0.0f;
	static float PLL_omega = 0.0f;
	static float PLL_var = 10.0f;
	static bool PLL_locked = false;

	//total flux linkage (x) estimator 
	float y[2] =	{ -motor.R * Ialpha + Valpha
					, -motor.R * Ibeta  + Vbeta };

	float eta[2] = 	{ x[0] - motor.L * Ialpha
					, x[1] - motor.L * Ibeta };

	float scalefactor = 0.5f * sensorless_gamma * (motor.lambda * motor.lambda - (eta[0] * eta[0] + eta[1] * eta[1]));

	for (int i = 0; i < 2; ++i) {
		float xdot = y[i] + scalefactor * eta[i];
		x[i] += xdot * dt;
	}


	//PLL
	float PM_flux_linkage[2] = { x[0] - motor.L * Ialpha, x[1] - motor.L * Ibeta };
	float PM_flux_angle = fast_atan2(PM_flux_linkage[1], PM_flux_linkage[0]);
	float delta_theta = wrap_pm_pi(PM_flux_angle - PLL_theta);
	
	PLL_theta += (PLL_omega + sensorless_PLL_Kp * delta_theta) * dt;
	PLL_theta = wrap_pm_pi(PLL_theta);
	*theta_out = PLL_theta;

	PLL_omega += sensorless_PLL_Ki * delta_theta * dt;
	*omega_out = PLL_omega;

	
	//PLL lock detection
	//variance LPF
	PLL_var *= (1-PLL_lockfilter_c);
	PLL_var += PLL_lockfilter_c * (delta_theta * delta_theta);

	float PM_flux_magsqr = PM_flux_linkage[0] * PM_flux_linkage[0] + PM_flux_linkage[1] * PM_flux_linkage[1];
	
	//Hysteresis lock check
	if (PLL_locked) {
		if ( PLL_var > PLL_unlock_var || PM_flux_magsqr < PLL_unlock_magsqr) {
			PLL_locked = false;
		}
	} else {
		if ( PLL_var < PLL_lock_var && PM_flux_magsqr > PLL_lock_magsqr) {
			PLL_locked = true;
		}
	}

	return PLL_locked;

}