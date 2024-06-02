#include "airspeed.h"

AirSpeedDataStruct AirSpeedData;
double DiffPressure;

MS5525_Status AirSpeedGet(void)
{
	MS5525_Ret = MS5525_Measure();
	if(MS5525_Ret == MS5525_ERR) return MS5525_ERR;
	else
	{
		MS5525_Converse(&MS5525_StaticData);
		MS5525_Converse(&MS5525_TotalData);
	}
	return MS5525_OK;
}

void AirSpeedCalibration(void)
{
	uint8_t i;
	MS5525_Reset(0x76);
	MS5525_Reset(0x77);
	MS5525_Calibration(0x76,&MS5525_StaticData);
	MS5525_Calibration(0x77,&MS5525_TotalData);
	MS5525_StaticData.pre_init = 0;
	MS5525_TotalData.pre_init = 0;
	for(i=0;i<10;i++)
	{
		MS5525_Ret = MS5525_Measure();
		MS5525_Converse(&MS5525_StaticData);
		MS5525_Converse(&MS5525_TotalData);
		MS5525_StaticData.pre_init += MS5525_StaticData.pre;
		MS5525_TotalData.pre_init += MS5525_TotalData.pre;
	}
	MS5525_StaticData.pre_init = MS5525_StaticData.pre_init/10.0;
	MS5525_TotalData.pre_init = MS5525_TotalData.pre_init/10.0;
	AirSpeedData.AltitudeInit = AltitudeGet(MS5525_StaticData.pre_init);
}

double AltitudeGet(double pre)
{
	return 288.15*(1-pow(pre/101325,0.0065*287.05287/9.8))/0.0065;
}

float calc_IAS_corrected(float tube_len, float tube_dia_mm, float differential_pressure, float pressure_ambient, float temperature_celsius)
{
	tube_len = 0;
	tube_dia_mm = 1.5;
	volatile double DiffPre = differential_pressure;
	// air density in kg/m3
	volatile float rho_air = get_air_density(pressure_ambient, temperature_celsius);
	volatile float dp = fabsf(differential_pressure);
	volatile float dp_tot = dp;
	volatile float dv = 0.0f;
	
//	//第一种补偿方法
//	// assumes a metal pitot tube with round tip as here: https://drotek.com/shop/2986-large_default/sdp3x-airspeed-sensor-kit-sdp31.jpg
//	// and tubing as provided by px4/drotek (1.5 mm diameter)
//	// The tube_len represents the length of the tubes connecting the pitot to the sensor.
//	volatile const float dp_corr = dp * 96600.0f / pressure_ambient;
//	// flow through sensor
//	volatile float flow_SDP33 = (300.805f - 300.878f / (0.00344205f * powf(dp_corr, 0.68698f) + 1.0f)) * 1.29f / rho_air;

//	// for too small readings the compensation might result in a negative flow which causes numerical issues
//	if (flow_SDP33 < 0.0f) 
//	{
//		flow_SDP33 = 0.0f;
//	}
//	volatile float dp_pitot = 0.0f;		
//	dp_pitot = (0.0032f * flow_SDP33 * flow_SDP33 + 0.0123f * flow_SDP33 + 1.0f) * 1.29f / rho_air;
//	// pressure drop through tube
//	volatile const float dp_tube = (flow_SDP33 * 0.674f) / 450.0f * tube_len * rho_air / 1.29f;
//	// speed at pitot-tube tip due to flow through sensor
//	dv = 0.125f * flow_SDP33;
//	// sum of all pressure drops
//	dp_tot = dp_corr + dp_tube + dp_pitot;

	
	//第二种补偿方法
	
	// Pressure loss compensation as defined in https://goo.gl/UHV1Vv.
	// tube_dia_mm: Diameter in mm of the pitot and tubes, must have the same diameter.
	// tube_len: Length of the tubes connecting the pitot to the sensor and the static + dynamic port length of the pitot.

	// check if the tube diameter and dp is nonzero to avoid division by 0
//	if ((tube_dia_mm > 0.0f) && (dp > 0.0f)) 
//	{
//		float d_tubePow4 = powf(tube_dia_mm * 1e-3f, 4);
//		float denominator = M_PI_F * d_tubePow4 * rho_air * dp;
//		// avoid division by 0
//		float eps = 0.0f;
//		if (fabsf(denominator) > 1e-32f) 
//		{
//			float viscosity = (18.205f + 0.0484f * (temperature_celsius - 20.0f)) * 1e-6f;
//			// 4.79 * 1e-7 -> mass flow through sensor
//			// 59.5 -> dp sensor constant where linear and quadratic contribution to dp vs flow is equal
//			eps = -64.0f * tube_len * viscosity * 4.79f * 1e-7f * (sqrtf(1.0f + 8.0f * dp / 59.3319f) - 1.0f) / denominator;
//		}
//		// range check on eps
//		if (fabsf(eps) >= 1.0f) 
//		{
//			eps = 0.0f;
//		}
//		// pressure correction
//		dp_tot = dp / (1.0f + eps);
//	}
//	
//	// computed airspeed without correction for inflow-speed at tip of pitot-tube
//	volatile float airspeed_uncorrected = sqrtf(2.0f * dp_tot / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

//	// corrected airspeed
//	volatile float airspeed_corrected = airspeed_uncorrected + dv;

//	// return result with correct sign
//	return (differential_pressure > 0.0f) ? airspeed_corrected : -airspeed_corrected;
	return sqrtf(2.0f * dp_tot / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
}

float get_air_density(float static_pressure, float temperature_celsius)
{
	return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
}



