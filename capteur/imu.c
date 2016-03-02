/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu.c
 * Inertial Measurement Unit interface.
 */

#include "imu.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>
#include "navdata.h"
#include "conf_capteur.h"

struct Imu imu;
struct ImuFloat imuF;

void imu_init(void) {
  
		imu_data_available = FALSE;
		navdata_init();

// Set to 0 

	imuF.eulers.theta = 0.;
	imuF.eulers.phi = 0. ;
	imuF.eulers.psi = 0. ;
	imuF.time = 0.;
	imuF.eulers_prevLPF.theta = 0.;
	imuF.eulers_prevLPF.phi = 0.;
	imuF.eulers_prevLPF.psi = 0.;
	imuF.tauFeulers =50.;
		
  /*
    Compute quaternion and rotation matrix
    for conversions between body and imu frame
  */
  struct Int32Eulers eulers =
    { ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_PHI),
      ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_THETA),
      ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_PSI) };
  INT32_QUAT_OF_EULERS(imu.body_to_imu_quat, eulers);
  INT32_QUAT_NORMALIZE(imu.body_to_imu_quat);
  INT32_RMAT_OF_EULERS(imu.body_to_imu_rmat, eulers);
  imu_data_available = FALSE;
  INT_RATES_ZERO(imu.gyro_prev);
  INT_VECT3_ZERO(imu.accel_prev);
}

void imu_flatTrim(void) {
	
	int16_t i = 0;
	int32_t vx = 0 ;
	int32_t vy = 0 ; 
	int32_t vz = 0 ;
	uint32_t ax = 0 ;
	uint32_t ay = 0 ;
	uint32_t az = 0 ;
	int32_t mx = 0 ;
	int32_t my = 0 ;
	int32_t mz = 0 ;
	int32_t us = 0 ;


  // initialises neutrals
	printf("*** Neutral Calibration ***\n");

	while(i<30){	
		navdata_update(); 
		if (navdata_imu_available == TRUE) {
    			navdata_imu_available = FALSE;
			
			vx = vx + navdata->vx;
			vy = vy + navdata->vy;
			vz = vz + navdata->vz;
			ax = ax + navdata->ax;
			ay = ay + navdata->ay;
			az = az + navdata->az;
			mx = mx + navdata->mx;
			my = my + navdata->my;
			mz = mz + navdata->mz;
    			
			// To reject points that have a value close to 34000
			if(navdata->ultrasound <10000){
			us = us + navdata->ultrasound;
			}
			else{
			us = us + us;
			}

			i=i+1;
			imu_data_available = TRUE;
		}
		else {
		    	imu_data_available = FALSE;
		}
	}
	

  RATES_ASSIGN(imu.gyro_neutral, vx/(i),  vy/(i),  vz/(i));

  VECT3_ASSIGN(imu.accel_neutral, ax/(i), ay/(i), az/(i));

// On sait que le neutre du capteur ultrasound est de 875
  imu.neutral_us = 875;

#if defined IMU_MAG_X_NEUTRAL && defined IMU_MAG_Y_NEUTRAL && defined IMU_MAG_Z_NEUTRAL
  VECT3_ASSIGN(imu.mag_neutral, mx/(i), my/(i), mz/(i));
#else
#if USE_MAGNETOMETER
INFO("Magnetometer neutrals are set to zero, you should calibrate!")
#endif
  INT_VECT3_ZERO(imu.mag_neutral);
#endif



	printf("*** Neutral Calibrated ***\n");
}

void imu_periodic(void) {
  navdata_update();
  //checks if the navboard has a new dataset ready
  if (navdata_imu_available == TRUE) {
    navdata_imu_available = FALSE;
    RATES_ASSIGN(imu.gyro_unscaled, navdata->vx, navdata->vy, navdata->vz);
    VECT3_ASSIGN(imu.accel_unscaled, navdata->ax, navdata->ay, navdata->az);
    VECT3_ASSIGN(imu.mag_unscaled, navdata->mx, navdata->my, navdata->mz);
    
    // To reject points that have a value close to 34000
    if(navdata->ultrasound <10000){
    imu.alt_unscaled_us = navdata->ultrasound;
    }

    imu_data_available = TRUE;
  }
  else {
    imu_data_available = FALSE;
  }
}


void imu_periodicScaledInt2Float(void){

	imuF.time = imuF.time + 1./PERIODIC_FREQUENCY;
    	ImuScaleGyro(imu);
	ImuScaleAccel(imu);
	ImuScaleMag(imu);
	ImuScaleUltrasound(imu);
	RATES_ASSIGN(imuF.gyro, (float)imu.gyro.p*2000./(pow(2,15)), (float)imu.gyro.q*2000./(pow(2,15)), (float)imu.gyro.r*2000./(pow(2,15)));
	VECT3_ASSIGN(imuF.accel, (float)imu.accel.x*9.81/512, (float)imu.accel.y*9.81/512, (float)imu.accel.z*9.81/512);
	
	// non linear sensor, 2 gains for 2 cases : <40 cm et >40 cm

	if(imu.alt_us < 407){     // means <40 cm
	imuF.raw_altitude = (float)imu.alt_us * 0.0009828009;
	}
	else{
	imuF.raw_altitude = (float)imu.alt_us * 0.000282748694 + 0.283959;
	}

	// Passe bas sur l'altitude

	imuF.altitude = imuF.prev_altitude + 10/PERIODIC_FREQUENCY*(imuF.raw_altitude - imuF.prev_altitude);
	imuF.prev_altitude = imuF.altitude;
}


void imu_euler(void){
	// using complementary filter over phi and theta. Only gyro for psi (TODO: add magnetometer for psi)

	imuF.eulers.phi = 0.98*(imuF.eulers.phi+ imuF.gyro.p*1./PERIODIC_FREQUENCY)+0.02*(atan2f(imuF.accel.y , sqrt(pow(imuF.accel.x,2)+pow(imuF.accel.z,2)))*180./3.1415);
	imuF.eulers.theta = 0.98*(imuF.eulers.theta + imuF.gyro.q*1./PERIODIC_FREQUENCY)+0.02*(atan2f(imuF.accel.x,sqrt(pow(imuF.accel.y,2)+pow(imuF.accel.z,2)))*180./3.1415);
	imuF.eulers.psi = imuF.eulers.psi + imuF.gyro.r*1./PERIODIC_FREQUENCY;

	
	// Inutile, il faut retravailler le filtre complémentaire plutot que rajouter un autre filtre passe bas.
	/*// Low pass filter

	imuF.eulers.phi = imuF.eulers_prevLPF.phi + imuF.tauFeulers/PERIODIC_FREQUENCY*(imuF.eulers_CF.phi - imuF.eulers_prevLPF.phi);
	imuF.eulers.theta = imuF.eulers_prevLPF.theta + imuF.tauFeulers/PERIODIC_FREQUENCY*(imuF.eulers_CF.theta - imuF.eulers_prevLPF.theta);
	imuF.eulers.psi = imuF.eulers_prevLPF.psi + imuF.tauFeulers/PERIODIC_FREQUENCY*(imuF.eulers_CF.psi - imuF.eulers_prevLPF.psi);

	imuF.eulers_prevLPF.phi = imuF.eulers.phi;
	imuF.eulers_prevLPF.theta = imuF.eulers.theta;
	imuF.eulers_prevLPF.psi = imuF.eulers.psi;
	*/

	
	// compute rad eulers

	imuF.eulers_rad.phi = imuF.eulers.phi*2*3.14159265/360;
	imuF.eulers_rad.theta = imuF.eulers.theta*2*3.14159265/360;
	imuF.eulers_rad.psi = imuF.eulers.psi*2*3.14159265/360;
}


void imu_printNeutral(void){
	// Affichage valeur neutres
		printf("Navdata Neutral : \n");
		printf("vx : %d , vy : %d , vz : %d \n", imu.gyro_neutral.p, imu.gyro_neutral.q, imu.gyro_neutral.r);		
		printf("ax : %d , ay : %d , az : %d \n", imu.accel_neutral.x, imu.accel_neutral.y, imu.accel_neutral.z); 
		printf("mx : %d , my : %d , mz : %d \n", imu.mag_neutral.x, imu.mag_neutral.y, imu.mag_neutral.z); 
}


void imu_printRawNavdata(void){
	// Affichage valeur actuelles
		printf("RawNavdata : \n");
		printf("gyr_p : %d , gyr_q : %d , gyr_r : %d \n", imu.gyro_unscaled.p, imu.gyro_unscaled.q, imu.gyro_unscaled.r);
		printf("acc_x : %d , acc_y : %d , acc_z : %d \n", imu.accel_unscaled.x, imu.accel_unscaled.y, imu.accel_unscaled.z); 
		printf("mag_x : %d , mag_y : %d , mag_z : %d \n", imu.mag_unscaled.x, imu.mag_unscaled.y, imu.mag_unscaled.z);
		printf("altitude : %d \n",imu.alt_unscaled_us);
}

void imu_printIntNavdata(void){
	// Affichage valeur actuelles
		printf("IntNavdata : \n");
		printf("gyr_p : %d , gyr_q : %d , gyr_r : %d \n", imu.gyro.p, imu.gyro.q, imu.gyro.r);
		printf("acc_x : %d , acc_y : %d , acc_z : %d \n", imu.accel.x, imu.accel.y, imu.accel.z); 
		printf("mag_x : %d , mag_y : %d , mag_z : %d \n", imu.mag.x, imu.mag.y, imu.mag.z);
		printf("theta : %f , phi : %f , psi : %f \n", imuF.eulers.theta, imuF.eulers.phi, imuF.eulers.psi);		
		printf("altitude : %d \n",imu.alt_us);

}

void imu_printFloatNavdata(void){
	// Affichage valeur actuelles
		printf("FloatNavdata : \n");
		printf("gyr_p : %f , gyr_q : %f , gyr_r : %f \n", imuF.gyro.p, imuF.gyro.q, imuF.gyro.r);
		printf("acc_x : %f , acc_y : %f , acc_z : %f \n", imuF.accel.x, imuF.accel.y, imuF.accel.z); 
		printf("mag_x : %f , mag_y : %f , mag_z : %f \n", imuF.mag.x, imuF.mag.y, imuF.mag.z);
		printf("theta : %f , phi : %f , psi : %f \n", -imuF.eulers.theta, imuF.eulers.phi, imuF.eulers.psi);		// explication signe moins dans stabilisation.c
		printf("altitude : %f \n",imuF.altitude);

}

void imu_dataInit(void){
	FILE* sensorData = NULL;

		sensorData = fopen("sensorData.txt","w+");
		fprintf(sensorData, "# 0time	1gyr_p	2gyr_q	3gyr_r	4acc_x	5acc_y	6acc_z	7mag_x	8mag_y	9mag_z	10altitude	11theta	12phi	13psi\n");
		fclose(sensorData);
}

void imu_dataCollect(void){
	FILE* sensorData = NULL;

		sensorData = fopen("sensorData.txt","a");
		fprintf(sensorData, "%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%d	%f	%f	%f	%f 	%f	%f\n",imuF.time, imuF.gyro.p, imuF.gyro.q, imuF.gyro.r,imuF.accel.x, imuF.accel.y, imuF.accel.z, imuF.mag.x, imuF.mag.y, imuF.mag.z, imu.alt_us, imuF.eulers.theta, imuF.eulers.phi, imuF.eulers.psi,imuF.eulers_CF.theta,imuF.eulers_CF.phi,imuF.eulers_CF.psi);
		fclose(sensorData);
	
}


