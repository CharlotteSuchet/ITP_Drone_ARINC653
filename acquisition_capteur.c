#include <stdio.h>   /* Standard input/output definitions */
#include <stdbool.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>
#include <float.h>

#include "sys_time.h"
#include "log.h"
#include "state.h"
#include "commands.h"
#include "conf.h"

//bloc capteur
#include "capteur/imu.h"

//bloc util : pour l'interface sur la console
#include "util/util.h"

//bloc moteur
#include "motorboard/mot.h"
#include "motorboard/motorboard.h"

tid_t main_tid;
tid_t main_periodic_tid;
tid_t print_tid;
tid_t land_tid;
int fin=0;
int inc =0;
int32_t altitude=0; 

struct FloatEulers *euler_angles = NULL;
struct Imu imu ;
struct ImuFloat imuF ;

int main(){

	printf("*** Initialisation ***\n");

	mot_Init();	
	mot_SetLeds(MOT_LEDRED,MOT_LEDRED,MOT_LEDRED,MOT_LEDRED);

	sys_time_init();
	log_init();
	stateInit();
	
		
	imu_init();
	imu_flatTrim();
	imu_printNeutral();
	imu_dataInit();

	stab_init();

 	// Timers
	main_tid = sys_time_register_timer(10,NULL);
	main_periodic_tid = sys_time_register_timer((1./PERIODIC_FREQUENCY),NULL);
	print_tid = sys_time_register_timer((1./500.),NULL);
	land_tid = sys_time_register_timer((0.25),NULL);
	
	mot_SetLeds(MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN);

	printf("*** Initialisation Done ***\n");



	while(!sys_time_check_and_ack_timer(main_tid)){
		//periodic acquisition
		if(sys_time_check_and_ack_timer(main_periodic_tid)){
			imu_periodic();
			imu_periodicScaledInt2Float();
			imu_euler();
			imu_dataCollect();
		}	
		//periodic print
		if(sys_time_check_and_ack_timer(land_tid)){
			imu_printFloatNavdata();

		}
	}
	printf("Fin acquisition\n");
	log_close();
	actuators_ardrone_close();
}

