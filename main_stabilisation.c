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

//bloc util : pour l'interface sur la console ( A faire)
#include "util/util.h"
#include "util/type.h"

//bloc moteur
#include "motorboard/mot.h"
#include "motorboard/motorboard.h"

//bloc stabilisation
#include "stabilisation/stabilisation.h"

tid_t main_tid;
tid_t main_periodic_tid;
tid_t print_tid;
tid_t land_tid;
int fin=0;
int inc =0;


struct FloatEulers *euler_angles = NULL;
struct Imu imu ;
struct ImuFloat imuF ;
struct StabRef ref;
struct PID PIDz;
struct PID PIDtheta;
struct PID PIDphi;
struct PID PIDpsi;
struct Erreur E;
struct Commande u;
struct PWM pwm;
int c;

int main(){
	// Initialisation
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
	stab_dataInit();

 		// Timers
	main_tid = sys_time_register_timer(30,NULL);
	main_periodic_tid = sys_time_register_timer((1./PERIODIC_FREQUENCY),NULL);
	print_tid = sys_time_register_timer((1),NULL);

	
	mot_SetLeds(MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN);

	printf("*** Initialisation Done ***\n");

	// Message de bienvunue
	printf("Alternative stabilisation software for Parrot ARDrone 2.0\n \n");


		ref.zRef=0.8;
		printf("décollage...\n");

		// Boucle 
		while(!sys_time_check_and_ack_timer(main_tid)){
			//periodic acquisition
			if(sys_time_check_and_ack_timer(main_periodic_tid)){
				imu_periodic();
				imu_periodicScaledInt2Float();
				imu_euler();
				stab_update();
				stab_PID();
				stab_computePWM();
				stab_dataCollect();
				imu_dataCollect();
				//stab_setPWM();
			}	

			
			//periodic print
			if(sys_time_check_and_ack_timer(print_tid)){
				system("clear");				
				imu_printFloatNavdata();
				//stab_printStabData();

			}
		}
		
		// procédure d'attérissance, on aurait aussi pu penser à une rampe jusqu'a z=0	
		land_tid = sys_time_register_timer((3),NULL);
		while(!sys_time_check_and_ack_timer(land_tid)){
			if(sys_time_check_and_ack_timer(main_periodic_tid)){			
				//stab_land();
			}
		}


		printf("Fin acquisition\n");
		log_close();
	
}

