#include "../capteur/imu.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>
#include "stabilisation.h"

#include "../motorboard/mot.h"
#include "../motorboard/motorboard.h"
#include "../capteur/conf_capteur.h"


struct ImuFloat imuF;
struct StabRef ref;
struct PID PIDz;
struct PID PIDtheta;
struct PID PIDphi;
struct PID PIDpsi;
struct Erreur E;
struct Commande u;
struct PWM pwm;
pthread_t mot_thread;
pthread_mutex_t mot_mutex;



void stab_init(void){

	//Initialise PID Gains

	PIDz.kP = 500;
	PIDz.kI = 300;
	PIDz.kD = 200;
	PIDz.Tf = 100;

	PIDtheta.kP = 13.2;
	PIDtheta.kI = 1.2;
	PIDtheta.kD = 12;
	PIDtheta.Tf = 100;

	PIDphi.kP = 13.2;
	PIDphi.kI = 1.2;
	PIDphi.kD = 12;
	PIDphi.Tf = 100;

	PIDpsi.kP = 13.2;
	PIDpsi.kI = 1.2;
	PIDpsi.kD = 12;
	PIDpsi.Tf = 100;
	
	//Initialise References

	ref.zRef = 0;
	ref.thetaRef = 0;
	ref.phiRef = 0;
	ref.psiRef = 0;

	// Initialise Erreurs

	E.prev_Ez = 0;     // Erreur avant
	E.Ez = 0;          // Erreur actuelle
        E.prev2_EzF = 0;
	E.prev_EzF = 0;
	E.EIz = 0;         // Intégrale de l'erreur
	E.EDz = 0;         // Dérivée de l'erreur 
	E.prev_Etheta = 0;
	E.Etheta = 0;
        E.prev2_EthetaF = 0;
	E.prev_EthetaF = 0;
	E.EItheta = 0;
	E.EDtheta = 0;
	E.prev_Ephi = 0;
	E.Ephi = 0;
        E.prev2_EphiF = 0;
	E.prev_EphiF = 0;
	E.EIphi = 0;
	E.EDphi = 0;
	E.prev_Epsi = 0;
	E.Epsi = 0;
        E.prev2_EpsiF = 0;
	E.prev_EpsiF = 0;
	E.EIpsi = 0;
	E.EDpsi = 0;

	// Initialise commandes

	u.puissance = 0;
	u.prevPuissance = 0;
	u.theta = 0; 
	u.prevTheta = 0;
	u.phi = 0;
	u.prevPhi = 0;
	u.psi = 0;
	u.prevPsi = 0;
	u.puissanceD = 0;
	u.thetaD = 0; 
	u.phiD = 0;
	u.psiD = 0;
	u.puissanceDf = 0;
	u.thetaDf = 0; 
	u.phiDf = 0;
	u.psiDf = 0;
	u.prevPuissanceDf = 0;
	u.prevThetaDf = 0;
	u.prevPhiDf = 0;
	u.prevPsiDf = 0;


}

void stab_update(void){
	
		// Calcul des erreurs	
	E.Ez = ref.zRef - imuF.altitude;
	E.Etheta = ref.thetaRef + imuF.eulers_rad.theta;    // Car on neut pas changer le signe avant avec le filtre, imuF.eulerus_rad.theta est dans le mauvais sens des angles définis positifs
	E.Ephi = ref.phiRef - imuF.eulers_rad.phi;
	E.Epsi = ref.psiRef - imuF.eulers_rad.psi;

	
		// Intégration trapézoidale
	E.EIz = E.EIz + (E.Ez+E.prev_Ez)/2*1/PERIODIC_FREQUENCY;
	E.EItheta = E.EItheta + (E.Etheta+E.prev_Etheta)/2*1/PERIODIC_FREQUENCY;
	E.EIphi = E.EIphi + (E.Ephi+E.prev_Ephi)/2*1/PERIODIC_FREQUENCY;
	E.EIpsi = E.EIpsi + (E.Epsi+E.prev_Epsi)/2*1/PERIODIC_FREQUENCY;

		// Dérivée à gauche
	E.EDz = (E.EzF - E.prev_EzF)*PERIODIC_FREQUENCY;
	E.EDtheta = (E.EthetaF - E.prev_EthetaF)*PERIODIC_FREQUENCY;
	E.EDphi = (E.EphiF - E.prev_EphiF)*PERIODIC_FREQUENCY;
	E.EDpsi = (E.EpsiF - E.prev_EpsiF)*PERIODIC_FREQUENCY;
	


}


void stab_PID(void){

	/* Sert a rien
	// Calcul des commandes
		u.puissance = u.prevPuissance + PIDz.kP*(E.Ez-E.prev_Ez) + PIDz.kI/PERIODIC_FREQUENCY * E.Ez + PIDz.kD*PERIODIC_FREQUENCY*(E.EzF - 2*E.prev_EzF + E.prev2_EzF) ;
	
		if(u.puissance > 500.){
			u.puissance = 500.;
		}

		u.theta = u.prevTheta + PIDtheta.kP*(E.Etheta-E.prev_Etheta) + PIDtheta.kI/PERIODIC_FREQUENCY * E.Etheta + PIDtheta.kD*PERIODIC_FREQUENCY*(E.EthetaF - 2*E.prev_EthetaF + E.prev2_EthetaF) ;
		u.phi = u.prevPhi + PIDphi.kP*(E.Ephi-E.prev_Ephi) + PIDphi.kI/PERIODIC_FREQUENCY * E.Ephi + PIDphi.kD*PERIODIC_FREQUENCY*(E.EphiF - 2*E.prev_EphiF + E.prev2_EphiF) ;
		u.psi = u.prevPsi + PIDpsi.kP*(E.Epsi-E.prev_Epsi) + PIDpsi.kI/PERIODIC_FREQUENCY * E.Epsi + PIDpsi.kD*PERIODIC_FREQUENCY*(E.EpsiF - 2*E.prev_EpsiF + E.prev2_EpsiF) ;

	// mémorisation de la commande précédente
		u.prevPuissance = u.puissance;
		u.prevTheta = u.theta;
		u.prevPhi = u.phi;
		u.prevPsi = u.psi;

	*/
	
	// Calcul des termes dérivés filtrés
		u.puissanceD = PIDz.kD*E.EDz;
		u.puissanceDf = u.prevPuissanceDf + PIDz.Tf/PERIODIC_FREQUENCY * (u.puissanceD - u.prevPuissanceDf);
		u.thetaD = PIDtheta.kD*E.EDtheta;
		u.thetaDf = u.prevThetaDf + PIDtheta.Tf/PERIODIC_FREQUENCY * (u.thetaD - u.prevThetaDf);
		u.phiD = PIDphi.kD*E.EDphi;
		u.phiDf = u.prevPhiDf + PIDphi.Tf/PERIODIC_FREQUENCY * (u.phiD - u.prevPhiDf);
		u.psiD = PIDpsi.kD*E.EDpsi;
		u.psiDf = u.prevPsiDf + PIDpsi.Tf/PERIODIC_FREQUENCY * (u.psiD - u.prevPsiDf);
	
	
	// Calcul des commandes
		u.puissance = 250;//PIDz.kP*E.Ez + PIDz.kI*E.EIz + u.puissanceDf ;
	
		if(u.puissance > 500.){
			u.puissance = 500.;
		}

		u.theta = PIDtheta.kP*E.Etheta + PIDtheta.kI*E.EItheta + u.thetaDf;
		u.phi = PIDphi.kP*E.Ephi + PIDphi.kI*E.EIphi + u.phiDf;
		u.psi = 0; // kP.PIDpsi*E.Epsi + PIDpsi.kI*E.EIpsi + u.psiDf;


	// mémorisation de la commande dérivée précédente pour calcul des filtres
		u.prevPuissanceDf = u.puissanceDf;
		u.prevThetaDf = u.thetaDf;
		u.prevPhiDf = u.phiDf;
		u.prevPsiDf = u.psiDf;


	
}

void stab_computePWM(void){

	pwm.M1 = u.puissance - u.theta - u.phi + u.psi;
	pwm.M2 = u.puissance - u.theta + u.phi - u.psi;
	pwm.M3 = u.puissance + u.theta + u.phi + u.psi;
	pwm.M4 = u.puissance + u.theta - u.phi - u.psi;

}

void stab_setPWM(void){

	mot_Run(pwm.M1/511, pwm.M2/511, pwm.M3/511, pwm.M4/511);

}

void stab_land(void){

	pwm.M1 = pwm.M1-0.08;
	pwm.M2 = pwm.M2-0.08;
	pwm.M3 = pwm.M3-0.08;
	pwm.M4 = pwm.M4-0.08;

	mot_Run(pwm.M1/511, pwm.M2/511, pwm.M3/511, pwm.M4/511);
}

void stab_printStabData(void){

printf("Stabdata: \n");
printf("zRef : %f, thetaRef : %f, phiRef : %f,psiRef : %f\n", ref.zRef, ref.thetaRef, ref.phiRef, ref.psiRef);
printf("Ez : %f, Etheta : %f, Ephi : %f, Epsi : %f\n", E.Ez, E.Etheta, E.Ephi, E.Epsi);
printf("Upuissance : %f , UTheta: %f, UPhi : %f, UPsi : %f\n", u.puissance, u.theta, u.phi, u.psi);
printf("PWM1 : %f, PWM2 : %f, PWM3 : %f, PWM4 : %f\n", pwm.M1, pwm.M2, pwm.M3, pwm.M4);

}

void stab_dataInit(void){
	FILE* stabData = NULL;

		stabData = fopen("stabData.txt","w+");
		fprintf(stabData, "# 1time	2zRef	3thetaRef	4phiRef	5psiRef	6Ez	7Etheta	8Ephi	9Epsi	10Upuissance	11utheta	12uphi	13upsi	14PWMM1	15PWMM2	16PWMM3	17PWMM4\n");
		fclose(stabData);
}

void stab_dataCollect(void){
	FILE* stabData = NULL;

		stabData = fopen("stabData.txt","a");
		fprintf(stabData, "%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",imuF.time, ref.zRef, ref.thetaRef, ref.phiRef, ref.psiRef,E.Ez, E.Etheta, E.Ephi, E.Epsi,u.puissance, u.theta, u.phi, u.psi,pwm.M1, pwm.M2, pwm.M3, pwm.M4);
		fclose(stabData);
}
