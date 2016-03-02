#include <stdlib.h>
#include "../math/pprz_algebra_float.h"

//références pour la régulation

struct StabRef{
float zRef;
float thetaRef;
float phiRef;
float psiRef;
float sampleTime;
};

struct Erreur{
float prev_Ez;     // Erreur précédente
float Ez;          // Erreur actuelle
float prev2_EzF;
float prev_EzF;
float EzF;
float EIz;         // Intégrale de l'erreur
float EDz;         // Dérivée de l'erreur 
float prev_Etheta;
float Etheta;
float prev2_EthetaF;
float prev_EthetaF;
float EthetaF;
float EItheta;
float EDtheta;
float prev_Ephi;
float Ephi;
float prev2_EphiF;
float prev_EphiF;
float EphiF;
float EIphi;
float EDphi;
float prev_Epsi;
float Epsi;
float prev2_EpsiF;
float prev_EpsiF;
float EpsiF;
float EIpsi;
float EDpsi;
};

struct Commande{
float puissance;
float prevPuissance;
float puissanceD;
float puissanceDf;
float prevPuissanceDf;
float theta;
float prevTheta;
float thetaD;
float thetaDf;
float prevThetaDf;
float phi;
float prevPhi;
float phiD;
float phiDf;
float prevPhiDf;
float psi;
float prevPsi;
float psiD;
float psiDf;
float prevPsiDf;
};


struct PID{
float kP;
float kI;
float kD;
float Tf;    //constante de temps du filtre
};

struct PWM{
float M1;
float M2;
float M3;
float M4;
};



