#ifndef AHRS_MAGNETIC_FIELD_MODEL_H
#define AHRS_MAGNETIC_FIELD_MODEL_H

#include "../conf_capteur.h"

#ifndef AHRS_H_X
#ifdef AHRS_H_Y
#error Either define both AHRS_H_X and AHRS_H_Y or none, but not half
#endif
#define AHRS_H_X 1
#define AHRS_H_Y 0
#endif

#endif
