/*
    mot.h - AR.Drone motor thread

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/
#ifndef _MOT_H
#define _MOT_H

#include "../util/type.h"
#include "motorboard.h"

//m0=Front Left m1=Front Right m3=Rear Right m4=Rear Left
struct mot_struct
{
  float mot[4]; //motor speed setting. 0.0=min power, 1.0=full power
  u16 pwm[4];   //motor speed 0x00-0x1ff.  -- protected by mutex
  u08 led[4];   //led 0=off 1=red 2=green 3=orange -- protected by mutex
  u08 NeedToSendLedCmd;
};


int mot_Init();
void mot_SetLed(u08 mot_id, u08 led);
void mot_SetLeds(u08 led0, u08 led1, u08 led2, u08 led3);
void mot_Stop();
void mot_Run(float m0, float m1, float m2, float m3);
void mot_GetMot(float *m);
void mot_Close();

#endif
