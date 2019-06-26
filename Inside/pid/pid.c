#include "pid.h"

float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{
  float PIDInc;
 
  PID->Ek = SetValue - ActualValue;
  PIDInc = (PID->Kp * PID->Ek) - PID->Ek1 + (PID->Kd * PID->Ek2);
 
  PID->Ek2 = PID->Ek1;
  PID->Ek1 += PID->Ki *PID->Ek;
	if(PID->Ek1>1.1f)PID->Ek1=1.1f;
	else if(PID->Ek1<-1.1f)PID->Ek1=-1.1f;
	
	return PIDInc;
}

void PID_Init(PID_IncTypeDef *PID){
	PID->Kp = 0;
	PID->Ki = 0;
	PID->Kd = 0;
	PID->Ek = 0;
	PID->Ek1 = 0;
	PID->Ek2 = 0;
}

void PID_Set_Value(PID_IncTypeDef *PID, float Kp, float Ki, float Kd){

	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;

}