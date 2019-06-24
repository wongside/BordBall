#ifndef __PID_H_
#define __PID_H_

typedef struct
{
  float Kp;
  float Ki;
  float Kd;

  float Ek;
  float Ek1;
  float Ek2;
}PID_IncTypeDef;

float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID);
void PID_Init(PID_IncTypeDef *PID);
void PID_Set_Value(PID_IncTypeDef *PID, float Kp, float Ki, float Kd);

#endif