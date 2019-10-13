#include <math.h>
#include <stdlib.h>
#include "pid.h"

//================================================================
void setPoint(pid_t *wheel, float reference)
	{
		 wheel->setPoint = reference;
			
	}
//================================================================
void setInputBoder(pid_t *wheel,float minima,float maxima)
	{
		wheel->minInput=minima;
		wheel->maxInput=maxima;
	}
//================================================================
void setOutputBoder(pid_t *wheel, float minima,float maxima)
	{
		wheel->minOutput=minima;
		wheel->maxOutput=maxima;
	}
//================================================================	
float actuatorLimit(pid_t *wheel, float *value2check,int what2check)
{
				float err;
			if(what2check==0)
			{				
				if((*value2check) > wheel->maxInput)
				{
						err=(wheel->maxInput - (*value2check));
						*value2check=wheel->maxInput;
					//	return err;	
				}
				else if ((*value2check) < wheel->minInput)
				{
						err=(wheel->minInput - (*value2check));
						*value2check = wheel->minInput;
					//	return err;
				}	
			}
			else if(what2check==1)
				{
				if((*value2check) > wheel->maxOutput)
				{
					err =  (wheel->maxOutput - (*value2check));
					wheel->PIDResult=wheel->maxOutput;
					//return err;
				}
				else if ((*value2check) < wheel->minOutput)
				{
					err=(wheel->minOutput - (*value2check));
					wheel->PIDResult = wheel->minOutput;
			//		return err;
				}
				else
					{
					wheel->PIDResult = wheel->PIDResultTemp;
					return 0;
					}				
			}
				else
					return 0;
		return 0;
}	
//================================================================	
void setProcessVariable(pid_t *wheel, float value)
	{
				wheel->processVariable=value;
	}
//================================================================	
float getProcessVariale(pid_t *wheel)
{
	return wheel->processVariable;
}	
//================================================================	
//float pidFunction(pid_t *wheel, float kp,float ki,float kd, float dt)
//	{
//		wheel->Kp=kp;
//		wheel->Ki=ki;
//		wheel->Kd=kd;
//		wheel->dt=dt;
//		float pPart,iPart,dPart;
//	//	static float iError;
//		
////		actuatorLimit(wheel, &(wheel->setPoint) ,0); //0==INPUT
//		wheel->PIDError = wheel->setPoint - getProcessVariale(wheel);
//		pPart = wheel->Kp * wheel->PIDError;
//		dPart = wheel->Kd * ( wheel->PIDError - wheel->PIDErrorTemp1)/ wheel->dt;
//		iPart += wheel->Ki*wheel->dt*0.5f*(wheel->PIDError+wheel->PIDErrorTemp1);
//		wheel->PIDResultTemp= pPart + iPart + dPart;

////		wheel->PIDResult = actuatorLimit(wheel, &(wheel->PIDResultTemp), 1);
//		
//		wheel->PIDErrorTemp1 = wheel->PIDError;
////		wheel->PIDResultTemp = wheel->PIDResult;
//		return wheel->PIDResult;
//	}
////================================================================	
//float pidCompensatorFunction(pid_t *wheel, float kp,float ki,float kd,float dt)
//{
//	(*param).Kp=kp;
//	(*param).Ki=ki;
//	(*param).Kd=kd;
//	(*param).dt=dt;
//	
//	if((*param).Kd)
//		(*param).Kt=1/sqrt(fabs((*param).Kd/(*param).Ki));
//	else
//		(*param).Kt=1/sqrt(fabs((*param).Kp/(*param).Ki));
//	float dPart,pPart,iPart;
//	actuatorLimit(&(*param).setPoint,0);
//	(*param).PIDError=(*param).setPoint-getProcessVariale();
//	
//	pPart=(*param).Kp*((*param).PIDError-(*param).PIDErrorTemp1);
//	iPart=((*param).Ki)*((*param).dt)*((*param).PIDErrorTemp1)+((*param).Kt*(*param).dt*actuatorLimit(&(*param).PIDResultTemp1,1));
//	dPart=(*param).Kd*((*param).PIDError+(*param).PIDErrorTemp2-(2*((*param).PIDErrorTemp1)))/(*param).dt;
//	
//	(*param).PIDResultTemp=(*param).PIDResultTemp1+pPart+dPart+iPart;
//	
//	actuatorLimit(&(*param).PIDResultTemp,1);
//	
//	(*param).PIDResultTemp1=(*param).PIDResultTemp;
//	(*param).PIDErrorTemp2=(*param).PIDErrorTemp1;
//	(*param).PIDErrorTemp1=(*param).PIDError;
//	
//	return (*param).PIDResult;
//}	
//================================================================
float pidFunction2(pid_t *wheel, float kp,float ki,float kd,float dt)
{
	wheel->Kp=kp;
	wheel->Ki=ki;
	wheel->Kd=kd;
	wheel->dt=dt;
	
	float dPart,pPart,iPart;
	wheel->PIDError = wheel->setPoint-getProcessVariale(wheel);
	if((wheel->PIDError)>500||(wheel->PIDError)<-500)	
	{
		pidReset(wheel);
//		wheel->PIDError = wheel->PIDErrorTemp1;
	}
	pPart =wheel->Kp*(wheel->PIDError-wheel->PIDErrorTemp1);
	dPart =wheel->Kd*(wheel->PIDError-2*wheel->PIDErrorTemp1+wheel->PIDErrorTemp2)/wheel->dt;
	iPart = wheel->Ki * wheel->dt * 0.5f * (wheel->PIDError+wheel->PIDErrorTemp1);
	wheel->PIDResultTemp = wheel->PIDResultTemp1+pPart+dPart+iPart;
	
	actuatorLimit(wheel, &(wheel->PIDResultTemp), 1);
	
	wheel->PIDResultTemp1 = wheel->PIDResult;
	wheel->PIDErrorTemp2 = wheel->PIDErrorTemp1;
	wheel->PIDErrorTemp1 = wheel->PIDError;
	return wheel->PIDResult;
}
//================================================================	
void pidReset(pid_t *wheel)
{
		wheel->iError=0.0f;
		wheel->PIDError=0.0f;
		wheel->PIDErrorTemp1=0.0f;
		wheel->PIDErrorTemp2=0.0f;
		wheel->PIDResultTemp=0.0f;
		wheel->PIDResult=0.0f;
		wheel->PIDResultTemp1=0.0f;
	
}
