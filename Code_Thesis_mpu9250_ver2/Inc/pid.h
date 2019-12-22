#ifndef PID_H_
#define PID_H_
#include <stdint.h>
#include <stdbool.h>
	
//================= PID Parameters ===============================
typedef struct{
		float Kp;
		float Ki;
		float Kd;
		float dt;
		float Kt;
		float setPoint;
		
		float minInput;
		float maxInput;
		float minOutput;
		float maxOutput;
		
		float PIDResultTemp;
		float PIDResultTemp1;
		
		float PIDResult;
		
		float PIDError;
		float PIDErrorTemp1;
		float PIDErrorTemp2;	
		float iError;
		float processVariable;
		} pid_t;

//=============== PID API ===================================

		void setPoint(pid_t *wheel, float reference);
		void setInputBoder(pid_t *wheel, float minima, float maxima);
		void setOutputBoder(pid_t *wheel, float minima, float maxima);
		float actuatorLimit(pid_t *wheel, float *value2check, int what2check);		
		void setProcessVariable(pid_t *wheel, float value);
		float getProcessVariale(pid_t *wheel);
		float pidFunction(pid_t *wheel, float kp, float ki, float kd, float dt);
		float pidFunction2(pid_t *wheel, float kp, float ki, float kd, float dt);
		float pidCompensatorFunction(pid_t *wheel, float kp, float ki, float kd, float dt);
		void pidReset(pid_t *wheel);
#endif /* PID_H_ */
