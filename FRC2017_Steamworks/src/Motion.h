#pragma once
#include "CxTimer.h"

class Motion
{
private:

	CxTimer m_t;
	double m_fpo;
	double m_spo;
	double m_dto;
	bool m_done;
	double m_tol;


	//int m_encoderIP;
	//double m_initTime;
	//double m_runTime;
	float m_k;
	//bool m_active;
	//float m_totalMove;
	
public:
	//Constructors
	Motion();
	Motion(float k);
	
	double P345(double t);					//Position 345 Polynomial
	double V345(double t);					//Velocity 345 Polynomial
	
	float FB(float pa, float ps);//Feedback Function
	
	void Reset(float k);
	void Reset();
	
	//Return Functions
	float GetPower(float mcp, float mfp, float mdt, float mk, bool mmon, float mtol);
	float Getk();
	bool GetDone();
};
