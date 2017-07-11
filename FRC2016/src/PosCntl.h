#pragma once


// simple positional feedback control based on velocity is square root of positional error
// a single gain factor
// 
class PosCntl
{
private:
	//int targetp;  //spare not used
	float k;
	
public:
	//Constructors
	PosCntl();
	PosCntl(float k);
	
	
	float FB(float cp, float tp);	//Feedback Function
	bool chkDone(float cp, float tp, float tol);
	
	void reset(float k);
	void reset();
	
	//Return Functions
};
