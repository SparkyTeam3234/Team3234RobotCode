#include "poscntl.h"
#include "math.h"

PosCntl::PosCntl()
{
	PosCntl::k = 0.0;

}

PosCntl::PosCntl(float x)
{
	PosCntl::k = x;

}
bool PosCntl::chkDone(float cp, float tp, float tol)
{
	int mydiff=cp-tp;
	if(mydiff<0) mydiff=-mydiff;
	if(mydiff<tol) return true;
	return false;
}

float PosCntl::FB(float cp, float tp)	//Feedback Function
{
	// k 	: 	gain factor;error scalar
	// cp	:	actual position
	// tp	:	target position
	float vo=0.0;
	if((cp-tp)<3 && (cp-tp)>-3) return vo;  // dead band if close return 0 output
	
	// velocity is set to the square root of positional error times gain
	if(cp>tp) vo=-k*sqrt(cp-tp);  
	if(cp<tp) vo= k*sqrt(tp-cp);
	// clamp values to max range
	if(vo>1) vo=1.0;
	if(vo<-1.0) vo=-1.0;
	return vo;
}

void PosCntl::reset(float x)
{
	PosCntl::k = x;
}
void PosCntl::reset()
{
	PosCntl::k = 0;
}
