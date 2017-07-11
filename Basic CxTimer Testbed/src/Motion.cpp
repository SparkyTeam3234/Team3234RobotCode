#include <iostream>

#include "Motion.h"
#include "math.h"

//CxTimer m_t;
float tst;

Motion::Motion()
{
	m_k = 0.0;
	m_fpo =0;
	m_spo =0;
	m_dto =1;
	m_done =true;
	m_tol =0;

}

Motion::Motion(float k)
{
	m_k = k;
	m_fpo =0;
	m_spo =0;
	m_dto =1;
	m_done =true;
	m_tol =0;

}

double Motion::P345(double t) // t ranges 0-1 and returns position 0-1
{
	double t2=t*t;
	double t3=t2*t;
	return (10*t3)-(15*t2*t2)+(6*t2*t3);
}

double Motion::V345(double t) // returns velocity a point in motion
{
	double t2=t*t;
	double t3=t2*t;
	return (30*t2)-(60*t3)+(30*t2*t2);
}

float Motion::FB(float pa, float ps)	//Feedbacm_k Algorithm
{
	// m_k 	: 	gain factor;error scalar
	// pa	:	actual position
	// ps	:	ideal position
	float v=0;
	if(pa>ps) v=-m_k*sqrt(pa-ps);
	if(pa<ps) v= m_k*sqrt(ps-pa);
	if(v<-1) v=-1.0;
	if(v>1)  v=1.0;
	return v;
}


float Motion::GetPower(float cp, float fp, float dt, float k, bool mon, float tol, CxTimer &m_t)
{
	long dtx =0;
	if (m_fpo != fp) // when final position changes start the move
	{
		m_fpo = fp; // save final position
		m_spo = cp; // save current position as start position
		m_dto = dt; // get how long the move is going to take
		m_k = k;
		m_tol=tol; // save the tolerance to how close the final position must be
		m_t.Update();
		m_t.Reset(); // reset the timer
		m_done = false; // not done
	}

	dtx= (int) m_dto*100; // get ticks in hundreds of a second
	//m_t.UpdateCurrent();

	if(!m_done)
	{
		double t = m_t.GetTime(); // get time
		double tp = (m_fpo - m_spo)*P345(t/m_dto) + m_spo; //calculate target position
		m_done = m_t.CkTime(!m_done, dtx); //check if time is up
		if(abs(cp-fp)<m_tol) m_done=true; // check if close enough
		tst++;
		return FB(cp, tp); // return velocity based on position error
	}
	else
	{
		return 0;
	}
}





void Motion::Reset(float k)
{
	m_k = k;
	m_fpo =0;
	m_spo =0;
	m_dto =1;
	m_done =true;
	m_tol =0;
}

void Motion::Reset()
{
	m_k = 0.0;
	m_fpo =0;
	m_spo =0;
	m_dto =1;
	m_done =true;
	m_tol =0;
}

//Get Functions
float Motion::Getk(){return Motion::m_k;}
bool Motion::GetDone(){return m_done;}

//float Motion::Gett(){return m_t.GetTimeSec();}
//float Motion::Gettp(){return (m_fpo - m_spo)*P345(m_t.GetTimeSec()/m_dto) + m_spo;}
float Motion::Gettst(){return tst;}
