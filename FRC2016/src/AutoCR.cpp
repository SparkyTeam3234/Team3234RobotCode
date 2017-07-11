#include <cmath>
#include "WPILib.h"
#include "Poscntl.h"
#include "Motion.h"
#include "CxTimer.h"
using namespace std;
/*
 * AutoCR.cpp
 *
 *  Created on: Feb 15, 2016
 *      Author: Not Dave and The Breadman Scam
 */
enum dashbutton {BUTTON_4BITENCODED,BUTTON_1,BUTTON_2,BUTTON_3,BUTTON_4};
class AutoCorrect
{

	double dsuba;
	double const ShrtDist = 98; //placeholder until known
public:
	void RelPos(double arcLng, double idPos)
	{
		double theta;
		double alpha;
		double ofstdist;
		theta = arcLng / 27; //subject to change
		alpha = M_PI_2 - theta;
		ofstdist = ShrtDist / tan(alpha);
		dsuba = ofstdist + idPos;
	};
	int GoalDist(int goal)
	{
		double dTotal;
		double dsubo;
		double csubi=10;
		double f=1;
		dTotal = sqrt(pow(csubi,2) - pow(ShrtDist,2));
		return dsubo = int (dTotal-(dsuba-f));
	};
	int GoalAngle(int goal)
	{
		double theta;
		double alpha;
		double h=5;
		double csubi;
		theta = asin ((h/csubi));
		return alpha = int (M_PI_2 - theta);
	};
	int ConvertToLinearTick(double DistinInch)
	{
		const double LNRCNVRTE = 0;							//Needs to Be Defined
		return int(DistinInch*LNRCNVRTE);
	}
	int ConvertToAngularTick(double DistinRadians)
	{
		const double ANGCNVRTE = 0;							//Needs to Be Defined
		return int(DistinRadians * ANGCNVRTE);
	}
	double ConvertToLinearIn(int DistinTicks)
	{
		const double INLNCNV = 0;
		return DistinTicks * INLNCNV;
	}

	int GetAutoButtons(dashbutton button=BUTTON_4BITENCODED)
	{
		int out;
		bool b1, b2, b3, b4;
		b1 = SmartDashboard::GetBoolean("DB/Button 0", false);
		b2 = SmartDashboard::GetBoolean("DB/Button 1", false);
		b3 = SmartDashboard::GetBoolean("DB/Button 2", false);
		b4 = SmartDashboard::GetBoolean("DB/Button 3", false);
		switch(button)
		{
		case BUTTON_4BITENCODED: out=(b1)+(b2*2)+(b3*4)+(b4*8); break;
		case BUTTON_1: out=b1; break;
		case BUTTON_2: out=b2; break;
		case BUTTON_3: out=b3; break;
		case BUTTON_4: out=b4; break;
		}
		return out;
	};
	int DecodeAutoButtons(int encodedbuttons,dashbutton button=BUTTON_4BITENCODED)
		{
			int out;
			bool b1, b2, b3, b4;
			if (encodedbuttons>=8){b4=1,encodedbuttons-=8;}
			if (encodedbuttons>=4){b3=1,encodedbuttons-=4;}
			if (encodedbuttons>=2){b2=1,encodedbuttons-=2;}
			if (encodedbuttons>=1){b1=1,encodedbuttons-=1;}
			switch(button)
			{
			case BUTTON_4BITENCODED: out=encodedbuttons; break;
			case BUTTON_1: out=b1; break;
			case BUTTON_2: out=b2; break;
			case BUTTON_3: out=b3; break;
			case BUTTON_4: out=b4; break;
			}
			return out;
		};
};
