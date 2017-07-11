#include <thread>
#include <iostream>
#include <memory>
#include <string>
#include <Talon.h>
#include <Joystick.h>
#include <Timer.h>
#include <WPILib.h>
#include <math.h>
#include <ADXRS450_Gyro.h>
#include "Motion.h"
#include <CameraServer.h>
#include "time.h"

#include <SPI.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "gyrofunctions.cpp"



using namespace frc;
using namespace std;

class Robot: public IterativeRobot
{
	bool brendancontrols = false;
	bool bckrst = true;
	bool m_autosw1 = false;
	bool m_autosw2 = false;
	bool m_autosw3 = false;

	float m_culEnc = 0;
	float m_curEnc = 0;
	int gyrocrrt = 0;
	int m_autostp = 0;
	int m_automax=0;
	int m_autopgnum=0;
	float gyroAngle = 0;
	float drstk_x=0;
	float drstk_y=0;
	float drstk_z=0;
	float drstk_t=0;
	float m_lstkz=0;
	float m_rng=0;
	float m_curbckEnc=0;
	float m_lstt=0;
	float m_dt=0;
	bool toplim=false;
	bool bttlim=false;

	long ldtx =0;
	float m_lfpo=0;
	float m_lspo=0;
	float m_ldto=0;
	float m_ltol=0;

	long rdtx =0;
	float m_rfpo=0;
	float m_rspo=0;
	float m_rdto=0;
	float m_rtol=0;
	/*
	float Time_GetTime;
	float Time_GetTimeSec;
	float Output;

	float m_fpo;
	float fp;
	float m_spo;
	float m_dto;
	float m_tol;
	float dt;
	float cp;
	float tol;
	double t;
	double tp;
*/
	float m_k = 0.04;
	float m_lEnc = 0;

	bool m_ldone = false;
	bool m_rdone = false;

	Joystick drstk {0};
	Joystick lstk {1};
	Talon lDrv {1};
	Talon rDrv {0};
	Talon bckspn {2};
	Talon flppddl {3};
	Talon Succ {4};
	Talon gear {5};
	Talon popcorn {6};
	CxTimer m_t1;
	CxTimer m_t2;
	Timer m_t3;
	Encoder lEnc {2,3};
	Encoder rEnc {0,1};
	Encoder bckEnc {4,5};
	RobotDrive myrobot;
	ADXRS450_Gyro gyro;
	GyroCorrect ljhgyro;
	AnalogInput rngfndr {0};
	DigitalInput autosw1 {7};
	DigitalInput autosw2 {8};
	DigitalInput autosw3 {9};
	DigitalInput grtplmt {10};
	DigitalInput grbttmlmt {11};

	Motion lDrvMotion;
	Motion rDrvMotion;
	Motion bckspnMotion;

public:
	Robot():
	myrobot (rDrv, lDrv)
	//gyro (SPI::Port::kOnboardCS0)
	{
		myrobot.SetExpiration(0.1);
	}
	void RobotInit() {
		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		cs::UsbCamera camera=CameraServer::GetInstance()->StartAutomaticCapture();
		cs::UsbCamera camera2=CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(320, 240);
		//camera2.SetResolution(160, 120);
		//camera2.SetFPS(15);
		camera.SetFPS(15);
		//gyro.Reset();
		//gyro.Calibrate();
		//gyro.StartLiveWindowMode();
		/*
		cs::UsbCamera camera2=CameraServer::GetInstance()->StartAutomaticCapture();
		camera2.SetResolution(320, 240);
		*/
		m_t1.Update();
		m_t1.Reset();
		m_t2.Update();
		m_t2.Reset();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override
	{
		/*
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		cout << "Auto selected: " << autoSelected << endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
		*/
		m_t1.Update();
		m_t1.Reset();
		m_t2.Update();
		m_t2.Reset();

		rEnc.Reset();
		lEnc.Reset();
		//int autopgnum=0;


	}
	void AutonomousPeriodic()
	{
		UpdateInputs();
		/*
		if (autoSelected == autoNameCustom)
		{
			// Custom Auto goes here
		}
		else
		{
			// Default Auto goes here
		}
		*/

		m_autopgnum=0;
		if(m_autosw1) m_autopgnum=1;
		if(m_autosw2) m_autopgnum +=2;
		if(m_autosw3) m_autopgnum +=4;

		switch(m_autopgnum)
		{
		case 0: //do nothing
			{
				m_automax=2;
				Autonomous0();
				break;
			}
		case 1:
		{
			m_automax=2;
			Autonomous1();
			break;
		}
		case 2:
		{
			m_automax=4;
			Autonomous2();
			break;
		}
		case 3:
		{
			m_automax=4;
			Autonomous3();
			break;
		}
		case 4:
		{
			m_automax=4;
			Autonomous4();
			break;
		}
		case 5:
		{
			m_automax=4;
			Autonomous5();
			break;
		}
		case 6:
		{
			m_automax=8;
			Autonomous6();
			break;
		}
		case 7:
		{
			m_automax=8;
			break;
		}
		}

		if(m_autostp >= m_automax)
				{
					m_autostp = m_automax;
					lDrv.Set(0);
					rDrv.Set(0);
					bckspn.Set(0);
					flppddl.Set(0);
					popcorn.Set(0);
				}


	}
	void Autonomous0()
	{
		/*
		float tp;
		float t= m_t1.GetFPGATimestamp();
		tp = 1414*(10*(pow(t/5,3))-15*(pow(t/5,4))+6*(pow(t/5,5)));

		if(m_culEnc>tp)
			lDrv.Set(sqrt(m_culEnc-tp));
		else
			lDrv.Set(sqrt(tp-m_culEnc));

		if(m_curEnc>tp)
			rDrv.Set(sqrt(m_curEnc-tp));
		else
			rDrv.Set(sqrt(tp-m_curEnc));

		if(abs(m_culEnc)>1414 && abs(m_curEnc) >1414)
			m_autostp++;
			*/
		//SmartDashboard::PutNumber("MotionGetTime",LMotion(m_culEnc, 1414, 3, .04, true, 100));

		switch(m_autostp)
		{
		case 0:
		{
			bckspn.Set(-.540);
			m_t3.Start();
			if(m_t3.Get()>=1)
				m_autostp++;
			break;
		}
		case 1:
		{
			flppddl.Set(1);
			popcorn.Set(-.55);
			if(m_t3.Get()>=15)
				m_autostp++;
			break;
		}
		}
	}
	void Autonomous1() 	//center gear
		{
			switch(m_autostp)
			{
				case 0:
				{
					//1414
					lDrv.Set(LMotion(m_culEnc, 1414, 2, .055, true, 20));
					rDrv.Set(-RMotion(m_curEnc, 1414, 2, .055, true, 20));
					//if(m_ldone && m_rdone)
					m_autostp++;
					break;
				}

				case 1:
				{
					/*
					m_autostp++;
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);
					*/
					if(m_ldone && m_rdone) // if done next step
						{
							m_autostp++;
							lDrv.Set(0); // stop all motion
							rDrv.Set(0);
						}
					else // do action for step
					{
						lDrv.Set(LMotion(m_culEnc, 1414, 2, .055, true, 20));
						rDrv.Set(-RMotion(m_curEnc, 1414, 2, .055, true, 20));
					}
					break;
				}
			}
		}
	void Autonomous2()	//gear (left)
	{
		switch(m_autostp)
		{
			case 0:
			{
				lDrv.Set(LMotion(m_culEnc, 1848, 3, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1848, 3, .04, true, 20));
				m_autostp++;
				break;
			}
			case 1:
			{
				if(m_ldone && m_rdone)
				{
					lDrv.Set(LMotion(m_culEnc, 2088, 1, .04, true, 20));
					rDrv.Set(-RMotion(m_curEnc, 1608, 1, .04, true, 20));
					m_autostp++;
				}
				else
				{
					//1848
					lDrv.Set(LMotion(m_culEnc, 1829, 3, .04, true, 20));
					rDrv.Set(-RMotion(m_curEnc, 1829, 3, .04, true, 20));
				}
				break;
			}
			case 2:
			{
				if(m_ldone && m_rdone)
				{
					lDrv.Set(LMotion(m_culEnc, 2528, 2, .04, true, 20));
					rDrv.Set(-RMotion(m_curEnc, 2046, 2, .04, true, 20));
					m_autostp++;
				}
				else
				{
					lDrv.Set(LMotion(m_culEnc, 2088, 1, .04, true, 20));
					rDrv.Set(-RMotion(m_curEnc, 1608, 1, .04, true, 20));
				}
				break;
			}
			case 3:
			{
				if(m_ldone && m_rdone)
				{
					lDrv.Set(0);
					rDrv.Set(0);
					m_autostp++;
				}
				else
				{
					lDrv.Set(LMotion(m_culEnc, 2528, 2, .04, true, 20));
					rDrv.Set(-RMotion(m_curEnc, 2046, 2, .04, true, 20));
				}

			}
		}
	}
	void Autonomous3()	//gear (right)
	{
		switch(m_autostp)
		{
		case 0:
		{
			lDrv.Set(LMotion(m_culEnc, 1829, 3, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 1829, 3, .04, true, 20));
			m_autostp++;
			break;
		}
		case 1:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 1608, 1, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 2088, 1, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 1829, 3, .04, true, 20)); //18429
				rDrv.Set(-RMotion(m_curEnc, 1829, 3, .04, true, 20));
			}
			break;
		}
		case 2:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 2046, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 2528, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 1608, 1, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 2088, 1, .04, true, 20));
			}
			break;
		}
		case 3:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(0);
				rDrv.Set(0);
				m_autostp++;
			}
			else
			{
			lDrv.Set(LMotion(m_culEnc, 2046, 2, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 2528, 2, .04, true, 20));
			}
			break;
		}
		}
	}
	void Autonomous4()	//red goal + move
	{
		switch(m_autostp)
		{
		case 0:
			{
				bckspn.Set(-.540);
				m_t3.Start();
				if(m_t3.Get()>=1)
					m_autostp++;
				break;
			}
		case 1:
			{
				flppddl.Set(1);
				popcorn.Set(-.55);
				if(m_t3.Get()>=11)
				{
					m_autostp++;
					flppddl.Set(0);
					popcorn.Set(0);
					lDrv.Set(LMotion(m_culEnc, 249, 1, .04, true, 20));
					rDrv.Set(-RMotion(m_culEnc, -249, 1, .04, true, 20));
				}
				break;
			}
		case 2:
		{
			lDrv.Set(LMotion(m_culEnc, 249, 1, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, -249, 1, .04, true, 20));
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 2249, 3, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1751, 3, .04, true, 20));
				m_autostp++;
			}
			break;
		}
		case 3:
		{
			lDrv.Set(LMotion(m_culEnc, 2249, 3, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 1751, 3, .04, true, 20));
			if(m_ldone && m_rdone)
			{
				lDrv.Set(0);
				rDrv.Set(0);
				m_autostp++;
			}
			break;
		}
		}
	}
	void Autonomous5() // blue goal + move
	{
		switch(m_autostp)
		{
		case 0:
			{
				bckspn.Set(-.540);
				m_t3.Start();
				if(m_t3.Get()>=1)
					m_autostp++;
				break;
			}
		case 1:
			{
				flppddl.Set(1);
				popcorn.Set(-.55);
				if(m_t3.Get()>=11)
				{
					m_autostp++;
					flppddl.Set(0);
					popcorn.Set(0);
					lDrv.Set(LMotion(m_culEnc, -249, 1, .04, true, 20));
					rDrv.Set(-RMotion(m_culEnc, 249, 1, .04, true, 20));
				}
				break;
			}
		case 2:
		{
			lDrv.Set(LMotion(m_culEnc, -249, 1, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 249, 1, .04, true, 20));
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 1751, 3, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 2249, 3, .04, true, 20));
				m_autostp++;
			}
			break;
		}
		case 3:
		{
			lDrv.Set(LMotion(m_culEnc, 1751, 3, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 2166, 3, .04, true, 20));
			if(m_ldone && m_rdone)
			{
				lDrv.Set(0);
				rDrv.Set(0);
				m_autostp++;
			}
			break;
		}
		}
	}
	void Autonomous6() // cross defense and stop
		{
			m_automax = 3;

			float d=8736;
			float tm=12.5;

			switch(m_autostp)
			{

			case 0:  //start move
			{
				lDrv.Set(-LMotion(m_culEnc, d, tm, .04, true, 100));
				rDrv.Set(RMotion(m_curEnc, d, tm, .04, true, 100));
				m_autostp++;
				break;
			}

			case 1: // move until over defense
			{
				if(m_ldone && m_rdone) // if done next step
				{
					m_autostp++;
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);

				}
				else // do action for step
				{
					lDrv.Set(-LMotion(m_culEnc, d, tm, .04, true, 100));
					rDrv.Set(RMotion(m_curEnc, d, tm, .04, true, 100));
				}
				break;
			}
			case 2:
			{
				m_autostp++;
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				break;
			}
			case 3:
			{
					m_autostp++;
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);
				break;
			}
			default:
			{
				m_autostp++;
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				break;
			}
			}// end step switch
		}// end autonomous6
	/*void Autonomous6()	//combo left blue
	{
		switch(m_autostp)
		{
		case 0:
		{
			lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
			m_autostp++;
			break;
		}
		case 1:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(RMotion(m_curEnc,-500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
			}
			break;
		}
		case 2:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(RMotion(m_curEnc,-500, 2, .04, true, 20));
			}
			break;
		}
		case 3:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
			}
			break;
		}
		case 4:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
			}
			break;
		}
		case 5:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
			}
			break;
		}
		case 6:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(0);
				rDrv.Set(0);
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
			}
			break;
		}
		case 7:
		{
			if(m_t2.CkTime(true, 100))
			{
				bckspn.Set(0);
				m_autostp++;
			}
			else if(bckrst)
			{
				bckspn.Set(bckspnMotion.VelocityPID(m_curbckEnc,m_dt,15, true));
				bckrst= 0;
			}
			else
				bckspn.Set(bckspnMotion.VelocityPID(m_curbckEnc,m_dt,15, false));
			break;
		}
		}
	}
	*/
	void Autonomous7()
	{
		switch(m_autostp)
		{
		case 0:
		{
			lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
			rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
			m_autostp++;
			break;
		}
		case 1:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(RMotion(m_curEnc,-500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
			}
			break;
		}
		case 2:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(RMotion(m_curEnc,-500, 2, .04, true, 20));
			}
			break;
		}
		case 3:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 1000, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, 1000, 2, .04, true, 20));
			}
			break;
		}
		case 4:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
			}
			break;
		}
		case 5:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, 500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
			}
			break;
		}
		case 6:
		{
			if(m_ldone && m_rdone)
			{
				lDrv.Set(0);
				rDrv.Set(0);
				m_autostp++;
			}
			else
			{
				lDrv.Set(LMotion(m_culEnc, -500, 2, .04, true, 20));
				rDrv.Set(-RMotion(m_curEnc, -500, 2, .04, true, 20));
			}
			break;
		}
		case 7:
		{
/*
			if(m_t2.CkTime(true, 100))
			{
				bckspn.Set(0);
				m_autostp++;
			}
			else if(bckrst)
			{
				bckspn.Set(bckspnMotion.VelocityPID(m_curbckEnc,m_dt,15, true));
				bckrst= 0;
			}
			else
				bckspn.Set(bckspnMotion.VelocityPID(m_curbckEnc,m_dt,15, false));
*/
			break;
		}
		}
	}
	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{

		UpdateInputs();
		ljhgyro.setTol(1);
		if (!brendancontrols)
		{
			drstk_x=drstk_z;
		}

		if (abs(drstk_x)>=0.1)
		{
			ljhgyro.setTargetAngle(gyroAngle);
			myrobot.ArcadeDrive(drstk_y, -.55*drstk_x, true);
		}
		else
		{
			gyrocrrt = ljhgyro.goTargetAngle(gyroAngle);
			myrobot.Drive(drstk_y, -gyrocrrt);
		}
		if(drstk.GetRawButton(1))
		{
			if (abs(drstk_x)>=0.05)
			{
				ljhgyro.setTargetAngle(gyroAngle);
				myrobot.ArcadeDrive(.5*drstk_y, -.45*drstk_x, true);
			}
			else
			{
				gyrocrrt = ljhgyro.goTargetAngle(gyroAngle);
				myrobot.Drive(.5*drstk_y, -gyrocrrt);
			}
		}
		//Run w/o gyro
		//myrobot.ArcadeDrive(drstk_y, -.65*drstk_z, true);

		if(lstk.GetRawButton(2))
		{
			flppddl.Set(1); //.8
			popcorn.Set(-.55); //-.8
		}
		else
		{
			flppddl.Set(0);
			popcorn.Set(0);
		}

		if(lstk.GetRawButton(1))
		{
			bckspn.Set(-0.540);
			/*
			if(bckrst)
			{
			bckspn.Set(bckspnMotion.VelocityPID(m_curbckEnc, m_dt, 120, true));
			bckrst=false;
			}
			else
			{
			bckspn.Set(bckspnMotion.VelocityPID(m_curbckEnc, m_dt, 120, false));
			}
			*/
		}
		else
		{
			bckspn.Set(0);
			//bckrst =true;
		}

		if(lstk.GetRawButton(5))
		{
			if(toplim)
			{
				if(lstk.GetY()<0)
				{
					gear.Set(lstk.GetY());
				}
				else
				{
					gear.Set(0);
				}
			}
			else if(bttlim)
			{
				if(lstk.GetY()>0)
				{
					gear.Set(lstk.GetY());
				}
				else
				{
					gear.Set(0);
				}
			}
			else
			{
				gear.Set(lstk.GetY());
			}
		}
		else
		{
			gear.Set(0);
		}
		if(lstk.GetRawButton(8))
		{
			Succ.Set(-1);
		}
		else
		{
			if(lstk.GetRawButton(6))
					{
						Succ.Set(1);
					}
			else if(lstk.GetRawButton(7))
			{
				Succ.Set(0.5);
			}

			else
					{
						Succ.Set(0);
					}
		}

		/*
		if (lstk.GetRawButton(10))
		{
			gyro.Calibrate();
		}
		*/
		if (lstk.GetRawButton(11))
		{
			//gyro.Reset();
			bckEnc.Reset();
			lEnc.Reset();
			rEnc.Reset();
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void UpdateInputs()
	{
		SmartDashboard::PutNumber("m_autostp", m_autostp);
		//SmartDashboard::PutNumber("m_t1", m_t1.GetFPGATimestamp());
		SmartDashboard::PutNumber("FPGA", GetFPGATime());
		SmartDashboard::PutNumber("degrees", gyroAngle);
		SmartDashboard::PutNumber("target", ljhgyro.getTargetAngle());
		SmartDashboard::PutNumber("z_rotate", drstk_z);
		SmartDashboard::PutNumber("z_gyro", ljhgyro.goTargetAngle(gyroAngle));
		SmartDashboard::PutNumber("t_value", drstk_t);
		SmartDashboard::PutNumber("m_curEnc", m_curEnc);
		SmartDashboard::PutNumber("m_culEnc", m_culEnc);
		SmartDashboard::PutNumber("m_lstkz", m_lstkz);
		SmartDashboard::PutNumber("m_rng", m_rng);
		SmartDashboard::PutNumber("m_curbckEnc", m_curbckEnc);
		SmartDashboard::PutNumber("POV",drstk.GetPOV(0));
		SmartDashboard::PutNumber("autonum", m_autopgnum);
		SmartDashboard::PutNumber("m_lDrvMotion", lDrv.Get());
		SmartDashboard::PutNumber("m_rDrvMotion", rDrv.Get());

		SmartDashboard::PutBoolean("autosw1",m_autosw1);
		SmartDashboard::PutBoolean("autosw2",m_autosw2);
		SmartDashboard::PutBoolean("autosw3",m_autosw3);
		SmartDashboard::PutBoolean("grtplmt",grtplmt.Get());
		SmartDashboard::PutBoolean("grbttmlmt",grbttmlmt.Get());
		SmartDashboard::PutBoolean("toplim",toplim);
		SmartDashboard::PutBoolean("bttlim",bttlim);
		SmartDashboard::PutBoolean("m_lDrvDone", lDrvMotion.GetDone());
		SmartDashboard::PutBoolean("m_rDrvDone", rDrvMotion.GetDone());


		gyroAngle = (float) gyro.GetAngle();
		drstk_x= (float) drstk.GetX();
		drstk_y= (float) drstk.GetY();
		drstk_z= (float) drstk.GetZ();
		drstk_t= (float) drstk.GetThrottle();
		m_lstkz = (float) lstk.GetZ();
		m_culEnc= (float) -lEnc.Get();
		m_curEnc= (float) rEnc.Get();
		m_curbckEnc = (float) bckEnc.Get();
		m_rng =(float) rngfndr.GetVoltage();
		//m_dt= (float) m_t1.GetTimeSec()- m_lstt;
		//m_lstt= (float) m_t1.GetTimeSec();
		m_t1.Update();
		m_t2.Update();
		/*
		if (drstk.GetPOV(0)==0)
		{
			toplim=true;
			bttlim=false;
		}
		else
		{
		*/
			toplim=grtplmt.Get();
		/*
		}
		if (drstk.GetPOV(0)==180)
		{
			bttlim=true;
			toplim=false;
		}
		else
		{
		if (drstk.GetPOV(0)==-1)
		{
		*/
			bttlim=grbttmlmt.Get();
		//}

		//}
		if(lstk.GetRawButton(8))
			m_t1.Reset();
		m_autosw1= autosw1.Get();
		m_autosw2= autosw2.Get();
		m_autosw3= autosw3.Get();

	}

	float LMotion(float cp, float fp, float dt, float k, bool j, float tol)
	{
				m_k = .04;
					if (m_lfpo != fp) // when final position changes start the move
					{
						m_lfpo = fp; // save final position
						m_lspo = cp; // save current position as start position
						m_ldto = dt; // get how long the move is going to take
						m_k = .04;
						m_ltol= tol; // save the tolerance to how close the final position must be
						m_t1.Update();
						m_t1.Reset(); // reset the timer
						m_ldone = false; // not done
					}
					ldtx= (int) m_ldto*100; // get ticks in hundreds of a second

					if(!m_ldone)
					{
						double t = m_t1.GetTimeSec(); // get time
						double tp = (m_lfpo - m_lspo)*P345(t/m_ldto) + m_lspo; //calculate target position
						m_ldone = m_t1.CkTime(!m_ldone, ldtx); //check if time is up
						if(abs(cp-fp)<m_ltol) m_ldone=true; // check if close enough
						return FB(cp, tp); // return velocity based on position error
					}
					else
					{
						return 0;
					}
	}
	float RMotion(float cp, float fp, float dt, float k, bool j, float tol)
	{

				m_k = .04;


					if (m_rfpo != fp) // when final position changes start the move
					{
						m_rfpo = fp; // save final position
						m_rspo = cp; // save current position as start position
						m_rdto = dt; // get how long the move is going to take
						m_k = .04;
						m_rtol= tol; // save the tolerance to how close the final position must be
						m_t2.Update();
						m_t2.Reset(); // reset the timer
						m_rdone = false; // not done
					}

					rdtx= (int) m_rdto*100; // get ticks in hundreds of a second

					if(!m_rdone)
					{
						double t = m_t2.GetTimeSec(); // get time
						double tp = (m_rfpo - m_rspo)*P345(t/m_rdto) + m_rspo; //calculate target position
						m_rdone = m_t2.CkTime(!m_rdone, rdtx); //check if time is up
						if(abs(cp-fp)<m_rtol) m_rdone=true; // check if close enough
						return FB(cp, tp); // return velocity based on position error
					}
					else
					{
						return 0;
					}
	}



	float FB(float pa, float ps)	//Feedbacm_k Algorithm
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

	double P345(double t) // t ranges 0-1 and returns position 0-1
	{
		double t2=t*t;
		double t3=t2*t;
		return (10*t3)-(15*t2*t2)+(6*t2*t3);
	}
private:
	LiveWindow* lw = LiveWindow::GetInstance();
	SendableChooser<std::string> chooser;
	const string autoNameDefault = "Default";
	const string autoNameCustom = "My Auto";
	string autoSelected;
};

START_ROBOT_CLASS(Robot)
