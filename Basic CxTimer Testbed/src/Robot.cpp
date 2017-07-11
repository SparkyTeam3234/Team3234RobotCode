#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <Timer.h>
#include <WPIlib.h>
#include <CxTimer.h>
#include <math.h>
#include <Talon.h>
#include <Motion.h>

using namespace frc;
using namespace std;

class Robot: public frc::IterativeRobot
{
	bool brendancontrols;
	float Time_GetTime;
	float Time_GetTimeSec;
	float Output;
	float m_k;
	float m_fpo;
	float fp=1000;
	float m_spo;
	float m_dto;
	float m_tol;
	float dt;
	float cp;
	float tol;
	double t;
	double tp;

	float m_lEnc;

	bool m_done;

	CxTimer m_t;
	Encoder lDrvEnc {2,3};
	Talon lDrv {1};
	Motion lDrvMotion;
public:
	void RobotInit()
	{
	/*
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		SmartDashboard::PutData("Auto Modes", &chooser);
	*/
		m_t.Update();
		m_t.Reset();


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
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
		*/
		m_k = .04;
		dt=5;
		tol = 100;
		lDrvEnc.Reset();
		//m_t.Update();
		//m_t.Reset();
	}

	void AutonomousPeriodic()
	{
		/*
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
		*/
		UpdateInputs();

		cp= lDrvEnc.Get();
		long dtx =0;

			if (m_fpo != fp) // when final position changes start the move
			{
				m_fpo = fp; // save final position
				m_spo = lDrvEnc.Get(); // save current position as start position
				m_dto = dt; // get how long the move is going to take
				m_k = .04;
				m_tol= tol; // save the tolerance to how close the final position must be
				m_t.Update();
				m_t.Reset(); // reset the timer
				m_done = false; // not done
			}

			dtx= (int) m_dto*100; // get ticks in hundreds of a second

			if(!m_done)
			{
				t = m_t.GetTimeSec(); // get time
				tp = (m_fpo - m_spo)*P345(t/m_dto) + m_spo; //calculate target position
				m_done = m_t.CkTime(!m_done, dtx); //check if time is up
				if(abs(cp-fp)<m_tol) m_done=true; // check if close enough
				Output= FB(cp, tp); // return velocity based on position error
			}
			else
			{
				Output= 0;
			}
			lDrv.Set(-Output);

	}

	void TeleopInit()
	{
		m_t.Update();
		m_t.Reset();
		lDrvEnc.Reset();
		UpdateInputs();
	}

	void TeleopPeriodic()
	{
		UpdateInputs();
		lDrv.Set(-lDrvMotion.GetPower(m_lEnc, 1414, 3, .04, true, 50, m_t));
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void UpdateInputs()
	{
		m_t.Update();
		//m_t.UpdateCurrent();
		Time_GetTime = m_t.GetTime();
		Time_GetTimeSec = m_t.GetTimeSec();

		SmartDashboard::PutNumber("Time_GetTime", Time_GetTime);
		SmartDashboard::PutNumber("Time_GetTimeSec",Time_GetTimeSec);
		SmartDashboard::PutNumber("myCurTime", m_t.m_myCurTime);
		SmartDashboard::PutNumber("timeTicks", m_t.m_timeTicks);
		SmartDashboard::PutNumber("myLastTm", m_t.m_myLastTm);
		SmartDashboard::PutNumber("timeTicksRm", m_t.m_timeTicksRm);
		SmartDashboard::PutNumber("Output", Output);
		SmartDashboard::PutNumber("t",t);
		SmartDashboard::PutNumber("tp",tp);
		SmartDashboard::PutNumber("lEnc", lDrvEnc.Get());
		SmartDashboard::PutBoolean("m_done", m_done);

		m_lEnc=lDrvEnc.Get();

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
	SendableChooser<string> chooser;
	const string autoNameDefault = "Default";
	const string autoNameCustom = "My Auto";
	string autoSelected;
};

START_ROBOT_CLASS(Robot)
