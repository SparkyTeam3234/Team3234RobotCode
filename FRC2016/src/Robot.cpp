//WPILib Includes are in AutoCR.cpp to consolidate.
#include "AutoCR.cpp"
#include "WPILib.h"
#include "CxTimer.h"
#include "Motion.h"
#include "PosCntl.h"
/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */

//int autoTable [] ={ 1,24,3,24,5,24,7,24,9,24,11,24,13,24,15,24,17,24,19,24,21,24,23,24 };

class Robot: public IterativeRobot
{
	float m_culEnc=0;
	float m_curEnc=0;
	bool m_sol1=0;
	bool m_sol2=0;
	bool m_sol3=0;

	int m_autostep=0;	//autostep number
	int m_automax=0;	//autostep max
	int m_autoArcLng=0;
	bool m_autosw1=false;
	bool m_autosw2=false;
	bool m_autosw3=false;
	bool m_dr1stk_bttn1=false;	//Control Vars
	bool m_dr1stk_bttn2=false;
	bool m_dr1stk_bttn3=false;
	bool m_dr1stk_bttn4=false;
	bool m_dr1stk_bttn8=false;
	bool m_dr1stk_bttn11=false;
	bool m_dr2stk_bttn1=false;
	bool m_dr2stk_bttn2=false;
	bool m_dr2stk_bttn3=false;
	bool m_dr2stk_bttn4=false;
	bool m_dr2stk_bttn8=false;
	bool m_dr2stk_bttn11=false;
	bool m_lstk_bttn1=false;
	bool m_lstk_bttn2=false;
	bool m_lstk_bttn3=false;
	bool m_lstk_bttn4=false;
	bool m_lstk_bttn5=false;
	bool m_lstk_bttn6=false;
	bool m_lstk_bttn7=false;
	bool m_lstk_bttn8=false;
	bool m_lstk_bttn9=false;
	bool m_lstk_bttn11=false;
	float d1=0;
	float d2=0;
	float gh=0;
	float m_autoTable [6];
	float m_idPosTable [6];
	float d=0;
	float a=0;
	float tm=0;
	float fg=0;
	float hpwr=0;
	Talon lDrv;
	Talon rDrv;
	RobotDrive myRobot; // robot drive system
	Joystick dr1stick; 	// Primary joystick1
	Joystick dr2stick;  // Primary joystick2
	Joystick lstick; 	// Secondary joystick1
	CxTimer m_t1;
	CxTimer m_t2;
	CxTimer m_t3;
	Talon hookmec;		// Hook Mechanism
	Talon winchmec;		// Winch Mechanism
	Talon rollmec;		// Ball Roller Mechanism
	Talon sith;			// Defense Breacher (Cam's sith army knife)
	Talon hshot;
	Talon untitledmec;		// Untitled Mechanism
	Solenoid cylinder1;
	Solenoid cylinder2;
	Solenoid cylinder3;
	DigitalInput autoswitch1;
	DigitalInput autoswitch2;
	DigitalInput autoswitch3;
	DigitalInput posswitch1;
	DigitalInput posswitch2;
	DigitalInput posswitch3;
	DigitalInput lPhoto;
	DigitalInput rPhoto;
	Encoder lEnc;
	Encoder rEnc;

	Motion lDrvMotion;
	Motion rDrvMotion;


public:

	Robot() :
		lDrv(1),
		rDrv(0),
		myRobot(lDrv, rDrv),	// initialize the RobotDrive to use motor controllers on ports 0 and 1
		dr1stick(0),
		dr2stick(1),
		lstick(2),
		m_t1(),
		m_t2(),
		m_t3(),
		hookmec(2),
		winchmec(3),
		rollmec(4),
		sith(7),
		hshot(8),
		untitledmec(4),
		cylinder1(0),
		cylinder2(1),
		cylinder3(2),
		autoswitch1(0),
		autoswitch2(1),
		autoswitch3(2),
		posswitch1(3),
		posswitch2(4),
		posswitch3(5),
		lPhoto(6),
		rPhoto(7),
		lEnc(10,11),
		rEnc(12,13),
		lDrvMotion(.1125),
		rDrvMotion(.1125)

{
		myRobot.SetExpiration(0.1);


}

	/**
	 * Runs the motors with arcade steering.
	 */
	void RobotInit()
	{
		//lw = LiveWindow::GetInstance();
		myRobot.SetSafetyEnabled(false);

		m_t1.Update();
		m_t1.Reset();


		UpdateInputs();

	/*
		//Camera Initialize
		CameraServer::GetInstance()->SetQuality(15);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
	*/
	}

	void AutonomousInit()
	{
		m_t1.Update(); //done in UpdateInputs
		m_t1.Reset();
		lEnc.Reset();
		rEnc.Reset();
		m_autostep=0;
		m_automax = 0;

		UpdateInputs();

	}

	void AutonomousPeriodic()
	{

		int autopgnum=0;

		UpdateInputs();
		if(m_autosw1) autopgnum=1;
		if(m_autosw2)	autopgnum +=2;
		if(m_autosw3) autopgnum +=4;



		switch (autopgnum)
		{

		case 0: //do nothing
		{
			Autonomous0();
			break;
		}
		case 1: //slot low bar
		{
			m_automax = 5;

			d=12200;
			a=-785;
			tm=12.5;
			fg=100;
			Autonomous1_5();
			break;
		}
		case 2: // slot 2
		{
			m_automax = 5;

			d=12600;
			a=-755;
			tm=12.5;
			fg=100;
			Autonomous1_5();
			break;
		}
		case 3: //slot 3
		{
			m_automax = 5;

			d=13000;
			a=-810;
			tm=12.5;
			fg=-400;
			Autonomous1_5();
			break;
		}
		case 4: //slot 4
		{
			m_automax = 5;

			d=12600;
			a=960;
			tm=12.5;
			fg=400;

			Autonomous1_5();
			break;
		}
		case 5: //slot 5
		{
			m_automax = 5;

			d=12200;
			a=420;
			tm=12.5;
			fg=00;
			Autonomous1_5();
			break;
		}
		case 6: // cross do nothing
		{
			m_automax = 3;

			d=8736;
			tm=12.5;
			a=0;
			fg=0;

			Autonomous6();
			break;
		}
		case 7: //spare
		{
			m_automax = 5;

			Autonomous7();
			break;

		}



		} // end of switch



		if(m_autostep >= m_automax)
		{
			m_autostep = m_automax;
			lDrv.Set(0);
			rDrv.Set(0);
			rollmec.Set(0);
		}


	}// end autonnomous periodic


	void Autonomous0() // do nothing
	{
		m_automax = 1;
		m_autostep++;
	}// end autonomous0

	void Autonomous1_5() //slot x
	{


		switch(m_autostep)
		{

		case 0: //start move with curve
		{
			lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d+fg, tm, .04, true, 100));
			rDrv.Set(rDrvMotion.GetPower(m_curEnc, d, tm, .04, true, 100));
			m_autostep++;
			break;
		}
		case 1: // move until end of motion
		{
			if(lDrvMotion.GetDone() && rDrvMotion.GetDone()) // if done next step
			{
				m_autostep++; // start turn
				lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d+fg-a, 2, .04, true, 100)); // turn
				rDrv.Set(rDrvMotion.GetPower(m_curEnc, d+a, 2, .04, true, 100));
			}
			else // do action for step
			{
				lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d+fg, tm, .04, true, 100));
				rDrv.Set(rDrvMotion.GetPower(m_curEnc, d, tm, .04, true, 100));
			}
			break;
		}
		case 2:
		{
			if(lDrvMotion.GetDone() && rDrvMotion.GetDone()) // if turn done
			{
				m_autostep++;
				rollmec.Set(-1); //shoot
				m_t1.Reset();
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
			}
			else // do action for step
			{   // turn to goal
				lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d+fg-a, 2, .04, true, 20));
				rDrv.Set(rDrvMotion.GetPower(m_curEnc, d+a, 2, .04, true, 20));
			}
			break;
		}
		case 3:
		{
			if(m_t1.CkTime(true, 100)) // if done next step time for 1 seconds
				{
				m_autostep++;
				rollmec.Set(0); //no shoot
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				}
			else // do action for step
			{
				rollmec.Set(-1); //shoot
			}
			break;
		}
		case 4:
		{
			if(1) // if done next step
				{
				m_autostep++;
				rollmec.Set(0); //no shoot
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				}
			else // do action for step
			{

			}
			break;
		}
		default:
		{
			m_autostep++;
			rollmec.Set(0); //no shoot
			lDrv.Set(0); // stop all motion
			rDrv.Set(0);
			break;
		}
		}// end step switch
	}// end autonomous1_5







	void Autonomous6() // cross defense and stop
		{
			m_automax = 3;

			float d=8736;
			float tm=12.5;

			switch(m_autostep)
			{

			case 0:  //start move
			{
				lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d, tm, .04, true, 100));
				rDrv.Set(rDrvMotion.GetPower(m_curEnc, d, tm, .04, true, 100));
				m_autostep++;
				break;
			}

			case 1: // move until over defense
			{
				if(lDrvMotion.GetDone() && rDrvMotion.GetDone()) // if done next step
				{
					m_autostep++;
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);
					rollmec.Set(0); //stop shoot

				}
				else // do action for step
				{
					lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d, tm, .04, true, 100));
					rDrv.Set(rDrvMotion.GetPower(m_curEnc, d, tm, .04, true, 100));
				}
				break;
			}
			case 2:
			{
				m_autostep++;
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				rollmec.Set(0); //stop shoot
				break;
			}
			case 3:
			{
					m_autostep++;
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);
					rollmec.Set(0); //stop shoot
				break;
			}
			default:
			{
				m_autostep++;
				rollmec.Set(0); //stop shoot
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				break;
			}
			}// end step switch
		}// end autonomous6


	void Autonomous7() // spare case
		{

		d=2535;
		float d2=12000;
		a=0;
		tm=4.5;
		fg=0;




			switch(m_autostep)
			{

			case 0: //start move with curve
			{
				lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d+fg, tm, .04, true, 100));
				rDrv.Set(rDrvMotion.GetPower(m_curEnc, d, tm, .04, true, 100));
				m_autostep++;
				break;
			}
			case 1: // move until end of motion
			{
				if(lDrvMotion.GetDone() && rDrvMotion.GetDone()) // if done next step
				{
					m_autostep++;
					sith.Set(.4); // arm down
					m_t1.Reset();
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);
				}
				else // do action for step
				{
					lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d+fg, tm, .04, true, 100));
					rDrv.Set(rDrvMotion.GetPower(m_curEnc, d, tm, .04, true, 100));
				}
				break;
			}
			case 2:
			{
				if(m_t1.CkTime(true, 200)) // arm down
				{
					m_autostep++;
					m_t1.Reset(); // reset timer
					sith.Set(0); // stop arm down
					lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d2+fg, tm, .04, true, 100));  //start moving again
					rDrv.Set(rDrvMotion.GetPower(m_curEnc, d2, tm, .04, true, 100));
				}
				else // do action for step
				{   //
					sith.Set(.4); // arm down
				}
				break;
			}
			case 3:
			{
				if(lDrvMotion.GetDone() && rDrvMotion.GetDone()) // if done next step time for 1 seconds
				{
					m_autostep++;
					sith.Set(0); //no arm
					lDrv.Set(0);
					rDrv.Set(0);

				}
				else // do action for step
				{
					lDrv.Set(-lDrvMotion.GetPower(m_culEnc, d2+fg, tm, .04, true, 100));  //start moving again
					rDrv.Set(rDrvMotion.GetPower(m_curEnc, d2, tm, .04, true, 100));
					if(m_culEnc> d+625)
					{
						if(m_t1.CkTime(true, 200))
						{
							sith.Set(0);
						}
						else
						{
							sith.Set(-.4);
						}



					}
				}
				break;
			}
			case 4:
			{
				if(1) // if done next step
					{
					m_autostep++;
					rollmec.Set(0); //no shoot
					lDrv.Set(0); // stop all motion
					rDrv.Set(0);
					}
				else // do action for step
				{

				}
				break;
			}
			default:
			{
				m_autostep++;
				rollmec.Set(0); //no shoot
				lDrv.Set(0); // stop all motion
				rDrv.Set(0);
				break;
			}
			}// end step switch
		}// end autonomous7


	void TeleopInit()
	{
		m_t1.Reset();
		UpdateInputs();

	}

	void TeleopPeriodic()
	{

			/*
			DriveStick1, Button 3: Hold to make X axis on DriveStick1 control turning motion.
			DriveStick2, Button 3: Hold to make Y axis on DriveStick2 control forward and backward motion.
			DriveStick1, Button 10: Move Utilizing Encoders???
			DriveStick1, Button 11: Zero Encoders
			DriveStick2, Button 11: Zero Encoders
			LeftStick, Button 1: Roller Mechanism Shoot Out.
			LeftStick, Button 3: Roller Mechanism Draw In.
			LeftStick, Button 2: Defense Breacher Mechanism is controlled by the Y axis.
			LeftStick, Button 3: Winch Mechanism is controlled by the Y axis.
			LeftStick, Button 4: Hook Mechanism is controlled by the Y axis.
			LeftStick, Button 7: Cylinder1 changes state when held. (Expand/Retract)
			LeftStick, Button 8: Cylinder2 changes state when held. (Expand/Retract)
			LeftStick, Button 9: Cylinder3 changes state when held. (Expand/Retract)
			*/
			UpdateInputs();


			if (m_dr2stk_bttn3)
				myRobot.TankDrive(dr2stick,dr2stick,true);
			else if (m_dr1stk_bttn3)
				myRobot.TankDrive(-1*dr1stick.GetX(), dr1stick.GetX(), true);
			else
				myRobot.TankDrive(dr1stick,dr2stick,true); // drive with tank style (use right sticks)



			// ball pick up and shoot: roller
			if (m_lstk_bttn1 || m_dr1stk_bttn1 || m_dr2stk_bttn1)
				{
				rollmec.Set(-1); // shoots
				}
			else if (m_lstk_bttn3 || m_dr1stk_bttn2 || m_dr2stk_bttn2)
				{ // pick up boulder
				rollmec.Set(0.6);
				}
			else if (m_lstk_bttn5 || m_dr1stk_bttn4 || m_dr2stk_bttn4)
				{ // Soft shot
				rollmec.Set(-0.4);
				}

			else
				rollmec.Set(0);
			/*
			//high goal shooter
			if (m_lstk_bttn1)
			{//fire high shot
				hshot.Set(-hpwr);
			}
			else if (m_lstk_bttn6)
			{//reverse high goal shooter
				hshot.Set(0.3);
			}
			else
				hshot.Set(0);
			 */
			// check winch code


			if (m_lstk_bttn11)
				winchmec.Set(-1*lstick.GetY());
			else if (m_lstk_bttn7) // let cable out
				winchmec.Set(.1);
			else if (m_lstk_bttn8 || m_dr1stk_bttn8 || m_dr2stk_bttn8) // pull cable in raise robot
				winchmec.Set(-1);
			else
				winchmec.Set(0);



			// hook four bar control
			if (m_lstk_bttn4)
				hookmec.Set(-lstick.GetY()); // pull stick raises hook --- push stick lower hook
			else
				hookmec.Set(0);

			if (m_lstk_bttn9)
				untitledmec.Set(-lstick.GetY()); // pull stick raises hook --- push stick lower hook
			else
				untitledmec.Set(0);

			// roller arm
			if (m_lstk_bttn2)
				sith.Set(-lstick.GetY());
			else
				sith.Set(0);
/*
			if (lstick.GetRawButton(7)) cylinder1.Set(true); else cylinder1.Set(false);
			if (lstick.GetRawButton(8)) cylinder2.Set(true); else cylinder2.Set(false);
			if (lstick.GetRawButton(9)) cylinder3.Set(true); else cylinder3.Set(false);
*/
			if(m_dr1stk_bttn11 || m_dr2stk_bttn11)
				lEnc.Reset(),
				rEnc.Reset();
			Wait(0.005);				// wait for a motor update time

	}

	void UpdateInputs()
	{
		//m_t1.Update();
		m_culEnc = (float) lEnc.Get();
		m_curEnc = (float) rEnc.Get();
		m_sol1 = (bool) cylinder1.Get();
		m_sol2 = (bool) cylinder2.Get();
		m_sol3 = (bool) cylinder3.Get();
		m_autosw1 = (bool) autoswitch1.Get();
		m_autosw2 = (bool) autoswitch2.Get();
		m_autosw3 = (bool) autoswitch3.Get();
		m_dr1stk_bttn1 = dr1stick.GetRawButton(1);
		m_dr1stk_bttn2 = dr1stick.GetRawButton(2);
		m_dr1stk_bttn3 = dr1stick.GetRawButton(3);
		m_dr1stk_bttn4 = dr1stick.GetRawButton(4);
		m_dr1stk_bttn8 = dr1stick.GetRawButton(8);
		m_dr1stk_bttn11 = dr1stick.GetRawButton(11);
		m_dr2stk_bttn1 = dr2stick.GetRawButton(1);
		m_dr2stk_bttn2 = dr2stick.GetRawButton(2);
		m_dr2stk_bttn3 = dr2stick.GetRawButton(3);
		m_dr2stk_bttn4 = dr2stick.GetRawButton(4);
		m_dr2stk_bttn8 = dr2stick.GetRawButton(8);
		m_dr2stk_bttn11 = dr2stick.GetRawButton(11);
		m_lstk_bttn1 = lstick.GetRawButton(1);
		m_lstk_bttn2 = lstick.GetRawButton(2);
		m_lstk_bttn3 = lstick.GetRawButton(3);
		m_lstk_bttn4 = lstick.GetRawButton(4);
		m_lstk_bttn5 = lstick.GetRawButton(5);
		m_lstk_bttn6 = lstick.GetRawButton(6);
		m_lstk_bttn7 = lstick.GetRawButton(7);
		m_lstk_bttn8 = lstick.GetRawButton(8);
		m_lstk_bttn9 = lstick.GetRawButton(9);
		m_lstk_bttn11 = lstick.GetRawButton(11);
		hpwr = (lstick.GetThrottle()+1)/2;
		SmartDashboard::PutNumber("m_culEnc", m_culEnc);
		SmartDashboard::PutNumber("m_curEnc", m_curEnc);
		SmartDashboard::PutNumber("d1", d1);
		SmartDashboard::PutNumber("d2", d2);
		SmartDashboard::PutNumber("gh", gh);
		SmartDashboard::PutBoolean("cylinder1", m_sol1);
		SmartDashboard::PutBoolean("cylinder2", m_sol2);
		SmartDashboard::PutBoolean("cylinder3", m_sol3);
		SmartDashboard::PutBoolean("autosw1", m_autosw1);
		SmartDashboard::PutBoolean("autosw2", m_autosw2);
		SmartDashboard::PutBoolean("autosw3", m_autosw3);
		SmartDashboard::PutNumber("army",-lstick.GetY());
		SmartDashboard::PutNumber("hpwr", hpwr);
	}
	bool RobotMotion(float time, float dR, float dL, float k = .02)
		{
			float tol = 2.5;

			lDrv.Set(lDrvMotion.GetPower(m_culEnc, dL, time, k, true, tol));
			rDrv.Set(rDrvMotion.GetPower(m_curEnc, dR, time, k, true, tol));


			return lDrvMotion.GetDone() && rDrvMotion.GetDone();
		}

};

START_ROBOT_CLASS(Robot)
