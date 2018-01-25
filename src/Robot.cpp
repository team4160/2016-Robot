#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CanTalonSRX.h>

class Robot: public frc::IterativeRobot {
		const int kDoubleSolenoidForward = 1;
		const int kDoubleSolenoidReverse = 4;
		const int ramping = 35;
		const float turnSensitivity = 0.6;

		double spinnerSpeed = 0;
		double leftStick = 0;
		double turn = 0;
		double driveSpeed;bool driveMode = true;bool driveModeSwitch = false;  //bool Cam = false;bool CamSwitch = false;

		class CanTalonSRX *leftDrive1, *rightDrive1, *leftDrive2, *rightDrive2, *ramp, *shooterAngleM, *shooterMoter1, *shooterMoter2, *launcher;
		Encoder *encoder;
		class Joystick *joystick, *joystick2;
//	class PowerDistributionPanel *_pdp;
		class DoubleSolenoid *m_doubleSolenoid;
	private:
		frc::LiveWindow* lw = LiveWindow::GetInstance();
		frc::SendableChooser<std::string> chooser;
		const std::string autoNameDefault = "Low Bar 0%";
		const std::string autoNameCustom = "Rock 80%";
		std::string autoSelected;

		void RobotInit() {
			chooser.AddDefault(autoNameDefault, autoNameDefault);
			chooser.AddObject(autoNameCustom, autoNameCustom);
			frc::SmartDashboard::PutData("Auto Modes", &chooser);


			leftDrive1 = new CanTalonSRX(1);
			leftDrive2 = new CanTalonSRX(2);
			rightDrive1 = new CanTalonSRX(3);
			rightDrive2 = new CanTalonSRX(4);
			ramp = new CanTalonSRX(5);
			shooterAngleM = new CanTalonSRX(6);
			shooterMoter1 = new CanTalonSRX(7);
			shooterMoter2 = new CanTalonSRX(8);
			launcher = new CanTalonSRX(9);
			encoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
			joystick = new Joystick(0);
			joystick2 = new Joystick(1);
//		_pdp = new PowerDistributionPanel(1);
			m_doubleSolenoid = new DoubleSolenoid(0, 1);

			leftDrive1->SetRampThrottle(ramping);
			leftDrive2->SetRampThrottle(ramping);
			rightDrive1->SetRampThrottle(ramping);
			rightDrive2->SetRampThrottle(ramping);
			ramp->SetRampThrottle(20);

//		CameraServer::GetInstance()->SetQuality(50);
//		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
//		system("/etc/init.d/mjpg-streamer1 start");

			encoder->SetMaxPeriod(.1);
			encoder->SetMinRate(10);
			encoder->SetDistancePerPulse(5);
			encoder->SetReverseDirection(true);
			encoder->SetSamplesToAverage(7);
		}

		void drive(double left, double right) {
			leftDrive1->Set(left);
			leftDrive2->Set(left);
			rightDrive1->Set(right * -1);
			rightDrive2->Set(right * -1);
		}

		/**
		 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
		 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
		 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
		 * below the Gyro
		 *
		 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
		 * If using the SendableChooser make sure to add them to the chooser code above as well.
		 */
		void AutonomousInit() {
			if (autoSelected == autoNameCustom) {  //rock
				Wait(2);
				drive(-.8, -.8);  //sorry its negative but it makes the joystick work
				Wait(3.75);
				drive(0, 0);
			} else {  //low bar
				Wait(2);
				drive(-.4, -.4);  //sorry its negative but it makes the joystick work
				Wait(6.5);
				drive(0, 0);

				Wait(2);
				drive(-.5, -.5);  //sorry its negative but it makes the joystick work
				Wait(6.1);
				drive(0, 0);
			}
		}

		void AutonomousPeriodic() {
//			if (autoSelected == autoNameCustom) {
				//Custom Auto goes here
//			} else {
				//Default Auto goes here
//			}
		}

		void TeleopInit() {

		}

		void TeleopPeriodic() {
			if (driveMode) {
				//ax ^ 3 + (1 - a)x
				leftStick = (joystick->GetRawAxis(0));
				driveSpeed = (joystick->GetRawAxis(5));

				turn = ((turnSensitivity * leftStick * leftStick * leftStick) + (1 - turnSensitivity) * leftStick);

				drive(driveSpeed + turn, driveSpeed - turn);
			} else {
				drive(joystick->GetRawAxis(5), joystick->GetRawAxis(1));
			}
			if (joystick->GetRawButton(8) && driveModeSwitch == false) {
				driveMode = !driveMode;
				driveModeSwitch = true;
			}
			if (!joystick->GetRawButton(8) && driveModeSwitch == true) {
				driveModeSwitch = false;
			}

			if (joystick2->GetRawButton(3))
				ramp->Set(1);
			else if (joystick2->GetRawButton(2))
				ramp->Set(-1);
			else
				ramp->Set(0);

//			if (joystick->GetRawButton(1)) {
//				ramp->Set(100);
//			} else if (joystick->GetRawButton(3)) {
//				ramp->Set(-100);
//			} else {
//				ramp->Set(0);
//			}
//
//		double pdpVoltage = _pdp->GetVoltage();
//		printf("PDP  V:%f\n", pdpVoltage);
//		printf("Ramp C:%f\n", _pdp->GetCurrent(3));
//
//
//		 In order to set the double solenoid, we will say that if neither
//		 button is pressed, it is off, if just one button is pressed,
//		 set the solenoid to correspond to that button, and if both
//		 are pressed, set the solenoid to Forwards.

			if (joystick2->GetRawButton(1))
				m_doubleSolenoid->Set(DoubleSolenoid::kForward);
//		else if (joystick->GetRawButton(kDoubleSolenoidReverse))
//			m_doubleSolenoid->Set(DoubleSolenoid::kReverse);
			else
//		 m_doubleSolenoid->Set(DoubleSolenoid::kOff);
				m_doubleSolenoid->Set(DoubleSolenoid::kReverse);
		}

		void TestPeriodic() {
//			lw->Run();
		}
};
START_ROBOT_CLASS(Robot)
