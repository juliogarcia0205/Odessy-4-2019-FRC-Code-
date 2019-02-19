#include <iostream>
#include <memory>
#include <string>

#include <GripPipeline.h>
#include <cameraserver/CameraServer.h>
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/RobotDrive.h>
#include <frc/SampleRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/Talon.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>

#define SRC_VISIONMETHODS_H_

using namespace frc;

// andrew
class Robot: public frc::SampleRobot
{
	// Motors
	Talon left {0}, right {1}, elevTT {6}; 
		
	// Controllers 
	XboxController controller {0}, ElevCont {1};

	// Solenoids
	DoubleSolenoid largeC {0, 1}, releaseC {2, 3};
	
	// AUTO CHOOSER
	SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoLeft        = "Left Auto";
	const std::string autoCenter      = "Center Auto";
	const std::string autoRight       = "Right Auto";
	const std::string autoTesting     = "Compressor ON";
	const std::string kR              = "kReverse";
	const std::string kF              = "kForward";
	const std::string kO              = "kOff";
	
	// Compressor
	Compressor *c = new Compressor(0);
	bool enabled        = c->Enabled();
	bool pressureSwitch = c->GetPressureSwitchValue();
	double current      = c->GetCompressorCurrent();

	// Control Variables
	float forwardVelocity, backwardsVelocity, turnVelocity;
	float elevFoward, elevBackward;
	double netLeft, netRight;
	double netFoward, netBackward;
	
	// Drive Train Gear Mod
	int gear = 3;
	double speedMod = 1;

	// Elevator Gear Mod
	int elevGear = 1;
	double elevSpeed = 1;

public:

	Robot() {
		// Note SmartDashboard is not initialized here,	\
		   wait until RobotInit to make SmartDashboard calls
	}

	void RobotInit()
	{
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoLeft, autoLeft);
		chooser.AddObject(autoCenter, autoCenter);
		chooser.AddObject(autoRight, autoRight);
		chooser.AddObject(autoTesting, autoTesting);
		chooser.AddObject(kF, kF);
		chooser.AddObject(kR, kR);
		chooser.AddObject(kO, kO);
		
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture();
		cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture();
		cs::CvSink GetVideo(const cs::VideoSource& camera);

		largeC.Set(DoubleSolenoid::Value::kOff);
		releaseC.Set(DoubleSolenoid::Value::kOff);
	}

	void Autonomous()
	{
		auto autoSelected = chooser.GetSelected();
		std::cout << "Auto selected: " << autoSelected << std::endl;
		
		if (autoSelected == autoLeft) {
			std::cout << "Running Left Lane Autonomous" << std::endl;
			right.Set(-0.5);
			left.Set(0.5);
            Wait(2);
            right.Set(0);
            left.Set(0);
		}
		else if (autoSelected == autoCenter) {
			std::cout << "Running Center Lane Autonomous" << std::endl;
		}
		else if (autoSelected == autoRight) {
			std::cout << "Running Right Lane Autonomous" << std::endl;
			right.Set(-1);
			left.Set(1);
            Wait(1);
            right.Set(0);
            left.Set(0);
		}
		else if (autoSelected == autoTesting) {
			std::cout << "Compressor 20Sec" << std::endl;
			c->Start();
			Wait(60);
			c->Stop();
		}
		else if (autoSelected == kR) {
			std::cout << "kReverse" << std::endl;
			largeC.Set(DoubleSolenoid::Value::kReverse);
			Wait(1);
			largeC.Set(DoubleSolenoid::Value::kOff);
		}
		else if (autoSelected == kF) {
			std::cout << "kForward" << std::endl;
			largeC.Set(DoubleSolenoid::Value::kForward);
			Wait(1);
			largeC.Set(DoubleSolenoid::Value::kOff);
		}
		else if (autoSelected == kO) {
			std::cout << "kOff" << std::endl;
			largeC.Set(DoubleSolenoid::Value::kOff);
		}
		else {
			std::cout << "Running Default Autonomous" << std::endl;
		}
	}

	// Code run when operating
	void OperatorControl() override {
		while (IsOperatorControl() && IsEnabled())
		{
			
			/** -------- Motor Control ------------------ */
			
			/// Drive train
			
			// Gear select
			if(controller.GetPOV() == 0) {
				gear = 3;
				speedMod = 1;
			}
			else if(controller.GetPOV() == 90) {
				gear = 2;
				speedMod = 0.5;
			}
			else if(controller.GetPOV() == 180){
				gear = 1;
				speedMod = 0.25;
			}
			
			// Movement
			forwardVelocity = controller.GetRawAxis(2);
			backwardsVelocity = controller.GetRawAxis(3);
			turnVelocity = controller.GetRawAxis(0);

			netLeft = (forwardVelocity - backwardsVelocity) + turnVelocity;
			netRight = (backwardsVelocity - forwardVelocity) + turnVelocity;

			left.Set(netLeft * speedMod);
			right.Set(netRight * speedMod);

			// Output Gear To SmartDashboard
			SmartDashboard::PutNumber("Speed Gear", gear);
			
			/// Elevator
			
			// Gear Select
			if(ElevCont.GetPOV() == 0) {
				elevGear = 3;
				elevSpeed = 1;
			}
			else if(ElevCont.GetPOV() == 90) {
				elevGear = 2;
				elevSpeed = 0.5;
			}
			else if(ElevCont.GetPOV() == 180){
				elevGear = 1;
				elevSpeed = 0.25;
			}
			
			// Movement
			elevFoward = ElevCont.GetRawAxis(3);
			elevBackward = ElevCont.GetRawAxis(2);

			netFoward = elevFoward - elevBackward;
			netBackward = (elevBackward - elevFoward) * -1;

			elevTT.Set(netFoward * elevSpeed);
			elevTT.Set(netBackward * elevSpeed);
			
			/** -------- Solenoid Control --------------- */
			
			// Large Cylinders
			if (ElevCont.GetRawButton(5)){
				largeC.Set(DoubleSolenoid::Value::kReverse);
				Wait(1);
				largeC.Set(DoubleSolenoid::Value::kOff);
			}
			
			if (ElevCont.GetRawButton(6)){
				largeC.Set(DoubleSolenoid::Value::kForward);
				Wait(1);
				largeC.Set(DoubleSolenoid::Value::kOff);
			}

			if (ElevCont.GetRawButton(2)){
				largeC.Set(DoubleSolenoid::Value::kOff);
			}

			// Small Cylinders AKA Release Function
			if (ElevCont.GetRawButton(3)){
				releaseC.Set(DoubleSolenoid::Value::kReverse);
				Wait(1);
				releaseC.Set(DoubleSolenoid::Value::kOff);
			}
			
			if (ElevCont.GetRawButton(4)){
				releaseC.Set(DoubleSolenoid::Value::kForward);
				Wait(1);
				releaseC.Set(DoubleSolenoid::Value::kOff);
			}

			if (ElevCont.GetRawButton(1)){
				releaseC.Set(DoubleSolenoid::Value::kOff);
			}
			
			frc::Wait(0.005);
		}
	}

	//Runs during test mode

	void Test() override {

	}
};

START_ROBOT_CLASS(Robot)
