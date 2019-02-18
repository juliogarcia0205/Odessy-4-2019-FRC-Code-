#include <iostream>
#include <memory>
#include <string>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Talon.h>
#include <frc/Spark.h>
#include <ctre/Phoenix.h>
#include <frc/SampleRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotDrive.h>
#include <cameraserver/CameraServer.h>
#include <frc/Timer.h>
#include <GripPipeline.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#define SRC_VISIONMETHODS_H_
using namespace frc;

//andrew
class Robot: public frc::SampleRobot {
	//Left And Right Motor
	Talon left {0}, right {1}, elevTT {6}; 
		
	//Controller And Joysticks 
	XboxController controller {0},ElevCont {1};
	

	//Solenoids
	DoubleSolenoid largeC {0, 1}, releaseC {2, 3};
	//AUTO CHOOSER
	SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoLeft = "Left Auto";
	const std::string autoCenter = "Center Auto";
	const std::string autoRight = "Right Auto";
	const std::string autoTesting = "Compressor ON";
	const std::string kR = "kReverse";
	const std::string kF= "kForward";
	const std::string kO= "kOff";
	Compressor *c = new Compressor(0);
	bool enabled = c->Enabled();
	bool pressureSwitch = c->GetPressureSwitchValue();
	double current = c->GetCompressorCurrent();


	//Drive Train Gear Mod
	int gear = 3;
	double speedMod = 1;

	//Elevator Gear Mod
	int elevGear = 1;
	double elevSpeed = 1;

	//Position Vars
	float xAxis = 0;
	float yAxis = 0;
	float zAxis = 0;


	 


public:

	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
	}

	void RobotInit() {

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

	void Autonomous() {
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
		else if (autoSelected == kR){
			std::cout << "kReverse" << std::endl;
			largeC.Set(DoubleSolenoid::Value::kReverse);
			Wait(1);
			largeC.Set(DoubleSolenoid::Value::kOff);

		}
		else if (autoSelected == kF){
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

	//Code run when operating

	void OperatorControl() override {
		while (IsOperatorControl() && IsEnabled()) {

			//Gear selector

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

			//Output speed and gear to SmartDashboard

			SmartDashboard::PutNumber("Speed Gear", gear);
			//SmartDashboard::PutNumber("Speed", (left.GetSpeed() + (-right.GetSpeed()))/2);

			//Movement variables

			float forwardVelocity = controller.GetRawAxis(2);
			float backwardsVelocity = controller.GetRawAxis(3);
			float turnVelocity = controller.GetRawAxis(0);

			float forwardVelocityLeft = forwardVelocity;
			float backwardsVelocityLeft = backwardsVelocity;
			float forwardVelocityRight = forwardVelocity;
			float backwardsVelocityRight = backwardsVelocity;

			double totalTurnVelocity = (turnVelocity * 1);// * 1.5;

			double netLeft = ((forwardVelocityLeft - backwardsVelocityLeft) + totalTurnVelocity);
			double netRight = ((backwardsVelocityRight - forwardVelocityRight) + totalTurnVelocity);

			//Move main motors

			left.Set(netLeft * speedMod);
			right.Set(netRight * speedMod);

			//Elevator Variable
			float elevFoward = ElevCont.GetRawAxis(3);
			float elevBackward = ElevCont.GetRawAxis(2);

			double netFoward = ((elevFoward - elevBackward));
			double netBackward = ((elevBackward - elevFoward) * -1);

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
			//Elevator Control
			
			elevTT.Set(netFoward * elevSpeed);
			elevTT.Set(netBackward * elevSpeed);
				
			//Solenoid Control
			
			//Large Cylinders
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

			//Small Cylinders AKA Release Function
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
