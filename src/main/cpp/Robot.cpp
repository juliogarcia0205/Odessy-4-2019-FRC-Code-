#include <iostream>
#include <memory>
#include <string>
#include <frc/Joystick.h>
#include <frc/Talon.h>
#include <frc/Spark.h>
#include <ctre/Phoenix.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <CameraServer.h>
#include <Timer.h>
#include <GripPipeline.h>
#include <Compressor.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>
#define SRC_VISIONMETHODS_H_
using namespace frc;

//andrew
class Robot: public frc::SampleRobot {
	//Left And Right Motor
	WPI_VictorSPX left {0};  //LEFT Motor Controllers
	WPI_TalonSRX right {1};	 //RIGHT Motor Controllers
	//Controller And Joysticks 
	Joystick controller {0}, joystick {0};
	//Solenoids
	DoubleSolenoid testBench {0, 1};
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



	int gear = 3;
	double speedMod = 1;
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

		testBench.Set(DoubleSolenoid::Value::kOff);


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
			testBench.Set(DoubleSolenoid::Value::kReverse);
			Wait(1);
			testBench.Set(DoubleSolenoid::Value::kOff);

		}
		else if (autoSelected == kF){
			std::cout << "kForward" << std::endl;
			testBench.Set(DoubleSolenoid::Value::kForward);
			Wait(1);
			testBench.Set(DoubleSolenoid::Value::kOff);

		}
		else if (autoSelected == kO) {
			std::cout << "kOff" << std::endl;
			testBench.Set(DoubleSolenoid::Value::kOff);
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
			float turnVelocity = joystick.GetRawAxis(0);

			float forwardVelocityLeft = forwardVelocity;
			float backwardsVelocityLeft = backwardsVelocity;
			float forwardVelocityRight = forwardVelocity;
			float backwardsVelocityRight = backwardsVelocity;

			double totalTurnVelocity = (turnVelocity * -1);// * 1.5;

			double netLeft = ((forwardVelocityLeft - backwardsVelocityLeft) + totalTurnVelocity);
			double netRight = ((backwardsVelocityRight - forwardVelocityRight) + totalTurnVelocity);

			//Move main motors

			left.Set(netLeft * speedMod);
			right.Set(netRight * speedMod);

			//Solenoid Control
			
			if (controller.GetRawButton(1)){
				testBench.Set(DoubleSolenoid::Value::kReverse);
				Wait(1);
				testBench.Set(DoubleSolenoid::Value::kOff);
			}
			
			if (controller.GetRawButton(2)){
				testBench.Set(DoubleSolenoid::Value::kForward);
				Wait(1);
				testBench.Set(DoubleSolenoid::Value::kOff);
			}

			if (controller.GetRawButton(3)){
				testBench.Set(DoubleSolenoid::Value::kOff);
			}

			

			frc::Wait(0.005);
		}
	}

	//Runs during test mode

	void Test() override {

	}
};

START_ROBOT_CLASS(Robot)
