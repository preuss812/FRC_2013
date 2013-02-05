#include "WPILib.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	DriverStation *m_ds;	// driver station object
	Joystick leftStick; // left
	Joystick rightStick; // right
	Joystick climbStick;
	Jaguar *encoderJaguar_1;
	Jaguar *encoderJaguar_2;
	Relay spike;
	Counter ferroDetector;
	Encoder encoder_1;
	Encoder encoder_2;
	PIDController *pid_controller_2;
	
public:
	RobotDemo(void):
	//	RobotDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
	//					UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
		//myRobot(3,4,2,1),
		myRobot(1,2),
		leftStick(1),		// as they are declared above.
		rightStick(2), // right stick
		climbStick(3),
		spike(5),
		ferroDetector(3),  // digital I/O port 3
		encoder_1(5,6),   // digital I/O port 5 == chan A, port 6 == chan B
		encoder_2(8,9)	// digital I/O port 8 == chan A, port 9 == chan B
	{
		myRobot.SetExpiration(0.1);
		encoderJaguar_1 = new Jaguar(3); // PWM 3
		encoderJaguar_2 = new Jaguar(4); // PWM 4
		float p = 0;
		float i = 0;
		float d = 0;
		PIDSource *source = /*dynamic_cast<PIDSource *>*/ &encoder_1;
		PIDOutput *output = /*dynamic_cast<PIDOutput *>*/ encoderJaguar_2;
		pid_controller_2 = new PIDController(p, i, d, source, output);
		pid_controller_2->SetSetpoint(0);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		// Distance = velocity / time; 
		// Assume instaneous acceleration to make things easy
		float distance = 20.0; // feet
		float velocity = 10.0; // feet / sec
		float drive_time = distance / velocity; // t (sec) = d (feet) / v (feet/sec)
		
		SmartDashboard::PutNumber("autonomous: drive_time", drive_time);

		// Default robot drive code w/single speed and curve parameters, no real
		// ability to adjust wheel speed based on encoder feedback other than
		// recalculate a curvature value.
		// Assuming we run the robot at full speed
		for (float curve = -1.0; curve<=1.0  ; curve += 0.1 ) {
			SmartDashboard::PutNumber("autonomous: curve", (double)curve);
			myRobot.Drive(1.0, curve);
			Wait(drive_time);
			myRobot.Drive(0.0, 0.0); 	// stop robot
		}


#ifdef dans_bad_code
		// Individual motor control
		Jaguar *left_motor = new Jaguar(1);
		Jaguar *right_motor = new Jaguar(2);
		
		left_motor->Set(1.0);
		right_motor->Set(0.5);
		Wait(drive_time);
		left_motor->Set(0.0);
		right_motor->iSet(0.0);
#endif

		/*
		myRobot.Drive(0.25, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(.25, 1.0);
		Wait(1.5);
		myRobot.Drive(.25, 0.0);
		Wait(2.0);
		*/
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		bool windowMotorOn = FALSE;
		float encoderMotorSpeed = 0.0;
		double sumEncoderRates_1;
		double sumEncoderRates_2;
		int countRates = 0;
		const int periods = 100;
		
		myRobot.SetSafetyEnabled(true);
		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true); //right
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true); //left
		ferroDetector.Start();
		encoder_1.Start();
		encoder_2.Start();
//		newEncoder.SetDistancePerPulse(6*3.14/4096.0);
		encoder_1.SetDistancePerPulse(360.0/4096.0);
		encoder_2.SetDistancePerPulse(360.0/4096.0);

//		newEncoder.SetMinRate(0.1);
		
		pid_controller_2->Reset();
		pid_controller_2->Enable();
		
		while (IsOperatorControl())
		{
			//myRobot.ArcadeDrive(rightStick); // drive with arcade style (use right stick)
		    myRobot.TankDrive(leftStick, rightStick);
			Wait(0.005);				// wait for a motor update time
			if(rightStick.GetRawButton(1))
			{
				encoderMotorSpeed += 0.05;
				if( encoderMotorSpeed > 0.5) {
					encoderMotorSpeed = 0.0;
				}
				encoderJaguar_1->SetSpeed(encoderMotorSpeed);
				//encoderJaguar_2->SetSpeed(encoderMotorSpeed);
				pid_controller_2->SetSetpoint(encoder_1.GetRate());
				fprintf(stderr,"OnTarget? %d", pid_controller_2->OnTarget());
			}
			if(leftStick.GetRawButton(1) )
			{
				spike.Set(Relay::kForward);
				windowMotorOn = TRUE;
			}
			if(leftStick.GetRawButton(2) )
			{
				spike.Set(Relay::kOff);
				windowMotorOn = FALSE;
			}
			if(leftStick.GetRawButton(3) )
			{
				spike.Set(Relay::kReverse);
				windowMotorOn = TRUE;
			}
			
			sumEncoderRates_1 += encoder_1.GetRate();
			sumEncoderRates_2 += encoder_2.GetRate();
			countRates += 1;
			if( countRates >= periods ) {
				fprintf(stderr,"motor: %f, enc_1 avg_1: %f, enc_1 deg: %f\n",
						encoderMotorSpeed, 
						sumEncoderRates_1/periods, 
						sumEncoderRates_1/periods/360.0*60.0);
				fprintf(stderr,"motor: %f, enc_2 avg_2: %f, enc_2 deg: %f\n", 
						encoderMotorSpeed, 
						sumEncoderRates_2/periods, 
						sumEncoderRates_2/periods/360.0*60.0);
				fprintf(stderr,"deltas: avg: %f, deg %f\n",
						sumEncoderRates_1/periods - sumEncoderRates_2/periods,
						(sumEncoderRates_2/periods/360.0*60.0) - (sumEncoderRates_1/periods/360.0*60.0)	);

						
						
				countRates = 0;
				sumEncoderRates_1 = 0;
				sumEncoderRates_2 = 0;
				encoder_1.Reset();
				encoder_2.Reset();
			}
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

