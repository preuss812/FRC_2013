#include "WPILib.h"

#define KP 5/10
#define KI 0
#define KD 0

class PIDController812
{
	float prev_err, integral_err;
public:
	float PIDCalc(float, float);
	void Reset(void);
};

float PIDController812::PIDCalc( float setPoint, float currentPoint) {
	float error;
	float delta_err;
	float p_out;
	float i_out;
	float d_out;
	float output;
		
	error = currentPoint - setPoint;
	delta_err = prev_err - error;
	integral_err += error;
	
	fprintf(stderr, "PID data: error %f, delta_err %f, integral_err %f\n",
			error,
			delta_err,
			integral_err);
	
	p_out = error * KP;
	i_out = integral_err * KI;
	d_out = delta_err * KD;
		
	output = p_out + i_out + d_out;
		
	return (output);
};

void PIDController812::Reset() {
	prev_err = 0.0;
	integral_err = 0.0;
};
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
//	RobotDrive myRobot; // robot drive system
	DriverStation *m_ds;	// driver station object
	Joystick leftStick; // left
	Joystick rightStick; // right
	Joystick climbStick;
	Jaguar *Motor1;	// left wheel
	Jaguar *Motor2;	// right wheel
	Jaguar *Motor3;
	Jaguar *Motor4;
	Jaguar *Motor5;
	Jaguar *Motor6;
	Jaguar *Motor7;
	RobotDrive *myRobot;
	Relay spike;
	Counter ferroDetector;
	Encoder encoder_1;
	Encoder encoder_2;
	PIDController812 *pid_controller_2;
	
public:
	RobotDemo(void):
	//	RobotDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
	//					UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
		//myRobot(3,4,2,1),
//		myRobot(1,2),
		leftStick(1),		// as they are declared above.
		rightStick(2), // right stick
		climbStick(3),
		spike(5),
		ferroDetector(3),  // digital I/O port 3
		encoder_1(5,6),   // digital I/O port 5 == chan A, port 6 == chan B
		encoder_2(8,9)	// digital I/O port 8 == chan A, port 9 == chan B
	{
//		myRobot.SetExpiration(0.1);  
		Motor1 = new Jaguar(1);
		Motor2 = new Jaguar(2);
		Motor3 = new Jaguar(3); // PWM 3
		Motor4 = new Jaguar(4); // PWM 4
		Motor5 = new Jaguar(5);
		Motor6 = new Jaguar(6);
		Motor7 = new Jaguar(7);

		pid_controller_2 = new PIDController812;

		myRobot = new RobotDrive(Motor1, Motor2);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
	//	myRobot.SetSafetyEnabled(false);
		// Distance = velocity / time; 
		// Assume instaneous acceleration to make things easy
		float distance = 40.0; // feet
		float velocity = 10.0; // feet / sec
		float drive_time = distance / velocity; // t (sec) = d (feet) / v (feet/sec)
		
		fprintf(stderr,"autonomous: drive_time %f\n", drive_time);
		Motor1->SetSpeed(0.5);
		Motor2->SetSpeed(-0.5);
		
		Wait(drive_time);
		
		Motor1->SetSpeed(0.0);
		Motor2->SetSpeed(0.0);

		// Default robot drive code w/single speed and curve parameters, no real
		// ability to adjust wheel speed based on encoder feedback other than
		// recalculate a curvature value.
		// Assuming we run the robot at full speed
/*		for (float curve = -1.0; curve<=1.0  ; curve += 0.1 ) {
			SmartDashboard::PutNumber("autonomous: curve", (double)curve);
						myRobot.Drive(1.0, curve);
			Wait(drive_time);
			myRobot.Drive(0.0, 0.0); 	// stop robot
	}
*/

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
		float jaguarSpeed_1 = 0.0;
		float jaguarSpeed_2 = 0.0;
		float jagAdjust;
		double sumEncoderRates_1;
		double sumEncoderRates_2;
		int countRates = 0;
		const int periods = 100;
		float adjustment;
		
//		myRobot.SetSafetyEnabled(true);
//		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true); //right
//		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true); //left
		ferroDetector.Start();
		encoder_1.Start();
		encoder_2.Start();
		encoder_1.SetDistancePerPulse(360.0/4096.0);
		encoder_2.SetDistancePerPulse(360.0/4096.0);
		
		pid_controller_2->Reset();
//		pid_controller_2->Enable();
		
		while (IsOperatorControl())
		{
			//myRobot.ArcadeDrive(rightStick); // drive with arcade style (use right stick)
//		    myRobot.TankDrive(leftStick, rightStick);
			Wait(0.005);				// wait for a motor update time
			if(rightStick.GetRawButton(1))
			{
				jaguarSpeed_1 += 0.05;
				if( jaguarSpeed_1 > 0.5) {
					jaguarSpeed_1 = 0.0;
				}
				jaguarSpeed_2 = jaguarSpeed_1;
				Motor3->SetSpeed(jaguarSpeed_1);
				Motor4->SetSpeed(jaguarSpeed_2);
//				pid_controller_2->SetSetpoint(encoder_1.GetRate());
//				fprintf(stderr,"OnTarget? %d\n", pid_controller_2->OnTarget());
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
				fprintf(stderr,"motor_1: %f, enc_1 avg_1: %f, enc_1 deg: %f\n",
						jaguarSpeed_1, 
						sumEncoderRates_1/periods, 
						sumEncoderRates_1/periods/360.0*60.0);
				fprintf(stderr,"motor_2: %f, enc_2 avg_2: %f, enc_2 deg: %f\n", 
						jaguarSpeed_2, 
						sumEncoderRates_2/periods, 
						sumEncoderRates_2/periods/360.0*60.0);
				fprintf(stderr,"deltas: avg: %f, deg %f\n",
						sumEncoderRates_1/periods - sumEncoderRates_2/periods,
						(sumEncoderRates_2/periods/360.0*60.0) - (sumEncoderRates_1/periods/360.0*60.0)	);

				
				adjustment = pid_controller_2->PIDCalc(
						encoder_1.GetRate(),
						encoder_2.GetRate());
				jagAdjust = adjustment*0.2/700.0;

				if( jagAdjust*jagAdjust > 0.05*0.05) {
					Motor4->SetSpeed(Motor4->Get() + jagAdjust);
					jaguarSpeed_2 = Motor4->Get();
					fprintf(stderr,"Jaguar adjustment: %f\n", jagAdjust);
				}
				
				if((jaguarSpeed_2 - jaguarSpeed_1) >= 0.2)
				{
					jaguarSpeed_2 = jaguarSpeed_1;
					Motor4->SetSpeed(jaguarSpeed_2);
					fprintf(stderr,"RESET: \n");
				}
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

