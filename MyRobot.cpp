#include "WPILib.h"
#include <math.h>

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
	prev_err = error;
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
	Jaguar *Motor5RightTower;
	Jaguar *Motor6LeftTower;
	Jaguar *Motor7;
	RobotDrive *myRobot;
//	Relay spike;
	Encoder encoder_1;
	Encoder encoder_2;
	PIDController812 *pid_controller_1;
	PIDController812 *pid_controller_2;
	Counter Counter6LeftTower;
	Counter Counter7RightTower;
//	Counter Counter3;
	
public:
	RobotDemo(void):
		leftStick(1),		// as they are declared above.
		rightStick(2), // right stick
		climbStick(3),
		encoder_1(1,2), // digital I/O port 1 == chan A, port 2 == chan B
		encoder_2(3,4),	// digital I/O port 3 == chan A, port 4 == chan B
		Counter6LeftTower(6),	// dio port 6 counter on left tower motor
		Counter7RightTower(7)		// dio port 7 counter on right tower motor
	{
		Motor1 = new Jaguar(1);
		Motor2 = new Jaguar(2);
		Motor3 = new Jaguar(3); // PWM 3
		Motor4 = new Jaguar(4); // PWM 4
		Motor5RightTower = new Jaguar(5);
		Motor6LeftTower = new Jaguar(6); // left tower jaguar
		Motor7 = new Jaguar(7); // right tower jaguar

		pid_controller_1 = new PIDController812;
		pid_controller_2 = new PIDController812;

		myRobot = new RobotDrive(Motor2, Motor1);
		myRobot->SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		fprintf(stderr, "in auto \n");
		float jaguarSpeed_1 = 0.0;
		float jaguarSpeed_2 = 0.0;
		float jagAdjust;
		double sumEncoderRates_1;
		double sumEncoderRates_2;
		int countRates = 0;
		int saveEncoder1, saveEncoder2;
		const int periods = 100;
		float adjustment;

		//	myRobot.SetSafetyEnabled(false);
		// Distance = velocity / time; 
		// Assume instaneous acceleration to make things easy
		float distance = 20.0; // feet
		float velocity = 4.2; // feet / sec
		float drive_time = distance / velocity - 0.2; // t (sec) = d (feet) / v (feet/sec) - deceleration time
		float time_driven = 0;
		float drive_speed = 0.5;
		float drive_curve = 0.0001;
		
		fprintf(stderr,"autonomous: drive_time %f\n", drive_time);
		encoder_1.Start();
		encoder_2.Start();
		encoder_1.SetDistancePerPulse(360.0/4096.0);
		encoder_2.SetDistancePerPulse(360.0/4096.0);
		encoder_1.Reset();
		encoder_2.Reset();
		
		fprintf(stderr,"start: encoder1 %d, encoder2 %d\n",
				encoder_1.Get(),
				encoder_2.Get());
		fprintf(stderr,"distance %f, velocity %f, drive_time %f\n", distance, velocity, drive_time);
		saveEncoder1 = encoder_1.Get();
		saveEncoder2 = encoder_2.Get();

		myRobot->SetSafetyEnabled(FALSE);
		myRobot->Drive(drive_speed,drive_curve);
		Wait(drive_time);
		
		for( float v = drive_speed; v > 0.0; v = v - 0.1) {
			myRobot->Drive(v,drive_curve);
			Wait(0.04);
		}
		myRobot->Drive(0.0,0.0);

		fprintf(stderr,"rates during: encoder1 %f, encoder2 %f\n",
				encoder_1.GetRate(),
				encoder_2.GetRate());
		fprintf(stderr,"initial: encoder1 %d, encoder2 %d\n",
				saveEncoder1,
				saveEncoder2);
		fprintf(stderr,"final: encoder1 %d, encoder2 %d\n",
				encoder_1.Get(),
				encoder_2.Get());

/*
		while(time_driven <= drive_time) {
			Wait(0.25);
			time_driven += 0.25;
			sumEncoderRates_1 += encoder_1.GetRate();
			sumEncoderRates_2 += encoder_2.GetRate();
			countRates += 1;

			if( countRates >= periods ) {
				fprintf(stderr,"a: motor_1: %f, enc_1 avg_1: %f, enc_1 deg: %f\n",
						jaguarSpeed_1, 
						sumEncoderRates_1/periods, 
						sumEncoderRates_1/periods/360.0*60.0);
				fprintf(stderr,"a: motor_2: %f, enc_2 avg_2: %f, enc_2 deg: %f\n", 
						jaguarSpeed_2, 
						sumEncoderRates_2/periods, 
						sumEncoderRates_2/periods/360.0*60.0);
				fprintf(stderr,"a: deltas: avg: %f, deg %f\n",
						sumEncoderRates_1/periods - sumEncoderRates_2/periods,
						(sumEncoderRates_2/periods/360.0*60.0) - (sumEncoderRates_1/periods/360.0*60.0)	);

				adjustment = pid_controller_2->PIDCalc(
						encoder_1.GetRate(),
						encoder_2.GetRate());
				jagAdjust = adjustment*0.2/700.0;

				if( jagAdjust*jagAdjust > 0.05*0.05) {
					Motor2->SetSpeed(Motor2->Get() + jagAdjust);
					jaguarSpeed_2 = Motor2->Get();
					fprintf(stderr,"a: Jaguar adjustment: %f\n", jagAdjust);
				}
			
				if((jaguarSpeed_2 - jaguarSpeed_1) >= 0.2)
				{
					jaguarSpeed_2 = jaguarSpeed_1;
					Motor2->SetSpeed(jaguarSpeed_2);
					fprintf(stderr,"RESET: \n");
				}
				countRates = 0;
				sumEncoderRates_1 = 0;
				sumEncoderRates_2 = 0;
				encoder_1.Reset();
				encoder_2.Reset();
			}
		}

		Motor1->SetSpeed(0.0);
		Motor2->SetSpeed(0.0);
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
		float plungerSpeed = 0.50; // speed of platform screw
		float liftSpeed5Right = 0.80; // speed of right tower motor
		float liftSpeed6Left = 0.80; // speed of left tower motor
		float liftPercentOfFull = 0.30; // slow down factor for adjusting tower motor
		float liftErrorThreshold = 0.1; // error threshold between tower counters
		int liftDirection;
		int CounterCur6LeftTower, CounterCur5RightTower;
		int CounterPrev6LeftTower, CounterPrev5RightTower;
		float MotorSpeed5, MotorSpeed6;
		float liftAdjust;
		float liftCounterRatio;
		enum {DOWN, STOP, UP} liftStatus;
		
		myRobot->SetSafetyEnabled(true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true); //right
		myRobot->SetInvertedMotor(RobotDrive::kRearRightMotor, true); //left

		Counter6LeftTower.Start();
		Counter7RightTower.Start();

		
		encoder_1.Start();
		encoder_2.Start();
		encoder_1.SetDistancePerPulse(360.0/4096.0);
		encoder_2.SetDistancePerPulse(360.0/4096.0);
		
		pid_controller_1->Reset();
		pid_controller_2->Reset();
		
		liftStatus = STOP;
		
		while (IsOperatorControl())
		{
			//myRobot.ArcadeDrive(rightStick); // drive with arcade style (use right stick)
		    myRobot->TankDrive(leftStick, rightStick);
			Wait(0.005);				// wait for a motor update time
			
			if(rightStick.GetRawButton(1))
			{
				Motor7->SetSpeed(plungerSpeed);
			}
			if(rightStick.GetRawButton(2))
			{
				Motor7->SetSpeed(-plungerSpeed);
			}
			if(rightStick.GetRawButton(3))
			{
				Motor5RightTower->SetSpeed(0.0);
				Motor6LeftTower->SetSpeed(0.0);
				Motor7->SetSpeed(0.0);
				Wait(0.3);

				Counter6LeftTower.Reset();
				Counter7RightTower.Reset();
				
				liftStatus = STOP;
			}
			
			if(rightStick.GetRawButton(4))
			{
				Motor5RightTower->SetSpeed(liftSpeed5Right);
				Motor6LeftTower->SetSpeed(liftSpeed6Left);
			//	Counter6LeftTower.Reset();
			//	Counter7RightTower.Reset();
				pid_controller_1->Reset();
				liftStatus = DOWN;
			}
			if(rightStick.GetRawButton(5))
			{
				Motor5RightTower->SetSpeed(-liftSpeed5Right);
				Motor6LeftTower->SetSpeed(-liftSpeed6Left);
			//	Counter6LeftTower.Reset();
			//	Counter7RightTower.Reset();
				pid_controller_1->Reset();
				liftStatus = UP;
			}
			
			CounterCur6LeftTower = Counter6LeftTower.Get();
			CounterCur5RightTower = Counter7RightTower.Get();
		
			// if there has been a change in the rotational position
			// of either motor, then we have a change to evaluate
			if( CounterCur6LeftTower != CounterPrev6LeftTower || 
				CounterCur5RightTower != CounterPrev5RightTower)
			{
				// Store the current values for future evaluation
				CounterPrev6LeftTower = CounterCur6LeftTower;
				CounterPrev5RightTower = CounterCur5RightTower;
				fprintf(stderr, "LeftTower speed %f, rightTower speed %f\n",
						Motor6LeftTower->Get(),
						Motor5RightTower->Get());
				fprintf(stderr,"LeftTower count %d, rightTower count %d\n",
						CounterCur6LeftTower, CounterCur5RightTower);
				
				// if the error (delta) between the counters is
				// greater than the defined threshold, then
				// some adjustments to speed are required to
				// bring the rotation count into alignment between
				// the two motors. Rotation translates to distance.
				if( fabs( (double)CounterCur6LeftTower - (double) CounterCur5RightTower) >= liftErrorThreshold &&
					liftStatus != STOP )
				{
					if( liftStatus == UP ) {
						liftDirection = -1;
					} else if( liftStatus == DOWN ) {
						liftDirection = 1;
					} else if( liftStatus == STOP ) {
						// why are we here?
					}
					if(CounterCur6LeftTower > CounterCur5RightTower) {
						fprintf(stderr, "Turning off LeftTower because Left is UP ahead of Right.\n");
							Motor6LeftTower->SetSpeed(liftDirection*liftSpeed6Left*liftPercentOfFull);
							Motor5RightTower->SetSpeed(liftDirection*liftSpeed5Right);
					} else if (CounterCur5RightTower > CounterCur6LeftTower) {
							fprintf(stderr, "Turning off RightTower because Right is UP ahead of Left.\n");
								Motor6LeftTower->SetSpeed(liftDirection*liftSpeed6Left);
								Motor5RightTower->SetSpeed(liftDirection*liftSpeed5Right*liftPercentOfFull);
						} else {
							Motor6LeftTower->SetSpeed(liftDirection*liftSpeed6Left);
							Motor5RightTower->SetSpeed(liftDirection*liftSpeed5Right);
						}
				}
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

