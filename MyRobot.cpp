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
	Jaguar *Motor5;
	Jaguar *Motor6;
	Jaguar *Motor7;
	RobotDrive *myRobot;
//	Relay spike;
	Encoder encoder_1;
	Encoder encoder_2;
	PIDController812 *pid_controller_1;
	PIDController812 *pid_controller_2;
	Counter Counter1;
	Counter Counter2;
//	Counter Counter3;
	
public:
	RobotDemo(void):
		leftStick(1),		// as they are declared above.
		rightStick(2), // right stick
		climbStick(3),
		encoder_1(1,2), // digital I/O port 1 == chan A, port 2 == chan B
		encoder_2(3,4),	// digital I/O port 3 == chan A, port 4 == chan B
		Counter1(6),	// dio port 6 counter on left tower motor
		Counter2(7)		// dio port 7 counter on right tower motor
	{
		Motor1 = new Jaguar(1);
		Motor2 = new Jaguar(2);
		Motor3 = new Jaguar(3); // PWM 3
		Motor4 = new Jaguar(4); // PWM 4
		Motor5 = new Jaguar(5);
		Motor6 = new Jaguar(6);
		Motor7 = new Jaguar(7);

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
		const int periods = 100;
		float adjustment;

		//	myRobot.SetSafetyEnabled(false);
		// Distance = velocity / time; 
		// Assume instaneous acceleration to make things easy
		float distance = 40.0; // feet
		float velocity = 10.0; // feet / sec
		float drive_time = distance / velocity; // t (sec) = d (feet) / v (feet/sec)
		float time_driven = 0;
		
		fprintf(stderr,"autonomous: drive_time %f\n", drive_time);
		encoder_1.Start();
		encoder_2.Start();
		encoder_1.SetDistancePerPulse(360.0/4096.0);
		encoder_2.SetDistancePerPulse(360.0/4096.0);
		
		jaguarSpeed_1 = 0.5;
		jaguarSpeed_2 = -0.5;
		Motor1->SetSpeed(jaguarSpeed_1);
		Motor2->SetSpeed(jaguarSpeed_2);

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
		float liftSpeed5 = 0.2;
		float liftSpeed6 = 0.45;
		int CounterPrev1, CounterPrev2;
		float MotorSpeed5, MotorSpeed6;
		float liftAdjust;
		int liftAdjustCycle = 100;
		
		myRobot->SetSafetyEnabled(true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true); //right
		myRobot->SetInvertedMotor(RobotDrive::kRearRightMotor, true); //left

		Counter1.Start();
		Counter2.Start();

		
		encoder_1.Start();
		encoder_2.Start();
		encoder_1.SetDistancePerPulse(360.0/4096.0);
		encoder_2.SetDistancePerPulse(360.0/4096.0);
		
		pid_controller_1->Reset();
		pid_controller_2->Reset();
		
		while (IsOperatorControl())
		{
			//myRobot.ArcadeDrive(rightStick); // drive with arcade style (use right stick)
		    myRobot->TankDrive(leftStick, rightStick);
			Wait(0.005);				// wait for a motor update time
			
			if(rightStick.GetRawButton(1))
			{
				Motor7->SetSpeed(0.9);
			}
			if(rightStick.GetRawButton(2))
			{
				Motor7->SetSpeed(-0.9);
			}
			if(rightStick.GetRawButton(3))
			{
				Motor5->SetSpeed(0.0);
				Motor6->SetSpeed(0.0);
				Motor7->SetSpeed(0.0);
				Wait(0.2);

				Counter1.Reset();
				Counter2.Reset();
			}
			
			if(rightStick.GetRawButton(4))
			{
				Motor5->SetSpeed(liftSpeed5);
				Motor6->SetSpeed(liftSpeed6);
				Counter1.Reset();
				Counter2.Reset();
				pid_controller_1->Reset();
			}
			if(rightStick.GetRawButton(5))
			{
				Motor5->SetSpeed(-liftSpeed5);
				Motor6->SetSpeed(-liftSpeed6);
				Counter1.Reset();
				Counter2.Reset();
				pid_controller_1->Reset();
			}
			
/* 2012-02-12
 * This code section was used for PID controller testing. It is disabled
 * in favor of TankDrive listed above
 * 
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
--- end of PID controller testing code 
*/
			if( Counter1.Get() != CounterPrev1 || Counter2.Get() != CounterPrev2)
			{
				CounterPrev1 = Counter1.Get();
				CounterPrev2 = Counter2.Get();
				fprintf(stderr,"lift counter1 count: %d\n", CounterPrev1);
				fprintf(stderr,"lift counter2 count: %d\n", CounterPrev2);
				liftAdjust=pid_controller_1->PIDCalc(Counter2.Get(),Counter1.Get());
				fprintf(stderr,"liftAdjust: %f\n", liftAdjust);
				MotorSpeed6 = Motor6->Get();
				MotorSpeed5 = Motor5->Get();
				fprintf(stderr,"motor5 speed: %f, motor6 speed: %f\n",MotorSpeed5, MotorSpeed6);
				Motor5->SetSpeed(MotorSpeed5 + MotorSpeed5*liftAdjust/100.0);
				fprintf(stderr,"motor5 speed: %f, motor6 speed: %f\n",
						Motor5->Get(),
						Motor6->Get());
			}	
/*
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
*/			
			sumEncoderRates_1 += encoder_1.GetRate();
			sumEncoderRates_2 += encoder_2.GetRate();
			countRates += 1;

/*			if( countRates >= periods ) {
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
			*/
			}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

