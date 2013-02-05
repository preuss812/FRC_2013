#include "WPILib.h"
/**
 * This is a demo program showing the use of the RobotBase class.  The
 * SimpleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at
 * the right time as controlled by the switches on the driver station
 * or the field controls.
 */
class RobotDemo : public SimpleRobot
{
  RobotDrive myRobot; // robot drive system
  DriverStation *m_ds; // driver station object
  Joystick leftStick; // left
  Joystick rightStick; // right
  Joystick climbStick;
  Relay spike;
  Counter ferroDetector;
  Encoder newEncoder;
public:
  RobotDemo(void):
// RobotDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
// UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
//myRobot(3,4,2,1),
    myRobot(1,2),
    leftStick(1), // as they are declared above.
    rightStick(2), // right stick
    climbStick(3),
    spike(5),
    ferroDetector(3),  // digital I/O port 3
    newEncoder(5,6)   // digital I/O port 1 == chan A, port 2 == chan B
  {
    myRobot.SetExpiration(0.1);
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
      myRobot.Drive(0.0, 0.0); // stop robot
    }


/*
  myRobot.Drive(0.25, 0.0); // drive forwards half speed
  Wait(2.0); //    for 2 seconds
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
    myRobot.SetSafetyEnabled(true);
    myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true); //right
    myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true); //left
    ferroDetector.Start();
    newEncoder.Start();
    newEncoder.SetDistancePerPulse(6*3.14/4096.0);
    // newEncoder.SetMinRate(0.1);
    while (IsOperatorControl())
      {
	//myRobot.ArcadeDrive(rightStick); // drive with arcade style (use right stick)
	myRobot.TankDrive(leftStick, rightStick);
	Wait(0.005); // wait for a motor update time
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
	SmartDashboard::PutNumber("Ferrite detecto count",
				  (double)ferroDetector.Get());
	SmartDashboard::PutNumber("New encoder rate", newEncoder.GetRate()/10.0);
	SmartDashboard::PutNumber("New encoder raw count",
				  (double)newEncoder.GetRaw());
	SmartDashboard::PutNumber("new encoder get distance",
				  newEncoder.GetDistance());
	SmartDashboard::PutNumber("Left Stick X Value",
				  (double)leftStick.GetX()*100.0);
	SmartDashboard::PutNumber("Left Stick Y Value",
				  (double)leftStick.GetY()*100.0);
	SmartDashboard::PutNumber("Left Stick Throttle Value",
				  (double)leftStick.GetThrottle());
	SmartDashboard::PutNumber("Left Stick Magnitude Value",
				  (double)leftStick.GetMagnitude());

      }
  }
 /**
 * Runs during test mode
 */
void Test() {

}
};

START_ROBOT_CLASS(RobotDemo);
