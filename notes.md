# CODE REPOSITORY
The Github repository from 2010/2011 has been resurrected.

## 2012 
https://github.com/preuss812/FRC_2012

## 2013
https://github.com/preuss812/FRC_2013

If you want to check-in code, please send me your SSH public key.


PID CONTROLLER
The PID controller class [1] is something we are looking at using to synchronize the rotation distance across two Jaguar controlled motors.

Sample code is available from the Nerdherd team from 2012 [2], please take a look. Additionally there is a discussion thread on Chief Delphi about using the PID controller [3].

1. http://www.virtualroadside.com/WPILib/class_p_i_d_controller.html
2. https://github.com/nerdherd/frc-2012/blob/master/drive.cpp
3. http://www.chiefdelphi.com/forums/showthread.php?t=72918


AS514B ENCODER
This is a magnetic field sensing encoder [4]. The wiring diagram for FRC usage [5] is correct. We failed to ground the CSn pin at first, but figured that out and fixed it.

The AS514B encoder samples 4096 steps per 360 degrees, therefore the angular distance traveled per sample is 360/4096 or 0.087 degrees. This is reflected in the code.

4. http://ams.com/eng/Products/Magnetic-Position-Sensors/Magnetic-Rotary-Position-Sensors/AS5145B
5. http://wpilib.screenstepslive.com/s/3120/m/8559/l/91404-using-the-as5145b-magnetic-encoder-with-the-frc-control-system



WPILib 
This is a high-level robotics programming library for FIRST provided by the Worcester Polytechnic Institute (WPI). It is available in Java and C++. Team812 uses the C++ version.

A WPIlib Users Guide [6] is provided by WPI. It has not been updated since 2008, however, it remains generally applicable to the 2013 code base and items provided in the Kit Of Parts (KOP).

The most useful reference for WPILib that I've found is on the website virtualroadside.com [7]. It is valuable because it contains the class documentation and cross-references to the WPILib source file that implements the classes and methods.

A complete list of Classes is provided [8]. This class list is exhaustive, for example, check the documentation on PIDController [9] interspersed with the class and method descriptions are links to source code. The method SetSetPoint(float setpoint) is defined on line 373 of PIDController.cpp [10] 

7. http://www.virtualroadside.com/WPILib/index.html
8. http://www.virtualroadside.com/WPILib/annotated.html
9. http://www.virtualroadside.com/WPILib/class_p_i_d_controller.html
10. http://www.virtualroadside.com/WPILib/_p_i_d_controller_8cpp_source.html#l00373
