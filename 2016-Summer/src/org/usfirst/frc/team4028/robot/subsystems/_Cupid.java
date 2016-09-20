package org.usfirst.frc.team4028.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;

/**
 * This class defines the behavior of the Cupid Arrow shooting mechanism
 */
public class _Cupid 
{
	private Servo _cupidServo;
	
	public _Cupid(int victorPWMPort)
	{
    	_cupidServo = new Servo(victorPWMPort);
	}
}
