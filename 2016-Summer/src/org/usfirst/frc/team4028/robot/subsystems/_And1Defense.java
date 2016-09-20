package org.usfirst.frc.team4028.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;

/**
 * This class defines the behavior of the "And1" defense
 * 
 * Physically it consists of "U" shaped plastic extension that rotates down and up to extend our perimeter
 * The controllable element is a small ServoMotor
 * 
 */
public class _And1Defense
{
	public enum AND1_POSITION
	{
		UNDEFINED,
		DOWN,
		UP
	}

	private Servo _and1Servo;
	private AND1_POSITION _and1CurrentPosition;
	private AND1_POSITION _and1TargetPosition;
	
	private final double and1DownCmd = 1.0;
	private final double and1UpCmd = 0.0;
	
	/**
	 * Create  new instance of the "And1" subsystem
	*/		
	public _And1Defense(int victorPWMPort) 
	{
    	_and1Servo = new Servo(victorPWMPort);
    	
    	_and1CurrentPosition = AND1_POSITION.UNDEFINED;
    	_and1TargetPosition = AND1_POSITION.UNDEFINED;
	}
	
	/**
	 * Return the current position
	*/
	public AND1_POSITION getAnd1CurrentPosition()
	{
		return _and1CurrentPosition;
	}
	
	/**
	 * Set the target position
	*/
	public void setAnd1TargetPosition(AND1_POSITION and1TargetPosition)
	{
		_and1TargetPosition = and1TargetPosition;
	}
	
	/**
	 * Update our knowledge of the current state of this object
	*/
	public void ReadInputs()
	{
		// TODO: implement this
		// _and1CurrentPosition = 
	}
	
	/**
	 * Send our desired state out to the physical robot
	*/
	public void PushOutputs()
	{
		switch(_and1TargetPosition)
		{
			case UP:
				_and1Servo.setPosition(and1UpCmd);
				break;
				
			case DOWN:
				_and1Servo.setPosition(and1DownCmd);
				break;
				
			default:
				DriverStation.reportError("And1 Target Position is undefined!", false);
				break;
		}

		_and1CurrentPosition = _and1TargetPosition;
	}
}
