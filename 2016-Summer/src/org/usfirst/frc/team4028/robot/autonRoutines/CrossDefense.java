package org.usfirst.frc.team4028.robot.autonRoutines;

import java.util.Date;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Cross_Defense_Type;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Puma;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CrossDefense {
	private Infeed _infeed;
	private Chassis _chassis;
	private Puma _puma;
	
	private double _autonTargetDriveTimeMSecs;
	private double _autonDriveThrottlePercent;
	private Value _autonPumaFrontPosition;
	private Value _autonPumaBackPosition;
	private long _autonStartTime;
		
	public CrossDefense(Infeed infeed, Chassis chassis, Puma puma, Auton_Cross_Defense_Type autonCrossDefenseType)
	{
		_infeed = infeed;
		_chassis = chassis;
		_puma = puma;
		
		switch(autonCrossDefenseType)
		{
			case MOAT:
				_autonTargetDriveTimeMSecs = 0.0;
				_autonDriveThrottlePercent = 0.0;
				_autonPumaFrontPosition = RobotMap.PUMA_FRONT_UP_POSITION;
				_autonPumaBackPosition = RobotMap.PUMA_BACK_UP_POSITION;
				break;
			
			case RAMPARTS:
				_autonTargetDriveTimeMSecs = 0.0;
				_autonDriveThrottlePercent = 0.0;
				_autonPumaFrontPosition = RobotMap.PUMA_FRONT_UP_POSITION;
				_autonPumaBackPosition = RobotMap.PUMA_BACK_UP_POSITION;
				break;
				
			case ROCKWALL:
				_autonTargetDriveTimeMSecs = 0.0;
				_autonDriveThrottlePercent = 0.0;
				_autonPumaFrontPosition = RobotMap.PUMA_FRONT_UP_POSITION;
				_autonPumaBackPosition = RobotMap.PUMA_BACK_UP_POSITION;
				break;
				
			case ROUGH_TERRAIN:
				_autonTargetDriveTimeMSecs = 0.0;
				_autonDriveThrottlePercent = 0.0;
				_autonPumaFrontPosition = RobotMap.PUMA_FRONT_UP_POSITION;
				_autonPumaBackPosition = RobotMap.PUMA_BACK_UP_POSITION;
				break;
				
			case UNDEFINED:
				_autonTargetDriveTimeMSecs = 0.0;
				_autonDriveThrottlePercent = 0.0;
				_autonPumaFrontPosition = RobotMap.PUMA_FRONT_UP_POSITION;
				_autonPumaBackPosition = RobotMap.PUMA_BACK_UP_POSITION;
				break;
		}
		
		_autonStartTime = new Date().getTime();
	}
	
	public void Execute()
	{
		_puma.ChgPumaFrontPosition(_autonPumaFrontPosition);
		_puma.ChgPumaBackPosition(_autonPumaBackPosition);
		
		if(!_infeed.IsInfeedTiltAxisZeroedYet())
    	{
	    	// zero the infeed tilt axis (fyi this may be the same physical location as the stored position)
			_infeed.ZeroInfeedTiltAxisReentrant();
    	}
    	else
    	{
    		_infeed.MoveInfeedTiltAxisToCrossDefensePositionReentrant();
    	}
		
		
		long driveFwdElapsedTime = (new Date().getTime() - _autonStartTime);
		if (driveFwdElapsedTime  <= _autonTargetDriveTimeMSecs)
    	{
    		_chassis.RunDriveMotors(_autonDriveThrottlePercent, 0);
    	}
    	else
    	{
        	_chassis.StopDriveMotors();
    	}
		
		/*
		 * Add code here to release ball if we decide to also add that functionality to this auton routine instead of making another one
		 */
    
		
	}
}
