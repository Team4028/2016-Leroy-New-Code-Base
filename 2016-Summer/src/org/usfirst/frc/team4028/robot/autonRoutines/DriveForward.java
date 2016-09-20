package org.usfirst.frc.team4028.robot.autonRoutines;

import java.util.Date;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Drive_Throttle_Percent;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Drive_Time_In_Secs;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Pumas_Position;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Puma;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DriveForward {
	
	private Infeed _infeed;
	private Chassis _chassis;
	private Puma _puma;
	
	private double _autonTargetDriveTimeMSecs;
	private double _autonDriveThrottlePercent;
	private Value _autonPumaFrontPosition;
	private Value _autonPumaBackPosition;
	private long _autonStartTime;
		
	public DriveForward(Infeed infeed, Chassis chassis, Puma puma, Auton_Drive_Throttle_Percent autonDriveThrottlePercent, 
			Auton_Drive_Time_In_Secs autonDriveTimeInSecs, Auton_Pumas_Position autonPumasPosition)
	{
		_infeed = infeed;
		_chassis = chassis;
		_puma = puma;
		
		// determine the requested drive time
		
		switch(autonDriveThrottlePercent)
    	{
    		case PERCENT_10:
    			_autonDriveThrottlePercent = 0.10;
    			break;
    			
    		case PERCENT_20:
    			_autonDriveThrottlePercent = 0.20;
    			break;
    			
    		case PERCENT_30:
    			_autonDriveThrottlePercent = 0.30;
    			break;
    			
    		case PERCENT_40:
    			_autonDriveThrottlePercent = 0.40;
    			break;
    			
    		case PERCENT_50:
    			_autonDriveThrottlePercent = 0.50;
    			break;
    			
    		case PERCENT_60:
    			_autonDriveThrottlePercent = 0.60;
    			break;
    		
    		case PERCENT_70:
    			_autonDriveThrottlePercent = 0.70;
    			break;
    			
    		case PERCENT_80:
    			_autonDriveThrottlePercent = 0.80;
    			break;
    			
    		case PERCENT_90:
    			_autonDriveThrottlePercent = 0.90;
    			break;
    	}
		
		switch(autonDriveTimeInSecs)
    	{
    		case SECS_1:
    			_autonTargetDriveTimeMSecs = 1 * 1000;
    			break;
    			
    		case SECS_2:
    			_autonTargetDriveTimeMSecs = 2 * 1000;
    			break;	
    			
    		case SECS_3:
    			_autonTargetDriveTimeMSecs = 3 * 1000;
    			break;
    			
    		case SECS_4:
    			_autonTargetDriveTimeMSecs = 4 * 1000;
    			break;
    			
    		case SECS_5:
    			_autonTargetDriveTimeMSecs = 5 * 1000;
    			break;
    			
    		case SECS_6:
    			_autonTargetDriveTimeMSecs = 6 * 1000;
    			break;
    			
    		case SECS_7:
    			_autonTargetDriveTimeMSecs = 7 * 1000;
    			break;
    			
    		case SECS_8:
    			_autonTargetDriveTimeMSecs = 8 * 1000;
    			break;
    			
    		case SECS_9:
    			_autonTargetDriveTimeMSecs = 9 * 1000;
    			break;
    	}
		
		switch(autonPumasPosition)
    	{
    	    case PUMAS_UP:
    	    	_autonPumaFrontPosition = RobotMap.PUMA_FRONT_UP_POSITION;
    	    	_autonPumaBackPosition = RobotMap.PUMA_BACK_UP_POSITION;
    	     	break;
    	     
    	    case PUMAS_DOWN:
    	    	_autonPumaFrontPosition = RobotMap.PUMA_FRONT_DOWN_POSITION;
    	    	_autonPumaBackPosition = RobotMap.PUMA_BACK_DOWN_POSITION;
    	    	break;
    	    	 
    	    case UNDEFINED:
    	    	break;
    	    	
			default:
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
	}
}
