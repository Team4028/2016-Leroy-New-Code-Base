package org.usfirst.frc.team6204.robot.autonRoutines;

import java.util.Date;

import org.usfirst.frc.team6204.robot.constants.RobotMap;
import org.usfirst.frc.team6204.robot.constants.AutonConstants.Auton_Drive_Throttle_Percent;
import org.usfirst.frc.team6204.robot.constants.AutonConstants.Auton_Drive_Time_In_Secs;
import org.usfirst.frc.team6204.robot.constants.AutonConstants.Auton_Pumas_Position;
import org.usfirst.frc.team6204.robot.constants.AutonConstants.Auton_Shoot_Ball;
import org.usfirst.frc.team6204.robot.constants.AutonConstants.Auton_Infeed_Tilt_Position;
import org.usfirst.frc.team6204.robot.subsystems.Chassis;
import org.usfirst.frc.team6204.robot.subsystems.Infeed;
import org.usfirst.frc.team6204.robot.subsystems.Puma;
import org.usfirst.frc.team6204.robot.subsystemSequences.ShootBall;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/*
 * This class implements the Drive Forward Auton
 * 
 * Changes:
 * 	Refactored to change local string values w/ enums
 */
public class DriveForwardAuton 
{
	
	// define local instance variables to hold references to all subsystems that this Auton routine uses
	private Infeed 		_infeed;
	private Chassis 	_chassis;
	private Puma 		_puma;
	private ShootBall 	_shootBall;
	
	public enum Auton_Drive_Forward_State
	{
		UNDEFINED,
		SET_INITIAL_POSITIONS,
		DRIVE_FORWARD,
		SHOOT_BALL,
		SET_NEW_POSITION,
		DRIVE_REVERSE
	}
	
	private double 	_autonFwdTargetDriveTimeMSecs;
	private double 	_autonFwdDriveThrottlePercent;
	private double 	_autonRevTargetDriveTimeMSecs;
	private double 	_autonRevDriveThrottlePercent;
	private Auton_Infeed_Tilt_Position 	_autonInfeedTiltPosition;
	private Auton_Shoot_Ball 			_autonShootBall;
	private Value 	_autonPumaFrontPosition;
	private Value 	_autonPumaBackPosition;
	private long 	_autonStartTime;
	private long 	_autonReverseStartTime;

	// this keeps track of the current state
	private Auton_Drive_Forward_State _autonDriveForwardState;
	
	// constructor
	public DriveForwardAuton(Infeed infeed, Chassis chassis, Puma puma, ShootBall shootBall, Auton_Drive_Throttle_Percent autonFwdDriveThrottlePercent,
						Auton_Drive_Time_In_Secs autonFwdDriveTimeInSecs, Auton_Drive_Throttle_Percent autonRevDriveThrottlePercent, Auton_Drive_Time_In_Secs autonRevDriveTimeInSecs, 
						Auton_Pumas_Position autonPumasPosition, Auton_Infeed_Tilt_Position autonInfeedTiltPosition, Auton_Shoot_Ball autonShootBall)
	{
		_infeed = infeed;
		_chassis = chassis;
		_puma = puma;
		_shootBall = shootBall;
		
		// determine the requested fwd drive throttle %
		switch(autonFwdDriveThrottlePercent)
		{	
    		case PERCENT_30:
    			_autonFwdDriveThrottlePercent = 0.30;
    			break;
    			
    		case PERCENT_40:
    			_autonFwdDriveThrottlePercent = 0.40;
    			break;
    			
    		case PERCENT_50:
    			_autonFwdDriveThrottlePercent = 0.50;
    			break;
    			
    		case PERCENT_60:
    			_autonFwdDriveThrottlePercent = 0.60;
    			break;
    		
    		case PERCENT_70:
    			_autonFwdDriveThrottlePercent = 0.70;
    			break;
    			
    		case PERCENT_80:
    			_autonFwdDriveThrottlePercent = 0.80;
    			break;
    			
    		case PERCENT_90:
    			_autonFwdDriveThrottlePercent = 0.90;
    			break;
    			
    		case PERCENT_100:
    			_autonFwdDriveThrottlePercent = 1.00;
    	}
		
		// determine the requested fwd drive time
		switch(autonFwdDriveTimeInSecs)
    	{
    		case SECS_1:
    			_autonFwdTargetDriveTimeMSecs = 1 * 1000;
    			break;
    			
    		case SECS_1_5:
    			_autonFwdTargetDriveTimeMSecs = 1.5 * 1000;
    			break;
    			
    		case SECS_2:
    			_autonFwdTargetDriveTimeMSecs = 2 * 1000;
    			break;	
    			
    		case SECS_2_5:
    			_autonFwdTargetDriveTimeMSecs = 2.5 * 1000;
    			break;
    			
    		case SECS_3:
    			_autonFwdTargetDriveTimeMSecs = 3 * 1000;
    			break;
    			
    		case SECS_3_5:
    			_autonFwdTargetDriveTimeMSecs = 3.5 * 1000;
    			break;
    			
    		case SECS_4:
    			_autonFwdTargetDriveTimeMSecs = 4 * 1000;
    			break;
    			
    		case SECS_4_5:
    			_autonFwdTargetDriveTimeMSecs = 4.5 * 1000;
    			break;
    			
    		case SECS_5:
    			_autonFwdTargetDriveTimeMSecs = 5 * 1000;
    			break;
    			
    		case SECS_6:
    			_autonFwdTargetDriveTimeMSecs = 6 * 1000;
    			break;
    			
    		case SECS_7:
    			_autonFwdTargetDriveTimeMSecs = 7 * 1000;
    			break;
    	}
		
		// determine the requested rev drive throttle %
		switch(autonRevDriveThrottlePercent)
    	{
			case PERCENT_0: 
				_autonRevDriveThrottlePercent = 0.00;
				break;
    			
    		case PERCENT_30:
    			_autonRevDriveThrottlePercent = 0.30;
    			break;
    			
    		case PERCENT_40:
    			_autonRevDriveThrottlePercent = 0.40;
    			break;
    			
    		case PERCENT_50:
    			_autonRevDriveThrottlePercent = 0.50;
    			break;
    			
    		case PERCENT_60:
    			_autonRevDriveThrottlePercent = 0.60;
    			break;
    		
    		case PERCENT_70:
    			_autonRevDriveThrottlePercent = 0.70;
    			break;
    			
    		case PERCENT_80:
    			_autonRevDriveThrottlePercent = 0.80;
    			break;
    			
    		case PERCENT_90:
    			_autonRevDriveThrottlePercent = 0.90;
    			break;
    			
    		case PERCENT_100:
    			_autonRevDriveThrottlePercent = 1.00;
    	}
		
		// determine the requested rev drive time
		switch(autonRevDriveTimeInSecs)
    	{
			case SECS_0:
				_autonRevTargetDriveTimeMSecs = 0 * 1000;
				break;
				
    		case SECS_1:
    			_autonRevTargetDriveTimeMSecs = 1 * 1000;
    			break;
    			
    		case SECS_1_5:
    			_autonRevTargetDriveTimeMSecs = 1.5 * 1000;
    			break;
    			
    		case SECS_2:
    			_autonRevTargetDriveTimeMSecs = 2 * 1000;
    			break;
    			
    		case SECS_2_5:
    			_autonRevTargetDriveTimeMSecs = 2.5 * 1000;
    			break;
    			
    		case SECS_3:
    			_autonRevTargetDriveTimeMSecs = 3 * 1000;
    			break;
    			
    		case SECS_3_5:
    			_autonRevTargetDriveTimeMSecs = 3.5 * 1000;
    			break;
    			
    		case SECS_4:
    			_autonRevTargetDriveTimeMSecs = 4 * 1000;
    			break;
    			
    		case SECS_4_5:
    			_autonRevTargetDriveTimeMSecs = 4.5 * 1000;
    			break;
    			
    		case SECS_5:
    			_autonRevTargetDriveTimeMSecs = 5 * 1000;
    			break;
    	}
		
		// determine the requested puma positions
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
		
		// determine the requested infeed tilt position
		_autonInfeedTiltPosition = autonInfeedTiltPosition;
		/*switch(autonInfeedTiltPosition)
		{
			case INFEED_UP:
				_autonInfeedTiltPosition = "Infeed Up";
				break;
				
			case LOW_BAR:
				_autonInfeedTiltPosition = "Low Bar";
				break;
				
			case UNDEFINED:
				_autonInfeedTiltPosition = "";
				break;
		}*/
		
		// determine if we should "outfeed" the ball
		_autonShootBall = autonShootBall;
		/*switch(autonShootBall)
		{
			case YES:
				_autonShootBall = "Yes";
				break;
				
			case NO:
				_autonShootBall = "No";
				break;
			
			case UNDEFINED:
				_autonShootBall = "";
				break;
		} */
		
		// capture the start timestamp
		_autonStartTime = new Date().getTime();
		//DriverStation.reportError("Auton Drive Fwd Power: "  + Double.toString(_autonDriveThrottlePercent), false);
		//DriverStation.reportError("Auton Drive Fwd Time: "  + Double.toString(_autonTargetDriveTimeMSecs), false);
		
		// init the state machine
		_autonDriveForwardState = Auton_Drive_Forward_State.SET_INITIAL_POSITIONS;
	}
	
	// this is main method that implements the steps of this auton routine
	public void Execute()
	{
		// decide what state we are in
		switch(_autonDriveForwardState)
		{
			case SET_INITIAL_POSITIONS:
				_puma.ChgPumaFrontPosition(_autonPumaFrontPosition);
				_puma.ChgPumaBackPosition(_autonPumaBackPosition);
				
				if(!_infeed.IsInfeedTiltAxisZeroedYet())
		    	{
			    	// zero the infeed tilt axis (fyi this may be the same physical location as the stored position)
					_infeed.ZeroInfeedTiltAxisReentrant();
		    	}
		    	else
		    	{
	    			if(_autonInfeedTiltPosition == Auton_Infeed_Tilt_Position.LOW_BAR) // "Low Bar")
	    			{
	    				_infeed.MoveInfeedTiltAxisToDeployedPositionReentrant();
	    				DriverStation.reportError("Is Infeed Deploy Still Running:  " + Boolean.toString(_infeed.getIsInfeedTiltDeployStillRunning()), false);
	    				
	    				if (_infeed.getIsInfeedTiltDeployStillRunning() == false)
	    				{
	    					_autonDriveForwardState = Auton_Drive_Forward_State.DRIVE_FORWARD;
	    					DriverStation.reportError("Infeed Tilt Deploy no longer running..........", false);
			    			DriverStation.reportError("Switching to DRIVE FORWARD State" , false);
	    				}
	    			}
	    			else if(_autonInfeedTiltPosition == Auton_Infeed_Tilt_Position.INFEED_UP) //  "Infeed Up")
	    			{
	    				_infeed.MoveInfeedTiltAxisToCrossDefensePositionReentrant();
	    				if (_infeed.getIsInfeedTiltCrossDefenseStillRunning() == false)
	    				{
	    					_autonDriveForwardState = Auton_Drive_Forward_State.DRIVE_FORWARD;
			    			DriverStation.reportError("Switching to DRIVE FORWARD State" , false);
			    			DriverStation.reportError("Infeed in cross defense position......", false);
	    				}
	    			}
		    	}
				break;
			
			case DRIVE_FORWARD:
				long driveFwdElapsedTime = (new Date().getTime() - _autonStartTime);
				if (driveFwdElapsedTime  <= _autonFwdTargetDriveTimeMSecs)
		    	{
		    		_chassis.RunDriveMotors(_autonFwdDriveThrottlePercent, 0);
		    	}
		    	else
		    	{
		        	_chassis.StopDriveMotors();
		        	if (_autonShootBall == Auton_Shoot_Ball.YES)  //  "Yes")
		        	{
		        		_shootBall.Execute();
		        	}
		        	else
		        	{
		        	}
		        	_autonDriveForwardState = Auton_Drive_Forward_State.SHOOT_BALL;
		        	DriverStation.reportError("Switching to SHOOT BALL State", false);
		    	}
				break;
				
			case SHOOT_BALL:
				if(_autonShootBall == Auton_Shoot_Ball.YES)   //  "Yes")
				{
					if (_shootBall.getIsShootBallStillRunning())
					{
						_shootBall.Execute();
					}
					else 
					{
						_autonDriveForwardState = Auton_Drive_Forward_State.DRIVE_REVERSE;
						_autonReverseStartTime = new Date().getTime();
						DriverStation.reportError("Switching to DRIVE REVERSE State", false);
					}
				}
				else if (_autonShootBall == Auton_Shoot_Ball.NO)  //  "No")
				{
					_autonDriveForwardState = Auton_Drive_Forward_State.DRIVE_REVERSE;
					_autonReverseStartTime = new Date().getTime();
					DriverStation.reportError("Switching to DRIVE REVERSE State", false);
				}
				break;
				
			case SET_NEW_POSITION:
				if(_autonInfeedTiltPosition == Auton_Infeed_Tilt_Position.LOW_BAR) //  "Low Bar")
    			{
    				_infeed.MoveInfeedTiltAxisToDeployedPositionReentrant();
    				if (_infeed.getIsInfeedTiltDeployStillRunning() == false)
    				{
    					_autonReverseStartTime = new Date().getTime();
    					_autonDriveForwardState = Auton_Drive_Forward_State.DRIVE_REVERSE;
		    			DriverStation.reportError("Switching to DRIVE REVERSE State" , false);
    				}
    			}
    			else if(_autonInfeedTiltPosition == Auton_Infeed_Tilt_Position.INFEED_UP) //   "Infeed Up")
    			{
    				_infeed.MoveInfeedTiltAxisToCrossDefensePositionReentrant();
    				if (_infeed.getIsInfeedTiltCrossDefenseStillRunning() == false)
    				{
    					_autonReverseStartTime = new Date().getTime();
    					_autonDriveForwardState = Auton_Drive_Forward_State.DRIVE_REVERSE;
    					DriverStation.reportError("Switching to DRIVE REVERSE State" , false);
    				}
    			}
				break;
				
			case DRIVE_REVERSE:
				long driveRevElapsedTime = (new Date().getTime() - _autonReverseStartTime);
				if (driveRevElapsedTime  <= _autonRevTargetDriveTimeMSecs)
		    	{
		    		_chassis.RunDriveMotors(-1.0 * _autonRevDriveThrottlePercent, 0);
		    	}
		    	else
		    	{
		        	_chassis.StopDriveMotors();
		        	//DriverStation.reportError("Auton Completed...", false);
		    	}
				break;
		}
	}
}
