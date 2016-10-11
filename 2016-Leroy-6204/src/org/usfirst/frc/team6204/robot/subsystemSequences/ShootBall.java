package org.usfirst.frc.team6204.robot.subsystemSequences;

import java.util.Date;

import org.usfirst.frc.team6204.robot.constants.RobotMap;
import org.usfirst.frc.team6204.robot.subsystems.Infeed;
import org.usfirst.frc.team6204.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class ShootBall {
	private Infeed _infeed;
	private Shooter _shooter;
	
	public long _shootBallSequenceStartTime;
	private static final double TIME_TO_INFEED_IN_MS = 0.5 * 1000;
	private static final double TIME_TO_SPIN_SHOOTER_WHEELS_IN_MS = 1 * 1000;
	private static final double TIME_TO_INFEED_OUT_IN_MS = 0.5 * 1000;
	
	public boolean _isShootBallStillRunning;
	
	public enum SHOOT_BALL_STATE
	{
		UNDEFINED,
		SET_VARIABLES,
		INFEED_IN,
		SPIN_SHOOTER_WHEELS,
		INFEED_OUT,
		SEQUENCE_COMPLETE,
	}
	
	private SHOOT_BALL_STATE _shootBallState;
		
	public ShootBall(Infeed infeed, Shooter shooter)
	{
		_infeed = infeed;
		_shooter = shooter;
		
		_shootBallState = SHOOT_BALL_STATE.UNDEFINED;
		_isShootBallStillRunning = false;
	}
	
	public void Execute()
	{
		long sequenceElapsedTime = 0;
		switch(_shootBallState)
		{
			case UNDEFINED:
				_shootBallState = SHOOT_BALL_STATE.SET_VARIABLES;
				_isShootBallStillRunning = true;
				break;
				
			case SET_VARIABLES:
				_shootBallSequenceStartTime = new Date().getTime();
				DriverStation.reportError("IN SET VARIABLES MODE", false);
				_shootBallState = SHOOT_BALL_STATE.INFEED_IN;
				break;
				
			case INFEED_IN:
				sequenceElapsedTime = (new Date().getTime() - _shootBallSequenceStartTime);
				
				_infeed.RunInfeedAcquireMotorIn();
				
				if (sequenceElapsedTime > TIME_TO_INFEED_IN_MS)
				{
					DriverStation.reportError("Changing to Spin Shooter Wheels State", false);
					_shootBallState = SHOOT_BALL_STATE.SPIN_SHOOTER_WHEELS;
				}
				break;
				
			case SPIN_SHOOTER_WHEELS:
				sequenceElapsedTime = (new Date().getTime() - _shootBallSequenceStartTime);
				
				_shooter.RunShooterMotors(-1.0);
				
				if (sequenceElapsedTime > (TIME_TO_INFEED_IN_MS + TIME_TO_SPIN_SHOOTER_WHEELS_IN_MS))
				{
					_shootBallState = SHOOT_BALL_STATE.INFEED_OUT;
					DriverStation.reportError("Changing to Infeed Out State", false);
				}
				break;
				
			case INFEED_OUT:
				sequenceElapsedTime = (new Date().getTime() - _shootBallSequenceStartTime);
				
				_infeed.RunInfeedAcquireMotorOut();
				
				if (sequenceElapsedTime > (TIME_TO_INFEED_IN_MS + TIME_TO_SPIN_SHOOTER_WHEELS_IN_MS + TIME_TO_INFEED_OUT_IN_MS))
				{
					_shootBallState = SHOOT_BALL_STATE.SEQUENCE_COMPLETE;
					DriverStation.reportError("Shoot Ball Sequence Completed", false);
				}
				break;
			
			case SEQUENCE_COMPLETE:
				_infeed.RunInfeedAcquireMotor(0.0);
				_shooter.RunShooterMotors(0.0);
				_isShootBallStillRunning = false;
				_shootBallState = SHOOT_BALL_STATE.UNDEFINED;
				break;
		}
	}
	
	public void setIsShootBallStillRunning(boolean bool)
	{
		 bool = _isShootBallStillRunning; 
	}
	
	public void setShootBallSequenceStartTime(long startTime)
	{
		startTime = _shootBallSequenceStartTime;
	}
	
	public void setShootBallSequenceState(SHOOT_BALL_STATE shootBallState)
	{
		shootBallState = _shootBallState;
	}
	
	public boolean getIsShootBallStillRunning()
	{
		return _isShootBallStillRunning;
	}
}
