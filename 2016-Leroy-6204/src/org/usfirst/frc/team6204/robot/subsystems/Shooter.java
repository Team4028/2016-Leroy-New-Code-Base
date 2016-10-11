package org.usfirst.frc.team6204.robot.subsystems;

import org.usfirst.frc.team6204.robot.Utilities;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;

/**
 * This class defines the behavior of the Shooter Subsystem
 */
public class Shooter 
{
	// ======================================
	// Define constants for Shooter Motors
	// ======================================	
	private VictorSP _shooterLeftMtr;
	private VictorSP _shooterRightMtr;
	
	private static final double ACC_DEC_TOT_TIME_IN_SECS = 0.25;
	private static final double VELOCITY_CMD_TARGET_THRESHHOLD = 0.01;	
	
	//acc dec working variables
	private long _lastVelocityCmdChgTimeStamp;
	private double _lastCycleVelocityCmdRaw;			// snapshot of cmd target during the previous cycle (to tell if it chgd in this cycle)
	private double _currentVelocityCmdRaw;				// new target after latest chg in cmd input
	private double _previousVelocityCmdRaw;				// old target before latest chg in cmd input 
	private double _currentVelocityCmdAccDec;			// current cmd on acc / dec ramp
	private double _currentWheelVelocityCmd;			// backer for public property
	
	private final boolean _isAccDecModeEnabled = false;	// set as a constant since we are NOT mapping this to a GamePad button, enable/disable acc/dec mode here!
	private boolean _isAccDecModeActive;				// are we in the middle of an acc/dec ramp ?
	
	// ===================================
	// Constructors follow
	// ===================================
	public Shooter(int leftMtrPWMPort, int rightMtrPWMPort)
	{
		// ======================
		// config left motor
		// ======================
		_shooterLeftMtr = new VictorSP(leftMtrPWMPort);
    	
    	// ======================
    	// config right motor
    	// ======================
		_shooterRightMtr = new VictorSP(rightMtrPWMPort);
		
    	_isAccDecModeActive = false;
	}
	
	//============================================================================================
	// Methods follow (methods make the robot do something (ie: push changes to the robot hardware)
	//============================================================================================
	public void StopShooterMotors()
	{
		this.RunShooterMotors(0.0);
	}
	
	public void RunShooterMotorsIn()
	{
		this.RunShooterMotors(1.0);
	}
	
	public void RunShooterMotorsOut()
	{
		this.RunShooterMotors(-1.0);
	}
	
	public void RunShooterMotors(double newVelocityCmdRaw)
	{		
		if(Math.abs(newVelocityCmdRaw - _lastCycleVelocityCmdRaw) > VELOCITY_CMD_TARGET_THRESHHOLD)
		{
			// the input command has changed, snapshot current (from last cycle) into previous
			if(_isAccDecModeActive)
			{
				// if we were in the middle of an acc/dec ramp, we want ramp from the acc/dec speed we are at
				_previousVelocityCmdRaw = _currentVelocityCmdAccDec;
			}
			else
			{
				// ramp from the last speed
				_previousVelocityCmdRaw = _currentVelocityCmdRaw;
			}
			
			// here is the new target
			_currentVelocityCmdRaw = newVelocityCmdRaw;
			
			if(_isAccDecModeEnabled)
			{
				// start acc/dec cycle
				_isAccDecModeActive = true;
				_lastVelocityCmdChgTimeStamp = System.currentTimeMillis();
			}
		}
			
		if(!_isAccDecModeEnabled  || !_isAccDecModeActive)
		{
			// acc/dec mode is disabled (or we already reached the target), just use the new target
			_currentWheelVelocityCmd = _currentVelocityCmdRaw;
		}
		else
		{
			// calc the accel / decel s-curve adj cmd
			_currentVelocityCmdAccDec = CalcAccDecThrottleCmd_V2(_previousVelocityCmdRaw, _currentVelocityCmdRaw, _lastVelocityCmdChgTimeStamp);
			
			_currentWheelVelocityCmd = _currentVelocityCmdAccDec;
			
			// when we reach the desired target, reset the previous snapshot and turn off acc/dec mode
			if(Math.abs(newVelocityCmdRaw - _currentWheelVelocityCmd) < VELOCITY_CMD_TARGET_THRESHHOLD)
			{
				_previousVelocityCmdRaw = _currentVelocityCmdRaw;
				_isAccDecModeActive = false;
			}
		}
		
		// send cmd to mtr controllers  (Remember: FWD stick: LEFT CCW, RIGHT CW)
		_shooterLeftMtr.set(_currentWheelVelocityCmd);
		_shooterRightMtr.set((_currentWheelVelocityCmd * -1.0));
		
		// snapshot the current command so we can tell if it changed next cycle
		_lastCycleVelocityCmdRaw = newVelocityCmdRaw;		
	}
	
	// implement linear accel / dec decel
	private double CalcAccDecThrottleCmd_V2(double rampFromThrottleCmd, double rampToThrottleCmd, long lastCmdChgTimeStamp)
	{
		/*
		 * Calc the slope
		 *   
		 *        chg in y
		 *   m = ===========
		 *        chg in x
		 *        
		 * Calc the new cmd using the point-slope formula      
		 *   y - yint = m * (x - xint)    
		 */
		
		double slope = (rampToThrottleCmd - rampFromThrottleCmd) / ACC_DEC_TOT_TIME_IN_SECS; // m

        double elapsedSecsSinceLastChg = (System.currentTimeMillis() - lastCmdChgTimeStamp) / 1000.0; // x
        
        // finally calc the adj cmd
        double accDecCmd = Utilities.RoundDouble(((slope * elapsedSecsSinceLastChg) + rampFromThrottleCmd), 3);
        
        String debugMsg = String.format("AccDec From: [%f] at: [%f] to: [%f]", rampFromThrottleCmd, accDecCmd, rampToThrottleCmd);
        //DriverStation.reportError(debugMsg, false);

        return accDecCmd;
	}
	
	//============================================================================================
	// Property Accessors follow (properties only change internal state) (ie: DO NOT push changes to the robot hardware)
	//============================================================================================

	public double getCurrentWheelVelocityCmd()
	{
		return _currentWheelVelocityCmd;
	}
}
