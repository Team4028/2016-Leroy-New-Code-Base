package org.usfirst.frc.team6204.robot.subsystems;

import java.math.BigDecimal;
import java.math.RoundingMode;

import org.usfirst.frc.team6204.robot.Utilities;
import org.usfirst.frc.team6204.robot.constants.RobotMap;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * This class defines the behavior of the Chassis / Drive Subsystem
 */
public class Chassis 
{
	// ===================================
	// define objects for drive motors
	// ===================================
	private CANTalon _leftDriveMasterMtr;
	private CANTalon _leftDriveSlave1Mtr;
	private CANTalon _leftDriveSlave2Mtr;
	private CANTalon _rightDriveMasterMtr;
	private CANTalon _rightDriveSlave1Mtr;
	private CANTalon _rightDriveSlave2Mtr;
	
	private RobotDrive _robotDrive;
	
	private double _currentArcadeDriveThrottleCmd;	// backer for public property
	private double _currentArcadeDriveTurnCmd;		// backer for public property
	private double _currentDriveSpeedScalingFactorClamped;
	
	//acc dec working variables
	private long _lastThrottleCmdChgTimeStamp;
	private double _lastCycleThrottleCmdScaled;		// snapshot of cmd target during the previous cycle (to tell if it chgd in this cycle)
	
	private double _turnSpeedScalingFactor = 0.8;
	
	private boolean _isAccDecModeEnabled;			// overall enable/disable of acc/dec mode, mapped to a gamepad button
	private boolean _isAccDecModeActive;			// are we in the middle of an acc/dec ramp ?
	
	private double _currentThrottleCmdScaled;		// new target after latest chg in cmd input
	private double _previousThrottleCmdScaled;		// old target before latest chg in cmd input 
	
	private double _currentThrottleCmdAccDec;		// current cmd on acc / dec ramp
	
	// constants	
	private static final double ACC_DEC_RATE_FACTOR = 5.0;
	private static final double ACC_DEC_TOT_TIME_IN_SECS = 0.5;
	private static final double THROTTLE_CMD_TARGET_THRESHHOLD = 0.01;
	
	// ===================================
	// define objects for shifter
	// ===================================
	private DoubleSolenoid _shifterSolenoid;
	
	private Value _currentShifterPosition;
	
	// ===================================
	// Constructors follow
	// ===================================
	
	// 1 motor per side configuration
	public Chassis(int talonLeftMasterMtrCanBusAddr, int talonRightMasterMtrCanBusAddr, 
			int pcmCanBusAddress, int shifterSolenoidHighPCMPort, int shifterSolenoidLowPCMPort) 
	{
		this(talonLeftMasterMtrCanBusAddr, 0, 0, talonRightMasterMtrCanBusAddr, 0, 0, 
				pcmCanBusAddress, shifterSolenoidHighPCMPort, shifterSolenoidLowPCMPort);
	}
	
	// 2 motor per side configuration
	public Chassis(int talonLeftMasterMtrCanBusAddr, int talonLeftSlave1MtrCanBusAddr,
			int talonRightMasterMtrCanBusAddr, int talonRightSlave1MtrCanBusAddr, 
			int pcmCanBusAddress, int shifterSolenoidHighPCMPort, int shifterSolenoidLowPCMPort)
	{
		this(talonLeftMasterMtrCanBusAddr, talonLeftSlave1MtrCanBusAddr, 0, 
				talonRightMasterMtrCanBusAddr, talonRightSlave1MtrCanBusAddr, 0, 
				pcmCanBusAddress, shifterSolenoidHighPCMPort, shifterSolenoidLowPCMPort);
	}
	
	// 3 motor per side configuration
	public Chassis(int talonLeftMasterMtrCanBusAddr, int talonLeftSlave1MtrCanBusAddr, int talonLeftSlave2MtrCanBusAddr,
					int talonRightMasterMtrCanBusAddr, int talonRightSlave1MtrCanBusAddr, int talonRightSlave2MtrCanBusAddr,
					int pcmCanBusAddress, int shifterSolenoidHighPCMPort, int shifterSolenoidLowPCMPort)
	{
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
    	_leftDriveMasterMtr = new CANTalon(talonLeftMasterMtrCanBusAddr);
    	_leftDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
    	_leftDriveMasterMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	//_leftDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_leftDriveMasterMtr.reverseSensor(false);  							// do not invert encoder feedback
    	_leftDriveMasterMtr.enableLimitSwitch(false, false);
    	//_leftDriveMasterMtr.reverseOutput(true);

    	// if we have this motor
    	if(talonLeftSlave1MtrCanBusAddr > 0)
    	{
	    	_leftDriveSlave1Mtr = new CANTalon(talonLeftSlave1MtrCanBusAddr);	
	    	_leftDriveSlave1Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
	    	_leftDriveSlave1Mtr.set(talonLeftMasterMtrCanBusAddr);
	    	_leftDriveSlave1Mtr.enableBrakeMode(false);							// default to brake mode DISABLED
	    	_leftDriveSlave1Mtr.enableLimitSwitch(false, false);
	    	//_leftDriveSlaveMtr.reverseOutput(true);
    	}
    	
    	// if we have this motor
    	if(talonLeftSlave2MtrCanBusAddr > 0)
    	{
	    	_leftDriveSlave2Mtr = new CANTalon(talonLeftSlave2MtrCanBusAddr);
	    	_leftDriveSlave2Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);   // set this mtr ctrlr as a slave
	    	_leftDriveSlave2Mtr.set(talonLeftMasterMtrCanBusAddr);
	    	_leftDriveSlave2Mtr.enableBrakeMode(false);
	    	_leftDriveSlave2Mtr.enableLimitSwitch(false, false);
	    	//_leftDriveSlave2Mtr.reverseOutput(true);
    	}
    	
    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CCW = Drive FWD
    	// ===================
    	_rightDriveMasterMtr = new CANTalon(talonRightMasterMtrCanBusAddr);
    	_rightDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle.
    	_rightDriveMasterMtr.enableBrakeMode(false);						// default to brake mode DISABLED
    	//_rightDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_rightDriveMasterMtr.reverseSensor(true);  							// invert encoder feedback
    	_rightDriveMasterMtr.enableLimitSwitch(false, false);
    	//_rightDriveMasterMtr.reverseOutput(true);
    	
    	// if we have this motor
    	if(talonRightSlave1MtrCanBusAddr > 0)
    	{
	    	_rightDriveSlave1Mtr = new CANTalon(talonRightSlave1MtrCanBusAddr);	
	    	_rightDriveSlave1Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
	    	_rightDriveSlave1Mtr.set(talonRightMasterMtrCanBusAddr);
	    	_rightDriveSlave1Mtr.enableBrakeMode(false);							// default to brake mode DISABLED
	    	_rightDriveSlave1Mtr.enableLimitSwitch(false, false);
	    	//_rightDriveSlaveMtr.reverseOutput(true);
    	}
    	
    	// if we have this motor
    	if(talonRightSlave2MtrCanBusAddr > 0)
    	{
	    	_rightDriveSlave2Mtr = new CANTalon(talonRightSlave2MtrCanBusAddr);
	    	_rightDriveSlave2Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
	    	_rightDriveSlave2Mtr.set(talonRightMasterMtrCanBusAddr);
	    	_rightDriveSlave2Mtr.enableBrakeMode(false);
	    	_rightDriveSlave2Mtr.enableLimitSwitch(false, false);
	    	//_rightDriveSlave2Mtr.reverseOutput(true);
    	}
    	
    	// Shifter
    	_shifterSolenoid = new DoubleSolenoid(pcmCanBusAddress, shifterSolenoidHighPCMPort, shifterSolenoidLowPCMPort);
    	
    	//====================
    	// Arcade Drive
    	//====================
    	// Arcade Drive configured to drive in three motor setup, other two motors follow as slaves 
    	_robotDrive = new RobotDrive(_leftDriveMasterMtr, _rightDriveMasterMtr);
    	
    	//set default scaling factor
    	_currentDriveSpeedScalingFactorClamped = 1.0;
    	
    	_isAccDecModeActive = false;
	}

	//============================================================================================
	// Methods follow (methods make the robot do something (ie: push changes to the robot hardware)
	//============================================================================================
	
	public void StopDriveMotors()
	{
		RunDriveMotors(0.0, 0.0);
	}
	
	// This is the main drive method
	public void RunDriveMotors(double newThrottleCmdRaw, double newTurnCmdRaw)
	{
		double newThrottleCmdScaled = Utilities.RoundDouble((newThrottleCmdRaw * _currentDriveSpeedScalingFactorClamped), 3);
		
		if(Math.abs(newThrottleCmdScaled - _lastCycleThrottleCmdScaled) > THROTTLE_CMD_TARGET_THRESHHOLD)
		{
			// the input command has changed, snapshot current (from last cycle) into previous
			if(_isAccDecModeActive)
			{
				// if we were in the middle of acc/dec we want ramp from the acc/dec speed we are at
				_previousThrottleCmdScaled = _currentThrottleCmdAccDec;
			}
			else
			{
				// ramp from the last scaled speed
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
			}
			
			// here is the new target
			_currentThrottleCmdScaled = newThrottleCmdScaled;
			
			if(_isAccDecModeEnabled)
			{
				// start acc/dec cycle
				_isAccDecModeActive = true;
				_lastThrottleCmdChgTimeStamp = System.currentTimeMillis();
			}
		}
			
		if(!_isAccDecModeEnabled  || !_isAccDecModeActive)
		{
			// acc/dec mode is disabled (or we already reached the target), just use the scaled command
			_currentArcadeDriveThrottleCmd = _currentThrottleCmdScaled;
		}
		else
		{
			// calc the accel / decel s-curve adj cmd
			_currentThrottleCmdAccDec = CalcAccDecThrottleCmd_V1(_previousThrottleCmdScaled, _currentThrottleCmdScaled, _lastThrottleCmdChgTimeStamp);
			
			_currentArcadeDriveThrottleCmd = _currentThrottleCmdAccDec;
			
			// when we reach the desired target, reset the previous snapshot and turn off acc/dec mode
			if(Math.abs(newThrottleCmdScaled - _currentArcadeDriveThrottleCmd) < THROTTLE_CMD_TARGET_THRESHHOLD)
			{
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
				_isAccDecModeActive = false;
			}
		}

		// calc turn command
		_currentArcadeDriveTurnCmd = Utilities.RoundDouble((newTurnCmdRaw * _turnSpeedScalingFactor), 3);
		
		// send cmd to mtr controllers
		_robotDrive.arcadeDrive(-1.0 * _currentArcadeDriveThrottleCmd, _currentArcadeDriveTurnCmd);
		
		// snapshot the current command so we can tell if it changed next cycle
		_lastCycleThrottleCmdScaled = newThrottleCmdScaled;
	}
	
	// implement s-curve accel / decel
	private double CalcAccDecThrottleCmd_V1(double rampFromThrottleCmd, double rampToThrottleCmd, long lastCmdChgTimeStamp)
	{
		/*
		 * Sigmoid function
		 * 
		 * 				1
		 *   = 	===================
		 *       (1 + e^(-k*(x-a))
		 * 
		 */
		
		double accDecMidpointTimeSecs = ACC_DEC_TOT_TIME_IN_SECS / 2.0;    // a

        double minusK = -1.0 * ACC_DEC_RATE_FACTOR;
        
        double elapsedSecsSinceLastChg = (System.currentTimeMillis() - lastCmdChgTimeStamp) / 1000.0; // x
        
        double xMinusA = elapsedSecsSinceLastChg - accDecMidpointTimeSecs;

        double scaleFactor = 1.0 / ( 1.0 + Math.exp(minusK * xMinusA) );

        // finally calc the adj cmd
        double accDecCmd = Utilities.RoundDouble(rampFromThrottleCmd + ((rampToThrottleCmd - rampFromThrottleCmd) * scaleFactor), 3);
        
        String debugMsg = String.format("AccDec From: [%f] at: [%f] to: [%f]", rampFromThrottleCmd, accDecCmd, rampToThrottleCmd);
        //DriverStation.reportError(debugMsg, false);
        
        return accDecCmd;
	}
	
	// toggle the acc/dec mode enabled/disabled
	public void ToggleAccDecMode()
	{
		if(this.getIsAccDecModeEnabled())
		{
			this.setAccDecModeEnabled(false);
		}
		else
		{
			this.setAccDecModeEnabled(true);
		}
	}
	
	public void ToggleTurnSpeedScaling()
	{
		if(_turnSpeedScalingFactor == 0.8)
		{
			_turnSpeedScalingFactor = 0.6;
		}
		else
		{
			_turnSpeedScalingFactor = 0.8;
		}
	}
	
	// toggle the shifter high / low gear
	public void ToggleGear()
	{
		if (getCurrentGear() == RobotMap.SHIFTER_LOW_GEAR)
		{
			ShiftGear(RobotMap.SHIFTER_HIGH_GEAR);
		}
		else {
			ShiftGear(RobotMap.SHIFTER_LOW_GEAR);
		}
	}
	
	public void ShiftGear(Value newShifterPosition)
	{		
		_shifterSolenoid.set(newShifterPosition);
		_currentShifterPosition = newShifterPosition;
		
		if(_currentShifterPosition == RobotMap.SHIFTER_HIGH_GEAR)
		{
			DriverStation.reportError("Shift to HIGH gear", false);
		}
		else
		{
			DriverStation.reportError("Shift to Low gear", false);
		}
	}
	
	public void ZeroLeftDriveAxisEncoderPosition()
	{
		_leftDriveMasterMtr.setPosition(0.0);
	}
	
	//public void setLeftMasterMtrEncoderPosition(double position)
	//{
	//	_leftDriveMasterMtr.setPosition(position);
	//}
	
	public void ZeroRightDriveAxisEncoderPosition()
	{
		_rightDriveMasterMtr.setPosition(0.0);
	}
	
	//public void setRightMasterMtrEncoderPosition(double position)
	//{
	//	_rightDriveMasterMtr.setPosition(position);
	//}
	
	//============================================================================================
	// Property Accessors follow (properties only change internal state) (ie: DO NOT push changes to the robot hardware)
	//============================================================================================
		
	public void setDriveSpeedScalingFactor(double speedScalingFactor)
	{
		// for safety, clamp the scaling factor to max of +1, -1
		if (speedScalingFactor > 1.0){
			speedScalingFactor = 1.0;
		}
		else if (speedScalingFactor < -1.0){
			speedScalingFactor = -1.0;
		}
		
		_currentDriveSpeedScalingFactorClamped = speedScalingFactor;
		
		// 2017 season std: when we chg a modal condition, write to dashboard
		String logMsg = String.format("Drive speed scaling set to %f", _currentDriveSpeedScalingFactorClamped);
		DriverStation.reportError(logMsg, false);
	}
		
	public double getCurrentArcadeDriveThrottleCmd()
	{
		return _currentArcadeDriveThrottleCmd;
	}
	
	public double getCurrentArcadeDriveTurnCmd()
	{
		return _currentArcadeDriveTurnCmd;
	}
	
	public double getCurrentLeftDriveAxisEncoderPosition()
	{
		return _leftDriveMasterMtr.getPosition();
	}
	
	public double getCurrentRightDriveAxisEncoderPosition()
	{
		return _rightDriveMasterMtr.getPosition();
	}
	
	public double getCurrentSpeedScalingFactor()
	{
		return _currentDriveSpeedScalingFactorClamped;
	}
	
	public Value getCurrentGear()
	{
		return _currentShifterPosition;
	}
	
	public boolean getIsAccDecModeEnabled()
	{
		return _isAccDecModeEnabled;
	}
	
	public void setAccDecModeEnabled(boolean isModeEnabled)
	{
		_isAccDecModeEnabled = isModeEnabled;
		
		// 2017 season std: when we chg a modal condition, write to dashboard
		if(_isAccDecModeEnabled)
		{
			DriverStation.reportError("Acc/Dec Mode ENABLED", false);
		}
		else
		{
			DriverStation.reportError("Acc/Dec Mode DISABLED", false);
		}
	}
}
