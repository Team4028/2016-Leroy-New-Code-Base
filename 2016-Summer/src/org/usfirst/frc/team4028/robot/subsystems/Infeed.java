package org.usfirst.frc.team4028.robot.subsystems;

import java.util.Date;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;

import org.usfirst.frc.team4028.robot.Utilities;

/**
 * This class defines the behavior of the Infeed subsystem
 * 	This subsystem has two (2) parts
 * 		Tilt		>	Motor + Encoder + L/S
 * 		Acquisition	>	Motor
 */
public class Infeed 
{
	
	private CANTalon _infeedTiltMtr;
	
	private VictorSP _infeedAcqMtr;
	private DigitalInput _ballInPositionLimitSwitch;
	
	private boolean _isInfeedTiltAxisZeroedYet = false;			// has this axis already been zeroed? optimization used in auton -> telop state transition
	private boolean _isInfeedTiltDeployStillRunning = false;	// "state" variable used to complete long running actions across multiple scan cycles
	private boolean _isInfeedTiltChevalStillRunning = false;		// "state" variable used to complete long running actions across multiple scan cycles
	private boolean _isInfeedTiltZeroStillRunning = false;		// "state" variable used to complete long running actions across multiple scan cycles
	private boolean _isInfeedTiltShootStillRunning = false;
	private boolean _isInfeedTiltCrossDefenseStillRunning = false;
	
		
	// enum to support Infeed Tilt Homing State Machine
	public enum INFEED_TILT_HOMING_STATE
	{
		UNDEFINED,
		ZERO_INIT,
		MOVING_TO_HOME,
		AT_HOME,
		MOVING_TO_STORED_POSITION,
		AT_STORED_POSITION,
		TIMEOUT
	}
	
	private double _infeedTiltTargetPosition;
	private double _infeedAcqTargetVelocity;
	
	private long _infeedTiltAxisZeroStateStartTime;
	private boolean _isInfeedTiltAxisZeroTimedOut;
	private double _infeedTiltAxisEncoderPositionLastScanCycle = 0;
	private int _infeedTiltAxisNotMovingCycleCounter = 0;
	
	private double _infeedTiltAxisZeroPreviousCycleCmd;
	
	private INFEED_TILT_HOMING_STATE _infeedTiltAxisZeroCurrentState;
	private INFEED_TILT_HOMING_STATE _infeedTiltAxisZeroPreviousState;
	
	private static final double MOVE_TO_HOME_VELOCITY_CMD = 0.25;
	private static final long MAX_MOVE_TO_HOME_TIME_IN_MSECS = 8000;
	private static final long MAX_MOVE_TO_STORED_TIME_IN_MSECS = 5000;

	private static final double INFEED_TILT_AXIS_HOME_POSITION_IN_ROTATIONS = 0.0;
	public static final double INFEED_TILT_AXIS_CROSS_DEFENSE_POSITION_IN_ROTATIONS = -0.122;			// this is approx 90 deg
	public static final double INFEED_TILT_AXIS_SHOOTING_POSITION_IN_ROTATIONS = -0.407;
	public static final double INFEED_TILT_AXIS_DEPLOYED_POSITION_IN_ROTATIONS = -0.505;		// this is approx 0 deg
	public static final double INFEED_TILT_AXIS_CHEVAL_POSITION_IN_ROTATIONS = -0.642;			// this is approx ?? deg
	private static final double INFEED_TILT_AXIS_DOWN_SOFT_LIMIT_IN_ROTATIONS = -0.670;
	
	private static final double INFEED_TILT_AXIS_IN_POSITION_THRESHHOLD_IN_ROTATIONS = 0.01;
	private static final double INFEED_TILT_AXIS_VELOCITY_CMD_DEADBAND = 0.1;
	private static final double INFEED_TILT_AXIS_ANALOG_MODE_MOVE_INCREMENT = 0.01;
	private static final int INFEED_TILT_AXIS_ENCODER_COUNTS_PER_REV = 1024;	
	
	private static final double INFEED_TILT_AXIS_NOT_MOVING_BUMP_FACTOR = 0.01;
	
	private static final double INFEED_TILT_KP = 8.0; //0.4;
	private static final double INFEED_TILT_KI = 0.0;
	private static final double INFEED_TILT_KD = 0.0;
	private static final double INFEED_TILT_KF = 0.0;
	private static final int INFEED_TILT_IZONE = 0;
	private static final double INFEED_TILT_RAMPRATE = 1;
	private static final int INFEED_TILT_PROFILE = 0;
	private static final double INFEED_FORWARD_VOLTAGE = 12.0f;
	private static final double INFEED_REVERSE_VOLTAGE = -4.0f;
	
	private static final double INFEED_HOME_TARGET_VELOCITY = 6.0;
	private static final double INFEED_HOME_TARGET_VELOCITY_DEADBAND = 0.1; //0.5
	private static final double INFEED_HOME_TARGET_BASE_CMD = 0.2;
	private static final double INFEED_HOME_TARGET_BUMP_UP_CMD = 0.009; //0.0125; //0.025;
	private static final double INFEED_HOME_TARGET_BUMP_DOWN_CMD = 0.004; //0.00625; //0.0125; //0.025;
	private static final double INFEED_HOME_TARGET_BUMP_DOWN_LARGE_CMD = 0.009; //0.0125;
	
	
	
	// ===================================
	// Constructors follow
	// ===================================
	public Infeed(int tiltMtrTalonCanBusAdddress, int acqVictorPWMPort, int ballInPositionSwitchDIOPort)
	{
    	_infeedTiltMtr = new CANTalon(tiltMtrTalonCanBusAdddress);
    	_infeedTiltMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	//_infeedTiltMtr.configEncoderCodesPerRev(INFEED_TILT_AXIS_ENCODER_COUNTS_PER_REV);	// try to enable Unit Scaling
    	_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// we start in % vbus mode until we zero then we swap to position mode
    	_infeedTiltMtr.enableBrakeMode(false);
    	_infeedTiltMtr.enableLimitSwitch(true, false);		// we are using the REV limit switch
    	_infeedTiltMtr.reverseSensor(true);
    	_infeedTiltMtr.ConfigFwdLimitSwitchNormallyOpen(false);
    	
		_infeedAcqMtr = new VictorSP(acqVictorPWMPort);
		
		_ballInPositionLimitSwitch = new DigitalInput(ballInPositionSwitchDIOPort);
	}
	
	//============================================================================================
	// Methods follow (methods make the robot do something (ie: push changes to the robot hardware)
	//============================================================================================
	
	// ===================
	// TILT Motor Methods
	// ===================
	public void ZeroInfeedTiltAxisReentrant()
	{
		// only do when this method is initially called
		if(!_isInfeedTiltZeroStillRunning)
		{
			_infeedTiltAxisZeroStateStartTime = System.currentTimeMillis();
			_isInfeedTiltAxisZeroTimedOut = false;
			
			// determine initial state
			if(this.getIsTiltAxisAtHomePosition())
			{
				_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.AT_HOME;
			}
			else
			{
				_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.ZERO_INIT;				
			}
				
			_infeedTiltAxisEncoderPositionLastScanCycle = 0.0;
			_infeedTiltAxisNotMovingCycleCounter = 0;
			
			DriverStation.reportError("Zero Infeed Tilt Axis", false);
		}
		
		_isInfeedTiltZeroStillRunning = true;
		
		/// implement state machine
		switch (_infeedTiltAxisZeroCurrentState)
		{
			case ZERO_INIT:
				_infeedTiltAxisZeroPreviousCycleCmd = INFEED_HOME_TARGET_BASE_CMD;
				_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.MOVING_TO_HOME;
				break;
				
			case MOVING_TO_HOME:
				if(!this.getIsTiltAxisAtHomePosition())
				{
    		    	long elapsedTimeMSecs = (new Date().getTime() - _infeedTiltAxisZeroStateStartTime);
    		    	
    		    	
    		    	// if we are not on the limit switch, drive up until we hit it but only wait for a max time
    		    	if (elapsedTimeMSecs < MAX_MOVE_TO_HOME_TIME_IN_MSECS)	   
    		    	{
    		    		// if reqd, chg to %VBUS mode
    		    		if (_infeedTiltMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus)
    		    		{
    		    			_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		    		}
    		    		
    		    		double infeedAxisCurrentVelocityCmd = _infeedTiltAxisZeroPreviousCycleCmd;
    		    		
    		    		// Assumption: INFEED_HOME_TARGET_VELOCITY >= 0
    		    		double infeedTiltAxisActualTargetDelta = _infeedTiltMtr.getSpeed() - INFEED_HOME_TARGET_VELOCITY;
    		    		
    		    		if(infeedTiltAxisActualTargetDelta > INFEED_HOME_TARGET_VELOCITY)
    		    		{
    		    			// More than 2x the target velocity
    		    			infeedAxisCurrentVelocityCmd = Utilities.RoundDouble((infeedAxisCurrentVelocityCmd - INFEED_HOME_TARGET_BUMP_DOWN_LARGE_CMD), 3);
    		    			//DriverStation.reportError("BUMPING DOWN LARGE   MOTOR COMMAND: " + Double.toString(infeedAxisCurrentVelocityCmd)
    		    			//+ "      VELOCITY: " + Double.toString(_infeedTiltMtr.getSpeed()), false);
    		    		}
    		    		else if(infeedTiltAxisActualTargetDelta > INFEED_HOME_TARGET_VELOCITY_DEADBAND)
    		    		{    		    			
    		    			// Going faster than the target velocity + deadband
    		    			infeedAxisCurrentVelocityCmd = Utilities.RoundDouble((infeedAxisCurrentVelocityCmd - INFEED_HOME_TARGET_BUMP_DOWN_CMD), 3);
    		    			//DriverStation.reportError("BUMPING DOWN    MOTOR COMMAND: " + Double.toString(infeedAxisCurrentVelocityCmd)
    		    			//+ "      VELOCITY: " + Double.toString(_infeedTiltMtr.getSpeed()), false);
    		    		}
    		    		else if(infeedTiltAxisActualTargetDelta < (-1.0 * INFEED_HOME_TARGET_VELOCITY_DEADBAND))
		    			{
    		    			// Going slower than target velocity - deadband
    		    			infeedAxisCurrentVelocityCmd = Utilities.RoundDouble((infeedAxisCurrentVelocityCmd + INFEED_HOME_TARGET_BUMP_UP_CMD), 3);
    		    			//DriverStation.reportError("BUMPING UP-----MOTOR COMMAND: " + Double.toString(infeedAxisCurrentVelocityCmd)
    		    			//+ "      VELOCITY: " + Double.toString(_infeedTiltMtr.getSpeed()), false);
		    			}
    		    		else
    		    		{
    		    			// Going within the deadband for the target velocity
    		    			//DriverStation.reportError("------------NO CHANGE---------", false);
    		    		}
		    					
    		    		// drive the axis "up" towards the home location
    		    		_infeedTiltMtr.set(infeedAxisCurrentVelocityCmd);	
    		    		
    		    		_infeedTiltAxisZeroPreviousCycleCmd = infeedAxisCurrentVelocityCmd;
    		    		// snapshot the position in this scan so we can tell if it moved by the next scan
    		    		//_infeedTiltAxisEncoderPositionLastScanCycle = infeedTiltAxisEncoderPositionThisScanCycle;
    		    	}
    		    	else
    		    	{
    					_infeedTiltAxisZeroPreviousState = _infeedTiltAxisZeroCurrentState;
    		    		_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.TIMEOUT;
    		    	}					
				}
				else
				{
					_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.AT_HOME;
				}
				
				break;
				
			case AT_HOME:
				// immediately stop motor, set to Position Mode
				if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
				{
	    			_infeedTiltMtr.set(0.0);
	    			_infeedTiltMtr.configPeakOutputVoltage(INFEED_FORWARD_VOLTAGE, INFEED_REVERSE_VOLTAGE);
	    			// setup the PID Loop
	    			_infeedTiltMtr.setPID(INFEED_TILT_KP, INFEED_TILT_KI, INFEED_TILT_KD, INFEED_TILT_KF, INFEED_TILT_IZONE, INFEED_TILT_RAMPRATE, INFEED_TILT_PROFILE);
	    	    	_infeedTiltMtr.setProfile(INFEED_TILT_PROFILE);
	    			_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.Position);
				}
				
    	    	// once we hit the home switch, reset the encoder	- this is at approx ??? degrees
    	    	_infeedTiltMtr.setPosition(INFEED_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);	

    	    	try {
    	    		// sleep 1 mSec to let the zero occur
    				Thread.sleep(1);
    			} catch (InterruptedException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			}
    	    

    	    	//_infeedTiltMtr.setCloseLoopRampRate(INFEED_TILT_RAMPRATE);
    	    	//_infeedTiltMtr.setVoltageRampRate(5);
    	    	
    	    	// write to the operator's console log window
    	    	DriverStation.reportError("..Infeed Tilt At Home, Chg to PID Position Ctrl Mode.", false);
    	    	
    	    	_infeedTiltAxisZeroStateStartTime = System.currentTimeMillis();
    	    	_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.MOVING_TO_STORED_POSITION;
				break;
				
			case MOVING_TO_STORED_POSITION:
				// set new target position
				_infeedTiltMtr.set(INFEED_TILT_AXIS_CROSS_DEFENSE_POSITION_IN_ROTATIONS);
				
				// are we NOT within the deadband of the target position?
				int currentTargetClosedLoopError = _infeedTiltMtr.getClosedLoopError();
				
				if(currentTargetClosedLoopError > INFEED_TILT_AXIS_IN_POSITION_THRESHHOLD_IN_ROTATIONS)
				{
					long elapsedTimeMSecs = (new Date().getTime() - _infeedTiltAxisZeroStateStartTime);
					
					// have we taken too long to get there?
			    	if (elapsedTimeMSecs  >= MAX_MOVE_TO_STORED_TIME_IN_MSECS)	   
			    	{
						_infeedTiltAxisZeroPreviousState = _infeedTiltAxisZeroCurrentState;
			    		_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.TIMEOUT;
			    	}
				}	
				else
				{
					_infeedTiltAxisZeroCurrentState = INFEED_TILT_HOMING_STATE.AT_STORED_POSITION;	
				}

				break;
				
			case AT_STORED_POSITION:
    	    	// finally mark the axis as zeroed
    	    	_isInfeedTiltAxisZeroedYet = true;
    	    	_isInfeedTiltZeroStillRunning = false;
    	    	
				DriverStation.reportError("..Infeed Tilt At Stored Location, Homing cycle complete.", false);
				
				break;
				
			case TIMEOUT:
    			if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    			{
    				_infeedTiltMtr.set(0.0);
    			}
    			_isInfeedTiltZeroStillRunning = false;
    			DriverStation.reportError("Error....Infeed Tilt Axis Zero timed out while in State: " + _infeedTiltAxisZeroPreviousState.toString(), false);
    			
				break;
		}
	}
	
	public boolean MoveInfeedTiltAxisToDeployedPositionReentrant()
	{
		if(!_isInfeedTiltDeployStillRunning)
		{
			// only do when this method is initially called
			DriverStation.reportError("Deploy Infeed Tilt Axis", false);
		}		
		
		_infeedTiltTargetPosition = INFEED_TILT_AXIS_DEPLOYED_POSITION_IN_ROTATIONS;
		_infeedTiltMtr.set(_infeedTiltTargetPosition);
		
		if(Math.abs(_infeedTiltMtr.getPosition()) - Math.abs(_infeedTiltTargetPosition) > INFEED_TILT_AXIS_IN_POSITION_THRESHHOLD_IN_ROTATIONS)
		{
			_isInfeedTiltDeployStillRunning = true;
			
			return false;
		}
		else
		{
			_isInfeedTiltDeployStillRunning = false;
			DriverStation.reportError("Infeed Tilt Axis at Deployed Position", false);
			
			return true;
		}
	}
	
	public boolean MoveInfeedTiltAxisToChevalPositionReentrant()
	{
		if(!_isInfeedTiltChevalStillRunning)
		{
			// only do when this method is initially called
			DriverStation.reportError("Move Infeed Tilt to Cheval", false);
		}		
		
		_infeedTiltTargetPosition = INFEED_TILT_AXIS_CHEVAL_POSITION_IN_ROTATIONS;
		_infeedTiltMtr.set(_infeedTiltTargetPosition);
		
		if(Math.abs(_infeedTiltMtr.getPosition()) - Math.abs(_infeedTiltTargetPosition) > INFEED_TILT_AXIS_IN_POSITION_THRESHHOLD_IN_ROTATIONS)
		{
			_isInfeedTiltChevalStillRunning = true;
			
			return false;
		}
		else
		{
			_isInfeedTiltChevalStillRunning = false;
			DriverStation.reportError("Infeed Tilt Axis at Cheval Position", false);
			
			return true;
		}
	}
	
	public boolean MoveInfeedTiltAxisToShootingPositionReentrant()
	{
		if(!_isInfeedTiltShootStillRunning)
		{
			// only do when this method is initially called
			DriverStation.reportError("Shoot Infeed Tilt Axis", false);
		}		
		
		_infeedTiltTargetPosition = INFEED_TILT_AXIS_SHOOTING_POSITION_IN_ROTATIONS;
		_infeedTiltMtr.set(_infeedTiltTargetPosition);
		
		if(Math.abs(_infeedTiltMtr.getPosition()) - Math.abs(_infeedTiltTargetPosition) > INFEED_TILT_AXIS_IN_POSITION_THRESHHOLD_IN_ROTATIONS)
		{
			_isInfeedTiltShootStillRunning = true;
			
			return false;
		}
		else
		{
			_isInfeedTiltShootStillRunning = false;
			DriverStation.reportError("Infeed Tilt Axis at Shooting Position", false);
			
			return true;
		}
	}
	
	public boolean MoveInfeedTiltAxisToCrossDefensePositionReentrant()
	{
		if(!_isInfeedTiltCrossDefenseStillRunning)
		{
			// only do when this method is initially called
			DriverStation.reportError("Shoot Infeed Tilt Axis", false);
		}		
		
		_infeedTiltTargetPosition = INFEED_TILT_AXIS_CROSS_DEFENSE_POSITION_IN_ROTATIONS;
		_infeedTiltMtr.set(_infeedTiltTargetPosition);
		
		if(Math.abs(_infeedTiltMtr.getPosition()) - Math.abs(_infeedTiltTargetPosition) > INFEED_TILT_AXIS_IN_POSITION_THRESHHOLD_IN_ROTATIONS)
		{
			_isInfeedTiltCrossDefenseStillRunning = true;
			
			return false;
		}
		else
		{
			_isInfeedTiltCrossDefenseStillRunning = false;
			DriverStation.reportError("Infeed Tilt Axis at Cross Defense Position", false);
			
			return true;
		}
	}
	
	public void RunInfeedTiltMotor(Double newVelocityCmdRaw)
	{
		/*
		 *	To enable better ability to hold desired position, we keep the axis in position control mode
		 *		and just move the target position (at little each scan) ahead or behind of the current 
		 *		position to cause the axis to move
		 */
		
		_infeedTiltTargetPosition = _infeedTiltMtr.getPosition();
		String debugMsg = "";
		
		if(_isInfeedTiltAxisZeroedYet)
		{
			if(_infeedTiltTargetPosition <= INFEED_TILT_AXIS_DOWN_SOFT_LIMIT_IN_ROTATIONS)
			{
				if(newVelocityCmdRaw > INFEED_TILT_AXIS_VELOCITY_CMD_DEADBAND)
				{
					_infeedTiltTargetPosition += INFEED_TILT_AXIS_ANALOG_MODE_MOVE_INCREMENT;
				}
				else if (newVelocityCmdRaw < (-1.0 *INFEED_TILT_AXIS_VELOCITY_CMD_DEADBAND))
				{
					_infeedTiltTargetPosition -= INFEED_TILT_AXIS_ANALOG_MODE_MOVE_INCREMENT;
				}
				
				//_infeedTiltMtr.setPosition(_infeedTiltTargetPosition);
				debugMsg = String.format("Infeed Tilt Motor Velocity Cmd: [%f]", newVelocityCmdRaw);
			}
			else
			{
				debugMsg = "Infeed Tilt Axis at Down Soft Limit!";
			}
		}
		else
		{
			debugMsg = "RunInfeedTiltMotor cmd is NOT available until after the Axis has been zeroed!";
		}

		DriverStation.reportError(debugMsg, false);
	}
	
	// ===================
	// ACQUIRE Motor Methods
	// ===================
	public void StopInfeedAquireMotor()
	{
		this.RunInfeedAcquireMotor(0.0);
	}
	
	public void RunInfeedAcquireMotorIn()
	{
		this.RunInfeedAcquireMotor(-1.0);
	}
	
	public void RunInfeedAcquireMotorOut()
	{
		this.RunInfeedAcquireMotor(1.0);
	}
	
	public void RunInfeedAcquireMotor(double newVelocityCmdRaw)
	{
		_infeedAcqTargetVelocity = newVelocityCmdRaw;
		
		_infeedAcqMtr.set(_infeedAcqTargetVelocity);
	}
	
	//============================================================================================
	// Property Accessors follow (properties only change internal state) (ie: DO NOT push changes to the robot hardware)
	//============================================================================================
	
	//=======
	// Tilt
	//=======
	public boolean IsInfeedTiltAxisZeroedYet() 
	{
		return _isInfeedTiltAxisZeroedYet;
	}
	
	public boolean getIsInfeedTiltZeroStillRunning()
	{
		return _isInfeedTiltZeroStillRunning;	
	}
	
	public boolean getIsInfeedTiltDeployStillRunning()
	{
		return _isInfeedTiltDeployStillRunning;
	}
	
	public boolean getIsInfeedTiltShootStillRunning()
	{
		return _isInfeedTiltShootStillRunning;
	}
	
	public boolean getIsInfeedTiltCrossDefenseStillRunning()
	{
		return _isInfeedTiltCrossDefenseStillRunning;
	}
	
	public boolean getIsInfeedTiltChevalStillRunning()
	{
		return _isInfeedTiltChevalStillRunning;
	}
		
	public boolean getIsTiltAxisAtHomePosition()
	{
		// note: we are reading real time from the DIO port
		//		  switch is normally closed, so invert signal
		return !_infeedTiltMtr.isFwdLimitSwitchClosed();
	}
	
	public double getInfeedTiltTargetPosition()
	{
		return _infeedTiltTargetPosition;
	}
	
	//=======
	// Acq
	//=======
	public double getInfeedAcqTargetVelocity()
	{
		return _infeedAcqTargetVelocity;
	}
	
	public boolean getIsBallInPosition()
	{
		// note: we are reading real time from the DIO port
		//		DIO input is pulled up internally, we use NC (Normally Closed) switch 
		//		Ball NOT on Switch 	=> NC 	=> pulls input low 				=> false
		//		Ball ON Switch		=> Open	=> input pulled internally high	=>	true
		return _ballInPositionLimitSwitch.get();
	}
}
