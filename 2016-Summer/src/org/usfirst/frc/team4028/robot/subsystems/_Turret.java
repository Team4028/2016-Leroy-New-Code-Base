package org.usfirst.frc.team4028.robot.subsystems;

import java.util.Date;

import org.usfirst.frc.team4028.robot.constants.LogitechF310;
//import org.usfirst.frc.team4028.robot.interfaces.IPositionTargetAxis;
//import org.usfirst.frc.team4028.robot.interfaces.IRWRobotIO;
//import org.usfirst.frc.team4028.robot.interfaces.IZeroableAxis;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;

/**
 * This class defines the behavior of the Turret Subsystem
 */
public class _Turret //implements IRWRobotIO, IZeroableAxis, IPositionTargetAxis
{
	// private Robot physical objects
	private CANTalon _turretMtr;
	private DigitalInput _turretHomeLimitSwitch;
	private DigitalInput _turretApproachingHomeLimitSwitch;
	
	private enum Turret_Zero_State
	{
		UNDEFINED,
		BEFORE_APPROACHING_SWITCH,
		ON_APPROACHING_SWITCH,
		BEFORE_HOME_SWITCH,
		ON_HOME_SWITCH,
		TIMEOUT
	}
	
	private enum Target_Auton_Position_State
	{
		UNDEFINED,
		GROSS_TURN_TO_TARGET,
		COARSE_TURN_TO_TARGET,
		FINE_TURN_TO_TARGET,
		IN_POSITION
	}
	
	// private state variables
	private TurretStateValues _currentScanValues;
	private TurretStateValues _previousScanValues;
	private boolean _isAxisZeroedYet = false;
	private Turret_Zero_State _turretZeroState = Turret_Zero_State.UNDEFINED;
	private Target_Auton_Position_State _tagetPositionState = Target_Auton_Position_State.UNDEFINED;
	private long _turretZeroStartTime = 0;
	private double _targetPositionCmd = 0;
	private double _targetVelocityCmd = 0;
	private double _targetPosition = 0;
	private CANTalon.TalonControlMode _targetControlMode;
	
	private static final double IN_POSITION_DEADBAND = 50;	
	
	/**
	 * Create a new instance of the Turret and setup default configuration
	 * 
	 * @param talonCanBusAdddress
	 * @param turretApproachingHomeLimitSwitchDIOPort
	 * @param turretHomeLimitSwitchDIOPort
	 */
	public _Turret(int talonCanBusAdddress, int turretApproachingHomeLimitSwitchDIOPort, int turretHomeLimitSwitchDIOPort)
	{
    	_turretMtr = new CANTalon(talonCanBusAdddress);
    	_turretMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	_turretMtr.reverseSensor(false);
    	_turretMtr.enableBrakeMode(true);
    	_turretMtr.enableLimitSwitch(false, false);
    	//_turretMtr.configNominalOutputVoltage(+0.0f, -0.0f);
    	//_turretMtr.configPeakOutputVoltage(+5.0f, -5.0f);
    	
    	_turretApproachingHomeLimitSwitch = new DigitalInput(turretApproachingHomeLimitSwitchDIOPort);
    	_turretHomeLimitSwitch = new DigitalInput(turretHomeLimitSwitchDIOPort);
    	
    	_targetControlMode = _turretMtr.getControlMode();
	}
	
	// ================================================================================================
	// Public Methods 
	// ================================================================================================
	public void ReadInputs()
	{
		if (_currentScanValues != null)
		{
			// clone the old _currentScanValues as the new _previousScanValues
			_previousScanValues = new TurretStateValues(_currentScanValues);
		}
		
		// read the inputs
		_currentScanValues = new TurretStateValues(_turretMtr, _turretHomeLimitSwitch, _turretApproachingHomeLimitSwitch);
		
		
		if(_previousScanValues == null)
		{
			// if we do not previous values, use the current ones
			_previousScanValues = new TurretStateValues(_currentScanValues);			
		}
	}
	
	/**
	 * Short method to read input and return current values
	 * 	(this method cannot be part of IRWRobotIO since it has a specific return type)
	 * @return
	 */
	public TurretStateValues ReadAndReturnInputs()
	{
		this.ReadInputs();
		return CurrentScanValues();
	}
		
	/**
	 * Output the new commands to the physical robot objects
	 */
	public void PushOutputs()
	{
		// chg the control mode if necessary
		if (_targetControlMode != _currentScanValues.TalonControlMode())
		{
			_turretMtr.changeControlMode(_targetControlMode);
		}
		
    	if (_targetControlMode == CANTalon.TalonControlMode.Position)
    	{
    		_turretMtr.set(_targetPositionCmd);
    	}
    	else if (_targetControlMode == CANTalon.TalonControlMode.PercentVbus)
    	{
    		_turretMtr.set(_targetVelocityCmd);
    	}
	}
	
	// ================================================================================================
	// property Accessors 
	// ================================================================================================
	
	//@Override
	public boolean IsAxisZeroedYet()
	{
		return _isAxisZeroedYet;
	}
	
	public TurretStateValues CurrentScanValues()
	{
		return _currentScanValues;
	}
	
	public TurretStateValues PreviousScanValues()
	{
		return _previousScanValues;
	}
	
	//@Override
	public boolean IsAxisAtTarget() 
	{
		if(Math.abs(_currentScanValues.EncoderPosition() - _targetPosition) < IN_POSITION_DEADBAND)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// ================================================================================================
	// Target Position Methods
	// ================================================================================================
	
	/**
	 * Moves the axis to the target position
	 * @param targetPosition
	 * @return True if the axis is at the target position, else false.
	 */
	//@Override
	public boolean GotoTargetPositionReentrant(double targetPosition) 
	{
		_targetPosition = targetPosition;
		
		if(this.IsAxisAtTarget())
		{
			return true;
		}
		else
		{
		// TODO: implement tbis method
			
			return false;
		}
	}

	// ================================================================================================
	// Axis Zeroing Methods
	// ================================================================================================
	//@Override
	public void InitZeroAxisReentrant()
	{
		_turretZeroStartTime = System.currentTimeMillis();
		_turretZeroState = Turret_Zero_State.BEFORE_APPROACHING_SWITCH;
		
		DriverStation.reportError("..Chg to Turret Zero State[0]: BEFORE_APPROACHING_SWITCH.", false);
	}
	
	/**
	 * Re-enterable Axis Homing Return that supports incremental %VBus command bumping if axis is not moving (ex Lower Battery Voltage)
	 */
	//@Override
	public void ZeroAxisReentrant()
	{
    	final long maxTimeInMSec = 8000; // 8 secs
		
		// refresh current state
    	TurretStateValues currentScanValues = ReadAndReturnInputs();
    	TurretStateValues previousScanValues = this.PreviousScanValues();
    	
    	switch (_turretZeroState)
    	{
    		case BEFORE_APPROACHING_SWITCH:
    	    	// start out in %VBUS mode
    			if(currentScanValues.TalonControlMode() != CANTalon.TalonControlMode.PercentVbus )
    			{
    				// chg to control mode and continue in the eext loop
    				_targetControlMode = CANTalon.TalonControlMode.PercentVbus;
    				break;
    			}
    			    
    			// make sure we are not alrady on one of the limit switches
    			if(!currentScanValues.IsOnHomeLimitSwitch() && !currentScanValues.IsOnApproachingHomeLimitSwitch())
    	    	{	    	
    		    	// if we are not on the limit switch, drive up until we hit it but only wait for 5 secs max
    		    	long elapsedTime = (new Date().getTime() - _turretZeroStartTime);
		    		
		    		if (elapsedTime  >= maxTimeInMSec)
		    		{
		    			_turretZeroState = Turret_Zero_State.TIMEOUT;
		    			
		    			DriverStation.reportError("..Chg to Turret Zero State[-1]: TIMEOUT.", false);
		    			break;
		    		}
		    		
			    	// drive the axis at 15%
			    	_targetVelocityCmd = 0.15;
			    	_turretMtr.set(_targetVelocityCmd);
    	    	}
    			else if(currentScanValues.IsOnApproachingHomeLimitSwitch())
    			{
    				_turretZeroState = Turret_Zero_State.ON_APPROACHING_SWITCH;
    				
    				DriverStation.reportError("..Chg to Turret Zero State[1]: ON_APPROACHING_SWITCH", false);
    			}
    			else if(currentScanValues.IsOnHomeLimitSwitch())
    			{
    				_turretZeroState = Turret_Zero_State.ON_HOME_SWITCH;
    				
    				DriverStation.reportError("..Chg to Turret Zero State[4]: ON_HOME_SWITCH", false);    				
    			}
    			break;
    		
    		case ON_APPROACHING_SWITCH:
    			_turretZeroStartTime = System.currentTimeMillis();
    			_turretZeroState = Turret_Zero_State.BEFORE_HOME_SWITCH;
    			
    			DriverStation.reportError("Turret Approaching Switch hit", false);
    			break;
    			
    		case BEFORE_HOME_SWITCH:
    	    	long elapsedTime = 0L;
    	    	//long maxTimeInMSec = 5000; // 5 secs
    	    	
    	    	// if we are not on the limit switch, drive up until we hit it but only wait for 5 secs max
    	    	if(!currentScanValues.IsOnHomeLimitSwitch())
    	    	{
    	        	elapsedTime = (new Date().getTime() - _turretZeroStartTime);
    	    		if (elapsedTime  >= maxTimeInMSec)
    	    		{
    	    			_turretZeroState = Turret_Zero_State.TIMEOUT;
    	    		}

    	    	}
    	    	else
    	    	{
    	    		_turretZeroState = Turret_Zero_State.ON_HOME_SWITCH;
    	    		DriverStation.reportError("Turret Zero State: ON_HOME_SWITCH", false);
    	    	}
    			break;
    			
    		case ON_HOME_SWITCH:
    			// stop driving the axis
    			_targetVelocityCmd = 0;
    	    	_turretMtr.set(_targetVelocityCmd);
        		
    	    	// now switch to position loop mode
    	    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.Position);
    	    	
    	    	// once we hit it reset the encoder
    	    	double homePosition = 0;
    	    	_turretMtr.setPosition(homePosition);	

    	    	try {
    	    		// sleep a little to let the zero occur
    				Thread.sleep(1);
    			} catch (InterruptedException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			}
    	    	
    	    	// setup the PID Loop
    	    	//_turretMtr.setPID(RobotMap.TURRET_SLOW_KP, RobotMap.TURRET_SLOW_KI, RobotMap.TURRET_SLOW_KD, RobotMap.TURRET_SLOW_KF, RobotMap.TURRET_SLOW_IZONE, RobotMap.TURRET_SLOW_RAMPRATE, RobotMap.TURRET_SLOW_PROFILE);
    	    	//_turretMtr.setProfile(RobotMap.TURRET_SLOW_PROFILE);
    	    	DriverStation.reportError("Turret set to Slow Profile", false);
    	    	
    	    	// write to the operator's console log window
    	    	DriverStation.reportError("..Turret Axis Zeroed, Chg to Positon Ctrl Mode.", false);
    	    	
    	    	// finally mark the axis as zeroed
    	    	_isAxisZeroedYet = true;
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportError("Turret Axis Zero timed out", false);
    			_targetVelocityCmd = 0.0;
		    	_turretMtr.set(_targetVelocityCmd);
    			break;
    	}	
    	
    	PushOutputs();
	}
	

	/**
	 * Immutable class to hold state data
	 */
	public final class TurretStateValues
	{
		private boolean _isHomeLimitSwitchClosed = false;
		private boolean _isApproachingHomeLimitSwitchClosed = false;
		private double _encoderPosition = 0;
		private CANTalon.TalonControlMode _controlNode;
		
		protected TurretStateValues(CANTalon turretMtr, DigitalInput turretHomeLimitSwitch, DigitalInput turretApproachingHomeLimitSwitch)
		{
	    	// get values from Limit Switches on DIO Ports
			_isHomeLimitSwitchClosed = _turretHomeLimitSwitch.get();
			_isApproachingHomeLimitSwitchClosed = _turretApproachingHomeLimitSwitch.get();
			
	    	// get values from motor controllers
			_encoderPosition = _turretMtr.getPosition();
			_controlNode = _turretMtr.getControlMode();
		}
		
		/**
		 * C;one constructor
		 * @param turretValues
		 */
		protected TurretStateValues(TurretStateValues turretStateValues)
		{
			_isHomeLimitSwitchClosed = turretStateValues.IsOnHomeLimitSwitch();
			_isApproachingHomeLimitSwitchClosed = turretStateValues.IsOnApproachingHomeLimitSwitch();
			_encoderPosition = turretStateValues.EncoderPosition();
			_controlNode = turretStateValues.TalonControlMode();
		}
		
		public boolean IsOnHomeLimitSwitch()
		{
			return _isHomeLimitSwitchClosed;
		}
		
		public boolean IsOnApproachingHomeLimitSwitch()
		{
			return _isApproachingHomeLimitSwitchClosed;
		}
		
		public double EncoderPosition()
		{
			return _encoderPosition;
		}
		
		public CANTalon.TalonControlMode TalonControlMode()
		{
			return _controlNode;
		}
	}






}
