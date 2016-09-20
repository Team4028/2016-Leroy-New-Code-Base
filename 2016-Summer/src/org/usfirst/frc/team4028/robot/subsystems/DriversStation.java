package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.AutonConstants;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Mode;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Cross_Defense_Type;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Drive_Throttle_Percent;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Drive_Time_In_Secs;
import org.usfirst.frc.team4028.robot.constants.AutonConstants.Auton_Pumas_Position;
import org.usfirst.frc.team4028.robot.constants.LogitechF310;
import org.usfirst.frc.team4028.robot.constants.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class defines the behavior of the Driver's Station
 * 	it encapsulates all interaction with the GamePads, the Dashboard and the Auton Sendable Choosers
 * 
 * Internally it keeps track of the current and previous scan values for all gamepads
 * 	but that is completely transparent to users of this class
 */
public class DriversStation 
{	
	// Driver & Operator station gamepads
	private Joystick _driverGamepad;
	private Joystick _operatorGamepad;
	
	private DriversStationInputs _currentValues;
	private DriversStationInputs _previousValues;
	
	// Smart Dashboard choosers
	private SendableChooser _autonModeChooser;
	private SendableChooser _autonPumasPositionChooser;
	private SendableChooser _autonDriveTimeChooser;
	private SendableChooser _autonDriveThrottleChooser;
	private SendableChooser _autonCrossDefenseTypeChooser;

	private final double INFEED_TILT_TRIGGER_THRESHHOLD = 0.05;
	private final double INFEED_ACQUIRE_TRIGGER_THRESHOLD = 0.05;
	private final double SHOOTER_JOYSTICK_THRESHHOLD = 0.05;
	
	/**
	 * Create a new instance of the Driver's Station
	 * @param driverGamePadUsbPort
	 * @param operatorGamePadUsbPort
	 */
	public DriversStation(int driverGamePadUsbPort, int operatorGamePadUsbPort)
	{
		_driverGamepad = new Joystick(driverGamePadUsbPort);				// std Logitech F310 Gamepad  
		_operatorGamepad = new Joystick(operatorGamePadUsbPort);			// std Logitech F310 Gamepad  
		
		ConfigAutonModeChoosers();
	}

	/**
	 * Returns the locally cached copy of the Current (this scan) Driver's Station Input Values
	 * @return
	 */
	//public DriversStationInputs getCurrentValues()
	//{
	//	return _currentValues;
	//}
	
	/**
	 *  Returns the locally cached copy of the Previous (previous scan) Driver's Station Input Values
	 * @return
	 */
	//public DriversStationInputs getPreviousValues()
	//{
	//	return _previousValues;
	//}
	
	public void UpdateDashboard()
    {
		// Drive Motors
		SmartDashboard.putString("Genl:TEAM_NUMBER", "8024");
		SmartDashboard.putString("Genl:ROBOT_NAME", "LEROY");
    }
	
	private void ConfigAutonModeChoosers() 
	{
        //===================
        // Smart DashBoard User Input
        //   http://wpilib.screenstepslive.com/s/3120/m/7932/l/81109-choosing-an-autonomous-program-from-smartdashboard
        //===================
        _autonModeChooser = new SendableChooser();
        _autonModeChooser.addDefault("Do Nothing", AutonConstants.Auton_Mode.DO_NOTHING);
        _autonModeChooser.addObject("Zero All Axis", AutonConstants.Auton_Mode.ZERO_ALL_AXIS);
        _autonModeChooser.addObject("Drive Fwd", AutonConstants.Auton_Mode.DRIVE_FWD);
        //_autonModeChooser.addObject("Cross Defense", AutonConstants.AutonMode.CROSS_DEFENSE);
        SmartDashboard.putData("Auton mode chooser", _autonModeChooser);
        
        _autonPumasPositionChooser = new SendableChooser();
        _autonPumasPositionChooser.addDefault("Puma Down", AutonConstants.Auton_Pumas_Position.PUMAS_DOWN);
        _autonPumasPositionChooser.addObject("Puma Up", AutonConstants.Auton_Pumas_Position.PUMAS_UP);
        SmartDashboard.putData("Auton Pumas Position", _autonPumasPositionChooser);
        
        _autonDriveTimeChooser = new SendableChooser();
        _autonDriveTimeChooser.addDefault("1 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_1);
        _autonDriveTimeChooser.addObject("2 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_2);
        _autonDriveTimeChooser.addObject("3 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_3);
        _autonDriveTimeChooser.addObject("4 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_4);
        _autonDriveTimeChooser.addObject("5 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_5);
        _autonDriveTimeChooser.addObject("6 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_6);
        _autonDriveTimeChooser.addObject("7 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_7);
        _autonDriveTimeChooser.addObject("8 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_8);
        _autonDriveTimeChooser.addObject("9 sec", AutonConstants.Auton_Drive_Time_In_Secs.SECS_9);
        SmartDashboard.putData("Auton Drive Time", _autonDriveTimeChooser);
        
    	_autonDriveThrottleChooser = new SendableChooser();
    	_autonDriveThrottleChooser.addDefault("10 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_10);
    	_autonDriveThrottleChooser.addObject("25 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_20);
    	_autonDriveThrottleChooser.addObject("30 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_30);
    	_autonDriveThrottleChooser.addObject("40 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_40);
    	_autonDriveThrottleChooser.addObject("50 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_50);
    	_autonDriveThrottleChooser.addObject("60 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_60);
    	_autonDriveThrottleChooser.addObject("70 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_70);
    	_autonDriveThrottleChooser.addObject("80 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_80);
    	_autonDriveThrottleChooser.addObject("90 %", AutonConstants.Auton_Drive_Throttle_Percent.PERCENT_90);
    	SmartDashboard.putData("Auton Drive", _autonDriveThrottleChooser);
    	
    	_autonCrossDefenseTypeChooser = new SendableChooser();
    	_autonCrossDefenseTypeChooser.addObject("MOAT", AutonConstants.Auton_Cross_Defense_Type.MOAT);
    	_autonCrossDefenseTypeChooser.addObject("RAMPARTS", AutonConstants.Auton_Cross_Defense_Type.RAMPARTS);
    	_autonCrossDefenseTypeChooser.addObject("ROCKWALL", AutonConstants.Auton_Cross_Defense_Type.ROCKWALL);
    	_autonCrossDefenseTypeChooser.addObject("ROUGH_TERRAIN", AutonConstants.Auton_Cross_Defense_Type.ROUGH_TERRAIN);
    	SmartDashboard.putData("Auton Cross Defense", _autonCrossDefenseTypeChooser);	    	
	}

	/**
	 * Refreshes the locally cached copy of the Current (this scan) Driver's Station Input Values
	 */
	public DriversStationInputs ReadCurrentScanCycleValues()
	{
		// always set previousValues object to null (so that object can be gc'd)
		_previousValues = null;
		
		if(_currentValues != null)
		{
			// if we have _currentValues (From the last scan) push them into _previosuValues
			_previousValues = _currentValues;
		}
		
		// update _currentValues by reading from the gamepads
		_currentValues = new DriversStationInputs(_driverGamepad, _operatorGamepad);
		
		if(_previousValues == null)
		{
			// if we DO NOT have _previousValues (From the last scan) set it equal to _currentValues
			_previousValues = _currentValues;
		}
		
		return _currentValues;
	}
	
	// Acc / Dec Mode Togle Btn
	public boolean getIsAccelDecelBtnJustPressed()
	{
		if(_currentValues.getIsAccDecModeToggleBtnPressed()
    			&& !_previousValues.getIsAccDecModeToggleBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// Shifter Toggle Btn
	public boolean getIsShifterToggleBtnJustPressed()
	{
    	if(_currentValues.getIsShifterToggleBtnPressed() 
    			&& !_previousValues.getIsShifterToggleBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// Front Puma Toggle Button
	public boolean getIsPumaFrontToggleBtnJustPressed()
	{
		if(_currentValues.getIsPumaFrontToggleBtnPressed() 
				&& !_previousValues.getIsPumaFrontToggleBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// Back Puma Toggle Button
	public boolean getIsPumaBackToggleBtnJustPressed()
	{
		if(_currentValues.getIsPumaBackToggleBtnPressed() 
				&& !_previousValues.getIsPumaBackToggleBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// Both Pumas Toggle Button
	public boolean getIsPumaBothToggleBtnJustPressed()
	{
		if(_currentValues.getIsPumaBothToggleBtnPressed() 
				&& !_previousValues.getIsPumaBothToggleBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsAccDecModeToggleBtnJustPressed()
	{
    	if(_currentValues.getIsAccDecModeToggleBtnPressed()
    			&& !_previousValues.getIsAccDecModeToggleBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsOnlyScaleDriveSpeedUpBtnJustPressed()
	{
    	if(_currentValues.getIsScaleDriveSpeedUpBtnPressed()
    			&& !_previousValues.getIsScaleDriveSpeedUpBtnPressed()
    			&& !_currentValues.getIsScaleDriveSpeedDownBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsOnlyScaleDriveSpeedDownBtnJustPressed()
	{
    	if(_currentValues.getIsScaleDriveSpeedDownBtnPressed()
    			&& !_previousValues.getIsScaleDriveSpeedDownBtnPressed()
    			&& !_currentValues.getIsScaleDriveSpeedUpBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsReleaseBtnJustPressed()
	{
		if(_currentValues.getIsReleaseBtnPressed() 
				&& !_previousValues.getIsReleaseBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsInfeedTiltAxisZeroBtnJustPressed()
	{
		if(_currentValues.getIsInfeedTiltZeroBtnPressed()
				&& !_previousValues.getIsInfeedTiltZeroBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// make sure only 1 button is pressed & 0 triggers
	public boolean getIsOnlyInfeedTiltDeployBtnJustPressed()
	{
    	if(_currentValues.getIsAcquireBtnPressed()
    			&& !_previousValues.getIsAcquireBtnPressed()
    			&& !_currentValues.getIsReleaseBtnPressed()
    			&& !_currentValues.getIsInfeedTiltChevalBtnPressed()
    			&& !_currentValues.getIsInfeedTiltCrossDefenseBtnPressed()
    			&& Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD
    			&& Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// make sure only 1 button is pressed & 0 triggers
	public boolean getIsOnlyInfeedTiltShootBtnJustPressed()
	{
    	if(_currentValues.getIsReleaseBtnPressed()
    			&& !_previousValues.getIsReleaseBtnPressed()
    			&& !_currentValues.getIsAcquireBtnPressed()
    			&& !_currentValues.getIsInfeedTiltChevalBtnPressed()
    			&& !_currentValues.getIsInfeedTiltCrossDefenseBtnPressed()
    			&& Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD
    			&& Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// make sure only 1 button is pressed & 0 triggers
	public boolean getIsOnlyInfeedTiltCrossDefenseBtnJustPressed()
	{
    	if(_currentValues.getIsInfeedTiltCrossDefenseBtnPressed()
    			&& !_previousValues.getIsInfeedTiltCrossDefenseBtnPressed()
    			&& !_currentValues.getIsAcquireBtnPressed()
    			&& !_currentValues.getIsInfeedTiltChevalBtnPressed()
    			&& !_currentValues.getIsReleaseBtnPressed()
    			&& Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD
    			&& Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// make sure only 1 button is pressed & 0 triggers
	public boolean getIsOnlyInfeedTiltChevalBtnJustPressed()
	{
    	if(_currentValues.getIsInfeedTiltChevalBtnPressed()
    			&& !_previousValues.getIsInfeedTiltChevalBtnPressed()
    			&& !_currentValues.getIsAcquireBtnPressed()
    			&& !_currentValues.getIsInfeedTiltCrossDefenseBtnPressed()
    			&& !_currentValues.getIsReleaseBtnPressed()
    			&& Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD
    			&& Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	// make sure only 1 trigger is pressed & 0 buttons
	public boolean getIsOnlyInfeedTiltThrottlePressed()
	{
    	if(!_currentValues.getIsAcquireBtnPressed()
    			&& !_currentValues.getIsReleaseBtnPressed()
    			&& !_currentValues.getIsInfeedTiltChevalBtnPressed()
    			&& !_currentValues.getIsInfeedTiltCrossDefenseBtnPressed()
    			&& ((Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) > INFEED_TILT_TRIGGER_THRESHHOLD
    							&& Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD)
    				||
    				(Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD
							&& Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) > INFEED_TILT_TRIGGER_THRESHHOLD)))
		{
			return true;
		}
		else
		{
			return false;
		}		
	}
	
	// make sure only 1 trigger is pressed
	public double getInfeedTiltRawVelocityCmd()
	{
		if(Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD
				&& (Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) < INFEED_TILT_TRIGGER_THRESHHOLD))
		{
			// neither is pressed
			return 0.0;
		}
		else if(Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) > INFEED_TILT_TRIGGER_THRESHHOLD
				&& (Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) > INFEED_TILT_TRIGGER_THRESHHOLD))
		{
			// both are pressed
			return 0.0;
		}
		else if (Math.abs(_currentValues.getInfeedTiltUpVelocityCmd()) > INFEED_TILT_TRIGGER_THRESHHOLD)
    	{
			// only up trigger
    		return _currentValues.getInfeedTiltUpVelocityCmd();
    	}
    	else if (Math.abs(_currentValues.getInfeedTiltDownVelocityCmd()) > INFEED_TILT_TRIGGER_THRESHHOLD)
    	{
    		// only down trigger
    		return _currentValues.getInfeedTiltDownVelocityCmd();
    	}
    	else
    	{
    		return 0.0;
    	}
	}
	
	/*
	public boolean getIsOnlyInfeedAcquireBtnPressed()
	{
    	if(_currentValues.getIsInfeedAcquireBtnPressed()
    			&& !_currentValues.getIsInfeedReleaseBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsOnlyInfeedReleaseBtnPressed()
	{
    	if(!_currentValues.getIsInfeedAcquireBtnPressed()
    			&& _currentValues.getIsInfeedReleaseBtnPressed())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	*/
	public boolean getIsAcquireBtnPressed()
	{
		return _currentValues.getIsAcquireBtnPressed();
	}
	
	public boolean getIsReleaseBtnPressed()
	{
		return _currentValues.getIsReleaseBtnPressed();
	}
	
	public double getInfeedAcquireVelocityCmd()
	{
		return _currentValues.getInfeedAcquireVelocityCmd();
	}
	public double getShooterRawVelocityCmd()
	{
		return _currentValues.getShooterRawVelocityCmd();
	}
	
	public double getArcadeDriveRawThrottleCmd()
	{
		return _currentValues.getArcadeDriveRawThrottleCmd();		
	}
	
	public double getArcadeDriveRawTurnCmd()
	{
		return _currentValues.getArcadeDriveRawTurnCmd();		
	}	
	
	public Auton_Mode getAutonModeRequested()
	{
		return _currentValues.getAutonModeRequested();
	}
	
	public Auton_Pumas_Position getAutonPumasPositionRequested()
	{
		return _currentValues.getAutonPumasPositionRequested();
	}
	
	public Auton_Drive_Time_In_Secs getAutonDriveTimeInSecsRequested()
	{
		return _currentValues.getAutonDriveTimeInSecsRequested();
	}
	
	public Auton_Drive_Throttle_Percent getAutonDriveThrottlePercentRequested()
	{
		return _currentValues.getAutonDriveThrottlePercentRequested();
	}
	
	public Auton_Cross_Defense_Type getAutonCrossDefenseTypeRequested()
	{
		return _currentValues.getAutonCrossDefenseTypeRequested();	
	}
	
	/**
	 * 
	 * Immutable class to hold data read from the gamepads
	 *
	 */
	public final class DriversStationInputs
	{
		// ======================================
		// driver's station gamepad config
		// ======================================
		private static final int DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK = LogitechF310.LEFT_Y_AXIS;		
		private static final int DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK = LogitechF310.RIGHT_X_AXIS;
		//private static final int DRIVER_GAMEPAD_WINCH_AXIS = LogitechF310.RIGHT_Y_AXIS;
		private static final int DRIVER_GAMEPAD_PUMA_BOTH_TOGGLE_BTN = LogitechF310.GREEN_BUTTON_A;
		private static final int DRIVER_GAMEPAD_SHIFTER_TOGGLE_BTN = LogitechF310.YELLOW_BUTTON_Y;	
		private static final int DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN = LogitechF310.START_BUTTON;
		private static final int DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN = LogitechF310.BACK_BUTTON;
		private static final int DRIVER_GAMEPAD_ACC_DEC_MODE_TOGGLE_BTN = LogitechF310.LEFT_BUMPER;
		//private static final int DRIVER_GAMEPAD_CUPID_OPEN_BTN = LogitechF310.LEFT_BUMPER;
		private static final int DRIVER_GAMEPAD_INFEED_TILT_DOWN_AXIS = LogitechF310.RIGHT_TRIGGER;
		private static final int DRIVER_GAMEPAD_INFEED_TILT_UP_AXIS = LogitechF310.LEFT_TRIGGER;
		
		private static final int OPERATOR_GAMEPAD_SHOOTER_AXIS = LogitechF310.LEFT_Y_AXIS;
		private static final int OPERATOR_GAMEPAD_INFEED_AXIS = LogitechF310.RIGHT_Y_AXIS;
		private static final int OPERATOR_GAMEPAD_INFEED_CHEVAL_BTN = LogitechF310.GREEN_BUTTON_A;
		//private static final int OPERATOR_GAMEPAD_ELEVATOR_TIMER_OVERRIDE_BTN = LogitechF310.RED_BUTTON_B;
		private static final int OPERATOR_GAMEPAD_INFEED_ZERO_BTN = LogitechF310.RED_BUTTON_B;
		private static final int OPERATOR_GAMEPAD_INFEED_CROSS_DEFENSE_BTN = LogitechF310.YELLOW_BUTTON_Y;
		//private static final int OPERATOR_GAMEPAD_SLIDER_FWD_BTN = LogitechF310.START_BUTTON;
		//private static final int OPERATOR_GAMEPAD_SLIDER_REV_BTN = LogitechF310.BACK_BUTTON;	
		private static final int OPERATOR_GAMEPAD_ACQUIRE_BTN = LogitechF310.RIGHT_BUMPER;
		private static final int OPERATOR_GAMEPAD_RELEASE_BTN = LogitechF310.LEFT_BUMPER;
		private static final int OPERATOR_GAMEPAD_INFEED_ACQUIRE_AXIS = LogitechF310.RIGHT_TRIGGER;	
		private static final int OPERATOR_GAMEPAD_INFEED_RELEASE_AXIS = LogitechF310.LEFT_TRIGGER;
		
		// private backing fields
    	private final boolean _isPumaFrontToggleBtnPressed;
    	private final boolean _isPumaBackToggleBtnPressed;
    	private final boolean _isPumaBothToggleBtnPressed;
    	//private final boolean _isCupidCameraBtnPressed;
    	//private final boolean _isCupidToggleBtnPressed;
    	//private final boolean _isClimbEnabledBtnPressed;
    	private final boolean _isShifterToggleBtnPressed;
    	private final boolean _isAccDecModeToggleBtnPressed;
    	
    	//private final boolean _isSliderFwdBtnPressed;
    	//private final boolean _isSliderRevBtnPressed;
    	private final boolean _isAcquireBtnPressed;
    	private final boolean _isReleaseBtnPressed;
    	private final boolean _isInfeedTiltChevalBtnPressed;
    	private final boolean _isInfeedZeroBtnPressed;
    	private final boolean _isInfeedTiltCrossDefenseBtnPressed;
    	//private final boolean _isElevatorTimerOverrideBtnPressed;
    	
    	private final boolean _isScaleDriveSpeedUpBtnPressed;
    	private final boolean _isScaleDriveSpeedDownBtnPressed;
    	
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	private final double _arcadeDriveRawThrottleCmd;
    	private final double _arcadeDriveRawTurnCmd;
    	
    	private final double _shooterRawVelocityCmd;
    	private final double _infeedRawVelocityCmd;
    	private final double _infeedAcquireRawVelocityCmd;
    	private final double _infeedReleaseRawVelocityCmd;
    	
    	private final double _infeedTiltUpVelocityCmd;
    	private final double _infeedTiltDownVelocityCmd;
    	//private final double _winchRawCmd;
    
    	private Auton_Mode _autonModeRequested = Auton_Mode.CROSS_DEFENSE.UNDEFINED;
    	private Auton_Pumas_Position _autonPumasPositionRequested = Auton_Pumas_Position.UNDEFINED;
    	private Auton_Drive_Time_In_Secs _autonDriveTimeInSecsRequested = Auton_Drive_Time_In_Secs.UNDEFINED;
    	private Auton_Drive_Throttle_Percent _autonDriveThrottlePercentRequested = Auton_Drive_Throttle_Percent.UNDEFINED;
    	private Auton_Cross_Defense_Type _autonCrossDefenseTypeRequested = Auton_Cross_Defense_Type.UNDEFINED;
		
		/**
		 * Create an entirely new instance by reading from the Gamepads
		 * @param driverGamepad
		 * @param operatorGamepad
		 */
		protected DriversStationInputs(Joystick driverGamepad, Joystick operatorGamepad)
		{
	    	// ==========================
	    	// get values from the gamepads
	    	// ==========================
			
			_isPumaFrontToggleBtnPressed = _driverGamepad.getRawButton(DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN);
	    	_isPumaBackToggleBtnPressed = _driverGamepad.getRawButton(DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN);
	    	_isPumaBothToggleBtnPressed = _driverGamepad.getRawButton(DRIVER_GAMEPAD_PUMA_BOTH_TOGGLE_BTN);
	    	_isShifterToggleBtnPressed = _driverGamepad.getRawButton(DRIVER_GAMEPAD_SHIFTER_TOGGLE_BTN);
	    	//_isCupidCameraBtnPressed = false; //_driverGamepad.getRawButton(DRIVER_GAMEPAD_CUPID_CAMERA_BTN);
	    	//_isCupidToggleBtnPressed = false; //_driverGamepad.getRawButton(DRIVER_GAMEPAD_CUPID_OPEN_BTN);
	    	//_isClimbEnabledBtnPressed = false; //_driverGamepad.getRawButton(DRIVER_GAMEPAD_CLIMB_ENABLE_BTN);
	    	_isAccDecModeToggleBtnPressed = _driverGamepad.getRawButton(DRIVER_GAMEPAD_ACC_DEC_MODE_TOGGLE_BTN);
	    	
	    	//_isSliderFwdBtnPressed = false; //_operatorGamepad.getRawButton(OPERATOR_GAMEPAD_SLIDER_FWD_BTN);
	    	//_isSliderRevBtnPressed = false; //_operatorGamepad.getRawButton(OPERATOR_GAMEPAD_SLIDER_REV_BTN);
	    	_isAcquireBtnPressed = _operatorGamepad.getRawButton(OPERATOR_GAMEPAD_ACQUIRE_BTN);
	    	_isReleaseBtnPressed = _operatorGamepad.getRawButton(OPERATOR_GAMEPAD_RELEASE_BTN);
	    	_isInfeedTiltChevalBtnPressed = _operatorGamepad.getRawButton(OPERATOR_GAMEPAD_INFEED_CHEVAL_BTN);
	    	//_isElevatorTimerOverrideBtnPressed = false; //_operatorGamepad.getRawButton(OPERATOR_GAMEPAD_ELEVATOR_TIMER_OVERRIDE_BTN);
	    	_isInfeedZeroBtnPressed = _operatorGamepad.getRawButton(OPERATOR_GAMEPAD_INFEED_ZERO_BTN);
	    	_isInfeedTiltCrossDefenseBtnPressed = _operatorGamepad.getRawButton(OPERATOR_GAMEPAD_INFEED_CROSS_DEFENSE_BTN);
			
	    	_isScaleDriveSpeedUpBtnPressed = false;
	    	_isScaleDriveSpeedDownBtnPressed = false;
	    	
	    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
	    	_arcadeDriveRawThrottleCmd = _driverGamepad.getRawAxis(DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK);
	    	_arcadeDriveRawTurnCmd = _driverGamepad.getRawAxis(DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK);
	    	_infeedTiltUpVelocityCmd = _driverGamepad.getRawAxis(DRIVER_GAMEPAD_INFEED_TILT_UP_AXIS);
	    	_infeedTiltDownVelocityCmd = _driverGamepad.getRawAxis(DRIVER_GAMEPAD_INFEED_TILT_DOWN_AXIS);
	    	
	    	_shooterRawVelocityCmd = _operatorGamepad.getRawAxis(OPERATOR_GAMEPAD_SHOOTER_AXIS);
	    	_infeedAcquireRawVelocityCmd = _operatorGamepad.getRawAxis(OPERATOR_GAMEPAD_INFEED_RELEASE_AXIS);
	    	_infeedReleaseRawVelocityCmd = _operatorGamepad.getRawAxis(OPERATOR_GAMEPAD_INFEED_ACQUIRE_AXIS);
	    	_infeedRawVelocityCmd = _operatorGamepad.getRawAxis(OPERATOR_GAMEPAD_INFEED_AXIS);
	    	
	    	//_winchRawCmd = 0.0; //_driverGamepad.getRawAxis(DRIVER_GAMEPAD_WINCH_AXIS);	
	    
	    	// read auton values
	    	_autonModeRequested = (Auton_Mode) _autonModeChooser.getSelected();
	    	_autonPumasPositionRequested = (Auton_Pumas_Position)_autonPumasPositionChooser.getSelected();
	    	_autonDriveTimeInSecsRequested = (Auton_Drive_Time_In_Secs)_autonDriveTimeChooser.getSelected();
	    	_autonDriveThrottlePercentRequested = (Auton_Drive_Throttle_Percent)_autonDriveThrottleChooser.getSelected();
	    	_autonCrossDefenseTypeRequested = (Auton_Cross_Defense_Type)_autonCrossDefenseTypeChooser.getSelected();
	    	
		}
		
    	public Auton_Mode getAutonModeRequested()
    	{
    		return _autonModeRequested;
    	}
    	
    	public Auton_Pumas_Position getAutonPumasPositionRequested()
    	{
    		return _autonPumasPositionRequested;
    	}
    	
    	public Auton_Drive_Time_In_Secs getAutonDriveTimeInSecsRequested()
    	{
    		return _autonDriveTimeInSecsRequested;
    	}
    	
    	public Auton_Drive_Throttle_Percent getAutonDriveThrottlePercentRequested()
    	{
    		return _autonDriveThrottlePercentRequested;
    	}
    	
    	public Auton_Cross_Defense_Type getAutonCrossDefenseTypeRequested()
    	{
    		return _autonCrossDefenseTypeRequested;		
    	}
			
    	public boolean getIsPumaFrontToggleBtnPressed()
    	{
    		return _isPumaFrontToggleBtnPressed;
    	}
    	
    	public boolean getIsPumaBackToggleBtnPressed()
    	{
    		return _isPumaBackToggleBtnPressed;
    	}
    	
    	public boolean getIsPumaBothToggleBtnPressed()
    	{
    		return _isPumaBothToggleBtnPressed;
    	}
    	
    	public boolean getIsShifterToggleBtnPressed()
    	{
    		return _isShifterToggleBtnPressed;
    	}
    	
    	public boolean getIsInfeedTiltChevalBtnPressed()
    	{
    		return _isInfeedTiltChevalBtnPressed;
    	}
    	
    	public boolean getIsInfeedTiltCrossDefenseBtnPressed()
    	{
    		return _isInfeedTiltCrossDefenseBtnPressed;
    	}
    	
    	public boolean getIsInfeedTiltZeroBtnPressed()
    	{
    		return _isInfeedZeroBtnPressed;
    	}
    	
    	//public boolean getIsCupidCameraBtnPressed()
    	//{
    	//	return _isCupidCameraBtnPressed;
    	//}
    	
    	//public boolean getIsCupidToggleBtnPressed()
    	//{
    	//	return _isCupidToggleBtnPressed;
    	//}
    	
    	//public boolean getIsClimbEnabledBtnPressed()
    	//{
    	//	return _isClimbEnabledBtnPressed;
    	//}
    	
    	public boolean getIsAccDecModeToggleBtnPressed()
    	{
    		return _isAccDecModeToggleBtnPressed;
    	}
    	
    	//public boolean getIsSliderFwdBtnPressed()
    	//{
    	//	return _isSliderFwdBtnPressed;
    	//}
    	
    	//public boolean getIsSliderRevBtnPressed()
    	//{
    	//	return _isSliderRevBtnPressed;
    	//}
    	
    	public boolean getIsAcquireBtnPressed()
    	{
    		return _isAcquireBtnPressed;
    	}
    	
    	public boolean getIsReleaseBtnPressed()
    	{
    		return _isReleaseBtnPressed;
    	}
    	
    	//public boolean getIsCameraSwitchBtnPressed()
    	//{
    	//	return _isCameraSwitchBtnPressed;
    	//}
    	
    	//public boolean getIsElevatorTimerOverrideBtnPressed()
    	//{
    	//	return _isElevatorTimerOverrideBtnPressed;
    	//}
    	
    	//public boolean getIsShooterTargetSpeedToggleBtnPressed()
    	//{
    	//	return _isShooterTargetSpeedToggleBtnPressed;
    	//}
    	
    	//public boolean getIsShooterAltModeEnableBtnPressed()
    	//{
    	//	return _isShooterAltModeEnableBtnPressed;
    	//}
    	
    	public boolean getIsScaleDriveSpeedUpBtnPressed()
    	{
    		return _isScaleDriveSpeedUpBtnPressed;
    	}
    	
    	public boolean getIsScaleDriveSpeedDownBtnPressed()
    	{
    		return _isScaleDriveSpeedDownBtnPressed;
    	}
    	
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	public double getArcadeDriveRawThrottleCmd()
    	{
    		return -1.0 * _arcadeDriveRawThrottleCmd;
    	}
    	
    	public double getArcadeDriveRawTurnCmd()
    	{
    		return _arcadeDriveRawTurnCmd;
    	}

    	public double getShooterRawVelocityCmd()
    	{
    		if(Math.abs(_shooterRawVelocityCmd) > SHOOTER_JOYSTICK_THRESHHOLD)
    		{
    			return _shooterRawVelocityCmd;
    		}
    		else
    		{
    			return 0.0;
    		}
    		
    	}
    	
    	//public double getTurretCCWRawVelocityCmd()
    	//{
    	//	return _turretCCWRawVelocityCmd;
    	//}
    	
    	//public double getTurretCWRawVelocityCmd()
    	//{
    	//	return _turretCWRawVelocityCmd;
    	//}
    	
    	//public double getAnd1RawCmd()
    	//{
    	//	return _and1RawCmd;
    	//}
    	
    	public double getInfeedTiltUpVelocityCmd()
    	{
    		if(Math.abs(_infeedTiltUpVelocityCmd) > INFEED_TILT_TRIGGER_THRESHHOLD)
    		{
    			return _infeedTiltUpVelocityCmd;
    		}
    		else
    		{
    			return 0.0;
    		}
    	}
    	
    	public double getInfeedTiltDownVelocityCmd()
    	{
    		if(Math.abs(_infeedTiltDownVelocityCmd) > INFEED_TILT_TRIGGER_THRESHHOLD)
    		{
    			return _infeedTiltDownVelocityCmd;
    		}
    		else
    		{
    			return 0.0;
    		}
    	}
    	
    	public double getInfeedAcquireVelocityCmd()
    	{
    		if ((Math.abs(_infeedAcquireRawVelocityCmd) > INFEED_ACQUIRE_TRIGGER_THRESHOLD)
    				&& (Math.abs(_infeedReleaseRawVelocityCmd) < INFEED_ACQUIRE_TRIGGER_THRESHOLD))
    		{
    			return -1.0 * _infeedAcquireRawVelocityCmd;
    		}
    		else if ((Math.abs(_infeedAcquireRawVelocityCmd) < INFEED_ACQUIRE_TRIGGER_THRESHOLD)
    				&& (Math.abs(_infeedReleaseRawVelocityCmd) > INFEED_ACQUIRE_TRIGGER_THRESHOLD))
    		{
    			return _infeedReleaseRawVelocityCmd;
    		}
    		else
    		{
    			return 0.0;
    		}
    		
    	}
    	
    	//public double getWinchRawCmd()
    	//{
    	//	return _winchRawCmd;
    	//}	
	}
}
