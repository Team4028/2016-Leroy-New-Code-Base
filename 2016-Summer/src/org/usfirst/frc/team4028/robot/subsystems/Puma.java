package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Puma 
{
	// ======================================
	// define constants for air cylinder states / positions
	//	(map the physical air cylinder position to logical state)
	// ======================================
	private DoubleSolenoid _pumaFrontSolenoid;
	private DoubleSolenoid _pumaBackSolenoid;
	
	private Value _pumaFrontPosition;
	private Value _pumaBackPosition;
	
	// ===================================
	// Constructors follow
	// ===================================
	public Puma(int pcmCanBusAdddress, int frontSolenoidExtendPCMPort, int frontSolenoidRetractPCMPort, int backSolenoidExtendPCMPort, int backSolenoidRetractPCMPort)
	{
    	_pumaFrontSolenoid = new DoubleSolenoid(pcmCanBusAdddress, frontSolenoidExtendPCMPort, frontSolenoidRetractPCMPort);
    	
    	_pumaBackSolenoid = new DoubleSolenoid(pcmCanBusAdddress, backSolenoidExtendPCMPort, backSolenoidRetractPCMPort);
	}

	//============================================================================================
	// Methods follow (methods make the robot do something (ie: push changes to the robot hardware)
	//============================================================================================

	public void ToggleFrontPumaPosition()
	{
		if (getPumaFrontPosition() == RobotMap.PUMA_FRONT_DOWN_POSITION)
		{
			ChgPumaFrontPosition(RobotMap.PUMA_FRONT_UP_POSITION);
		}
		else {
			ChgPumaFrontPosition(RobotMap.PUMA_FRONT_DOWN_POSITION);
		}		
	}
	
	public void ToggleBackPumaPosition()
	{
		if (this.getPumaBackPosition() == RobotMap.PUMA_BACK_DOWN_POSITION)
		{
			ChgPumaBackPosition(RobotMap.PUMA_BACK_UP_POSITION);
		}
		else {
			ChgPumaBackPosition(RobotMap.PUMA_BACK_DOWN_POSITION);
		}		
	}
	
	public void ToggleBothPumasPosition()
	{
		if ((this.getPumaFrontPosition() == RobotMap.PUMA_FRONT_UP_POSITION)
				|| (this.getPumaBackPosition() == RobotMap.PUMA_BACK_UP_POSITION))
		{
			ChgPumaFrontPosition(RobotMap.PUMA_FRONT_DOWN_POSITION);
			ChgPumaBackPosition(RobotMap.PUMA_BACK_DOWN_POSITION);
		}
		else {
			ChgPumaFrontPosition(RobotMap.PUMA_FRONT_UP_POSITION);
			ChgPumaBackPosition(RobotMap.PUMA_BACK_UP_POSITION);
		}		
	}
	
	public void ChgPumaFrontPosition(Value newPumaPostion) 
	{
		_pumaFrontPosition = newPumaPostion;
		
		_pumaFrontSolenoid.set(_pumaFrontPosition);
		
		// 2017 season std: when we chg a modal condition, write to dashboard
		if(_pumaFrontPosition == RobotMap.PUMA_FRONT_UP_POSITION)
		{
			DriverStation.reportError("Puma Front: UP", false);
		}
		else
		{
			DriverStation.reportError("Puma Front: DOWN", false);
		}
	}

	public void ChgPumaBackPosition(Value newPumaPostion) 
	{
		this._pumaBackPosition = newPumaPostion;
		
		_pumaBackSolenoid.set(_pumaBackPosition);
		
		// 2017 season std: when we chg a modal condition, write to dashboard
		if(_pumaBackPosition == RobotMap.PUMA_BACK_UP_POSITION)
		{
			DriverStation.reportError("Puma Back: UP", false);
		}
		else
		{
			DriverStation.reportError("Puma Back: DOWN", false);
		}
	}
	
	//============================================================================================
	// Property Accessors follow (properties only change internal state) (ie: DO NOT push changes to the robot hardware)
	//============================================================================================
	
	public Value getPumaFrontPosition() 
	{
		return _pumaFrontPosition;
	}
	
	public Value getPumaBackPosition() 
	{
		return _pumaBackPosition;
	}

}
