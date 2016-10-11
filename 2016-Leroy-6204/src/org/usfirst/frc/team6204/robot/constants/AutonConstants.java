package org.usfirst.frc.team6204.robot.constants;

public class AutonConstants 
{
	// ==========================================================
	// define the different auton modes (selected on the Smart Dashboard)
	// ==========================================================
	public enum Auton_Mode
	{
		UNDEFINED,
		DO_NOTHING,
		ZERO_ALL_AXIS,
		DRIVE_FWD,
		CROSS_DEFENSE,
	}
	
	public enum Auton_Pumas_Position
	{
		UNDEFINED,
		PUMAS_DOWN,
		PUMAS_UP
	}
	
	public enum Auton_Infeed_Tilt_Position
	{
		UNDEFINED,
		INFEED_UP,
		LOW_BAR
	}
	
	public enum Auton_Drive_Time_In_Secs
	{
		UNDEFINED,
		SECS_0,
		SECS_1,
		SECS_1_5,
		SECS_2,
		SECS_2_5,
		SECS_3,
		SECS_3_5,
		SECS_4,
		SECS_4_5,
		SECS_5,
		SECS_6,
		SECS_7
	}
	
	public enum Auton_Drive_Throttle_Percent
	{
		UNDEFINED,
		PERCENT_0,
		PERCENT_30,
		PERCENT_40,
		PERCENT_50,
		PERCENT_60,
		PERCENT_70,
		PERCENT_80,
		PERCENT_90,
		PERCENT_100
	}
	
	public enum Auton_Shoot_Ball
	{
		UNDEFINED,
		YES,
		NO,
	}
	
	public enum Auton_Cross_Defense_Type
	{
		UNDEFINED,
		MOAT,
		RAMPARTS,
		ROCKWALL,
		ROUGH_TERRAIN
	}
}
