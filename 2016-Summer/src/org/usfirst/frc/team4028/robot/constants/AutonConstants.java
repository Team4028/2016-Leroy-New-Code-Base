package org.usfirst.frc.team4028.robot.constants;

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
	
	public enum Auton_Drive_Time_In_Secs
	{
		UNDEFINED,
		SECS_1,
		SECS_2,
		SECS_3,
		SECS_4,
		SECS_5,
		SECS_6,
		SECS_7,
		SECS_8,
		SECS_9
	}
	
	public enum Auton_Drive_Throttle_Percent
	{
		UNDEFINED,
		PERCENT_10,
		PERCENT_20,
		PERCENT_30,
		PERCENT_40,
		PERCENT_50,
		PERCENT_60,
		PERCENT_70,
		PERCENT_80,
		PERCENT_90
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
