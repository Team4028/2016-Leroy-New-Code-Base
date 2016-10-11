package org.usfirst.frc.team6204.robot.autonRoutines;

import edu.wpi.first.wpilibj.DriverStation;

public class DoNothing {

	public DoNothing()
	{
		DriverStation.reportError("Hello! I am not doing anything...", false);
	}
	
	public void Execute()
	{
	}
}
