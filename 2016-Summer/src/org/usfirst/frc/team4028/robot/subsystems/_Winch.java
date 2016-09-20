package org.usfirst.frc.team4028.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;

/**
 * This class defines the behavior of the Winch Subsystem
 */
public class _Winch 
{
	private VictorSP _winchMtr;
	
	public _Winch(int victorPWMPort)
	{
		_winchMtr = new VictorSP(victorPWMPort);
	}
}
