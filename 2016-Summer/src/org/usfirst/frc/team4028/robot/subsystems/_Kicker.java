package org.usfirst.frc.team4028.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;

/**
 * This class defines the behavior of the Kicker Subsystem
 */
public class _Kicker 
{
	private VictorSP _kickerMtr;
	
	public _Kicker(int victorPWMPort)
	{
    	_kickerMtr = new VictorSP(victorPWMPort);			
	}
}
