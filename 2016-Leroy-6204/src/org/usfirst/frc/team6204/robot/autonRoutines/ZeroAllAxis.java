package org.usfirst.frc.team6204.robot.autonRoutines;

import org.usfirst.frc.team6204.robot.subsystems.Infeed;

public class ZeroAllAxis
{
	private Infeed _infeed;
	
	public ZeroAllAxis(Infeed infeed)
	{
		_infeed = infeed;
	}
	
	public void Execute()
	{
		if(!_infeed.IsInfeedTiltAxisZeroedYet())
    	{
	    	// zero the infeed tilt axis (fyi this may be the same physical location as the stored position)
			_infeed.ZeroInfeedTiltAxisReentrant();
    	}
    	else
    	{
    		_infeed.MoveInfeedTiltAxisToCrossDefensePositionReentrant();
    	}
	}
}
