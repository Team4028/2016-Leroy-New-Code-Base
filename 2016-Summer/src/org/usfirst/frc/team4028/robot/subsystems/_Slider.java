package org.usfirst.frc.team4028.robot.subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;

/**
 * This class defines the behavior of the Slider
 */
public class _Slider 
{
	private static final int SLIDER_ENCODER_COUNTS_PER_REV = 250;
	
	private CANTalon _sliderMtr;
	
	private boolean _isAxisZeroedYet = false;
	
	public _Slider(int talonCanBusAdddress)
	{
    	_sliderMtr = new CANTalon(talonCanBusAdddress);
    	_sliderMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);			// default to %VBus on startup, chgs to Position in Axis Zero Function
    	_sliderMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	_sliderMtr.enableLimitSwitch(false, true);
    	_sliderMtr.ConfigRevLimitSwitchNormallyOpen(false);
    	_sliderMtr.reverseSensor(true);
    	_sliderMtr.enableBrakeMode(true);
    	_sliderMtr.configNominalOutputVoltage(+0.0f, -0.0f);
    	_sliderMtr.configPeakOutputVoltage(+12.0f, -12.0f);
    	
    	_sliderMtr.configEncoderCodesPerRev(SLIDER_ENCODER_COUNTS_PER_REV);	// try to enable Unit Scaling
    	    	
	}
	
	public boolean IsAxisZeroedYet() 
	{
		return _isAxisZeroedYet;
	}
	
	public void ZeroAxisReentrant()
	{	
	}
}
