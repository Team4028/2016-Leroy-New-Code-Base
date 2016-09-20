package org.usfirst.frc.team4028.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;

import com.kauailabs.navx.frc.AHRS;

/**
 * This class defines the behavior of the Navigation Sensor
 */
// http://www.pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
public class _NavigationSensor 
{
	private AHRS _navXSensor;
	
	public _NavigationSensor(edu.wpi.first.wpilibj.SerialPort.Port port)
	{
    	try 
    	{
            /* Communicate w/ navX MXP via one of the following ports                           */
            /*   				I2C.Port.kMXP, 												   	*/
            /* 					SerialPort.Port.kMXP, 										   	*/
            /* 					SerialPort.Port.kUSB										   	*/
            /* 					SPI.Port.kMXP   			plugged into mxp port on RoboRio	*/			
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.  */
            _navXSensor = new AHRS(port);
            
            DriverStation.reportError("..navX sensor connected" + " |" , false);
        } 
    	catch (RuntimeException ex ) 
    	{
            DriverStation.reportError("Error connecting to navX Sensor: " + ex.getMessage() + " |", true);
        }
	}
}
