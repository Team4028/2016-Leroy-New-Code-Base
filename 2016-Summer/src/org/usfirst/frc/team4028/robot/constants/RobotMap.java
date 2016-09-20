package org.usfirst.frc.team4028.robot.constants;

import org.usfirst.frc.team4028.robot.constants.LogitechF310;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RobotMap 
{
	// ======================================
	// Motor Controller Layout on the robot
	// ======================================
	/*
			Talon	Can Addr 20 (Infeed Tilt)			Victor	PWN Port 6  (Infeed Acq)
			Victor	PWN Port 8	(Left Shooter)			Victor	PWM Port 7  (Right Shooter)
			Talon	Can Addr 11 (Left Slave 2)			Talon	Can Addr 10 (Right Slave 2)
			Talon	Can Addr 15	(Left Slave 1)			Talon	Can Addr 13 (Right Slave 1)
			Talon	Can Addr 14	(Left Master)			Talon	Can Addr 12 (Right Master)
	*/
	
	// ======================================
	// Constants for CAN Bus Addresses
	// ======================================
	
	// define constant for PCM (Pneumatic Control Module)
	public static final int PCM_CAN_ADDR = 0;				
	
	// PDP (Power Distribution Panel) CAN Bus Address
	//public static final int PDB_CAN_ADDR = 3; 
	
	// define constants for Talon SRX CAN Bus Addresses
	public static final int LEFT_DRIVE_MASTER_MTR_TALON_CAN_ADDR = 14;
	public static final int LEFT_DRIVE_SLAVE_1_MTR_TALON_CAN_ADDR = 15;
	public static final int LEFT_DRIVE_SLAVE_2_MTR_TALON_CAN_ADDR = 11;
	public static final int RIGHT_DRIVE_MASTER_MTR_TALON_CAN_ADDR = 12;
	public static final int RIGHT_DRIVE_SLAVE_1_MTR_TALON_CAN_ADDR = 13;
	public static final int RIGHT_DRIVE_SLAVE_2_MTR_TALON_CAN_ADDR = 10;
	//public static final int TURRET_TALON_CAN_ADDR = 16;
	//public static final int SHOOTER_LEFT_MTR_TALON_CAN_ADDR = 17;
	//public static final int SHOOTER_RIGHT_MTR_TALON_CAN_ADDR = 19;
	//public static final int SLIDER_TALON_CAN_ADDR = 18;
	public static final int INFEED_TILT_MTR_TALON_CAN_ADDR = 20;
	
	// ======================================
	// define constants for PWM Ports on RobioRio
	// ======================================
	//public static final int AND1_SERVO_PWM_PORT = 1;
	//public static final int CUPID_SERVO_PWM_PORT = 6;
	//public static final int SCALING_MTR_PWM_PORT = 7;
	public static final int SHOOTER_LEFT_MTR_PWM_PORT = 8;
	public static final int SHOOTER_RIGHT_MTR_PWM_PORT = 7;
	public static final int INFEED_ACQ_MTR_PWM_PORT = 6;
	//public static final int KICKER_MTR_PWM_PORT = 9;
	
	// ======================================
	// define constants for DIO Ports on RoboRio
	// ======================================
	//public static final int TURRET_HOME_LIMIT_SWITCH_DIO_PORT = 0;
	//public static final int TURRET_APPROACHING_HOME_LIMIT_SWITCH_DIO_PORT = 1;
	public static final int BALL_IN_POSITION_LIMIT_SWITCH_DIO_PORT = 3;

	// ======================================
	// define constants for usb cameras
	// ======================================
	public static final String SHOOTER_CAMERA_NAME = "cam0";
	//public static final String INFEED_CAMERA_NAME = "cam1";
	//public static final String CUPID_CAMERA_NAME = "cam2";

	// ======================================
	// Define constants for solenoid ports on Pneumatic Control Module (PCM)
	// ======================================
	public static final int PUMA_FRONT_SOLENOID_RETRACT_PCM_PORT = 2;
	public static final int PUMA_FRONT_SOLENOID_EXTEND_PCM_PORT = 3;
	public static final int PUMA_BACK_SOLENOID_RETRACT_PCM_PORT = 0;   	// 3
	public static final int PUMA_BACK_SOLENOID_EXTEND_PCM_PORT = 1;		// 2
	public static final int SHIFTER_SOLENOID_RETRACT_PCM_PORT = 7;
	public static final int SHIFTER_SOLENOID_EXTEND_PCM_PORT = 6;
	
	// ======================================
	// Define constants for solenoid positions 
	// ======================================
	public static final Value PUMA_FRONT_UP_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value PUMA_FRONT_DOWN_POSITION = DoubleSolenoid.Value.kReverse;		
	public static final Value PUMA_BACK_UP_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value PUMA_BACK_DOWN_POSITION = DoubleSolenoid.Value.kReverse;
	public static final Value SHIFTER_LOW_GEAR = DoubleSolenoid.Value.kForward;
	public static final Value SHIFTER_HIGH_GEAR = DoubleSolenoid.Value.kReverse;
	
	// ======================================
	// Driver's station
	// ======================================
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
}
