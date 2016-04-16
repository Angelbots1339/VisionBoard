package org.usfirst.frc.team1339.robot;

public class Constants {
	//Drive PID Values
	public static final double kDriveKp = 0.07;
	public static final double kDriveKi = 0;
	public static final double kDriveKd = 0.05;
	// 0.05 , 0 , 0.05
	
	/*public static final double kDriveKp = 0.22; 
	public static final double kDriveKi = 0;
	public static final double kDriveKd = 0.1; */
	
	//Turn PID Values
	//Tuned for making a stationary turn (eg a 180)
	public static final double kTurnKp = 0.113; // 0.05, 0.1
	public static final double kTurnKi = 0.;
	public static final double kTurnKd = 0.04;
	//Shooter PID Values
	public static final double kShooterKp = 0.0004;//0.0003;
	public static final double kShooterKi = 0;
	public static final double kShooterKd = 0.0000;//.01;
	//Intake PID Values
	public static final double kArmKp = 0.0008;
	public static final double kArmKi = 0.000003;
	public static final double kArmKd = 0.005;
	//Vision PID Values
	public static final double kVisionKp = 0.05;
	public static final double kVisionKi = 0;
	public static final double kVisionKd = 0.4;
	//Gyro PID Values
	//Tuned for use while driving straight in auto
	public static final double kAutoGyroKp = 0.22;
	public static final double kAutoGyroKi = .0;
	public static final double kAutoGyroKd = .1;
	//Tuned for turning and driving at the same time
	public static final double kGyroKp = 0.035;
	public static final double kGyroKi = .0;
	public static final double kGyroKd = .4;
	//Acceleration
	public static final double kAccelKp = 0.00000035; //0.00000035
	public static final double kAccelKi = 0;
	public static final double kAccelKd = 0.;
	//Drive Setpoints
	public static int kDriverSetpoint = 1000;
	//Turn Setpoints
	
	//Shooter Setpoints
	public static int kShooterSetpointOne = 0;
	public static int kShooterSetpointTwo = 90;
	//Intake Setpoints
	
	//Vision Setpoints
	public static int kVisionSetpoint = 140;
	//Gyro Setpoints
	
	//
	
	//Joysticks + Axis
	public static int kRazerPort = 0;
	public static int kRazerLeftXAxis = 0;
	public static int kRazerLeftYAxis = 1;
	public static int kRazerRightXAxis = 4;
	public static int kRazerRightYAxis = 5;
	public static int kRazerRightTrigger = 3;
	public static int kRazerLeftTrigger = 2;

	public static int kMadCatzPort = 1;
	public static int kMadCatzXAxis = 0;
	public static int kMadCatzYAxis = 1;
	public static int kMadCatzZAxis = 2;
	public static int kMadCatzZRotate = 3;
	//Motor CAN ID's
	public static int kLeverMotor = 2; //savage
	public static int kAxleMotor = 9;
	public static int kLeftMotorOne = 4;
	public static int kLeftMotorTwo = 5;
	public static int kRightMotorOne = 6;
	public static int kRightMotorTwo = 7;
	public static int kShooterMotorOne = 3;
	public static int kShooterMotorTwo = 8;
	public static int kRollerMotor = 1;
	public static int kLiftMotor = 10;
	//Encoders
	public static int kRightDriveAEncoder = 10;
	public static int kRightDriveBEncoder = 11;
	public static int kLeftDriveAEncoder = 12;
	public static int kLeftDriveBEncoder = 13;
	//Halleffects
	public static int kLeftHallEffect = 5;
	public static int kRightHallEffect = 4;
	//Limit Switch
	public static int kLimitSwitch = 6;
	//Flashlight
	public static int kLightPort = 0;
	//Gyro
	public static int kGyro = 0;
	//Ultrasonic
	public static int kUltrasonicOnePing = 0;
	public static int kUltrasonicOneEcho = 1;
	public static int kUltrasonicTwoPing = 2;
	public static int kUltrasonicTwoEcho = 3;
	public static int kIntakeUltrasonicPing = 8;
	public static int kIntakeUltrasonicEcho = 9;
	//Arduino
	public static int kArduinoOne = 23;
	public static int kArduinoTwo = 24;
	public static int kArduinoThree = 25;
}
