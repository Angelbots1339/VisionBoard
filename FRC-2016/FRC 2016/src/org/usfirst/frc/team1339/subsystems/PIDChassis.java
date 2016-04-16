package org.usfirst.frc.team1339.subsystems;

import org.usfirst.frc.team1339.robot.Constants;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import java.security.acl.LastOwnerException;

import org.usfirst.frc.team1339.commands.*;
import org.usfirst.frc.team1339.lib.utils.AngelMath;
import org.usfirst.frc.team1339.lib.utils.VisionVariables;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDChassis extends Subsystem {
	
	public boolean omniDown = false;
	public boolean visionRunning = false;
	
	NetworkTable table = NetworkTable.getTable("/GRIP/myContoursReport");
	double leftLastSpeed, rightLastSpeed;
	double rightSpeed, leftSpeed;
	double rate = 0.085;
	
    // Initialize your subsystem here
    public PIDChassis() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    	
    	VisionVariables.kGyroInitialValue = 0;
    	VisionVariables.kDifference = 0;
    	VisionVariables.kGyroSetpoint = 0;
    }
    
    public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithArcadeJoystick());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void driveWithJoystick(double throttle, double turn){
       	double right = throttle;
    	double left = throttle;
    	double turningThrottleScale;
    	
    	if (java.lang.Math.abs(throttle) > 0.1) {
            turningThrottleScale = java.lang.Math.abs(throttle);
        }
        else{
        	turningThrottleScale = 0.75;
        }
    	turningThrottleScale *= 1.5;
    	
    	right += turn * turningThrottleScale;  
        left -= turn * turningThrottleScale;
        
        if(Math.abs(right) <= 0.05)
        	right = 0;
        if(Math.abs(left) <= 0.05)
        	left = 0;
        
        if(left >= leftLastSpeed + rate)
        	leftSpeed = leftLastSpeed + rate;
        else if (left <= leftLastSpeed - rate)
        	leftSpeed = leftLastSpeed - rate;
        else
        	leftSpeed = left;
        
        if(right >= rightLastSpeed + rate)
        	rightSpeed = rightLastSpeed + rate;
        else if (right <= rightLastSpeed - rate)
        	rightSpeed = rightLastSpeed - rate;
        else
        	rightSpeed = right;
        
        
        setMotorValues(leftSpeed, rightSpeed);
        
        leftLastSpeed = leftSpeed;
        rightLastSpeed = rightSpeed;
        //Arcade Drive with Throttle
    }
    
    
    public void driveForward(double speed){
    	HardwareAdapter.kLeftMotorOne.set(-speed);
    	HardwareAdapter.kLeftMotorTwo.set(speed);
    	HardwareAdapter.kRightMotorOne.set(-speed);
    	HardwareAdapter.kRightMotorTwo.set(speed);
    	//Simple drive forward command
    }
    
    public void setMotorValues(double left, double right){
    	HardwareAdapter.kLeftMotorOne.set(left);
    	HardwareAdapter.kLeftMotorTwo.set(-left);
    	HardwareAdapter.kRightMotorOne.set(right);
    	HardwareAdapter.kRightMotorTwo.set(-right);
    	//
    }

    public void PIDDriveGyro(){
    	double speed = HardwareAdapter.TurnGyroPID.calculate(HardwareAdapter.kSpartanGyro.getAngle());
    	if (Math.abs(speed) > 0.4){
    		if (speed < 0)
    			speed = -0.4;
    		else
    			speed = 0.4;
    	}
    	HardwareAdapter.kLeftMotorOne.set(-speed);
    	HardwareAdapter.kLeftMotorTwo.set(speed);
    	HardwareAdapter.kRightMotorOne.set(speed);
    	HardwareAdapter.kRightMotorTwo.set(-speed);
    	SmartDashboard.putNumber("Vision PID", speed);
    	
    	//Driving with only gyro using PID
    }
    public void PIDDriveEncoder(){
    	//double enc = (HardwareAdapter.getRightDriveEnc() + HardwareAdapter.getLeftDriveEnc())/2;
    	double rightSpeed = HardwareAdapter.RightDriveEncoderPID.calculate(HardwareAdapter.getRightDriveEnc());
    	double leftSpeed = HardwareAdapter.LeftDriveEncoderPID.calculate(HardwareAdapter.getLeftDriveEnc());
    	double gyroOutput = HardwareAdapter.GyroPID.calculate(HardwareAdapter.kSpartanGyro.getAngle());
    	rightSpeed -= gyroOutput;
    	leftSpeed += gyroOutput;
    	rightSpeed *= 0.5;
    	leftSpeed *= 0.5;
    	SmartDashboard.putNumber("Right PID Output", rightSpeed);
    	SmartDashboard.putNumber("Left PID Output", leftSpeed);
    	HardwareAdapter.kLeftMotorOne.set(-leftSpeed);
    	HardwareAdapter.kLeftMotorTwo.set(leftSpeed);
    	HardwareAdapter.kRightMotorOne.set(-rightSpeed);
    	HardwareAdapter.kRightMotorTwo.set(rightSpeed);
    	//Driving with only encoder using PID
    }
    //For turning left w/ the right encoder
public void PIDDriveEncoderRight(){
    	//double enc = (HardwareAdapter.getRightDriveEnc() + HardwareAdapter.getLeftDriveEnc())/2;
    	double rightSpeed = HardwareAdapter.RightDriveEncoderPID.calculate(HardwareAdapter.getRightDriveEnc());
    	double leftSpeed = rightSpeed;
    	double gyroOutput = HardwareAdapter.GyroPID.calculate(HardwareAdapter.kSpartanGyro.getAngle());
    	rightSpeed -= gyroOutput;
    	leftSpeed += gyroOutput;
    	rightSpeed *= 0.5;
    	leftSpeed *= 0.5;
    	SmartDashboard.putNumber("Right PID Output", rightSpeed);
    	SmartDashboard.putNumber("Left PID Output", leftSpeed);
    	HardwareAdapter.kLeftMotorOne.set(-leftSpeed);
    	HardwareAdapter.kLeftMotorTwo.set(leftSpeed);
    	HardwareAdapter.kRightMotorOne.set(-rightSpeed);
    	HardwareAdapter.kRightMotorTwo.set(rightSpeed);
    	//Driving with only encoder using PID
    }
    //For turning right w/ the left encoder
    public void PIDDriveEncoderLeft(){
    	//double enc = (HardwareAdapter.getRightDriveEnc() + HardwareAdapter.getLeftDriveEnc())/2;
    	double leftSpeed = HardwareAdapter.LeftDriveEncoderPID.calculate(HardwareAdapter.getLeftDriveEnc());
    	double rightSpeed = leftSpeed;
    	double gyroOutput = HardwareAdapter.GyroPID.calculate(HardwareAdapter.kSpartanGyro.getAngle());
    	rightSpeed -= gyroOutput;
    	leftSpeed += gyroOutput;
    	rightSpeed *= 0.5;
    	leftSpeed *= 0.5;
    	SmartDashboard.putNumber("Right PID Output", rightSpeed);
    	SmartDashboard.putNumber("Left PID Output", leftSpeed);
    	HardwareAdapter.kLeftMotorOne.set(-leftSpeed);
    	HardwareAdapter.kLeftMotorTwo.set(leftSpeed);
    	HardwareAdapter.kRightMotorOne.set(-rightSpeed);
    	HardwareAdapter.kRightMotorTwo.set(rightSpeed);
    	//Driving with only encoder using PID
    }
    public void PIDDriveEncoderGyro(double speed){
    	double input = (HardwareAdapter.getLeftDriveEnc()+ 
    		HardwareAdapter.getRightDriveEnc())/2;
    	//HardwareAdapter.DriveEncoderPID.setSetpoint(Constants.kDriverSetpoint);
    	HardwareAdapter.GyroPID.setSetpoint(0);
    	HardwareAdapter.kLeftMotorOne.set(speed);
    	HardwareAdapter.kLeftMotorTwo.set(-speed);
    	HardwareAdapter.kRightMotorOne.set(speed);
    	HardwareAdapter.kRightMotorTwo.set(-speed);
    	//HardwareAdapter.DriveEncoderPID.calculate(input);
    	HardwareAdapter.GyroPID.calculate(HardwareAdapter.kGyro.getAngle());
    	//Driving with encoder and gyro using PID
    }
    	
    //Vision
    public void runGetGrip(){
    	double xVal = 0;
		double[] centerXs = table.getNumberArray("centerX", VisionVariables.kDefaultValue);
		for (double centerX : centerXs) {
			System.out.println(centerX + " ");
			//SmartDashboard.putNumber("CenterX", centerX);
			VisionVariables.kPIDX[0] =  centerX;
			xVal = centerX;
			//Starting GRIP on raspberry pi
		}
		Timer.delay(1);
		//Using PID to correct vision
		SmartDashboard.putNumber("GRIP x-val", xVal);
    }
    
    public void calculateSpeedVision(){
    	double[] areas = table.getNumberArray("area", VisionVariables.kDefaultValue);
		for (double area : areas) {
			System.out.println(area + " ");
			//SmartDashboard.putNumber("CenterX", centerX);
			VisionVariables.kArea[0] =  area;
			
			//Starting GRIP on raspberry pi
		}
    }
    
    public void distanceCalculator(){
    	VisionVariables.kDistanceToGoal = 0;
    	double[] lengths = table.getNumberArray("length", VisionVariables.kDefaultValue);
		for (double length : lengths) {
			VisionVariables.kTPixel = length;
		//
		}
		VisionVariables.kFOVPixel = 300;
    	VisionVariables.kDistanceToGoal = (20/12)*VisionVariables.kFOVPixel/
    	(2*VisionVariables.kTPixel*(Math.tan(49)));
    	//
    }
    
    public void scaledVision(){
    	VisionVariables.kScaledDistance = 0;
    	double[] areas = table.getNumberArray("area", VisionVariables.kDefaultValue);
		for (double area : areas) {
			VisionVariables.kScaledDistance = area;
		//
		}
		VisionVariables.kScaledDistance = VisionVariables.kScaledDistanceArea*0.2;
		//
    }
    
    public void gyroVision(){
    	double[] centerXs = table.getNumberArray("centerX", VisionVariables.kDefaultValue);
    	double offsetX = 0;
		for (double centerX : centerXs) {
			offsetX = centerX;
		//
		}
		VisionVariables.kGyroInitialValue = HardwareAdapter.kGyro.getAngle();
    	VisionVariables.kDifference = Constants.kVisionSetpoint-offsetX;
    	VisionVariables.kGyroSetpoint = VisionVariables.kGyroInitialValue
    			+(VisionVariables.kDifference*VisionVariables.kScaledDistance);//MIGHT NOT WORK. NEEDS TESTING 
    	HardwareAdapter.GyroPID.setSetpoint(VisionVariables.kGyroSetpoint);
    	//Using PID with gyro to correct vision
    }
    
    public void gyroOffset(){
    	VisionVariables.kGyroUpdatingValue = HardwareAdapter.kGyro.getAngle();
    	SmartDashboard.putNumber("Gyro Updating Val", VisionVariables.kGyroUpdatingValue);
    	VisionVariables.kGyroDifference = VisionVariables.kGyroSetpoint
    			- VisionVariables.kGyroUpdatingValue;
    	//
    }
    
    public void calibrateGyro(){
    	HardwareAdapter.kGyro.calibrate();
    	//Calibrating gyro
    }
    
    public void publishSmartDashboard(){
    	SmartDashboard.putNumber("Gyro Radians", AngelMath.boundAngle0to2PiRadians(HardwareAdapter.kGyro.getAngle()));
    	SmartDashboard.putNumber("Gyro Degrees", AngelMath.boundAngle0to360Degrees(HardwareAdapter.kGyro.getAngle()));
    	SmartDashboard.putNumber("Spartan Gyro Radians", AngelMath.boundAngle0to2PiRadians(HardwareAdapter.kSpartanGyro.getAngle()));
    	SmartDashboard.putNumber("Spartan Gyro Degrees", (HardwareAdapter.kSpartanGyro.getAngle()));
    	SmartDashboard.putNumber("Shooter Encoder Test", HardwareAdapter.kShooterMotorOne.getPosition()/4000);
    	SmartDashboard.putNumber("Lever Encoder Test", HardwareAdapter.kLeverMotor.getPosition());
    }
    //Arduino code
    //0 = Disabled
    //1 = Teleop
    //2 = Startup
    //3 = shooter on
    //4 = Autonomous
    //5 = Intake on
    public void setArduino(int num){
    	switch(num){
    	case(0): {
    		HardwareAdapter.kArduinoOne.set(false);
    		HardwareAdapter.kArduinoTwo.set(false);
    		HardwareAdapter.kArduinoThree.set(false);
    		SmartDashboard.putBoolean("Arduino 1", false);
    		SmartDashboard.putBoolean("Arduino 2", false);
    		SmartDashboard.putBoolean("Arduino 3", false);
    	}
    	case(1): {
    		HardwareAdapter.kArduinoOne.set(true);
    		HardwareAdapter.kArduinoTwo.set(false);
    		HardwareAdapter.kArduinoThree.set(false);
    		SmartDashboard.putBoolean("Arduino 1", true);
    		SmartDashboard.putBoolean("Arduino 2", false);
    		SmartDashboard.putBoolean("Arduino 3", false);
    	}
    	case(2): {
    		HardwareAdapter.kArduinoOne.set(false);
    		HardwareAdapter.kArduinoTwo.set(true);
    		HardwareAdapter.kArduinoThree.set(false);
    		SmartDashboard.putBoolean("Arduino 1", false);
    		SmartDashboard.putBoolean("Arduino 2", true);
    		SmartDashboard.putBoolean("Arduino 3", false);
    	}
    	case(3): {
    		HardwareAdapter.kArduinoOne.set(true);
    		HardwareAdapter.kArduinoTwo.set(true);
    		HardwareAdapter.kArduinoThree.set(false);
    		SmartDashboard.putBoolean("Arduino 1", true);
    		SmartDashboard.putBoolean("Arduino 2", true);
    		SmartDashboard.putBoolean("Arduino 3", false);
    	}
    	case(4): {
    		HardwareAdapter.kArduinoOne.set(false);
    		HardwareAdapter.kArduinoTwo.set(false);
    		HardwareAdapter.kArduinoThree.set(true);
    		SmartDashboard.putBoolean("Arduino 1", false);
    		SmartDashboard.putBoolean("Arduino 2", false);
    		SmartDashboard.putBoolean("Arduino 3", true);
    	}
    	case(5): {
    		HardwareAdapter.kArduinoOne.set(true);
    		HardwareAdapter.kArduinoTwo.set(false);
    		HardwareAdapter.kArduinoThree.set(true);
    		SmartDashboard.putBoolean("Arduino 1", true);
    		SmartDashboard.putBoolean("Arduino 2", false);
    		SmartDashboard.putBoolean("Arduino 3", true);
    	}
    	case(6): {
    		HardwareAdapter.kArduinoOne.set(false);
    		HardwareAdapter.kArduinoTwo.set(true);
    		HardwareAdapter.kArduinoThree.set(true);
    		SmartDashboard.putBoolean("Arduino 1", false);
    		SmartDashboard.putBoolean("Arduino 2", true);
    		SmartDashboard.putBoolean("Arduino 3", true);
    	}
    	case(7): { 
    		HardwareAdapter.kArduinoOne.set(true);
    		HardwareAdapter.kArduinoTwo.set(true);
    		HardwareAdapter.kArduinoThree.set(true);
    		SmartDashboard.putBoolean("Arduino 1", true);
    		SmartDashboard.putBoolean("Arduino 2", true);
    		SmartDashboard.putBoolean("Arduino 3", true);
    		break;
    	}
    	}
    }
}
