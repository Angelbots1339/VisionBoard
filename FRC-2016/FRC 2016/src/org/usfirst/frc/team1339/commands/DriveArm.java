package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveArm extends CommandBase {
	double speed;
	Joystick stick;
	double counter = 0;

    public DriveArm() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDArm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	stick = HardwareAdapter.getRazerStick();
    	
    	speed = 0;
    	
		speed += (stick.getRawAxis(Constants.kRazerLeftTrigger) * 0.5);
		speed -= stick.getRawAxis(Constants.kRazerRightTrigger);
	
		speed *= 0.7;
    	
    	/*if(speed >= -0.05 && speed <= 0.05 && !PIDArm.armInit){
    		if(counter == 0){
    			HardwareAdapter.ArmPID.setSetpoint(HardwareAdapter.kLeverMotor.getPosition());
    			SmartDashboard.putNumber("PIDArm Setpoint", HardwareAdapter.kLeverMotor.getPosition());
    		}
    		PIDArm.stayStill();
    		SmartDashboard.putBoolean("ARMPID", true);
    		counter++;
    	}
    	if (speed >= 0.05 && speed <= -0.05){
    		PIDArm.armInit = false;
    	}
    	else{
    		PIDArm.control(speed);
    		SmartDashboard.putBoolean("ARMPID", false);
    		counter = 0;
    	}
    	*/
		PIDArm.control(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
