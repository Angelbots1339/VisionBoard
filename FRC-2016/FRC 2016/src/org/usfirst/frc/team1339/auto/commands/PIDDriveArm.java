package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;
import org.usfirst.frc.team1339.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class PIDDriveArm extends CommandBase {
	
	double m_setpoint, speed, counter = 0;
	Joystick stick;
	boolean pid = false;

    public PIDDriveArm(double setpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDArm);
    	
    	m_setpoint = setpoint;
    	
    	//setTimeout(4);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.ArmPID.setSetpoint(m_setpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	stick = HardwareAdapter.getRazerStick();
    	
    	speed = 0;
    	
		speed += (stick.getRawAxis(Constants.kRazerLeftTrigger) * 0.5);
		speed -= stick.getRawAxis(Constants.kRazerRightTrigger);
	
		speed *= 0.7;
		
		if(Math.abs(speed) > 0.1){
			pid = false;
		}
		if(HardwareAdapter.getXButton() && counter >= 10){
			pid = !pid;
			counter = 0;
		}
		if(PIDIntake.ball){
			pid = false;
		}
		counter++;
		if(pid){
			PIDArm.PIDDriveArm();
		}
		else{
			PIDArm.control(speed);
		}
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
