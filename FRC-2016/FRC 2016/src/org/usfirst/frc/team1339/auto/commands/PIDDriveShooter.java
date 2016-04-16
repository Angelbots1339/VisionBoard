package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDDriveShooter extends CommandBase {

	double RPSsetpoint;
	boolean auto;
	
	
    public PIDDriveShooter(double setpoint, boolean autonomous) {
        // Use requires() here to declare subsystem dependencies
    	requires(PIDShooter);
    	if(autonomous){
        	setTimeout(10);
    	}
    	RPSsetpoint = setpoint;
    	auto = autonomous;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.ShooterPID.setSetpoint(RPSsetpoint);
    	HardwareAdapter.AccelPID.setSetpoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putString("ShooterCommand", "Shooter");
    	PIDShooter.PIDShoot();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean done = false;
    	if(!auto){
    		done = !HardwareAdapter.get7Button();
    	}
    	else{
    		done = isTimedOut();
    	}
        return done;
    }

    // Called once after isFinished returns true
    protected void end() {
    	PIDShooter.shootBall(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	PIDShooter.shootBall(0);
    }
}
