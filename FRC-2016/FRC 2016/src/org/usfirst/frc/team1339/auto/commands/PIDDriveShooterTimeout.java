package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDDriveShooterTimeout extends CommandBase {

	double RPSsetpoint;
	boolean auto;
	
	
    public PIDDriveShooterTimeout(double setpoint, boolean autonomous, double timeout) {
        // Use requires() here to declare subsystem dependencies
    	requires(PIDShooter);
    	setTimeout(timeout);
    	RPSsetpoint = setpoint;
    	auto = autonomous;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.ShooterPID.setSetpoint(RPSsetpoint);
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
    		done = !HardwareAdapter.get1Button();
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
