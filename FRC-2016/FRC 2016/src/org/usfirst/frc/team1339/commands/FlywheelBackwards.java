package org.usfirst.frc.team1339.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FlywheelBackwards extends CommandBase {

    public FlywheelBackwards() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDShooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (HardwareAdapter.get3Button()){
    		PIDShooter.shootBall(-.5);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
