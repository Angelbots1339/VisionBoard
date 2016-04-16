package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.lib.utils.VisionVariables;

/**
 *
 */
public class DriveShooterVision extends CommandBase {
	
	
	double area = 0;
	double scaledSpeed = 0;
	
    public DriveShooterVision() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDChassis);
    	requires(PIDShooter);
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	area = VisionVariables.kArea[0];
    	scaledSpeed = area*1;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	PIDShooter.shootBall(scaledSpeed);
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
