package org.usfirst.frc.team1339.robot.commands;


/**
 *
 */
public class DriveWithButtons extends CommandBase {
	
	public double speed;

    public DriveWithButtons() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Motors);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(oi.getTwenty())
    		speed = .75;
    	else if(oi.getForty())
    		speed = .78;
    	else if(oi.getSixty())
    		speed = .81;
    	else if(oi.getEighty())
    		speed = .92;
    	else if(oi.getHundred())
    		speed = .95;
    	
    	speed *= -1;
    	
    	Motors.joystickDrive(speed, 0, 0);
    	
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
