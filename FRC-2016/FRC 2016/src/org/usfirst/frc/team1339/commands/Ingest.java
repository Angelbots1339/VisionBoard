package org.usfirst.frc.team1339.commands;

/**
 *
 */
public class Ingest extends CommandBase {
	
	private int counter = 0;
	
    public Ingest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDIntake);
    	setTimeout(4);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PIDIntake.intake(-.8);
    	if(HardwareAdapter.kIntakeUltrasonic.getRangeInches() > 5){
    		counter = 0;
    	}
    	else{
    		counter++;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return counter >= 3 || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
