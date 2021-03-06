package org.usfirst.frc.team1339.robot.commands;


/**
 *
 */
public class GyroVision extends CommandBase {

    public GyroVision() {
        // Use requires() here to declare subsystem dependencies
        requires(PIDChassis);
        
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	PIDChassis.gyroVision();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PIDChassis.gyroOffset();
    	System.out.println("crap");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	PIDChassis.resetConditionalPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
