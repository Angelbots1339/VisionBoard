package org.usfirst.frc.team1339.commands;

/**
 *
 */
public class ControlFlashlight extends CommandBase {

	boolean m_on;
	
    public ControlFlashlight(boolean on) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Light);
    	
    	m_on = on;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Light.setLightOn(m_on);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
