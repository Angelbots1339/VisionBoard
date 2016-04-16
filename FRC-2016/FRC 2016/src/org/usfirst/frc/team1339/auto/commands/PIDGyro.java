package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;

/**
 *
 */
public class PIDGyro extends CommandBase {
	double m_angle;
	double m_speed;
	
	
    public PIDGyro(double speed , double angle) {
        // Use requires() here to declare subsystem dependencies
    	requires(PIDChassis);
    	m_angle = HardwareAdapter.kSpartanGyro.getAngle() + angle;
    	m_speed = speed;
    	setTimeout(2.5);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.TurnGyroPID.setSetpoint(m_angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PIDChassis.PIDDriveGyro();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (HardwareAdapter.TurnGyroPID.onTarget(1) || isTimedOut());
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
