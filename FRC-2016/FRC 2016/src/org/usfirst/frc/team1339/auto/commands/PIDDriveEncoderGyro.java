package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;


/**
 *
 */
public class PIDDriveEncoderGyro extends CommandBase {

	double m_angle;
	int m_right;
	int m_left;
	
    public PIDDriveEncoderGyro(double angle , int rightClicks , int leftClicks , double timeout) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDChassis);
    	
    	setTimeout(timeout);
    	m_angle = angle;
    	m_right = rightClicks;
    	m_left = leftClicks;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.resetGyro();
    	HardwareAdapter.resetDriveEncoder();
    	HardwareAdapter.TurnGyroPID.setSetpoint(m_angle);
    	HardwareAdapter.LeftDriveEncoderPID.setSetpoint(m_left);
    	HardwareAdapter.RightDriveEncoderPID.setSetpoint(m_right);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PIDChassis.PIDDriveEncoder();
    	PIDChassis.PIDDriveGyro();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return HardwareAdapter.RightDriveEncoderPID.onTarget(10)
        		|| HardwareAdapter.LeftDriveEncoderPID.onTarget(10) 
        		|| HardwareAdapter.TurnGyroPID.onTarget(.5)
        		|| isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
