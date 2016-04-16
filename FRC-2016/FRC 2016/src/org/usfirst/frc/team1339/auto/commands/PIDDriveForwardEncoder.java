package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;
import org.usfirst.frc.team1339.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDDriveForwardEncoder extends CommandBase {

	double m_clicks;
	
    public PIDDriveForwardEncoder(double timeout, double clicks) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDChassis);
    	
    	m_clicks = clicks;
    	
    	setTimeout(timeout);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.GyroPID.setPID(Constants.kAutoGyroKp, Constants.kAutoGyroKi, Constants.kAutoGyroKd);
    	HardwareAdapter.RightDriveEncoderPID.setSetpoint(m_clicks+HardwareAdapter.getRightDriveEnc());
    	HardwareAdapter.LeftDriveEncoderPID.setSetpoint(m_clicks+HardwareAdapter.getLeftDriveEnc());
    	HardwareAdapter.GyroPID.setSetpoint(HardwareAdapter.kSpartanGyro.getAngle());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putString("AutoCommand", "Enc");
    	PIDChassis.PIDDriveEncoder();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return HardwareAdapter.RightDriveEncoderPID.onTarget(10)
        		|| HardwareAdapter.LeftDriveEncoderPID.onTarget(10)  || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	PIDChassis.driveForward(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	PIDChassis.driveForward(0);
    }
}
