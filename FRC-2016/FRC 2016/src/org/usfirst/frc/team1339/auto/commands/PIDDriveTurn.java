package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;
import org.usfirst.frc.team1339.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class PIDDriveTurn extends CommandBase {
	
	double m_clicks, m_angle;

    public PIDDriveTurn(double timeout, double clicks, double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDChassis);
    	
    	setTimeout(timeout);
    	m_angle = angle;
    	m_clicks = clicks;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HardwareAdapter.GyroPID.setPID(Constants.kGyroKp, Constants.kGyroKi, Constants.kGyroKd);
    	if(m_angle > 0){
    		HardwareAdapter.LeftDriveEncoderPID.setSetpoint(m_clicks+HardwareAdapter.getLeftDriveEnc());
    	}
    	else{
    		HardwareAdapter.RightDriveEncoderPID.setSetpoint(m_clicks+HardwareAdapter.getRightDriveEnc());
    	}
    	HardwareAdapter.GyroPID.setSetpoint(HardwareAdapter.kSpartanGyro.getAngle() + m_angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putString("AutoCommand", "Gyro");

    	if(m_angle > 0){
    		PIDChassis.PIDDriveEncoderLeft();
    	}
    	else{
    		PIDChassis.PIDDriveEncoderRight();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(m_angle > 0){
    		return HardwareAdapter.LeftDriveEncoderPID.onTarget(10) 
    				|| isTimedOut();
    	}
    	else{
    		return HardwareAdapter.RightDriveEncoderPID.onTarget(10) 
    				|| isTimedOut();
    	}
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
