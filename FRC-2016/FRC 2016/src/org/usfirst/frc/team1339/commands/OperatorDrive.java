package org.usfirst.frc.team1339.commands;


import org.usfirst.frc.team1339.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class OperatorDrive extends CommandBase {
	private Joystick arcadeJoystick;
	private double turn;
	private double throttle;
	

    public OperatorDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDChassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	arcadeJoystick = HardwareAdapter.getMadCatz();
    	
    	turn = arcadeJoystick.getRawAxis(Constants.kMadCatzXAxis);
    	throttle = arcadeJoystick.getRawAxis(Constants.kMadCatzYAxis);
    	
    	turn = turn*turn*turn*turn*turn;
    	throttle = throttle * throttle * throttle;
    	
    	PIDChassis.driveWithJoystick(throttle, turn);
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
