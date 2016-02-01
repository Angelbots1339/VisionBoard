package org.usfirst.frc.team1339.robot.commands;

import org.usfirst.frc.team1339.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class DriveWithJoystick extends CommandBase {
	
	Joystick stick;
	double speed1, speed2, speed3;

    public DriveWithJoystick() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Motors);
    	requires(PIDChassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	PIDChassis.gyroVision();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	stick = oi.getMadCatz();
    	
    	speed1 = stick.getRawAxis(RobotMap.madCatzY);
    	speed2 = stick.getRawAxis(RobotMap.madCatzX);
    	speed3 = stick.getRawAxis(RobotMap.madCatzZ);
    	
    	if(speed1 <= .05 && speed1 >= -.05)
    		speed1 = 0;
    	if(speed2 <= .05 && speed2 >= -.05)
    		speed2 = 0;
    	if(speed3 <= .05 && speed3 >= -.05)
    		speed3 = 0;
    	
    	Motors.joystickDrive(speed1, speed2, speed3);
    	
    	PIDChassis.gyroOffset();
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
