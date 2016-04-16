package org.usfirst.frc.team1339.auto.commands;

import org.usfirst.frc.team1339.commands.CommandBase;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class PushBallIntoShooter extends CommandBase {
	
	double m_speed;
	boolean timeout = false;
	double timeoutTime;

    public PushBallIntoShooter(double speed, double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDIntake);
    	
    	m_speed = speed;
    	timeoutTime = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PIDIntake.intake(m_speed);
    	Timer.delay(timeoutTime);
    	timeout = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timeout;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
