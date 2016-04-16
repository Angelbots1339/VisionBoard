package org.usfirst.frc.team1339.commands;


import org.usfirst.frc.team1339.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class DriveWithArcadeJoystick extends CommandBase {
	private Joystick arcadeJoystick;
	private double throttle;
	private double turn;
	private boolean down;
	private boolean GTA = false;
	private double GTASpeed = 0;
	private int cooldown = 10;
	private double GTATurn;
	

    public DriveWithArcadeJoystick() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(PIDChassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	arcadeJoystick = HardwareAdapter.getRazerStick();
    	cooldown++;
    	
    	
    	if(arcadeJoystick.getPOV() <= 225 && arcadeJoystick.getPOV() >= 135 && HardwareAdapter.getLeftStickButton() && cooldown >= 10){
    		down = true;
    		cooldown = 0;
    		System.out.println("Lets rek some shit");
    	}
    	else {
    		down = false;
    	}
    	
    	if(down){
    		GTA = !GTA;
    	}
    	
    	throttle = arcadeJoystick.getRawAxis(Constants.kRazerLeftYAxis);
    	turn = arcadeJoystick.getRawAxis(Constants.kRazerRightXAxis);
    	GTATurn = arcadeJoystick.getRawAxis(Constants.kRazerRightXAxis);
    	
    	if(arcadeJoystick.getRawAxis(Constants.kRazerRightTrigger) >= .5){
    		GTASpeed = (arcadeJoystick.getRawAxis(Constants.kRazerLeftYAxis) + 1) / 2;
    	}
    	else if(arcadeJoystick.getRawAxis(Constants.kRazerLeftTrigger) >= .5){
    		GTASpeed = (arcadeJoystick.getRawAxis(Constants.kRazerLeftYAxis) - 1) / -2;
    	}
    	else{
    		GTASpeed = 0;
    	}
    	
    	if(GTA){
    		PIDChassis.driveWithJoystick(GTASpeed, GTATurn);
    	}
    	if (GTA == false){
    		throttle = throttle*throttle*throttle;
        	turn = turn*turn*turn;
        	
        	
        	PIDChassis.driveWithJoystick(throttle, turn);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	PIDChassis.setMotorValues(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	PIDChassis.setMotorValues(0, 0);
    }
}
