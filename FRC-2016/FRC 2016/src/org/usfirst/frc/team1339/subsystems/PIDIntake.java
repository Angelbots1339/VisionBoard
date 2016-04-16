package org.usfirst.frc.team1339.subsystems;

import org.usfirst.frc.team1339.commands.CommandBase;
import org.usfirst.frc.team1339.commands.DriveIntake;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class PIDIntake extends Subsystem {
	
	public boolean running = false;
	public boolean ball = true;
	
    // Initialize your subsystem here
    public PIDIntake() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void initDefaultCommand() {
    	setDefaultCommand(new DriveIntake());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void intake(double speed) {
    	//if(HardwareAdapter.kIntakeUltrasonic.getRangeInches() < 5)
    	//	ball = true;
    	HardwareAdapter.kAxleMotor.set(speed);
    	SmartDashboard.putBoolean("Ball", ball);
    	if(speed > 0.1 || speed < -0.1){
    		CommandBase.PIDChassis.setArduino(5);
    		running = true;
    		ball = false;
    	}
    	else{
    		CommandBase.PIDChassis.setArduino(1);
    		running = false;
    	}
    }
}
