package org.usfirst.frc.team1339.subsystems;

import org.usfirst.frc.team1339.commands.CommandBase;
import org.usfirst.frc.team1339.commands.DriveArm;
import org.usfirst.frc.team1339.auto.commands.PIDDriveArm;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class PIDArm extends Subsystem {
	
	public boolean armInit;
    // Initialize your subsystem here
    public PIDArm() {
    	armInit = true;
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void initDefaultCommand() {
    	setDefaultCommand(new PIDDriveArm(-1600));
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void control(double speed) {
    	if (!HardwareAdapter.kRightHallEffect.get() || !HardwareAdapter.kLeftHallEffect.get()) {
			HardwareAdapter.kLeverMotor.setEncPosition(0);
    		if(speed > 0){
    			speed = 0;
    		}
    	}
    	if (!HardwareAdapter.kLimitSwitch.get()){
    		if (speed < 0){
    			speed = 0;
    		}
    	}
    	HardwareAdapter.kLeverMotor.set(speed);
    }
    public void PIDDriveArm(){
    	double speed = 0;
    	//if (!CommandBase.PIDIntake.running){
    		speed = HardwareAdapter.ArmPID.calculate(HardwareAdapter.kLeverMotor.getPosition());
    		if (!HardwareAdapter.kRightHallEffect.get() || !HardwareAdapter.kLeftHallEffect.get()) {
    			if(speed > 0){
    				speed = 0;
    			}
    		}
    		if (!HardwareAdapter.kLimitSwitch.get()){
    			if(speed < 0){
    				speed = 0;
    			}
    		//}
    	}
		HardwareAdapter.kLeverMotor.set(speed);
    }
}
