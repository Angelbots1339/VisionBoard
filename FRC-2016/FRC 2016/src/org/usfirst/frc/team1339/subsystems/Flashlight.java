package org.usfirst.frc.team1339.subsystems;

import org.usfirst.frc.team1339.commands.ControlFlashlight;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Flashlight extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new ControlFlashlight(true));
    }
    
    public void setLightOn(boolean on){
    	if(on){
    		HardwareAdapter.kLight.set(Relay.Value.kForward);
    	}
    	else {
    		HardwareAdapter.kLight.set(Relay.Value.kOff);
    	}
    }
}

