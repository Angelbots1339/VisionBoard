package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.lib.utils.EncoderConversion;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1339.subsystems.*;
/**
 *
 */

//@author Chad is a nerd
//Test 

public abstract class CommandBase extends Command {

	public static HardwareAdapter HardwareAdapter;
	public static EncoderConversion EncConv;
    // Create a single static instance of all of your subsystems
    // "static"  members belong to the class instead of a specific instance, 
    // meaning that you could not need to use the " = new" syntax to use the member
	
    public static PIDChassis PIDChassis = new PIDChassis();
    public static PIDIntake PIDIntake = new PIDIntake();
    public static PIDShooter PIDShooter = new PIDShooter();
    public static PIDArm PIDArm = new PIDArm();
    public static Flashlight Light = new Flashlight();

    
    public static void init() {
        // This MUST be here. If the OI creates Commands (which it very likely
        // will), constructing it during the construction of CommandBase (from
        // which commands extend), subsystems are not guaranteed to be
        // yet. Thus, their requires() statements may grab null pointers. Bad
        // news. Don't move it
        
        HardwareAdapter = new HardwareAdapter();
        EncConv = new EncoderConversion();
        // Show what command your subsystem is running on the SmartDashboard
        // This is optional
        
    }

    public CommandBase(String name) {
        super(name);
    }

    public CommandBase() {
        super();
    }
}