package org.usfirst.frc.team1339.robot.commands;

import org.usfirst.frc.team1339.robot.OI;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1339.robot.subsystems.*;

/**
 *
 */
public abstract class CommandBase extends Command {

	public static OI oi;
    // Create a single static instance of all of your subsystems
    // "static"  members belong to the class instead of a specific instance, 
    // meaning that you could not need to use the " = new" syntax to use the member
    //public static Chassis chassis = new Chassis();
	//public static PIDChassis PIDChassis = new PIDChassis();
    //public static PrintPot PrintPot = new PrintPot();
	public static Motors Motors = new Motors();
	public static PIDChassis PIDChassis = new PIDChassis();
    
    
    public static void init() {
        // This MUST be here. If the OI creates Commands (which it very likely
        // will), constructing it during the construction of CommandBase (from
        // which commands extend), subsystems are not guaranteed to be
        // yet. Thus, their requires() statements may grab null pointers. Bad
        // news. Don't move it
        
        oi = new OI();

        // Show what command your subsystem is running on the SmartDashboard
        // This is optional
        //SmartDashboard.putData(exampleSubsystem);
    }

    public CommandBase(String name) {
        super(name);
    }

    public CommandBase() {
        super();
        
    }
}
