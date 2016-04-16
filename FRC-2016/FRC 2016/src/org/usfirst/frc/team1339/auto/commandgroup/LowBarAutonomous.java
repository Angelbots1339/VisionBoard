package org.usfirst.frc.team1339.auto.commandgroup;

import org.usfirst.frc.team1339.auto.commands.ArmDown;
import org.usfirst.frc.team1339.auto.commands.DriveForward;
import org.usfirst.frc.team1339.auto.commands.PIDGyro;
import org.usfirst.frc.team1339.auto.commands.PIDShootBallEncoder;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LowBarAutonomous extends CommandGroup {
    
    public  LowBarAutonomous() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	addSequential(new DriveForward(.75, .4));
    	addSequential(new ArmDown(3));
    	addSequential(new DriveForward(.75, 2));
    	//addSequential(new PIDGyro(0.5, -15));
    	//addSequential(new DriveForward(.75, .5));
    	//addSequential(new CorrectVision(3));
    	//addSequential(new ShootBallPID(85));
    }
}
