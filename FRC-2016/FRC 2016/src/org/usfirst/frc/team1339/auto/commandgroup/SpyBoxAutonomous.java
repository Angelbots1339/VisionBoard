package org.usfirst.frc.team1339.auto.commandgroup;

import org.usfirst.frc.team1339.auto.commands.ArmDown;
import org.usfirst.frc.team1339.auto.commands.DriveForward;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SpyBoxAutonomous extends CommandGroup {
    
    public  SpyBoxAutonomous() {
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
    	addSequential(new DriveForward(.5 , 1));
    	addSequential(new CorrectVision(3, true));
    	addSequential(new ShootBallPID(85, true));
    }
}
