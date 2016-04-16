package org.usfirst.frc.team1339.auto.commandgroup;

import org.usfirst.frc.team1339.auto.commands.PIDDriveForwardEncoder;
import org.usfirst.frc.team1339.auto.commands.PIDDriveShooter;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MiddleOneBall extends CommandGroup {
    
    public  MiddleOneBall() {
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
    	addParallel(new PIDDriveShooter(85, true));
    	addSequential(new PIDDriveForwardEncoder(10, 6235));
    	addSequential(new CorrectVision(4, true));
    	addSequential(new AutomatedShootBall());
    	//addSequential(new PIDDriveForwardEncoder(10, -6035));
    }
}
