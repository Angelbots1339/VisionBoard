package org.usfirst.frc.team1339.auto.commandgroup;

import org.usfirst.frc.team1339.auto.commands.Chill;
import org.usfirst.frc.team1339.auto.commands.IngestBallPID;
import org.usfirst.frc.team1339.commands.DriveIntake;
import org.usfirst.frc.team1339.commands.DriveIntakeTimeout;
import org.usfirst.frc.team1339.commands.Ingest;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IngestGroup extends CommandGroup {
    
    public  IngestGroup() {
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
    	addSequential(new Ingest());
    	addSequential(new DriveIntakeTimeout(.4, .3));
    	addSequential(new DriveIntake());
    }
}
