package org.usfirst.frc.team1339.auto.commandgroup;

import org.usfirst.frc.team1339.auto.commands.ArmDown;
import org.usfirst.frc.team1339.auto.commands.Chill;
import org.usfirst.frc.team1339.auto.commands.DriveForward;
import org.usfirst.frc.team1339.auto.commands.PIDDriveForwardEncoder;
import org.usfirst.frc.team1339.auto.commands.PIDDriveShooter;
import org.usfirst.frc.team1339.auto.commands.PIDDriveTurn;
import org.usfirst.frc.team1339.auto.commands.PIDGyro;
import org.usfirst.frc.team1339.commands.CommandBase;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class OneBallAuto extends CommandGroup {
    
    public  OneBallAuto() {
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
        // arm.addSequential(new PIDDriveForwardEncoder(1, 314));
    	addParallel(new ArmDown(2));
    	//addSequential(new PIDDriveForwardEncoder(4.5, 580));
    	addSequential(new PIDDriveForwardEncoder(6, 6735));
    	//addSequential(new PIDGyro(0, 50));
    	//addSequential(new PIDDriveForwardEncoder(2, 170));
    	//addSequential(new PIDDriveTurn(6, 2250, 50));
    	addSequential(new PIDGyro(0, 58));
    	addSequential(new PIDDriveForwardEncoder(3, 2150));

    	addParallel(new PIDDriveShooter(85, true));
    	//addSequential(new CorrectVision(3, false));
    	addSequential(new AutomatedShootBall());
    }
}
