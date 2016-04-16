
package org.usfirst.frc.team1339.robot;

import org.usfirst.frc.team1339.auto.commandgroup.IngestGroup;
import org.usfirst.frc.team1339.auto.commandgroup.LowBarAutonomous;
import org.usfirst.frc.team1339.auto.commandgroup.OneBallAuto;
import org.usfirst.frc.team1339.auto.commandgroup.RockWallAutonomous;
import org.usfirst.frc.team1339.auto.commandgroup.SpyBoxAutonomous;
import org.usfirst.frc.team1339.auto.commands.PIDDriveForwardEncoder;
import org.usfirst.frc.team1339.commands.*;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    Command autonomousCommand;
    SendableChooser chooser;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//CommandBase.PIDChassis.setArduino(2);
        chooser = new SendableChooser();
    	CommandBase.PIDChassis.setArduino(0);
    	//autonomousCommand = new RockWallAutonoums();
        autonomousCommand = new OneBallAuto();
        //autonomousCommand = new LowBarAutonomous();
        //autonomousCommand = new RockWallAutonomous();
    	//autonomousCommand = new SpyBoxAutonomous();
        CommandBase.init();
        /*
        SmartDashboard.putData("Auto mode", chooser);
        SmartDashboard.putNumber("speed", 0);
        
        CommandBase.PIDChassis.publishSmartDashboard();
        SmartDashboard.putNumber("Shooter Encoder Test", HardwareAdapter.kShooterMotorOne.getEncPosition());
        */

        SmartDashboard.putBoolean("Low Bar", false);
        SmartDashboard.putBoolean("One Ball", false);
        SmartDashboard.putBoolean("Spy Box", false);
        SmartDashboard.putBoolean("Rock Wall", false);     
        HardwareAdapter.kShooterMotorOne.setPosition(0);
        HardwareAdapter.ArmPID.setSetpoint(0);
        HardwareAdapter.ShooterPID.setSetpoint(85);
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){
    	CommandBase.PIDChassis.setArduino(0);
		HardwareAdapter.kRightDriveEncoder.reset();
		HardwareAdapter.kLeftDriveEncoder.reset();
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		/*
		HardwareAdapter.kRightDriveEncoder.reset();
		HardwareAdapter.kLeftDriveEncoder.reset();
		SmartDashboard.putNumber("Right Encoder Disabled", HardwareAdapter.getRightDriveEnc());
        SmartDashboard.putNumber("Left Encoder Disabled", HardwareAdapter.getLeftDriveEnc());
        SmartDashboard.putNumber("Shooter RPS", CommandBase.PIDShooter.getEncRate());*/
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
        //autonomousCommand = (Command) chooser.getSelected();
		CommandBase.PIDChassis.setArduino(4);
		HardwareAdapter.kSpartanGyro.reset();
		SmartDashboard.putString("IntakeCommand", "not running");
		/*if(SmartDashboard.getBoolean("One Ball", false)){
			autonomousCommand = new OneBallAuto();
		}
		else if(SmartDashboard.getBoolean("Spy Box", false)){
			autonomousCommand = new SpyBoxAutonomous();
		}
		else if(SmartDashboard.getBoolean("Rock Wall", false)){
			autonomousCommand = new RockWallAutonomous();
		}
		else {
			autonomousCommand = new LowBarAutonomous();
		}*/
    	// schedule the autonomous command (example)*/
		//autonomousCommand = new LowBarAutonomous();
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        SmartDashboard.putData("PIDChassis", CommandBase.PIDChassis);
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    	CommandBase.PIDChassis.setArduino(1);
    	HardwareAdapter.resetDriveEncoder();
    	HardwareAdapter.kLeverMotor.setPosition(0);
    	CommandBase.PIDArm.armInit = true;
    	CommandBase.PIDIntake.ball = false;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        CommandBase.PIDChassis.publishSmartDashboard();
        SmartDashboard.putData("PIDDrive", new PIDDriveForwardEncoder(10, 2000));
        SmartDashboard.putNumber("Right Encoder", HardwareAdapter.getRightDriveEnc());
        SmartDashboard.putNumber("Left Encoder", HardwareAdapter.getLeftDriveEnc());
        SmartDashboard.putNumber("Shooter RPS", CommandBase.PIDShooter.getEncRate());
        SmartDashboard.putNumber("Ultra 1", HardwareAdapter.kUltrasonicOne.getRangeInches());
        SmartDashboard.putNumber("Ultra 2", HardwareAdapter.kUltrasonicTwo.getRangeInches());
        SmartDashboard.putData("PIDArm", CommandBase.PIDArm);
        SmartDashboard.putBoolean("Arm Init", CommandBase.PIDArm.armInit);
        SmartDashboard.putNumber("Current", HardwareAdapter.kLeverMotor.getOutputCurrent());
        //SmartDashboard.putNumber("Shooter Accel", CommandBase.PIDShooter.getAcceleration());
        SmartDashboard.putNumber("jerk", CommandBase.PIDShooter.PIDJerk());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
