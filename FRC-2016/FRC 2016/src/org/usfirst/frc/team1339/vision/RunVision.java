package org.usfirst.frc.team1339.vision;

import org.usfirst.frc.team1339.auto.commands.PIDGyro;
import org.usfirst.frc.team1339.commands.CommandBase;
import org.usfirst.frc.team1339.lib.utils.VisionVariables;
import org.usfirst.frc.team1339.robot.Constants;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class RunVision extends CommandBase {

	double startTime = 0;
	double turnAngle = 0;
	double m_angle;
	boolean m_auto;
	boolean done = false;
    public RunVision(double timeout, boolean auto) {
        // Use requires() here to declare subsystem dependencies
    	m_auto = auto;
        requires(PIDChassis);
    	setTimeout(timeout);
    	startTime = System.currentTimeMillis();
    	//SmartDashboard.putString("Vision Running", "Running");
    	//if (HardwareAdapter.get2Button()){
    		//PIDChassis.visionRunning = true;
    	//}
 
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    	//System.out.println("CenterX " + Double.toString(VisionVariables.kPIDX[0]) + " " + Double.toString(System.currentTimeMillis() - startTime));
    	PIDChassis.runGetGrip();
    	
    	double pixels = 300;
    	double target = 170;
    	double camAngle = 67; // degrees
    	double degPerPixel = camAngle / pixels;

    	// positive is clockwise
    	turnAngle = HardwareAdapter.kSpartanGyro.getAngle() + (VisionVariables.kPIDX[0] - target) * degPerPixel;

    	HardwareAdapter.TurnGyroPID.setSetpoint(turnAngle);
    	if(m_auto){
    		if(turnAngle > 20 || turnAngle < -20){
    			done = true;
    		}
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	

    	SmartDashboard.putNumber("centerX", VisionVariables.kPIDX[0]);    	

    	SmartDashboard.putNumber("turnAngle", turnAngle);
    	SmartDashboard.putString("AutoCommand", "Vision");

    	PIDChassis.PIDDriveGyro();
    	   	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (isTimedOut() || HardwareAdapter.TurnGyroPID.onTarget(.30) || done);
    }

    // Called once after isFinished returns true
    protected void end() {
    	HardwareAdapter.kLeftMotorOne.set(0);
    	HardwareAdapter.kLeftMotorTwo.set(0);
    	HardwareAdapter.kRightMotorOne.set(0);
    	HardwareAdapter.kRightMotorTwo.set(0);
    	PIDChassis.visionRunning = false;
    	//SmartDashboard.putString("Vision Running", "Not Running");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	HardwareAdapter.kLeftMotorOne.set(0);
    	HardwareAdapter.kLeftMotorTwo.set(0);
    	HardwareAdapter.kRightMotorOne.set(0);
    	HardwareAdapter.kRightMotorTwo.set(0);
    	PIDChassis.visionRunning = false;
    }
}
