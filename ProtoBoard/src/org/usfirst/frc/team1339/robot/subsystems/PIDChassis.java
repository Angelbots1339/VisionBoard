package org.usfirst.frc.team1339.robot.subsystems;

import org.usfirst.frc.team1339.robot.RobotMap;
import org.usfirst.frc.team1339.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team1339.robot.commands.RunVision;

import com.ni.vision.NIVision.SettingType;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDChassis extends PIDSubsystem {
	private static final double Kp = 0.05;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;
    NetworkTable table = NetworkTable.getTable("/GRIP/myContoursReport");
    double[] defaultValue = new double[0];
    //double[] PIDX = {0},
    double[] PIDY = {0};
    CANTalon motor4;

    
    double SETpoint = 140;
    
    
    // Initialize your subsystem here
    public PIDChassis() {
    	super("PIDChassis", Kp, Ki, Kd);
    	
    	motor4 = new CANTalon(RobotMap.motorFourPort);
    	
    	
    	setSetpoint(SETpoint);
    	//setAbsoluteTolerance(100);
    	
    	// Vision
    	//double[] PIDX = new double[0];
    	//double[] PIDY = new double[0];		
    	
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        enable(); //- Enables the PID controller.
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new RunVision());
    }
    
    public void runGetGrip(){
    	
    	double[] centerXs = table.getNumberArray("centerX", defaultValue);
		for (double centerX : centerXs) {
			System.out.println(centerX + " ");
			SmartDashboard.putNumber("CenterX", centerX);
			//PIDX[0] = centerX;
		}
		
		double[] centerYs = table.getNumberArray("centerY", defaultValue);
		for (double centerY : centerYs) {
			System.out.println(centerY + " ");
			SmartDashboard.putNumber("CenterY", centerY);
			PIDY[0] =  centerY;
		}
		
		;
		Timer.delay(1);
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	//SmartDashboard.putNumber("X", centerX);
    	double[] centerXs = table.getNumberArray("centerX", defaultValue);
    	double PIDX = 0;
		for (double centerX : centerXs) {
			SmartDashboard.putNumber("X", centerX);
			PIDX = centerX;
			SmartDashboard.putNumber("PIDX", PIDX);
			//return PIDX;
		}
		if(Math.abs(SETpoint-PIDX)<10){
			PIDX = SETpoint;
		}
		
    	return PIDX;
		
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        motor4.set(output);
    }
}
