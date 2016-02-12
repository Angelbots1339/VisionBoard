package org.usfirst.frc.team1339.robot.subsystems;

import org.usfirst.frc.team1339.robot.RobotMap;
import org.usfirst.frc.team1339.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team1339.robot.commands.GyroVision;
import org.usfirst.frc.team1339.robot.commands.RunVision;

import com.ni.vision.NIVision.SettingType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
    
    double conditionalPID;
    
    public AnalogGyro chassisGyro;
    
    double VisionSETpoint = 140;
    double gyroSetpoint;
    double diff;
    double gyroDiff;
    double gyroInitialVal;//Gyro Value
    double gyroUpdatingVal;
    
    double distanceToGoal; //for calculating exact distance to goal
    double scaledDistance;//for determining how far to turn with PID vision
    double scaledDistanceArea;
    
    double FOVpixel;
    double Tpixel;
    
    public static double gyroAngle;
    // Initialize your subsystem here
    public PIDChassis() {
    	super("PIDChassis", Kp, Ki, Kd);
    	
    	motor4 = new CANTalon(RobotMap.motorFourPort);
    	
    	chassisGyro = new AnalogGyro(0);
			
		gyroInitialVal = 0;
		
		diff = 0;
		
		gyroSetpoint = 0;
    	
		conditionalPID = 0;
    	
		gyroAngle = chassisGyro.getAngle();
		
		SmartDashboard.putNumber("GRYO", gyroAngle);
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
        setDefaultCommand(new DriveWithJoystick());
    }
    public void resetConditionalPID(){
    	conditionalPID = 0;
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
		conditionalPID = 1;
    }
    
    public void distanceCalculator(){
    	distanceToGoal = 0;
    	double[] lengths = table.getNumberArray("length", defaultValue);
		for (double length : lengths) {
			Tpixel = length;
		}
		FOVpixel = 300;
    	distanceToGoal = (20/12)*FOVpixel/(2*Tpixel*(Math.tan(49)));
    }
    
    public void scaledVision(){
    	scaledDistance = 0;
    	double[] areas = table.getNumberArray("area", defaultValue);
		for (double area : areas) {
			scaledDistanceArea = area;
		}
		scaledDistance = scaledDistanceArea*0.2;//NEEDS TESTING
    }
    
    public void gyroVision(){
    	double[] centerXs = table.getNumberArray("centerX", defaultValue);
    	double offsetX = 0;
		for (double centerX : centerXs) {
			offsetX = centerX;
			//PIDX[0] = centerX;
		}
		
		gyroInitialVal = chassisGyro.getAngle();
    	diff = VisionSETpoint-offsetX;
    	gyroSetpoint = gyroInitialVal+(diff*scaledDistance);//MIGHT NOT WORK. NEEDS TESTING 
    	setSetpoint(gyroSetpoint);
    	
    	conditionalPID = 2;
    }
    
    public void gyroOffset(){
    	
    	gyroUpdatingVal = chassisGyro.getAngle();
    	SmartDashboard.putNumber("Gyro Updating Val", gyroUpdatingVal);
    	gyroDiff = gyroSetpoint-gyroUpdatingVal;
    	conditionalPID = 2;
    	
    }
    
    public void calibrateGyro() {
    	chassisGyro.calibrate();
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage
    	
    	//PID to align with goal
    	double[] centerXs = table.getNumberArray("centerX", defaultValue);
    	double PIDX = 0;
    	gyroAngle = chassisGyro.getAngle();
    	SmartDashboard.putNumber("GRRRROOO", Math.round(gyroAngle));
    	
    	gyroUpdatingVal = chassisGyro.getAngle();
    	SmartDashboard.putNumber("Gyro Updating Val", gyroUpdatingVal);
    	SmartDashboard.putNumber("gyroSetPoint", gyroSetpoint);
    	SmartDashboard.putNumber("COnditionalPID", conditionalPID);
    	
    	gyroDiff = gyroSetpoint-gyroUpdatingVal;
    	
		for (double centerX : centerXs) {
			SmartDashboard.putNumber("X", centerX);
			PIDX = centerX;
			SmartDashboard.putNumber("PIDX", PIDX);
			//return PIDX;
		}
		SmartDashboard.putNumber("GyroDiff", gyroDiff);
		if(Math.abs(VisionSETpoint-PIDX)<10){
			PIDX = VisionSETpoint;  
			conditionalPID = 1;
		}
		if(conditionalPID == 1){
			return PIDX;
		}
		else if (conditionalPID == 2){
			return gyroAngle;
		}
		else return 0;
		
		
		
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        motor4.set(output);
    }
}
