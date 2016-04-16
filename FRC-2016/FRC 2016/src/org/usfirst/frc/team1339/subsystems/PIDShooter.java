package org.usfirst.frc.team1339.subsystems;


import org.usfirst.frc.team1339.commands.*;
import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDShooter extends Subsystem {
	
	double m_oldspeed = 0;
	double oldEncRate = 0;
	public double ultraSpeed = .9;
	double rate = 0, speed;
	double lastSpeed = 0, lastTime = Timer.getFPGATimestamp();
	

    // Initialize your subsystem here
    public PIDShooter() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void shootBall(double speed){
    	HardwareAdapter.kShooterMotorOne.set(speed);
    	HardwareAdapter.kShooterMotorTwo.set(-speed);
    	
    }
    
    public void shootBallUltra(){
    	double ultra = (HardwareAdapter.kUltrasonicOne.getRangeInches()+HardwareAdapter.kUltrasonicTwo.getRangeInches())/2;
    	double speed = ((ultra*.5)+61.5)/100;
    	HardwareAdapter.kShooterMotorOne.set(speed);
    	HardwareAdapter.kShooterMotorTwo.set(-speed);
    }
    
    //rev/sec
    public double getEncRate(){
    	return HardwareAdapter.kShooterMotorOne.getSpeed()/400;
    }
    public void initDefaultCommand() {
    	//setDefaultCommand(new DriveShooter());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void PIDShoot(){
    	double accel = HardwareAdapter.ShooterPID.calculate(getEncRate());
    	//if(HardwareAdapter.kShooterMotorOne.getSpeed() > 40){
    		//accel += PIDJerk();
    	//}
    	accel += PIDJerk();
    	double speed = accel + m_oldspeed;    	
    	if(speed > 1){
    		speed = 1;
    	}
    	if(speed < -1){
    		speed = -1;
    	}
    	
    	HardwareAdapter.kShooterMotorOne.set(speed);
    	HardwareAdapter.kShooterMotorTwo.set(-speed);
    	SmartDashboard.putNumber("Motor Output", speed);
    	
    	m_oldspeed = speed;
    	oldEncRate = getEncRate();
    }
    public double getAcceleration(){
    	double speed = HardwareAdapter.kShooterMotorOne.getSpeed() - lastSpeed;
    	double time = (Timer.getFPGATimestamp() - lastTime);
    	if (HardwareAdapter.kShooterMotorOne.getSpeed() < 0.05 && HardwareAdapter.kShooterMotorOne.getSpeed() > -0.05){
    		speed = 0;
    	}
    	SmartDashboard.putNumber("accel time", time);
    	SmartDashboard.putNumber("accel speed", speed);
    	SmartDashboard.putNumber("Last Time Jerk PID", lastTime);
    	SmartDashboard.putNumber("Timer.getFPGATimestamp()", Timer.getFPGATimestamp());
    	double accel = speed/time;
    	lastSpeed = HardwareAdapter.kShooterMotorOne.getSpeed();
    	lastTime = Timer.getFPGATimestamp();
    	SmartDashboard.putNumber("ShooterAccel", accel);
    	return accel;
    }
    public double PIDJerk(){
    	double jerk = HardwareAdapter.AccelPID.calculate(getAcceleration());
    	SmartDashboard.putNumber("shooter jerk PID", jerk);
    	return jerk;
    }
}
