package org.usfirst.frc.team1339.lib.utils;

import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AccelerometerConversion {

	public double accelerationConversion(double g){
		double meters = g/0.101972;
		return meters;
	}
	
	public void publishSmartDashboard(){
		SmartDashboard.putNumber("Acceleration Conversion", accelerationConversion(HardwareAdapter.kAccelerometer.getX()));
	}
}
