package org.usfirst.frc.team1339.lib.utils;

import org.usfirst.frc.team1339.robot.HardwareAdapter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EncoderConversion {
	
	public EncoderConversion(){
	}
	public double encoderConversion(int clicks){
		double rotation = clicks/256;
		rotation *= 4.5;
		return rotation;
	}
	public double getDistanceInches(int clicks){
		double rotation = clicks/256;
		rotation *= 4.5;
		double distance = rotation * 6 * Math.PI;
		return distance;
	}
	public double getDistanceFeet(int clicks){
		double feet = getDistanceInches(clicks)/12;
		return feet;
	}
	public int getClicksInches(double distance){
		double rotation = distance /(6*Math.PI);
		rotation /= (16/33);
		double clicks = rotation * 256;
		return (int) Math.round(clicks);
	}
	public void publishSmartDashboard(){
		SmartDashboard.putNumber("Left Drive Clicks", HardwareAdapter.kLeftDriveEncoder.get());
		SmartDashboard.putNumber("Right Drive Clicks", HardwareAdapter.kRightDriveEncoder.get());
		SmartDashboard.putNumber("Left Drive Rotations", encoderConversion(HardwareAdapter.kLeftDriveEncoder.get()));
		SmartDashboard.putNumber("Right Drive Rotations", encoderConversion(HardwareAdapter.kLeftDriveEncoder.get()));
	}
}
