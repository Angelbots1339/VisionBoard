package org.usfirst.frc.team1339.robot.subsystems;

import org.usfirst.frc.team1339.robot.RobotMap;
import org.usfirst.frc.team1339.robot.commands.*;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Motors extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	CANTalon motor1, motor2, motor3;
	
	public Motors(){
		motor1 = new CANTalon(RobotMap.motorOnePort);
    	motor2 = new CANTalon(RobotMap.motorTwoPort);
    	motor3 = new CANTalon(RobotMap.motorThreePort);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void joystickDrive(double speed1, double speed2, double speed3){
    	motor1.set(speed1);
    	motor2.set(speed2);
    	motor3.set(speed3*.5);
    }
}

