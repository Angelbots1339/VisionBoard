
package org.usfirst.frc.team1339.robot;

import org.usfirst.frc.team1339.robot.commands.CommandBase;
import org.usfirst.frc.team1339.robot.subsystems.PIDChassis;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static OI oi;

    Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    //NetworkTable table;
 /*
     NetworkTable table = NetworkTable.getTable("/GRIP/myContoursReport");
     double[] defaultValue = new double[0];
    */
    
    public void robotInit() {
		oi = new OI();
		CommandBase.init();
		
		
		
		
        // instantiate the command used for the autonomous period
		
    }
    
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
   
        	/*
			double[] areas = table.getNumberArray("area", defaultValue);
			for (double area : areas) {
				System.out.println(area + " ");
				SmartDashboard.putNumber("Area", area);
			}
			
			double[] centerXs = table.getNumberArray("centerX", defaultValue);
			for (double centerX : centerXs) {
				System.out.println(centerX + " ");
				SmartDashboard.putNumber("CenterX", centerX);
			}
			
			double[] centerYs = table.getNumberArray("centerY", defaultValue);
			for (double centerY : centerYs) {
				System.out.println(centerY + " ");
				SmartDashboard.putNumber("CenterY", centerY);
			}
			
			double[] heights = table.getNumberArray("height", defaultValue);
			for (double height : heights) {
				System.out.println(height + " ");
				SmartDashboard.putNumber("height", height);
			}
			
			double[] widths = table.getNumberArray("width", defaultValue);
			for (double width : widths) {
				System.out.println(width + " ");
				SmartDashboard.putNumber("width", width);
			}
			System.out.println();
			Timer.delay(1);
			*/
		} 
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
