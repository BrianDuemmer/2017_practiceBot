package org.usfirst.frc.team223.robot;

import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;

/**
 * This class contains all of the functions for autonomous, and each auto variant exists 
 * as a static function, which should be called in AutonomousInit()
 * 
 * @author Brian Duemmer
 */
public class Autonomous
{
	private static double towerDist = 9.5;
	private static double towerWidth = 3.918;
	private static double towerAngle = 30;
	private static double robotLength = 3;
	
	
	public void approachGear1(double approachDist)
	{
		double fwdDist = towerDist - approachDist - robotLength;
		
		Robot.drive.setDriveType(driveType.FULL_TRACTION);
		Robot.drive.drive_G1xyCartesian(0, fwdDist, 0);
		Robot.drive.drive_G2Arc(0.0001, -90, 0);
		
		Robot.drive.setDriveType(driveType.FULL_OMNI);
		Robot.drive.drive_G1xyCartesian(approachDist, 0, 0);
		
	}	
}
