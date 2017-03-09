package org.usfirst.frc.team223.robot;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.EnumPair;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import net.sf.microlog.core.Logger;

/**
 * This class contains all of the functions for autonomous, and each auto variant exists 
 * as a static function, which should be called in AutonomousInit()
 * 
 * @author Brian Duemmer
 */
public class Autonomous
{
	private static double towerDist = 10;
	private static double towerWidth = 3.918;
	private static double towerAngle = 30;
	private static double robotLength = 3;
	
	
	private static double distToGear1 = 8.702;
	private static double distToGear3 = 7.316;
	private static double sideGearApproachDist = 2.55;
	private static double frontGearApproachDist = 1.96;
	private static double lineCrossDist = 12;
	
	private static double boilerStrafeDist = 1;
	private static double boilerTurnAngle = 30;
	private static double boilerDwellTime = 8;
	
	private EnumPair autoSelect;
	private boolean passAutoLine;
	
	// utility
	private Logger log;
	
	private Alliance alliance;
	public static Alliance overrideAlliance = Alliance.Red;
	
	
	/**
	 * Loads all of the data and initialized autonomous
	 */
	public Autonomous(AdvancedXManager manager)
	{
		log = manager.getRoboLogger().getLogger("Autonomous");
		
		GXMLparser parser = manager.obtainParser();
		
		autoSelect = (EnumPair)parser.getKeyByPath("auto/mode", BASIC_TYPE.ENUM);
		passAutoLine = (boolean) parser.getKeyByPath("auto/crossLineAfter", BASIC_TYPE.BOOL);
		
		
	}
	
	
	public void execAuto()
	{
		log.info("===============================================================================");
		log.info("============================== Starting Auto... ===============================");
		log.info("===============================================================================");
		
		log.info("Mode is:  \"" +autoSelect.selection+ "\"");
		
		alliance = DriverStation.getInstance().getAlliance();
		if(alliance == Alliance.Invalid)
			alliance = overrideAlliance;
		log.info("Alliance is: " +alliance.toString());
		
		
		// run the proper auto
		switch(autoSelect.selection)
		{
			case("gear1"):
				approachGear1();
			
				// pass the auto line if it is selected
				if(passAutoLine)
				{
					log.info("Crossing line from gear 1...");
					Robot.drive.drive_G1xyCartesian(0, lineCrossDist, 0);
				}
				
			
			case("gear2"):
				approachGear2(frontGearApproachDist);
			
				if(passAutoLine)
					log.warn("No line cross has been programmed for gear 2!");
			
			case("gear3"):
				approachGear3();
			
				// pass the auto line if it is selected
				if(passAutoLine)
				{
					log.info("Crossing line from gear 1...");
					Robot.drive.drive_G1xyCartesian(0, lineCrossDist, 0);
				}
			
				
			case("crossLine"):
				log.info("Beginning cross line auto...");
				Robot.drive.drive_G1xyCartesian(0, lineCrossDist, 0);
				
			case("boiler"):
				log.info("Beginning boiler shot auto...");
				if(alliance ==  Alliance.Red)
				{
					// approach boiler
					Robot.drive.drive_G1xyCartesian(-1*boilerStrafeDist, 0, 0);
					Robot.drive.drive_G2Arc(0.000001, boilerTurnAngle, 0);
					
					// fire away
					Command shootCmd = new ShooterNoVision();
					shootCmd.start();
					Timer.delay(boilerDwellTime);
					shootCmd.cancel();
					
					// cross the line after
					Robot.drive.drive_G2Arc(0.000001, -1 * (90+boilerTurnAngle), 0);
					Robot.drive.drive_G1xyCartesian(0, lineCrossDist - boilerStrafeDist, 0);
				}
				
				else
				{
					// approach boiler
					Robot.drive.drive_G1xyCartesian(boilerStrafeDist, 0, 0);
					Robot.drive.drive_G2Arc(0.000001, -1*boilerTurnAngle, 0);
					
					// fire away
					Command shootCmd = new ShooterNoVision();
					shootCmd.start();
					Timer.delay(boilerDwellTime);
					shootCmd.cancel();
					
					// cross the line after
					Robot.drive.drive_G2Arc(0.000001, (90+boilerTurnAngle), 0);
					Robot.drive.drive_G1xyCartesian(0, lineCrossDist - boilerStrafeDist, 0);
				}
				
			
			case("doNothing"):
				log.info("Doing nothing for auto...");
		}
		
		
		
		
	}
	
	
	public void approachGear2(double approachDist)
	{
		log.info("Beginning approach to gear 2...");
		
		double fwdDist = towerDist - approachDist - robotLength;
		
		Robot.drive.setDriveType(driveType.FULL_TRACTION, true);
		
		log.info("Approaching tower");
		Robot.drive.drive_G1xyCartesian(0, fwdDist, 0);
		Robot.drive.drive_G2Arc(0.000001, -90, 0);
		 
		Robot.drive.setDriveType(driveType.FULL_OMNI, true);
		
		log.info("Approaching gear peg...");
		Robot.drive.drive_G1xyCartesian(-1*approachDist, 0, 0);
		Robot.gear.dropGear();
		Robot.drive.drive_G1xyCartesian(approachDist, 0, 0);
		
	}	
	
	
	
	
	public void approachGear1()
	{
		// approach peg
		log.info("Approaching tower");
		Robot.drive.setDriveType(driveType.FULL_TRACTION, true);
		Robot.drive.drive_G1xyCartesian(0, distToGear1 - robotLength, 0);
		
		// turn onto it
		Robot.drive.drive_G2Arc(0.0000001, -30, 0);
		
		// place the gear and pull out
		Robot.drive.setDriveType(driveType.FULL_OMNI, true);
		
		log.info("Dropping gear...");
		Robot.drive.drive_G1xyCartesian(-1*sideGearApproachDist, 0, 0);
		Robot.gear.dropGear();
		Robot.drive.drive_G1xyCartesian(sideGearApproachDist, 0, 0);
	}
	
	
	
	
	public void approachGear3()
	{
		// approach peg
		log.info("Approaching tower");
		Robot.drive.setDriveType(driveType.FULL_TRACTION, true);
		Robot.drive.drive_G1xyCartesian(0, -1*distToGear3, 0);
		
		// turn onto it
		Robot.drive.drive_G2Arc(0.0000001, 30, 0);
		
		// place the gear and pull out
		Robot.drive.setDriveType(driveType.FULL_OMNI, true);
		
		log.info("Depositing gear");
		Robot.drive.drive_G1xyCartesian(-1*sideGearApproachDist, 0, 0);
		Robot.gear.dropGear();
		Robot.drive.drive_G1xyCartesian(sideGearApproachDist, 0, 0);
	}
}
