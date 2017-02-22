package org.usfirst.frc.team223.robot;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.RoboLogManagerBase;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;
import org.usfirst.frc.team223.AdvancedX.vision.PiVisionClient;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive;
import org.usfirst.frc.team223.robot.driveTrain.DriveFromController;
import org.usfirst.frc.team223.robot.hangar.HangDebug;
import org.usfirst.frc.team223.robot.hangar.Hangar;
import org.usfirst.frc.team223.robot.intake.Intake;
import org.usfirst.frc.team223.robot.intake.IntakeControl;
import org.usfirst.frc.team223.robot.shooter.Shooter;
import org.usfirst.frc.team223.robot.shooter.ShooterManual;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import net.sf.microlog.core.Level;
import net.sf.microlog.core.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	// OI and Subsystems
	public static OI oi;
	
	public static ButterflyHDrive drive;
	public static Hangar hangar;
	public static Intake intake;
	public static Shooter shooter;
	
	public static PowerDistributionPanel pdp;
	
	public static PiVisionClient visionClient;
	
	// global debug mode flag
	public static boolean isDebug = true;
	private static DigitalInput debugPin;


	// AdvancedX Components
	public static AdvancedXManager manager;
	public static RoboLogManagerBase logBase;
	
	// parser for auto configuration file
	public static GXMLparser autoParser;

	// logging object (for Robot.java) only
	static Logger log;

	private static NetworkTable nt;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		// init the AdvancedX components
		// Setup the logger
		logBase = new RoboLogManagerBase("/media/sda1/logging17", 5801, Level.WARN);
		log = logBase.getLogger("ROBOT");


		// Setup the AdvancedXManager, and add the reload() and free() methods
		manager = new AdvancedXManager("/media/sda1/MainConfig.xml", logBase)
		{
			@Override
			public boolean load() 
			{	
				pdp = new PowerDistributionPanel((int) obtainParser().getKeyByPath("pdpID", BASIC_TYPE.INT));
				
				int visionPort = (int) obtainParser().getKeyByPath("VisionServer/port", BASIC_TYPE.INT);
				String visionAddress = (String) obtainParser().getKeyByPath("VisionServer/address", BASIC_TYPE.STRING);
				
				visionClient = new PiVisionClient(visionAddress, visionPort, this.getRoboLogger().getLogger("VisionClient"));
				debugPin = new DigitalInput((int) obtainParser().getKeyByPath("debugPin", BASIC_TYPE.INT));
				
				intake = new Intake(manager);
				hangar = new Hangar(manager);
				shooter = new Shooter(manager);
				drive = new ButterflyHDrive(manager);
				
				drive.setDefaultCommand(new DriveFromController());
				intake.setDefaultCommand(new IntakeControl());
				shooter.setDefaultCommand(new ShooterManual());
				hangar.setDefaultCommand(new HangDebug());
				
				oi = new OI();
				
				return true;
			}

			@Override
			public boolean free() 
			{
				drive.free();
				intake.free();
				hangar.free();
				shooter.free();

				return true;
			}
		};
		

		nt = manager.getNt();

		manager.start(1000);

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() 
	{
		log.info("Entering Disabled mode...");
	}

	
	
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		nt.putNumber("theTime", Timer.getFPGATimestamp());
		generalPeriodic();
	}


	@Override
	public void autonomousInit() 
	{
		log.info("Entering Autonomous...");

	}

	
	
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		generalPeriodic();
	}

	
	
	@Override
	public void teleopInit() 
	{
		log.info("Entering Teleop...");
	}

	
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		nt.putNumber("totalCurrent", pdp.getTotalCurrent());
		generalPeriodic();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() 
	{
		generalPeriodic();
	}
	
	
	
	/**
	 * Runs periodically for the duration of robot operation, regardless of 
	 * robot mode
	 */
	public void generalPeriodic()
	{
		isDebug = !debugPin.get();
		manager.getNt().putBoolean("debugEnabled", isDebug);
	}
	
}






