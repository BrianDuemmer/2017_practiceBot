package org.usfirst.frc.team223.robot;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.RoboLogManagerBase;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BasicType;
import org.usfirst.frc.team223.AdvancedX.vision.PiVisionClient;
import org.usfirst.frc.team223.robot.auto.Autonomous;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive;
import org.usfirst.frc.team223.robot.driveTrain.DriveFromController;
import org.usfirst.frc.team223.robot.gear.GearThing;
import org.usfirst.frc.team223.robot.hangar.HangControl;
import org.usfirst.frc.team223.robot.hangar.Hangar;
import org.usfirst.frc.team223.robot.intake.Intake;
import org.usfirst.frc.team223.robot.intake.IntakeControl;
import org.usfirst.frc.team223.robot.shooter.Shooter;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

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
	public static GearThing gear;
	
	public static PowerDistributionPanel pdp;
	
	public static PiVisionClient visionClient;
	
	public static Autonomous auto;
	
	// global debug mode flag
	public static boolean isDebug = true;
	private static DigitalInput debugPin;


	// AdvancedX Components
	public static AdvancedXManager manager;
	public static RoboLogManagerBase logBase;
	

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
		manager = new AdvancedXManager("/home/lvuser/MainConfig.xml", logBase)
		{
			@Override
			public boolean load() 
			{	
				pdp = new PowerDistributionPanel((int) obtainParser().getKeyByPath("pdpID", BasicType.INT));
				
				int visionPort = (int) obtainParser().getKeyByPath("VisionServer/port", BasicType.INT);
				String visionAddress = (String) obtainParser().getKeyByPath("VisionServer/address", BasicType.STRING);
				
//				visionClient = new PiVisionClient(visionAddress, visionPort, this.getRoboLogger().getLogger("VisionClient"));
				debugPin = new DigitalInput((int) obtainParser().getKeyByPath("debugPin", BasicType.INT));
				
				intake = new Intake(manager);
				hangar = new Hangar(manager);
				shooter = new Shooter(manager);
				drive = new ButterflyHDrive(manager);
				gear = new GearThing(manager);
				auto = new Autonomous(manager);
				
				drive.setDefaultCommand(new DriveFromController());
				intake.setDefaultCommand(new IntakeControl());
				shooter.setDefaultCommand(new ShooterNoVision());
				hangar.setDefaultCommand(new HangControl());
				
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
		

		nt = manager.initNT();
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
		generalInit();
	}

	
	
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		nt.putNumber("theTime", Timer.getFPGATimestamp());
		generalPeriodic();
		
		if(drive != null)
		{
			drive.resetPIDs();
		}
	}


	static boolean prevFinished;
	
	@Override
	public void autonomousInit() 
	{
		generalInit();
		auto.runAuto();
		
		//reset on each auto init
		prevFinished = false;
	}

	
	
	@Override
	public void autonomousPeriodic() 
	{
		Scheduler.getInstance().run();
		generalPeriodic();
		
		// run only once after auto finishes
		if(!auto.autoCommand.isRunning() && !prevFinished)
		{
			prevFinished = true;
			
			log.info("===============================================================================");
			log.info("=============================== Finished Auto =================================");
			log.info("===============================================================================");
		}
	}

	
	
	@Override
	public void teleopInit() 
	{
		log.info("Entering Teleop...");
		generalInit();
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
	
	
	
	public void generalInit()
	{
		// set the drive type
//		drive.setDriveType(oi.button_dR.get()  ?  driveType.FULL_TRACTION : driveType.FULL_OMNI, true);
	}
	
}






