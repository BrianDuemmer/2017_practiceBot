
package org.usfirst.frc.team223.robot;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.RoboLogManagerBase;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive;
import org.usfirst.frc.team223.robot.driveTrain.DriveFromController;
import org.usfirst.frc.team223.robot.hangar.Hangar;
import org.usfirst.frc.team223.robot.intake.Intake;
import org.usfirst.frc.team223.robot.intake.IntakeControl;
import org.usfirst.frc.team223.robot.shooter.Shooter;

import edu.wpi.first.wpilibj.IterativeRobot;
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
		manager = new AdvancedXManager("/media/sda1/MainConfig.xml", logBase)
		{
			@Override
			public boolean load() 
			{	
				
				intake = new Intake(manager);
				hangar = new Hangar(manager);
				shooter = new Shooter(manager);
				drive = new ButterflyHDrive(manager);
				
				drive.setDefaultCommand(new DriveFromController());
				intake.setDefaultCommand(new IntakeControl());
				
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
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		nt.putNumber("theTime", Timer.getFPGATimestamp());
	}


	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

	}
}
