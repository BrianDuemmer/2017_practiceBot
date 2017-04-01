package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.motionControl.DriveSide;
import org.usfirst.frc.team223.AdvancedX.robotParser.DriveSideData;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.MotorData;

import com.ctre.CANTalon;

import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BasicType;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

/**
 *
 */
public class Shooter extends Subsystem
{
	// NT Keys
	public static final String shooterSpeedkey = "shooterRPM";
	public static final String shooterPosKey = "shooterPos";
	public static final String shooterSetpointKey = "shooterSetpoint";
	public static final String shooterEnabledKey = "shooterOn";
	public static final String shooterOutputKey = "shooterOutput";
	
	public static final double augerHighTime = 4;
	public static final double augerLowTime = 1;
	


	DriveSideData shooterData;
	DriveSide shooterMotors;
	
	MotorData augerData;
	SpeedController augerMotor;
	
	
	
	// PID Source for the shooter PID
	private PIDSource shooterSrc = new PIDSource()
	{
		/**
		 * don't do anything, we will always want rate
		 */
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {}

		/**
		 * Always returns rate for the encoder type
		 */
		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kRate;
		}

		@Override
		public double pidGet() {
			return getShooterRPM();
		}

	};

	
	
	
	
	// target output speed for the shooter
	public double shooterTargetRPM;


	// this run() method will run continuously for the duration of this subsystem
	private Thread shooterPeriodic = new Thread()
	{
		public void run()
		{
			log.info("Starting shooter periodic");

			try
			{
				while(!shouldStop)
				{
					// Delay for a bit to not lag
					Timer.delay(0.1);

					// report some stuff via NT
					manager.getNt().putNumber(shooterSpeedkey, getShooterRPM());
					manager.getNt().putNumber(shooterSetpointKey, shooterMotors.getSetpoint());
					manager.getNt().putBoolean(shooterEnabledKey, shooterMotors.getPIDController().isEnabled());
					manager.getNt().putNumber(shooterOutputKey, Math.abs(shooterMotors.getMotors().get(0).get()));
					manager.getNt().putNumber("shootErr", shooterMotors.getPIDController().getError());
				}
			} catch(Exception e)
			{
				log.error("Exception encountered in shooterPeriodic : ", e);
			}
			
			log.info("Stopping Shooter periodic");
		}
	};



	// if true, the subsystem shuts down
	private boolean shouldStop = false;

	private AdvancedXManager manager;

	Logger log;





	public Shooter(AdvancedXManager manager)
	{
		log = manager.getRoboLogger().getLogger("SHOOTER");
		log.info("Initializing Shooter Subsystem...");
		this.manager = manager;

		// obtain parser and allocator
		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();


		// parse / allocate objects
		this.shooterTargetRPM = (Double)parser.getKeyByPath("Shooter/targetSpeed", BasicType.DOUBLE);
		this.shooterData = parser.parseDriveSide("Shooter/wheels");
		this.augerData = parser.parseMotor("Shooter/auger/motor");
		
		this.shooterMotors = allocator.allocateDriveSide(shooterData, "Shooter");
		this.augerMotor = allocator.allocateMotor(augerData);
		
		this.shooterMotors.setVelocityPIDSource(shooterSrc);

		// start the periodic function
		this.shooterPeriodic.start();

		log.info("Finished setting up shooter");
	}







	public void free()
	{
		// Tell the shooter to stop
		this.shouldStop = true;

		// log and shutdown
		log.info("Attempting to free shooter...");
		manager.destroy(shooterMotors);
		manager.destroy(augerMotor);
		log.info("Finished freeing shooter");
	}


	/**
	 * Leave this empty - we don't have anything running by default
	 */
	public void initDefaultCommand() {}
	
	
	
	public void setDefaultCommand(Command command) {   super.setDefaultCommand(command); }


	/**
	 * gets the current speed of the shooter wheels, in RPMs
	 */
	public double getShooterRPM()
	{
		double out = ((CANTalon)shooterMotors.getMotors().get(0)).getEncVelocity();
		out *= shooterData.encoder.distPerCount * (shooterData.encoder.invert  ?  -1 : 1);
		
		return out;
	}

	
	
	/**
	 * Brings the shooter up to speed. This is non-blocking
	 */
	public void bringUpToSpeed()
	{
		log.info("Bringing Shooter up to speed..");
		
		shooterMotors.setSetpoint(shooterTargetRPM);
	}
	
	
	
	/**
	 * Turns off the shooter. non-blocking.
	 */
	public void spinDown()
	{
		log.info("Spinning down shooter...");
		shooterMotors.setSetpoint(0);
		shooterMotors.setRawOutput(0);
	}
	
	
	/**
	 * Checks if the shooter wheel is up to speed
	 * @return true if the wheel is up to speed
	 */
	public boolean isUpToSpeed()
	{
		return Math.abs(getShooterRPM() - shooterTargetRPM) / shooterTargetRPM < 0.1;
	}
	
	
	
}

