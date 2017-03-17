package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.EncoderData;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.MotorData;
import org.usfirst.frc.team223.AdvancedX.robotParser.PIDData;

import com.ctre.CANTalon;

import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BasicType;
import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
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


	private EncoderData shooterEncoderData;
	MotorData shooterMotorData;
	CANTalon shooterMotor;

	MotorData augerMotorData;
	SpeedController augerMotor;


	

	// PID utilities
	private PIDController shooterPID;
	private PIDData shooterPIDData;
	
	// PIDOutput for the shooter
	private PIDOutput shooterOutput = new PIDOutput()
	{
		@Override
		public void pidWrite(double output) {
			shooterMotor.set(output);
		}
	};
	
	
	
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
					manager.getNt().putNumber(shooterPosKey, getShooterPos());
					manager.getNt().putNumber(shooterSetpointKey, getShooterPID().getSetpoint());
					manager.getNt().putBoolean(shooterEnabledKey, getShooterPID().isEnabled());
					manager.getNt().putNumber(shooterOutputKey, Math.abs(shooterMotor.get()));
					manager.getNt().putNumber("ShootKi", shooterPID.getI());

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
		this.shooterTargetRPM = (Double)parser.getKeyByPath("Shooter/wheels/targetSpeed", BasicType.DOUBLE);

		this.shooterMotorData = parser.parseMotor("Shooter/wheels/motor");
		this.shooterMotor = (CANTalon) allocator.allocateMotor(this.shooterMotorData);

		this.shooterEncoderData = parser.parseEncoder("Shooter/wheels/encoder");

		allocator.allocateCANEncoder(this.shooterEncoderData, this.shooterMotor);

		this.augerMotorData = parser.parseMotor("Shooter/auger/motor");
		this.augerMotor = allocator.allocateMotor(this.augerMotorData);

		this.shooterPIDData = parser.parsePID("Shooter/wheels/PID");

		this.shooterPID = allocator.allocatePID(shooterPIDData, shooterSrc, shooterOutput);

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
		manager.destroy(shooterMotor);
		manager.destroy(augerMotor);
		log.info("Finished freeing shooter");
	}


	/**
	 * Leave this empty - we don't have anything running by default
	 */
	public void initDefaultCommand() {}
	
	
	
	public void setDefaultCommand(Command command) {   super.setDefaultCommand(command); }


	/**
	 * Obtains a reference to the shooter PID
	 */
	public PIDController getShooterPID() {   return shooterPID;   }



	/**
	 * gets the current speed of the shooter wheels, in RPMs
	 */
	public double getShooterRPM()
	{
		double speed = shooterMotor.getEncVelocity() * shooterEncoderData.distPerCount;
		return speed;
	}
	
	
	
	
	public double getShooterPos()
	{
		return shooterMotor.getEncPosition() * shooterEncoderData.distPerCount;
	}
	
	
	/**
	 * Brings the shooter up to speed. This is non-blocking
	 */
	public void bringUpToSpeed()
	{
		log.info("Bringing Shooter up to speed..");
		
		shooterPID.setSetpoint(shooterTargetRPM);
		shooterPID.reset();
	}
	
	
	
	/**
	 * Turns off the shooter. non-blocking.
	 */
	public void spinDown()
	{
		log.info("Spinning down shooter...");
		shooterPID.disable();
		shooterMotor.set(0);
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

