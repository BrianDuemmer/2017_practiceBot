package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.MotorData;
import org.usfirst.frc.team223.AdvancedX.robotParser.PIDData;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

/**
 *
 */
public class Shooter extends Subsystem
{

	private MotorData shooterMotorData;
	private SpeedController shooterMotor;

	private MotorData augerMotorData;
	private SpeedController augerMotor;



	// PID utilities
	private PIDController shooterPID;
	private PIDData shooterPIDData;

	// PID Source for the shooter PID
	private PIDSource shooterSrc;

	// PIDOutput for the shooter
	private PIDOutput shooterOutput;
	
	// target output speed for the shooter
	public double shooterTargetRPM;





	private AdvancedXManager manager;

	Logger log;


	public Shooter(AdvancedXManager manager)
	{
		log = manager.getRoboLogger().getLogger("SHOOTER");
		log.info("Initializing Shooter Subsystem...");
		this.manager = manager;

		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();
		
		this.shooterTargetRPM = (Double)parser.getKeyByPath("Shooter/wheels/targetSpeed", BASIC_TYPE.DOUBLE);

		this.shooterMotorData = parser.parseMotor("Shooter/wheels/motor");
		this.shooterMotor = allocator.allocateMotor(this.shooterMotorData);

		this.augerMotorData = parser.parseMotor("Shooter/auger/motor");
		this.augerMotor = allocator.allocateMotor(this.augerMotorData);

		this.shooterPIDData = parser.parsePID("Shooter/wheels/PID");




		// Setup the shooter PID
		shooterSrc = new PIDSource()
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
				return 0;
			}

		};


		// setup the shooter PID output
		this.shooterOutput = new PIDOutput()
		{
			@Override
			public void pidWrite(double output) {
				shooterMotor.set(output);
			}
		};



		this.shooterPID = allocator.allocatePID(shooterPIDData, shooterSrc, shooterOutput);
		
		log.info("Finished setting up shooter");
	}


	public void free()
	{
		log.info("Attempting to free shooter...");
		manager.destroy(shooterMotor);
		manager.destroy(augerMotor);
		log.info("Finished freeing shooter");
	}


	/**
	 * Leave this empty - we don't have anything running by default
	 */
	public void initDefaultCommand() {}


	/**
	 * Obtains a reference to the shooter PID
	 */
	public PIDController getShooterPID() {   return shooterPID;   }





}

