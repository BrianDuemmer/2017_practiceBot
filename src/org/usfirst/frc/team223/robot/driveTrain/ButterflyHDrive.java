package org.usfirst.frc.team223.robot.driveTrain;


import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.motionControl.DriveSide;
import org.usfirst.frc.team223.AdvancedX.motionControl.GyroNavX;
import org.usfirst.frc.team223.AdvancedX.motionControl.LinearFeedInterpolator;
import org.usfirst.frc.team223.AdvancedX.motionControl.OmniDirectionalDrive;
import org.usfirst.frc.team223.AdvancedX.robotParser.DriveSideData;
import org.usfirst.frc.team223.AdvancedX.robotParser.Freeable;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;
import org.usfirst.frc.team223.AdvancedX.robotParser.PIDData;
import org.usfirst.frc.team223.AdvancedX.robotParser.SolenoidData;
import org.usfirst.frc.team223.AdvancedX.utility.AngleUtil;
import org.usfirst.frc.team223.robot.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import net.sf.microlog.core.Logger;

/**
 * Class for controlling an H drive
 * @author Brian Duemmer
 *
 */
public class ButterflyHDrive extends Subsystem implements OmniDirectionalDrive, Freeable 
{
	/////////// Constants and scalars ////////////
	private double maxOmniVel;
	private double maxOmniAcceleration;
	
	private double maxTractionVel;
	private double maxTractionAcceleration;
	
	private double maxCenterVel;
	private double maxCenterAcceleration;
	
	private double wheelbaseWidth;
	
	private double tractionReductionScalar;


	//////////////// Modal Data /////////////////

	/**
	 *  Enum that describes the driving mode that the robot is in
	 * @author Brian Duemmer
	 *
	 */
	public enum driveType 
	{
		FULL_TRACTION,
		FRONT_TRACTION,
		REAR_TRACTION,
		FULL_OMNI
	}

	private driveType currDriveType = driveType.FULL_OMNI;


	////////////// Physical Objects //////////////

	// DriveSides
	private DriveSideData leftSideData;
	private DriveSide leftDriveSide;

	private DriveSideData rightSideData;
	private DriveSide rightDriveSide;

	private DriveSideData centerSideData;
	private DriveSide centerDriveSide;

	// Solenoids
	private SolenoidData frontSolenoidData;
	private Solenoid frontSolenoid;

	private SolenoidData rearSolenoidData;
	private Solenoid rearSolenoid;
	
	
	private Gyro navx;


	////////////////// Utility //////////////////
	Logger log;
	private AdvancedXManager manager;
	private boolean shouldStop;

	// NT keys
	private static final String leftVelKey = "leftDriveVel";
	private static final String leftPosKey = "leftDrivePos";

	private static final String rightVelKey = "rightDriveVel";
	private static final String rightPosKey = "rightDrivePos";

	private static final String centerVelKey = "centerDriveVel";
	private static final String centerPosKey = "centerDrivePos";


	private static final String frontTractionKey = "frontTraction";
	private static final String rearTractionKey = "rearTraction";
	
	private static final String yawAngleKey = "robotYaw";
	
	private static final String leftSideCurrentKey = "leftCurrent";
	private static final String rightSideCurrentKey = "rightCurrent";
	private static final String centerSideCurrentKey = "centerCurrent";


	// runs periodically in the background during the duration of the shooter
	private Thread drivePeriodic = new Thread()
	{
		public void run()
		{
			while(!shouldStop)
			{
				try
				{
					// delay to save resources
					Timer.delay(0.1);

					// put some stuff to NT
					manager.getNt().putNumber(leftPosKey, getLeftSidePos());
					manager.getNt().putNumber(leftVelKey, leftDriveSide.getVel());

					manager.getNt().putNumber(rightPosKey, getRightSidePos());
					manager.getNt().putNumber(rightVelKey, rightDriveSide.getVel());

					manager.getNt().putNumber(centerPosKey, getCenterSidePos());
					manager.getNt().putNumber(centerVelKey, centerDriveSide.getVel());

					manager.getNt().putBoolean(rearTractionKey, rearSolenoid.get() ^ rearSolenoidData.invert);
					manager.getNt().putBoolean(frontTractionKey, frontSolenoid.get() ^ frontSolenoidData.invert);

					manager.getNt().putNumber(yawAngleKey, navx.getAngle());

					manager.getNt().putNumber("leftOut", leftDriveSide.getMotors().get(0).get());
					manager.getNt().putNumber("rightOut", rightDriveSide.getMotors().get(0).get());

					manager.getNt().putNumber("leftVelSet", leftDriveSide.getSetpoint());
					manager.getNt().putNumber("rightVelSet", rightDriveSide.getSetpoint());
					manager.getNt().putNumber("centerVelSet", centerDriveSide.getSetpoint());


					// update the current draw data
					manager.getNt().putNumberArray(leftSideCurrentKey, leftDriveSide.getCurrent(Robot.pdp, leftSideData));
					manager.getNt().putNumberArray(rightSideCurrentKey, rightDriveSide.getCurrent(Robot.pdp, rightSideData));
					manager.getNt().putNumberArray(centerSideCurrentKey, centerDriveSide.getCurrent(Robot.pdp, centerSideData));
				} catch(Exception e) // log the error and wait for a bit
				{
					log.error("Exception in Drive Periodic: ,e");
					Timer.delay(1);
				}
			}
		}
	};




	// PID Controller utilities
	
	// Slave loop actions
	private double fwdDistAction;
	private double turnPosAction;
	private double centerDistAction;
	private double fwdRateAction; 
	private double centerRateAction;

	
	private LinearFeedInterpolator distInterpolator;

	
	// Forward position PID Conroller
	private PIDController fwdPosPID;
	private PIDData fwdPosPIDData;
	
	private PIDOutput fwdPosPIDOutput = new PIDOutput()
			{
				@Override
				public void pidWrite(double output) {   fwdDistAction = output; }
			};
		
			
	private PIDSource fwdPosPIDSource = new PIDSource()
			{

				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {}

				@Override
				public PIDSourceType getPIDSourceType() {   return PIDSourceType.kDisplacement; }

				/**
				 * Gets the average distance between the encoder readings from both drivesides
				 */
				@Override
				public double pidGet() 
				{
					double leftPos = getLeftSidePos();
					double rightPos = getRightSidePos();
					
					double avgPos = 0.5 * (leftPos + rightPos);
					 
					return avgPos;
				}
		
			};
			
			
			
			
		
	// Forward position PID Conroller
	private PIDController centerPosPID;
	private PIDData centerPosPIDData;
	
	private PIDOutput centerPosPIDOutput = new PIDOutput()
			{
				@Override
				public void pidWrite(double output) {   centerDistAction = output; }
			};
		
			
	private PIDSource centerPosPIDSource = new PIDSource()
			{

				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {}

				@Override
				public PIDSourceType getPIDSourceType() {   return PIDSourceType.kDisplacement; }

				/**
				 * Gets distance traveled by the center driveside
				 */
				@Override
				public double pidGet() 
				{
					return getCenterSidePos();
				}
		
			};


			
	// Turn position PIDController
	private PIDController turnPosPID;
	private PIDData turnPosPIDData;
	
	private PIDOutput turnPosPIDOutput = new PIDOutput()
			{
				@Override
				public void pidWrite(double output) {   turnPosAction = output; }
			};
			
	private PIDSource turnPosPIDSource = new PIDSource()
			{

				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {}

				@Override
				public PIDSourceType getPIDSourceType() {   return PIDSourceType.kDisplacement; }

				/**
				 * Gets the current Navx angle as the PIDInput
				 */
				@Override
				public double pidGet() 
				{
					double currYaw = getAngle();
					return currYaw;
				}
				
			};
			
			
			
		
			
			
			
			
			
		
			
			
			
			
			

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Methods ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////



	public ButterflyHDrive(AdvancedXManager manager)
	{
		// obtain the logger and parser
		log = manager.getRoboLogger().getLogger("OmniHDrive");
		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();

		log.info("parser: " + parser + "  allocator: " +allocator);

		this.manager = manager;

		// log us entering the parse routine
		log.info("\r\n\r\n\r\n================= Initializing Butterfly H Drive =================");
		
		
		// parse the master PIDs
		fwdPosPIDData = parser.parsePID("Drive/fwdPosPID");
		centerPosPIDData = parser.parsePID("Drive/strafePosPID");
		turnPosPIDData = parser.parsePID("Drive/turnPID");


		// parse the objects
		this.leftSideData = parser.parseDriveSide("Drive/leftSide");
		this.rightSideData = parser.parseDriveSide("Drive/rightSide");
		this.centerSideData = parser.parseDriveSide("Drive/centerSide");

		this.frontSolenoidData = parser.parseSolenoid("Drive/frontSolenoid");
		this.rearSolenoidData = parser.parseSolenoid("Drive/rearSolenoid");



		log.info("Attempting to allocate objects...");

		// Allocate the objects
		
		// Master PIDs
		fwdPosPID = allocator.allocatePID(fwdPosPIDData, fwdPosPIDSource, fwdPosPIDOutput);
		centerPosPID = allocator.allocatePID(centerPosPIDData, centerPosPIDSource, centerPosPIDOutput);
		turnPosPID = allocator.allocatePID(turnPosPIDData, turnPosPIDSource, turnPosPIDOutput);
		
		// Left driveside
		log.info("Allocating left side...");
		this.leftDriveSide = allocator.allocateDriveSide(this.leftSideData, "leftSide");
		
		log.info("Setting Left Side PIDSource to motor L1");
		
		PIDSource leftVelPIDSource = new PIDSource()
		{
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {}

			@Override
			public PIDSourceType getPIDSourceType() { return PIDSourceType.kRate; }

			@Override
			public double pidGet() 
			{ 
				// get the raw velocity from the encoder
				double rawVel = ((CANTalon)leftDriveSide.getMotors().get(1)).getEncVelocity();
				
				// scale from 100ms period to 1s period
				rawVel *= 10;
				return rawVel * leftSideData.encoder.distPerCount * (leftSideData.encoder.invert  ?  -1 : 1);
			}
				
		};
				
		this.leftDriveSide.setVelocityPIDSource(leftVelPIDSource);
		((CANTalon)leftDriveSide.getMotors().get(1)).setEncPosition(0);
		
		log.info("Finished allocating left side");

		
		
		// right driveside
		log.info("Allocating right side...");
		this.rightDriveSide = allocator.allocateDriveSide(this.rightSideData, "rightSide");
		
		log.info("Setting Right Side PIDSource to motor R1");
		PIDSource rightVelPIDSource = new PIDSource()
		{
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {}

			@Override
			public PIDSourceType getPIDSourceType() { return PIDSourceType.kDisplacement; }

			@Override
			public double pidGet() 
			{ 
				double rawVel = ((CANTalon)rightDriveSide.getMotors().get(1)).getEncVelocity();
				
				// scale from 100ms period to 1s period
				rawVel *= 10;
				
				return rawVel * rightSideData.encoder.distPerCount * (rightSideData.encoder.invert  ?  -1 : 1); 
				
			}
				
		};
		
		this.rightDriveSide.setVelocityPIDSource(rightVelPIDSource);
		((CANTalon)rightDriveSide.getMotors().get(1)).setEncPosition(0);
		
		log.info("Finished allocating right side");

		
		
		//center driveside
		log.info("Allocating center side...");
		this.centerDriveSide = allocator.allocateDriveSide(this.centerSideData, "centerSide");
		
		log.info("Setting Center Side PIDSource to center drive motor");
		this.centerDriveSide.setVelocityPIDSource((CANTalon)centerDriveSide.getMotors().get(0));
		((CANTalon)centerDriveSide.getMotors().get(0)).setEncPosition(0);
		this.centerDriveSide.setPIDSrcScalings(centerSideData.encoder.distPerCount, centerSideData.encoder.invert);
		
		log.info("Finished allocating center side");

		
		
		// solenoids
		this.frontSolenoid = allocator.allocateSolenoid(this.frontSolenoidData);
		this.rearSolenoid = allocator.allocateSolenoid(this.rearSolenoidData);
		log.info("Finished allocating solenoids");

		
		log.info("Attempting to parse variables...");
		

		// Parse the variables
		maxCenterAcceleration = (Double)parser.getKeyByPath("Drive/center/maxAccel", BASIC_TYPE.DOUBLE);
		maxCenterVel = (Double)parser.getKeyByPath("Drive/center/maxVel", BASIC_TYPE.DOUBLE);
		
		maxOmniAcceleration = (Double)parser.getKeyByPath("Drive/omni/maxAccel", BASIC_TYPE.DOUBLE);
		maxOmniVel = (Double)parser.getKeyByPath("Drive/omni/maxVel", BASIC_TYPE.DOUBLE);
		
		maxTractionAcceleration = (Double)parser.getKeyByPath("Drive/traction/maxAccel", BASIC_TYPE.DOUBLE);
		maxTractionVel = (Double)parser.getKeyByPath("Drive/traction/maxVel", BASIC_TYPE.DOUBLE);
		
		wheelbaseWidth = (Double)parser.getKeyByPath("Drive/wheelbaseWidth", BASIC_TYPE.DOUBLE);
		
		tractionReductionScalar = (Double)parser.getKeyByPath("Drive/tractionReductionScalar", BASIC_TYPE.DOUBLE);
		
		
		// Allocate misc. objects
		navx = new GyroNavX(Port.kMXP);
		distInterpolator = new LinearFeedInterpolator(log, maxOmniAcceleration, maxOmniVel);

		// start the drive periodic
		drivePeriodic.start();

		log.info("Finished allocating ButterflyHDrive data");

	}





	public void free()
	{
		shouldStop = true;
		log.info("Attempting to free ButterflyHDrive...");

		log.info("Attempting to free Center DriveSide...");
		this.manager.destroy(centerDriveSide);
		log.info("Finished freeing center DriveSide");

		log.info("Attempting to free Left DriveSide...");
		this.manager.destroy(leftDriveSide);
		log.info("Finished freeing Left DriveSide");


		log.info("Attempting to free Right DriveSide...");
		this.manager.destroy(rightDriveSide);
		log.info("Finished freeing Right DriveSide");

		log.info("Attempting to free Solenoids...");
		this.manager.destroy(frontSolenoid);
		this.manager.destroy(rearSolenoid);
		log.info("Finished freeing Solenoids");

		log.info("Finished freeing ButterflyHDrive");
	}



	
	
	
	/**
	 * Sets the default command for the {@link ButterflyHDrive}. Do not call this 
	 * until after the subsystem has been initialized
	 */
	public void setDefaultCommand(Command cmd)
	{
		super.setDefaultCommand(cmd);
	}






	/**
	 * {@inheritDoc}
	 * <P/>
	 * If the robot is not in omni mode, <code> strafe </code> has no effect
	 */
	@Override
	public void setRawOutput(double fwd, double strafe, double turn) 
	{
		this.leftDriveSide.setRawOutput(fwd + turn);
		this.rightDriveSide.setRawOutput(fwd - turn);

		if(this.currDriveType == driveType.FULL_OMNI)
			this.centerDriveSide.setRawOutput(strafe);

		else
			this.centerDriveSide.setRawOutput(0);

	}
	
	
	
	
	
	/**
	 * Drives the robot in a G1(linear) fashion between two points, while maintaining the same
	 * heading angle
	 * 
	 * @param heading, in degrees relative to the robot in the navigation frame, that the robot will travel at
	 * @param distance the distance that the robot will travel
	 * @param finalVel the velocity that the robot will travel at when it reached the destination
	 */
	public void drive_G1xyPolar(double heading, double distance, double finalVel)
	{
		log.info("Starting G1XY move with heading " + heading + " deg and dist of " +distance);
		
		// reset the driveTrain
		this.resetPIDs();
		
		// Enable the PIDs
		fwdPosPID.enable();
		centerPosPID.enable();
		turnPosPID.enable();
		
		
		
		// set the turning PID to maintain the current yaw
		turnPosPID.setSetpoint(getAngle());
		
		// scalars for the forward / center velocities and positions
		double fwdScalar = Math.abs(Math.cos(heading));
		double centerScalar = Math.abs(Math.sin(heading));
		
		
		
		// calculate the resultant max acceleration and velocity, such that they will not surpass either the 
		// forward or center maximum limits. Make sure the scalars are not 0 before proceeding
		double newMaxAcc;
		double newMaxVel;
		
		// Forward scalar is the only limiting factor, and the center scalar is 1, so just use center constraints 
		if(fwdScalar == 0)
		{
			 newMaxAcc = maxCenterAcceleration;
			 newMaxVel = maxCenterVel;
		}
		
		// Center scalar is the only limiting factor, and the Forward scalar is 1, so just use forward constraints 
		else if(centerScalar == 0)
		{
			newMaxAcc = maxOmniAcceleration;
			newMaxVel = maxOmniVel;
		}
		
		// Both center and forward axes are moving, so calculate which has the lower relative constraints
		else
		{
			newMaxAcc = Math.min(Math.abs(maxCenterAcceleration / centerScalar), Math.abs(maxOmniAcceleration / fwdScalar));
			newMaxVel = Math.min(Math.abs(maxCenterVel / centerScalar), Math.abs(maxOmniVel / fwdScalar));
		}
		
		
		
		// Setup and start the LFI
		distInterpolator.setConstraints(newMaxAcc, newMaxVel);
		distInterpolator.start(getVelocity(), finalVel, distance);
		
		
		
		
		// loop until the LFI says we are done
		while(distInterpolator.isActive())
		{
			//delay for a bit
			Timer.delay(leftSideData.pid.period);
			
			// feed the master PIDs
			fwdPosPID.setSetpoint(distInterpolator.getTargetPos() * fwdScalar);
			centerPosPID.setSetpoint(distInterpolator.getTargetPos() * centerScalar);
			
			// base velocities calculated via the LFI
			fwdRateAction = distInterpolator.getTargetVel() * fwdScalar;
			centerRateAction = distInterpolator.getTargetVel() * centerScalar;
			
			// feed the slave PIDs
			this.feedSlavePIDs(fwdRateAction, fwdRateAction, centerRateAction, fwdDistAction, centerDistAction, turnPosAction);
		}
		
		log.info("Finished interpolation sequence");
	}
	
	
	
	
	
	
	/**
	 * Drives the robot in a G1(linear) fashion between two points, while maintaining the same
	 * heading angle
	 * @param xDist the horizontal distance to travel
	 * @param fwdDist the distance to travel forward
	 * @param finalVel the velocity of the robot when it reaches the destination position
	 */
	public void drive_G1xyCartesian(double xDist, double fwdDist, double finalVel)
	{
		// heading angle, with navigation orientation
		double heading = (Math.PI / 2) - Math.atan2(fwdDist, xDist);
		heading *= Math.PI / 180;
		
		// total distance to travel
		double distance = Math.sqrt(fwdDist*fwdDist + xDist*xDist);
		
		this.drive_G1xyPolar(heading, distance, finalVel);
	}
	
	
	
	
	
	
	/**
	 * Drives the robot in a G2(arc) interpolation, around a certain radius and for a certain number of degrees
	 * 
	 * @param radius the radius of the arc
	 * @param angleChange the angle, in degrees, of the arc that the robot will travel on
	 * @param finalVel the velocity of the robot when the interpolation is finished
	 */
	public void drive_G2Arc(double radius, double angleChange, double finalVel)
	{
		log.info("Starting a new G2Arc sequence with radius " +radius+ "  and angle of: " +angleChange + "  and final velocity of: " +finalVel);
		
		// gyro angle at the start of the interpolation
		double startAngle = Math.toRadians(getAngle());
		
		resetPIDs();
		fwdPosPID.enable();
		centerPosPID.enable();
		turnPosPID.enable();
		
		// check for illegal moves
		
		if(angleChange == 0) // Illegal move
		{
			log.error("Attempted G2Arc cycle with an angle of 0! Aborting move...");
			return;
		} 

		else if(radius <= 0) // illegal move
		{
			log.error("Attempted G2Arc cycle with a radius of <= 0! Aborting move...");
			return;
		} 

		// convert angle to radians
		angleChange = Math.toRadians(angleChange);
		
		
		// if angle is negative, make radius negative as well for easier math
		radius = angleChange > 0  ?  radius : -1*radius;
		
		
		// calculate the left, right, and nominal distances
		double leftDist = angleChange * (radius + 0.5*wheelbaseWidth);
		double rightDist = angleChange * (radius - 0.5*wheelbaseWidth);
		double nominalDist = angleChange * radius;
			
		// calculate LFI scalar, as the nominal robot velocity will be different from the velocities of the left and right sides
		double lfiScalar = Math.max(Math.abs(leftDist), Math.abs(rightDist)) / nominalDist;
		
		
		// Setup the LFI
		distInterpolator.setConstraints(maxOmniAcceleration / lfiScalar,  maxOmniVel / lfiScalar);
		distInterpolator.start(getVelocity(), finalVel, radius * angleChange);
		
		
		
		
		
		// loop for the duration of the interpolation
		while(distInterpolator.isActive())
		{
			//delay for a bit
			Timer.delay(leftSideData.pid.period);
			
			// nominal velocity / position
			double vNominal = distInterpolator.getTargetVel();
			double sNominal = distInterpolator.getTargetPos();
			
			// feed master PIDs
			fwdPosPID.setSetpoint(sNominal);
			turnPosPID.setSetpoint(Math.toDegrees((sNominal / nominalDist) * angleChange + startAngle));
			
			
			// calculate rate actions, factoring in for forward rate action
			double leftVel = ((vNominal + fwdDistAction)/radius) * (radius + 0.5*wheelbaseWidth);
			double rightVel = ((vNominal + fwdDistAction)/radius) * (radius - 0.5*wheelbaseWidth);
			
			// feed the slaves, knowing that the forward distance action has already been factored in
			feedSlavePIDs(leftVel, rightVel, 0, 0, 0, turnPosAction);
		}
		
	}
	
	
	
	
	/**
	 * Drives the robot in a point turn fashion. internally, this performs a {@link ButterflyHDrive#drive_G2Arc(double, double, double) drive_G2Arc(..)}
	 * with an extremely small radius, performing what is effectively a point turn
	 * @param angle The angle, in degrees, that the robot will turn
	 * @param isRelative if <code>true</code>, the robot turns <code>angle</code> degrees relative to its current 
	 * position. if <code>false</code>, turns to an absolute angle on the field
	 */
	public void drive_G2Point(double angle, boolean isRelative)
	{
		log.info("Starting point turn with angle of: " +angle+ "  and isRelative of: " +isRelative+ ". This will call a G2Arc interpolation.");
		
		// radius that the robot will use
		double radius = 0.0001;
		
		if(!isRelative)
			drive_G2Arc(radius, angle, 0);
		
		else
		{
			double angleChange = angle - AngleUtil.norm360(getAngle());
			drive_G2Arc(radius, angleChange, 0);
		}
		
		log.info("Finished G2Point interpolation");
	}
	
	
	
	
	
	
	/**
	 * Resets all of the PID loops for the drive, and shuts off all outputs
	 */
	public void resetPIDs()
	{
		fwdPosPID.reset();
		centerPosPID.reset();
		turnPosPID.reset();
		
		// reset the drivesides
		leftDriveSide.setRawOutput(0);
		rightDriveSide.setRawOutput(0);
		centerDriveSide.setRawOutput(0);
	}
	
	
	
	
	/**
	 * Updates the setpoints of the slave PID loops
	 */
	private void feedSlavePIDs(double leftVelAction, double rightVelAction, double centerVelAction, double fwdDistAction, double centerDistAction, double turnPosAction)
	{
		// Calculate the velocity setpoints
		double leftSideOut = fwdDistAction + leftVelAction + turnPosAction;
		double rightSideOut = fwdDistAction + rightVelAction - turnPosAction;
		double centerSideOut = centerDistAction + centerVelAction;
		
		// set the setpoints
		leftDriveSide.setSetpoint(leftSideOut);
		rightDriveSide.setSetpoint(rightSideOut);
		centerDriveSide.setSetpoint(centerSideOut);
	}





	///////////////////////////////////////////////////////////////////////////////////////////////////////	
	/////////////////////////////////////////// Getters / Setters /////////////////////////////////////////	
	///////////////////////////////////////////////////////////////////////////////////////////////////////





	
	@Override
	public double getAngle() {
		return navx.getAngle();
	}

	
	
	
	
	@Override
	public double getAngleRate() {
		return navx.getRate();
	}
	
	
	
	
	
	
	/**
	 * Gets the current robot velocity
	 */
	public double getVelocity()
	{
		double fwdVel = 0.5 * (leftDriveSide.getVel() + rightDriveSide.getVel());
		double centerVel = centerDriveSide.getVel();
		
		double resultVel = Math.sqrt(fwdVel*fwdVel + centerVel*centerVel);
		return resultVel;
	}

	
	
	
	
	
	@Override
	public double getY() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	
	
	
	
	@Override
	public double getYRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	
	
	
	
	@Override
	public double getX() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	
	
	
	
	@Override
	public double getXRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	
	
	
	
	@Override	
	protected void initDefaultCommand() 
	{
	}





	public driveType getDriveType() {
		return currDriveType;
	}





	/**
	 * Sets the driving mode of the drivetrain, by specifying which wheels should be omni 
	 * and which should be traction
	 * @param currDriveType
	 */
	public void setDriveType(driveType currDriveType) 
	{
		// make sure a null hasn't been passed
		if(currDriveType == null)
		{
			log.warn("A null value has been passed to setCurrDriveType. Drive type will not be changed.");
			return;
		}


		// log the change, if there is one
		if(currDriveType != this.currDriveType)
			log.info("Drive mode is now " + currDriveType);



		// set the proper solenoids

		// if front traction, set the rear wheels only as omni
		if(currDriveType.equals(driveType.FRONT_TRACTION))
		{
			this.frontSolenoid.set(false ^ this.frontSolenoidData.invert);
			this.rearSolenoid.set(true ^ this.rearSolenoidData.invert);
		}

		// if rear traction, set the front wheels only as omni
		else if(currDriveType.equals(driveType.REAR_TRACTION))
		{
			this.frontSolenoid.set(true ^ this.frontSolenoidData.invert);
			this.rearSolenoid.set(false ^ this.rearSolenoidData.invert);
		}

		// if full traction, set all wheels as traction
		else if(currDriveType.equals(driveType.FULL_TRACTION))
		{
			this.frontSolenoid.set(false ^ this.frontSolenoidData.invert);
			this.rearSolenoid.set(false ^ this.rearSolenoidData.invert);
		}

		// if full omni, set all wheels as omni
		else if(currDriveType.equals(driveType.FULL_OMNI))
		{
			this.frontSolenoid.set(true ^ this.frontSolenoidData.invert);
			this.rearSolenoid.set(true ^ this.rearSolenoidData.invert);
		}

		else
		{
			log.warn("Invalid parameter \"" + currDriveType + "\" has been passed to setCurrDriveType. Driving mode will not be updated.");
			return;
		}


		this.currDriveType = currDriveType;
	}





	public DriveSideData getLeftSideData() {
		return leftSideData;
	}





	public DriveSideData getRightSideData() {
		return rightSideData;
	}





	public DriveSideData getCenterSideData() {
		return centerSideData;
	}





	public SolenoidData getFrontSolenoidData() {
		return frontSolenoidData;
	}





	public SolenoidData getRearSolenoidData() {
		return rearSolenoidData;
	}
	
	
	
	/**
	 * gets the position of the left driveside
	 */
	public double getLeftSidePos()
	{
		double rawPos = ((CANTalon)leftDriveSide.getMotors().get(1)).getEncPosition();
		return rawPos * leftSideData.encoder.distPerCount * (leftSideData.encoder.invert  ?  -1 : 1);
	}
	
	
	
	/**
	 * gets the position of the right driveside
	 */
	public double getRightSidePos()
	{
		double rawPos = ((CANTalon)rightDriveSide.getMotors().get(1)).getEncPosition();
		return rawPos * rightSideData.encoder.distPerCount * (rightSideData.encoder.invert  ?  -1 : 1);
	}
	
	
	
	/**
	 * gets the position of the center driveside
	 */
	public double getCenterSidePos()
	{
		double rawPos = ((CANTalon)centerDriveSide.getMotors().get(0)).getEncPosition();
		return rawPos * centerSideData.encoder.distPerCount * (centerSideData.encoder.invert  ?  -1 : 1);
	}

}







