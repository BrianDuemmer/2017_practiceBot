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
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BasicType;
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
	
	double maxTractionVel;
	double maxTractionAcceleration;
	
	double maxCenterVel;
	double maxCenterAcceleration;
	
	double wheelbaseWidth;
	
	private double omniReductionScalar;


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


	private static final String frontTractionKey = "frontTraction";
	private static final String rearTractionKey = "rearTraction";
	
	private static final String yawAngleKey = "robotYaw";
	
	private static final String leftSideCurrentKey = "leftCurrent";
	private static final String rightSideCurrentKey = "rightCurrent";


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

					manager.getNt().putBoolean(rearTractionKey, rearSolenoid.get() ^ rearSolenoidData.invert);
					manager.getNt().putBoolean(frontTractionKey, frontSolenoid.get() ^ frontSolenoidData.invert);

					manager.getNt().putNumber(yawAngleKey, navx.getAngle());

					manager.getNt().putNumber("leftOut", leftDriveSide.getMotors().get(0).get() * -1);
					manager.getNt().putNumber("rightOut", rightDriveSide.getMotors().get(0).get());

					manager.getNt().putNumber("leftVelSet", leftDriveSide.getSetpoint());
					manager.getNt().putNumber("rightVelSet", rightDriveSide.getSetpoint());
					
					manager.getNt().putNumber("leftPosSet", fwdPosPID.getSetpoint());
					manager.getNt().putNumber("centerDrivePos", getCenterSidePos());
					manager.getNt().putNumber("centerPosSet", centerPosPID.getSetpoint());


					// update the current draw data
					manager.getNt().putNumberArray(leftSideCurrentKey, leftDriveSide.getCurrent(Robot.pdp, leftSideData));
					manager.getNt().putNumberArray(rightSideCurrentKey, rightDriveSide.getCurrent(Robot.pdp, rightSideData));
				} catch(Exception e) // log the error and wait for a bit
				{
					log.error("Exception in Drive Periodic: ", e);
					Timer.delay(10);
				}
			}
		}
	};




	// PID Controller utilities
	
	// Slave loop actions
	double fwdDistAction;
	double turnPosAction;
	double centerDistAction;
	double leftRateAction; 
	double rightRateAction; 

	
	LinearFeedInterpolator distInterpolator;

	
	// Forward position PID Conroller
	PIDController fwdPosPID;
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
			
			
			
			
		
	// Strafe position PID Conroller
	private double strafeSlipScalar;
	PIDController centerPosPID;
	private PIDData centerPosPIDData;
	
	private PIDOutput centerPosPIDOutput = new PIDOutput()
			{
				@Override
				public void pidWrite(double output) 
				{   
//					centerDistAction = output; 
//					log.info("out: "+ output);
					setRawOutput(0, output, 0);
					
				}
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
				public double pidGet() {   return getCenterSidePos();   }
		
			};


			
	// Turn position PIDController
	PIDController turnPosPID;
	PIDData turnPosPIDData;
	
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
			
			
			
		
			
		class MotorOutput
		{
			public double lf;
			public double rf;
			public double lr;
			public double rr;
		}
			
			
			
		
			
			
			
			
			

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
			public PIDSourceType getPIDSourceType() { return PIDSourceType.kDisplacement; }

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

		
		
		// solenoids
		this.frontSolenoid = allocator.allocateSolenoid(this.frontSolenoidData);
		this.rearSolenoid = allocator.allocateSolenoid(this.rearSolenoidData);
		log.info("Finished allocating solenoids");

		
		log.info("Attempting to parse variables...");
		

		// Parse the variables
		maxCenterAcceleration = (Double)parser.getKeyByPath("Drive/center/maxAccel", BasicType.DOUBLE);
		maxCenterVel = (Double)parser.getKeyByPath("Drive/center/maxVel", BasicType.DOUBLE);
		
		maxOmniAcceleration = (Double)parser.getKeyByPath("Drive/omni/maxAccel", BasicType.DOUBLE);
		maxOmniVel = (Double)parser.getKeyByPath("Drive/omni/maxVel", BasicType.DOUBLE);
		
		maxTractionAcceleration = (Double)parser.getKeyByPath("Drive/traction/maxAccel", BasicType.DOUBLE);
		maxTractionVel = (Double)parser.getKeyByPath("Drive/traction/maxVel", BasicType.DOUBLE);
		
		wheelbaseWidth = (Double)parser.getKeyByPath("Drive/wheelbaseWidth", BasicType.DOUBLE);
		
		strafeSlipScalar = (Double)parser.getKeyByPath("Drive/strafeSlipScalar", BasicType.DOUBLE);
		
		omniReductionScalar = (Double)parser.getKeyByPath("Drive/omniReductionScalar", BasicType.DOUBLE);
		
		
		// Allocate misc. objects
		navx = new GyroNavX(Port.kMXP);
		distInterpolator = new LinearFeedInterpolator(log, maxOmniAcceleration, maxOmniVel);

		// start the drive periodic
		drivePeriodic.start();
		
		//preliminarily reset the PIDs
		resetPIDs();

		log.info("Finished allocating ButterflyHDrive data");

	}





	public void free()
	{
		shouldStop = true;
		log.info("Attempting to free ButterflyHDrive...");


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
//		log.info("sro");
		
//		if(this.currDriveType == driveType.FULL_OMNI)
//			centerDriveSide.setRawOutput(0);
//		else
//			centerDriveSide.setRawOutput(strafe);
//		
//		 leftDriveSide.setRawOutput(fwd + turn);
//		 rightDriveSide.setRawOutput(fwd - turn);
		
		if(this.currDriveType != driveType.FULL_OMNI)
			strafe = 0;
		
		MotorOutput out = calcMotorOutputs(fwd, strafe, turn);
		setRawOutput(out);
	}
	
	
	private void setRawOutput(MotorOutput out)
	{
		// set the outputs
		leftDriveSide.getMotors().get(0).set(out.lf);
		leftDriveSide.getMotors().get(1).set(out.lr);
		
		rightDriveSide.getMotors().get(0).set(out.rf);
		rightDriveSide.getMotors().get(1).set(out.rr);
	}
	
	
	public MotorOutput calcMotorOutputs(double fwd, double strafe, double turn)
	{
		MotorOutput out = new MotorOutput();
		
		// calc the raw output for each wheel
		out.lf = fwd + strafe + turn;
		out.rf = fwd - strafe - turn;
		out.lr = fwd - strafe + turn;
		out.rr = fwd + strafe - turn;
		
		// get the wheel output with the largest abs()
		double scalar = Math.max(Math.max(Math.abs(out.rr), Math.abs(out.lr)), Math.max(Math.abs(out.rf), Math.abs(out.lf)));
		
		// divide by the scalar, if necessary
		if(scalar > 1)
		{
			out.lf /= scalar;
			out.rf /= scalar;
			out.lr /= scalar;
			out.rr /= scalar;
		}
		
		return out;
	}
	
	
	
	
	
	/**
	 * Drives the robot in a G1(linear) fashion between two points, while maintaining the same
	 * heading angle
	 * 
	 * @deprecated use the command version of this instead
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
		
		// set the turning PID to maintain the current yaw
//		turnPosPID.setSetpoint(getAngle());
		
		// convert heading to radians
		heading = Math.toRadians(heading);
		
		// scalars for the forward / center velocities and positions
		double fwdScalar = Math.cos(heading);
		double centerScalar = Math.sin(-1*heading);
		
		
		
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
		
		// Enable the PIDs, set the setpoints to 0, the current position
		fwdPosPID.enable();
		fwdPosPID.setSetpoint(fwdPosPIDSource.pidGet());
		
		centerPosPID.enable();
		centerPosPID.setSetpoint(centerPosPIDSource.pidGet());
		
		turnPosPID.enable();
		turnPosPID.setSetpoint(turnPosPIDSource.pidGet());
		
		// Setup and start the LFI
		distInterpolator.setConstraints(newMaxAcc, newMaxVel);
		
		double currVel = getVelocity();
		log.info("Current velocity is: " +currVel);
		distInterpolator.start(currVel, finalVel, distance);
		
		
		
		
		// loop until the LFI says we are done
		while(!Robot.oi.button_dBack.get() && (distInterpolator.isActive() || (!fwdPosPID.onTarget() && centerPosPID.onTarget())))
		{	
			// feed the master PIDs
			fwdPosPID.setSetpoint(distInterpolator.getTargetPos() * fwdScalar);
			centerPosPID.setSetpoint(distInterpolator.getTargetPos() * centerScalar);
			
			// base velocities calculated via the LFI
//			fwdRateAction = distInterpolator.getTargetVel() * fwdScalar;
//			centerRateAction = distInterpolator.getTargetVel() * centerScalar;
//			
//			// feed the slave PIDs
//			this.feedLRSlavePIDs(fwdRateAction, fwdRateAction, centerRateAction, fwdDistAction, centerDistAction, /*turnPosAction*/ 0);
		
			//delay for a bit
			Timer.delay(leftSideData.pid.period / 2);
		}
		
		log.info("Finished interpolation sequence");
	}
	
	
	
	
	
	
	/**
	 * Drives the robot in a G1(linear) fashion between two points, while maintaining the same
	 * heading angle
	 * 
	 * @deprecated use the command version of this instead
	 * 
	 * @param xDist the horizontal distance to travel
	 * @param fwdDist the distance to travel forward
	 * @param finalVel the velocity of the robot when it reaches the destination position
	 */
	public void drive_G1xyCartesian(double xDist, double fwdDist, double finalVel)
	{
		// heading angle, with navigation orientation
		double heading = (Math.PI / 2) - Math.atan2(fwdDist, xDist);
		heading = Math.toDegrees(heading);
		
		// total distance to travel
		double distance = Math.sqrt(fwdDist*fwdDist + xDist*xDist);
		
		this.drive_G1xyPolar(heading, distance, finalVel);
	}
	
	
	
	
	
	
	/**
	 * Drives the robot in a G2(arc) interpolation, around a certain radius and for a certain number of degrees
	 * 
	 * @deprecated use the command version of this instead
	 * 
	 * @param radius the radius of the arc
	 * @param angleChange the angle, in degrees, of the arc that the robot will travel on
	 * @param finalVel the velocity of the robot when the interpolation is finished
	 */
	public void drive_G2Arc(double radius, double angleChange, double finalVel)
	{
		log.info("Starting a new G2Arc sequence with radius " +radius+ "  and angle of: " +angleChange + "  and final velocity of: " +finalVel);
		
		// gyro angle at the start of the interpolation
		double startAngle = getAngle();
		
		// check for illegal moves
		
		if(angleChange == 0) // Illegal move
		{
			log.error("Attempted G2Arc cycle with an angle of 0! Aborting move...");
			return;
		} 

//		else if(radius <= 0) // illegal move
//		{
//			log.error("Attempted G2Arc cycle with a radius of <= 0! Aborting move...");
//			return;
//		} 


//		// if angle is negative, make radius negative as well for easier math
//		radius = angleChange > 0  ?  radius : -1*radius;
//		
//		angleChange = Math.abs(angleChange);
		
		
		// convert angle to radians
		angleChange = Math.toRadians(angleChange);		
		
		// calculate the left, right, and nominal distances
		double leftDist;
		double rightDist;
		
		if(angleChange > 0)
		{
			leftDist = angleChange * (radius + 0.5*wheelbaseWidth);
			rightDist = angleChange * (radius - 0.5*wheelbaseWidth);
		} else
		{
			leftDist = -angleChange * (radius - 0.5*wheelbaseWidth);
			rightDist = -angleChange * (radius + 0.5*wheelbaseWidth);
			log.info("negAngle");
		}
		
		double nominalDist = angleChange * radius;
			
		// calculate LFI scalar, as the nominal robot velocity will be different from the velocities of the left and right sides
		double lfiScalar = Math.max(Math.abs(leftDist), Math.abs(rightDist)) / Math.abs(nominalDist);
		
		// get the master PIDs ready
		resetPIDs();
		
		fwdPosPID.enable();
		centerPosPID.enable();
		turnPosPID.enable();
		
		
		// Setup the LFI, making sure to reduce everything by the lfiScalar value
		distInterpolator.setConstraints(maxOmniAcceleration / lfiScalar,  maxOmniVel / lfiScalar);
		distInterpolator.start(getVelocity() / lfiScalar, finalVel / lfiScalar, radius * Math.abs(angleChange));
		
		
		
		log.info("ldist: " + leftDist + "  rdist: " +rightDist+ "   lfiScl: " +lfiScalar);
		
		boolean pastSet = false;
		
		// loop for the duration of the interpolation
		while(!Robot.oi.button_dBack.get() && (distInterpolator.isActive()  || !pastSet))
		{
			//delay for a bit
			Timer.delay(leftSideData.pid.period);
			
			if(angleChange > 0)
				pastSet = getAngle() + turnPosPIDData.tolerance > startAngle + Math.toDegrees(angleChange);
			else
				pastSet = getAngle() - turnPosPIDData.tolerance < startAngle + Math.toDegrees(angleChange);
			
			manager.getNt().putBoolean("pastSet", pastSet);
			
			// nominal velocity / position
			double vNominal = distInterpolator.getTargetVel();
			double sNominal = distInterpolator.getTargetPos();
			
			// feed master PIDs
			fwdPosPID.setSetpoint(sNominal);
			turnPosPID.setSetpoint(Math.toDegrees((sNominal / Math.abs(nominalDist)) * angleChange) + startAngle);
			
			// calculate adjusted forward distance
			double adjFwd = fwdDistAction / lfiScalar;
			
			double leftVel;
			double rightVel;
		
			
			// calculate rate actions, factoring in for forward rate action
			leftVel = ((vNominal + adjFwd)/radius) * (radius + Math.signum(angleChange)*wheelbaseWidth * 0.5);
			rightVel = ((vNominal + adjFwd)/radius) * (radius - Math.signum(angleChange)*wheelbaseWidth * 0.5);
//				
//			double k = 1.4;
//			
//			if(angleChange > 0)
//			{
//				leftVel *= k;
//				rightVel /= k;
//			} else 
//			{
//				leftVel /= k;
//				rightVel *= k;
//			}
	
//			double velA = (radius + 0.5*wheelbaseWidth) * ((vNominal + adjFwd) / radius); 
//			double velB = (radius - 0.5*wheelbaseWidth) * ((vNominal + adjFwd) / radius); 
//			
//			if(angleChange > 0)
//			{
//				leftVel = velA;
//				rightVel = velB;
//			} else
//			{
//				leftVel = velB;
//				rightVel = velA;
//			}
			
			
			// feed the slaves, knowing that the forward distance action has already been factored in
			feedLRSlavePIDs(leftVel, rightVel, 0, turnPosAction);
		}
		
		resetPIDs();
		setRawOutput(0, 0, 0);
		
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
		// reset the position integrators
		((CANTalon)leftDriveSide.getMotors().get(1)).setEncPosition(0);
		((CANTalon)rightDriveSide.getMotors().get(1)).setEncPosition(0);
		
		fwdPosPID.reset();
		fwdPosPID.setSetpoint(0);
		
		centerPosPID.reset();
		centerPosPID.setSetpoint(0);
		
		turnPosPID.reset();
		turnPosPID.setSetpoint(0);
		
		// reset the drivesides
		leftDriveSide.setSetpoint(0);
		leftDriveSide.setRawOutput(0);
		
		rightDriveSide.setSetpoint(0);
		rightDriveSide.setRawOutput(0);

		
		// delay for a tad so the changes take effect
		Timer.delay(0.05);
	}
	
	
	
	
	/**
	 * Updates the setpoints of the slave PID loops
	 */
	void feedLRSlavePIDs(double leftVelAction, double rightVelAction, double fwdDistAction, double turnPosAction)
	{
		// Calculate the velocity setpoints
		double leftSideOut = fwdDistAction + leftVelAction + turnPosAction;
		double rightSideOut = fwdDistAction + rightVelAction - turnPosAction;
		
		// set the setpoints
		leftDriveSide.setSetpoint(leftSideOut);
		rightDriveSide.setSetpoint(rightSideOut);
	}
	
	
	
	/**
	 * Updates the setpoints of the slave PID loops
	 */
	public void feedLRSlavePIDs()
	{
		// Calculate the velocity setpoints
		double leftSideOut = fwdDistAction + leftRateAction + turnPosAction;
		double rightSideOut = fwdDistAction + rightRateAction - turnPosAction;
		
		// set the setpoints
		leftDriveSide.setSetpoint(leftSideOut);
		rightDriveSide.setSetpoint(rightSideOut);
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
		return fwdVel;
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
	public void setDriveType(driveType currDriveType, boolean force) 
	{
		// make sure a null hasn't been passed
		if(currDriveType == null)
		{
			log.warn("A null value has been passed to setCurrDriveType. Drive type will not be changed.");
			return;
		}


		// log the change, if there is one
		if(currDriveType != this.currDriveType || force)
			log.info("Drive mode is now " + currDriveType);
		
		// return if there is not a change
		else
			return;



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
			this.rearSolenoid.set(false ^ this.rearSolenoidData.invert);
			Timer.delay(0.25);
			this.frontSolenoid.set(false ^ this.frontSolenoidData.invert);
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
	 * gets the position of the center driveside. This is done by looking at the 
	 * forwards distance and estimating from there
	 */
	public double getCenterSidePos()
	{	
		// get the measured forwards distance, and use that to estimate the actual distance strafed
		return getLeftSidePos() * strafeSlipScalar * -1 * omniReductionScalar;
	}

}







