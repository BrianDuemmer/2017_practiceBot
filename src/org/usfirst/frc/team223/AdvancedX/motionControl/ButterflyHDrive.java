package org.usfirst.frc.team223.AdvancedX.motionControl;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.DriveSideData;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;
import org.usfirst.frc.team223.AdvancedX.robotParser.SolenoidData;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

/**
 * Class for controlling an H drive
 * @author develoer
 *
 */
public class ButterflyHDrive extends Subsystem implements OmniDirectionalDrive 
{
	/////////// Constants and scalars ////////////
	
	// slip constants - multiply these by the power outputs to estimate our slippage
	private double kFwdSlip = 0;
	private double kStrafeSlip = 0;

	// Moon interpolation scalar. The moon trigger is divided by this to calculate
	// the arc radius for mooning
	private double kMoonScalar = 0;	
	
	
	
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
	
	private driveType currDriveType;
	
	
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
	
	
	////////////////// Utility //////////////////
	private Logger log;
	private Command defaultCommand;
	
	
	
	public ButterflyHDrive(AdvancedXManager manager, Command defaultCommand)
	{
		// obtain the logger and parser
		log = manager.getRoboLogger().getLogger("OmniHDrive");
		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();
		
		// log us entering the parse routine
		log.info("\r\n\r\n\r\n================= Initializing Butterfly H Drive =================");
		
		log.info("Default Command is: " + defaultCommand);
		this.defaultCommand = defaultCommand;
		
		// parse the objects
		this.leftSideData = parser.parseDriveSide("Drive/leftSide");
		this.rightSideData = parser.parseDriveSide("Drive/rightSide");
		this.centerSideData = parser.parseDriveSide("Drive/centerSide");
		
		this.frontSolenoidData = parser.parseSolenoid("Drive/frontSolenoid");
		this.rearSolenoidData = parser.parseSolenoid("Drive/rearSolenoid");
		
		
		
		log.info("Attempting to allocate objects...");
		
		// Allocate the objects
		this.leftDriveSide = allocator.allocateDriveSide(this.leftSideData, "LeftSide");
		this.rightDriveSide = allocator.allocateDriveSide(this.rightSideData, "RightSide");
		this.centerDriveSide = allocator.allocateDriveSide(this.centerSideData, "CenterSide");
		
		this.frontSolenoid = allocator.allocateSolenoid(this.frontSolenoidData);
		this.rearSolenoid = allocator.allocateSolenoid(this.rearSolenoidData);
		
		
		// Parse the variables
		this.kFwdSlip = (Double)parser.getKeyByPath("Drive/kFwdSlip", BASIC_TYPE.DOUBLE);
		this.kStrafeSlip = (Double)parser.getKeyByPath("Drive/kStrafeSlip", BASIC_TYPE.DOUBLE);
		this.kMoonScalar = (Double)parser.getKeyByPath("Drive/kMoonScalar", BASIC_TYPE.DOUBLE);
		
		log.info("Finished allocating data");
		
	}
	
	
	
	
	
	public void free()
	{
		this.leftDriveSide.free();
		this.rightDriveSide.free();
		this.centerDriveSide.free();
		
		this.frontSolenoid.free();
		this.rearSolenoid.free();
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
		
		if(this.centerSideData.equals(driveType.FULL_OMNI))
			this.centerDriveSide.setRawOutput(strafe);
		
		else
			this.centerDriveSide.setRawOutput(0);

	}

	@Override
	public double getAngle() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getAngleRate() {
		// TODO Auto-generated method stub
		return 0;
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
		setDefaultCommand(this.defaultCommand);
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
		}
		
		
		
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
		
		
		// log the change
		log.info("Drive mode is now " + currDriveType);
		
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

}







