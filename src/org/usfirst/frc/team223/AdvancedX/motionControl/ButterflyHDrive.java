package org.usfirst.frc.team223.AdvancedX.motionControl;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.DriveSideData;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;
import org.usfirst.frc.team223.AdvancedX.robotParser.SolenoidData;

import edu.wpi.first.wpilibj.Solenoid;
import net.sf.microlog.core.Logger;

/**
 * Class for controlling an H drive
 * @author develoer
 *
 */
public class ButterflyHDrive implements OmniDirectionalDrive 
{
	/////////// Constants and scalars ////////////
	
	// slip constants - multiply these by the power outputs to estimate our slippage
	private double kFwdSlip = 0;
	private double kStrafeSlip = 0;

	// Moon interpolation scalar. The moon trigger is divided by this to calculate
	// the arc radius for mooning
	private double kMoonScalar = 0;	
	
	
	
	//////////////// Modal Data /////////////////
	
	// Traction settings
	private boolean isFullTraction = false;
	private boolean isHalfTraction = false;

	
	
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
	public void setOutput(double fwd, double strafe, double turn) {
		// TODO Auto-generated method stub

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










	public DriveSideData getLeftSideData() {
		return leftSideData;
	}










	public void setLeftSideData(DriveSideData leftSideData) {
		this.leftSideData = leftSideData;
	}










	public DriveSideData getRightSideData() {
		return rightSideData;
	}










	public void setRightSideData(DriveSideData rightSideData) {
		this.rightSideData = rightSideData;
	}










	public DriveSideData getCenterSideData() {
		return centerSideData;
	}










	public void setCenterSideData(DriveSideData centerSideData) {
		this.centerSideData = centerSideData;
	}










	public SolenoidData getFrontSolenoidData() {
		return frontSolenoidData;
	}










	public void setFrontSolenoidData(SolenoidData frontSolenoidData) {
		this.frontSolenoidData = frontSolenoidData;
	}










	public SolenoidData getRearSolenoidData() {
		return rearSolenoidData;
	}










	public void setRearSolenoidData(SolenoidData rearSolenoidData) {
		this.rearSolenoidData = rearSolenoidData;
	}

}
