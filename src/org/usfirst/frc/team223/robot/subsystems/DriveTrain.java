package org.usfirst.frc.team223.robot.subsystems;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;

import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

/**
 *
 */
public class DriveTrain extends Subsystem 
{
	private Logger log;
	
	
	
	
	
	
	public DriveTrain(AdvancedXManager manager)
	{
		// obtain the logger and parser
		log = manager.getRoboLogger().getLogger("OmniHDrive");
		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();
		
		// log us entering the parse routine
		log.info("\r\n\r\n\r\n================= Initializing Butterfly H Drive =================");
		
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



    public void initDefaultCommand() 
    {

    }
}

