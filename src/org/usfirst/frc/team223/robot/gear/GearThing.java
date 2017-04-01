package org.usfirst.frc.team223.robot.gear;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BasicType;
import org.usfirst.frc.team223.AdvancedX.robotParser.SolenoidData;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

public class GearThing extends Subsystem
{
	// time that the robot will dwell when dropping a gear
	double dropTime;
	double frontDropTime;
	
	SolenoidData puncherData;
	Solenoid puncher;
	
	SolenoidData jawsData;
	Solenoid jaws;
	SolenoidData frontMechData;
	public Solenoid frontMech;
	
	Logger log;

	public GearThing(AdvancedXManager manager) 
	{
		// get advancedx objects
		log = manager.getRoboLogger().getLogger("GearThing");
		GXMLparser parser = manager.obtainParser();
		
		// parse data
		log.info("Parsing gear data...");
		dropTime = (Double) parser.getKeyByPath("Gear/dropTime", BasicType.DOUBLE);
		frontDropTime = (Double) parser.getKeyByPath("Gear/frontDropTime", BasicType.DOUBLE);
		
		puncherData = parser.parseSolenoid("Gear/puncher");
		jawsData = parser.parseSolenoid("Gear/jaws");
		frontMechData = parser.parseSolenoid("Gear/frontMech");
		
		log.info("finished parsing gear data");
		
		
		// allocate the objects
		log.info("Allocating gear data...");
		
		GXMLAllocator allocator = manager.obtainAllocator();
		puncher = allocator.allocateSolenoid(puncherData);
		jaws =  allocator.allocateSolenoid(jawsData);
		frontMech = allocator.allocateSolenoid(frontMechData);
		
		log.info("Finished allocating gear data");
		
		frontMech.set(false);
	}

	@Override
	protected void initDefaultCommand() {}

}





