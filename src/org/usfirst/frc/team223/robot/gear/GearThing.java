package org.usfirst.frc.team223.robot.gear;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;

import edu.wpi.first.wpilibj.Timer;
import net.sf.microlog.core.Logger;

public class GearThing 
{
	// time that the robot will dwell when dropping a gear
	private double dropTime;
	
	Logger log;

	public GearThing(AdvancedXManager manager) 
	{
		// get advancedx objects
		log = manager.getRoboLogger().getLogger("GearThing");
		GXMLparser parser = manager.obtainParser();
		
		// parse data
		log.info("Parsing gear data...");
		dropTime = (Double) parser.getKeyByPath("Gear/dropTime", BASIC_TYPE.DOUBLE);
		log.info("finished parsing gear data");
		
	}
	
	
	
	/**
	 * deposits the gear onto the peg. For now this only is a delay, but it may later 
	 * incorporate actuations and vision targeting as we see fit
	 */
	public void dropGear()
	{
		log.info("Preparing to drop gear...");
		Timer.delay(dropTime);
		log.info("Finished dropping gear");
	}

}
