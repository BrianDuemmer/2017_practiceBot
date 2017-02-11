package org.usfirst.frc.team223.robot.hangar;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.MotorData;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

/**
 *
 */
public class Hangar extends Subsystem 
{
	
	private MotorData motorData;
	private SpeedController motor;
	private AdvancedXManager manager;
	
	Logger log;
	
	
	public Hangar(AdvancedXManager manager)
	{
		log = manager.getRoboLogger().getLogger("HANGAR");
		log.info("Initializing Hangar Subsystem...");
		
		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();
		this.manager = manager;
		
		this.motorData = parser.parseMotor("Hangar/motor");
		this.motor = allocator.allocateMotor(this.motorData);
	}
	
	
	public void free()
	{
		log.info("Attempting to free hangar...");
		manager.destroy(motor);
		log.info("Finished freeing hangar");
	}



    public void initDefaultCommand() 
    {
    }
    
    
    public void setDefaultCommand(Command cmd)
    {
    	super.setDefaultCommand(cmd);
    }
    
    
    public void setOutput(double output)
    {
    	this.motor.set(output);
    }
}

