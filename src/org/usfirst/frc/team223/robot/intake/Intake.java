package org.usfirst.frc.team223.robot.intake;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.MotorData;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import net.sf.microlog.core.Logger;

/**
 *
 */
public class Intake extends Subsystem 
{
	
	private MotorData motorData;
	private SpeedController motor;
	
	Logger log;
	
	
	public Intake(AdvancedXManager manager)
	{
		log = manager.getRoboLogger().getLogger("INTAKE");
		log.info("Initializing Intake Subsystem...");
		
		GXMLparser parser = manager.obtainParser();
		GXMLAllocator allocator = manager.obtainAllocator();
		
		this.motorData = parser.parseMotor("intakeMotor");
		this.motor = allocator.allocateMotor(this.motorData);
	}
	
	
	public void free()
	{
	}



    public void initDefaultCommand() 
    {

    }
    
    
    public void setOutput(double output)
    {
    	this.motor.set(output);
    }
}
