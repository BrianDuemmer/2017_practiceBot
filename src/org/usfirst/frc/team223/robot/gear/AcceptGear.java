package org.usfirst.frc.team223.robot.gear;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Accepts a gear from the human player station
 */
public class AcceptGear extends Command 
{
    public AcceptGear() 
    {
    	requires(Robot.gear);
    }

    
    protected void initialize() 
    {
    	Robot.gear.log.info("Preparing to accept gear...");
    	
		// make sure the jaws are open and the puncher is retracted
    	Robot.gear.puncher.set(false);
		Robot.gear.jaws.set(true);
    }


    protected void end() 
    {
    	Robot.gear.log.info("Finished accepting gear");
    	Robot.gear.jaws.set(false);
    }    
    
    
    protected boolean isFinished() { return false; }
}
