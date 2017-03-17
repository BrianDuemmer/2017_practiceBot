package org.usfirst.frc.team223.robot.gear;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * deposits the gear onto the peg. punches off the gear, and dwells for a little bit of 
 * time
 */
public class DropGear extends Command 
{
    public DropGear() 
    {
    	requires(Robot.gear);
    	setTimeout(Robot.gear.dropTime);
    }

    
    protected void initialize() 
    {
		// make sure the jaws are closed
		Robot.gear.jaws.set(false);
		
    	Robot.gear.log.info("Preparing to drop gear...");
		Robot.gear.puncher.set(true);
    }


    protected void end() 
    {
    	Robot.gear.log.info("Finished dropping gear");
    	Robot.gear.puncher.set(false);
    }    
    
    
    protected boolean isFinished() { return isTimedOut(); }
}
