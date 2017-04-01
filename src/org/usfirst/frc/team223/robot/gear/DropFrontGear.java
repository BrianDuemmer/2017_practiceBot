package org.usfirst.frc.team223.robot.gear;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * deposits the gear onto the peg from the auto mechanism.
 */
public class DropFrontGear extends Command 
{
    public DropFrontGear() 
    {
    	requires(Robot.gear);
    	setTimeout(Robot.gear.frontDropTime);
    }

    
    protected void initialize() 
    {	
    	Robot.gear.log.info("Preparing to drop gear...");
		Robot.gear.frontMech.set(true);
    }


    protected void end() 
    {
    	Robot.gear.log.info("Finished dropping gear");
//    	Robot.gear.frontMech.set(false);
    }    
    
    
    protected boolean isFinished() { return isTimedOut(); }
}
