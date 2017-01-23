package org.usfirst.frc.team223.robot.hangar;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HangControl extends Command {

    public HangControl() {
    	requires(Robot.hangar);
    }

    
    protected void initialize() 
    {
    	Robot.hangar.log.info("Starting Intake...");
    }

    
    protected void execute() 
    {
    	Robot.hangar.setOutput(1);
    }

    
    protected boolean isFinished() {
        return false;
    }

    
    protected void end() 
    {
    	Robot.hangar.log.info("Stopping Intake...");
    	Robot.hangar.setOutput(0);
    }

    
    protected void interrupted() 
    {
    	end();
    }
}
