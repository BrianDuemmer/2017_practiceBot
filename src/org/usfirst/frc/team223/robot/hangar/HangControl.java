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
    	Robot.hangar.log.info("Starting Hangar...");
    }

    
    protected void execute() 
    {
    	double out = Robot.oi.operatorController.getRawAxis(2);
    	Robot.hangar.setOutput(out);
    }

    
    protected boolean isFinished() {
        return false;
    }

    
    protected void end() 
    {
    	Robot.hangar.log.info("Stopping Hangar...");
    	Robot.hangar.setOutput(0);
    }

    
    protected void interrupted() 
    {
    	end();
    }
}
