package org.usfirst.frc.team223.robot.hangar;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HangDebug extends Command 
{
	private static final int FWD_POV = 0;
	private static final int REV_POV = 180;

    public HangDebug() {
    	requires(Robot.hangar);
    }

    
    protected void initialize() 
    {
    }

    
    protected void execute() 
    {
    	int povVal = Robot.oi.operatorController.getPOV(0);
    	
    	
    	//only run in debug mode
    	if(Robot.isDebug)
    	{
    		// if fwd pov pressed
    		if(povVal == FWD_POV)
    			Robot.hangar.setOutput(1);
    		
    		// if rev POV pressed
    		else if (povVal == REV_POV) 
    			Robot.hangar.setOutput(-1);
    		
    		// if POV is not pressed
    		else
    			Robot.hangar.setOutput(0);
			
    	}
    	
		// if not in debug
		else
			Robot.hangar.setOutput(0);
    	
    }

    
    protected boolean isFinished() {
        return false;
    }

    
    protected void end() 
    {
    	Robot.hangar.setOutput(0);
    }

    
    protected void interrupted() 
    {
    	end();
    }
}
