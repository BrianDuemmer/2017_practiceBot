package org.usfirst.frc.team223.robot.common;

import edu.wpi.first.wpilibj.command.Command;
import net.sf.microlog.core.Level;
import net.sf.microlog.core.Logger;

/**
 * Logs a single message using the provided Logger
 */
public class LogMsg extends Command 
{
	private Logger log;
	private Level lv;
	private Object msg;
	private Throwable t;

    public LogMsg(Logger log, Level lv, Object msg) 
    {
    	this(log, lv, msg, null);
    }
    
    public LogMsg(Logger log, Level lv, Object msg, Throwable t) 
    {
    	this.log = log;
    	this.lv = lv;
    	this.msg = msg;
    	this.t = t;
    }


    protected void initialize() 
    {
    	if(t != null) // if a throwable was provided
    		log.log(lv, msg, t);
    	
    	else // if it was not 
    		log.log(lv, msg);
    }


    protected boolean isFinished() { return true; }
}
