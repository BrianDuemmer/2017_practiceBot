package org.usfirst.frc.team223.AdvancedX.logging;

import net.sf.microlog.core.Level;
import net.sf.microlog.core.appender.ConsoleAppender;


/** Identical to the regular {@link ConsoleAppender} but this allows for appender
 * specific log level filtering
 * 
 * 
 * @author Duemmer
 *
 */
public class FilteredConsoleAppender extends ConsoleAppender 
{
	// Level cutoff that will be used to filter log messages
	private Level level;
	
	
	public FilteredConsoleAppender() {   super();   }
	
	
	/**
	 * Set the logging level of this specific appender. If this is not set, or it
	 * is set as null, all messages will be logged
	 * @param level
	 */
	public void setLevel(Level level){   this.level = level;   }
	
	
	
	
	
	/** 
	 * {@inheritDoc}. Automatically filters log messages based on the level specified
	 * with {@link #setLevel(Level)}. If {@link #setLevel(Level)} is not set, or it
	 * is set as null, all messages will be logged by default.
	 */
	public void doLog(String clientID, String name, long time, Level level, Object message, Throwable t) 
	{
		// Log if the level has not been set, or the priority of the message's log is higher than the one set in level
		if(this.level == null || this.level.toInt() <= level.toInt())
			super.doLog(clientID, name, time, level, message, t);
	}
}










