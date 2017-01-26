package org.usfirst.frc.team223.AdvancedX;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

import org.usfirst.frc.team223.AdvancedX.logging.FilteredConsoleAppender;
import org.usfirst.frc.team223.AdvancedX.logging.FilteredFileAppender;
import org.usfirst.frc.team223.AdvancedX.logging.FilteredSocketAppender;
import org.usfirst.frc.team223.AdvancedX.logging.RoboPatternFormatter;

import net.sf.microlog.core.Formatter;
import net.sf.microlog.core.Level;
import net.sf.microlog.core.Appender;
import net.sf.microlog.core.Logger;
import net.sf.microlog.core.LoggerFactory;




/**
 * Team class for logging robot data. Extend and configure this as needed, or use this 
 * if you don't want to configure it
 * @author Brian Duemmer
 *
 */
public class RoboLogManagerBase {

	// Format string for all loggers
	protected String fmtString = "[%P] %d{ABSOLUTE}___MATCH %M [%c{1}]   %m %T\r\n";
	
	// logging directory
	protected String logDir;
	
	// Appender stuff
	protected RoboPatternFormatter defaultFormPattern = new RoboPatternFormatter();
	
	// List of all appenders
	protected List<Appender> appenders = new ArrayList<Appender>();
	
	protected Level defaultLevel = Level.TRACE;
	
	
	
	
	
	
	/**
	 * Initializes the RoboLogManagerBase object
	 * @param logDir The directory that log files will be saved in. If null or empty string, disables
	 * file logging
	 * @param socketPort the port that clients can connect to to receive log messages. If the value is <=0 or 
	 * >= 65536, this feature is disabled
	 * @param dsConsoleLevel the minimum {@link Level} that will be logged to the DS Console
	 */
	public RoboLogManagerBase(String logDir, int socketPort, Level dsConsoleLevel) 
	{
		this.logDir = logDir;
		
		// Set the pattern
		this.defaultFormPattern.setPattern(fmtString);
		
		// Setup the appenders
		this.appenders.add(initSocketAppender(defaultFormPattern, this.defaultLevel, socketPort));
		this.appenders.add(initFileAppender(defaultFormPattern, this.defaultLevel, logDir));
		this.appenders.add(initConsoleAppender(defaultFormPattern, dsConsoleLevel));

		
		// Initialize the logger
		Logger log = LoggerFactory.getLogger(RoboLogManagerBase.class);
		configLogger(log);

	}
	
	
	
	
	
	public Logger getLogger(String name)
	{
		// Initialize the logger
		Logger log = LoggerFactory.getLogger(name);
		configLogger(log);
		
		return log;
	}
	
	
	
	
	private Logger configLogger(Logger log)
	{
		// set the root level. Set to trace so that the appenders can take control
		// of what gets logged and what doesn't
		log.setLevel(Level.TRACE);
		
		// only add non-null appenders to the logger
		for(Appender i: this.appenders)
			if(i != null)
				log.addAppender(i);
		
		
		// return the finished logger
		return log;
	}
	
	
	
	/**
	 * Creates the {@link FilteredSocketAppender} that the logger will use
	 */
	private FilteredSocketAppender initSocketAppender(Formatter fmt, Level minLevel, int port)
	{
		// if the port is out of bounds, return null
		if(port <= 0 || port >= 65536)
			return null;
		else
		{
			FilteredSocketAppender appender = new FilteredSocketAppender(port);
			appender.setFormatter(fmt);
			appender.setMinLevel(minLevel);
			
			return appender;
		}
	}
	
	
	
	
	/**
	 * Creates the {@link FilteredConsoleAppender} that the logger will use
	 */
	private FilteredConsoleAppender initConsoleAppender(Formatter fmt, Level minLevel)
	{

			FilteredConsoleAppender appender = new FilteredConsoleAppender();
			appender.setFormatter(fmt);
			appender.setLevel(minLevel);
			
			return appender;
	}
	
	
	
	
	
	/**
	 * Creates the {@link FilteredFileAppender} that the logger will use. If logDir is 
	 * null or empty, then file logging will be disabled
	 */
	private FilteredFileAppender initFileAppender(Formatter fmt, Level minLevel, String logDir)
	{
		if(logDir == null || logDir.equals(""))
			return null;
		
		// make sure the logging dir exists
		mkdirIfNeeded(logDir);
		
		// initialize the calendar to the date
		Date date = new Date();
		Calendar cal = Calendar.getInstance();
		cal.setTime(date);
		
		// get the date components
		int year = cal.get(Calendar.YEAR);
		int month = cal.get(Calendar.MONTH) + 1;
		int day = cal.get(Calendar.DAY_OF_MONTH);
		
		// format the subdirectory name
		String dirName = year +"-"+ month +"-" + day;
		
		// get the time components
		int hour = cal.get(Calendar.HOUR_OF_DAY);
		int minute = cal.get(Calendar.MINUTE);
		int second = cal.get(Calendar.SECOND);
		
		// format the log file name
		String logName = hour +"h-"+ minute +"m-"+ second + "s_roboLog.log";
		
		
		// make sure the logging subdirectory exists
		mkdirIfNeeded(logDir +"/"+ dirName);
		
		// full path to the log file
		String fullLogPath = logDir +"/"+ dirName +"/"+ logName;

		FilteredFileAppender appender = new FilteredFileAppender();
		appender.setFilePath(fullLogPath);
		appender.setFormatter(fmt);
		appender.setLevel(minLevel);
		
		return appender;
	}
	
	
	
	private static void mkdirIfNeeded(String directoryName)
	{
		File theDir = new File(directoryName); 
		if (!theDir.exists())
		    theDir.mkdirs();
	}

}












