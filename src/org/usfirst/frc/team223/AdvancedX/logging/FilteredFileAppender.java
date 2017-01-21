package org.usfirst.frc.team223.AdvancedX.logging;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import net.sf.microlog.core.Appender;
import net.sf.microlog.core.Formatter;
import net.sf.microlog.core.Level;





public class FilteredFileAppender implements Appender
{
	// PrintWriter to output to file
	private PrintWriter fileOut;
	
	// Formatter object
	private Formatter formatter;
	
	// if true, the file is open and ready to accept logs
	private boolean logOpen;
	
	// path to the log file
	private String filePath;
	
	// Base level for the appender
	private Level baseLevel;
	
	
	
	/**
	 * Creates a new appender to log to the file at the specified path
	 */
	public FilteredFileAppender()
	{
		// initialize instance variables
		this.filePath = "";
		logOpen = false;
		formatter = null;
		this.baseLevel = Level.TRACE;
		
		// try to initialize the PrintWriter
		try {
			open();
		} catch(Exception e) {
			System.err.println("Failed to initialize file appender. DETAILS: ");
			e.printStackTrace();
		}
	}

	
	
	
	/**
	 * perform the log operation
	 */
	@Override
	public void doLog(String clientID, String name, long time, Level level, Object message, Throwable t) 
	{
		// only proceed if the log is open, the level is at or above the base level, and the formatter isn't null
		if(this.logOpen && formatter != null && this.baseLevel.toInt() <= level.toInt())
		{
			// get the string to log to the file
			String msg = formatter.format(clientID, name, time, level, message, t);
			
			// Attempt to write it to the file
			try
			{
				fileOut.println(msg);
				fileOut.flush();
			} 
			catch(Exception e) {
				System.err.println("Failed to log message " + e);
			}
		}
	}

	
	
	
	
	/**
	 * Clears all of the contents out of the log file, to prepare it for being logged to again
	 */
	@Override
	public void clear() 
	{
		// make sure that the fileOut is closed
		if(this.fileOut != null)
			this.fileOut.close();
		
		// Re-open the fileOut. This will automatically overwrite the file
		try 
		{
			fileOut = new PrintWriter(this.filePath, "UTF-8");
		} catch (Exception e) {
			System.err.println("Error clearing file. DETAILS: " + e);
		}

	}

	
	
	
	
	
	/**
	 * Close the appender and flush everything to the log file
	 * @throws IOException
	 */
	@Override
	public void close() throws IOException 
	{
		// Close the printWriter if opened, and set the logOpen flag to false
		if(this.fileOut != null)
		{
			this.fileOut.flush();
			this.fileOut.close();
			this.logOpen = false;
		}
	}
	

	
	
	
	/**
	 * Open the {@link PrintWriter} and prepare the appender for the appending
	 */
	@Override
	public void open() throws IOException 
	{
		this.fileOut = new PrintWriter(filePath, "UTF-8");
		logOpen = true;
	}

	
	
	
	
	
	@Override
	public boolean isLogOpen() 
	{
		return this.logOpen;
	}

	
	
	
	
	
	
	@Override
	public long getLogSize() 
	{
		long len;
		
		// Create a new instance of the File class to get the length
		File file = new File(this.filePath);
		
		// If the file exists, get the length. If not, return 0
		if(file.exists())
			len = file.length();
		else
			len = 0;
		
		return len;
	}

	
	
	
	
	
	
	@Override
	public void setFormatter(Formatter formatter) 
	{
		this.formatter = formatter;
	}

	
	
	
	
	
	
	@Override
	public Formatter getFormatter() 
	{
		return this.formatter;
	}

	
	
	
	
	
	
	@Override
	public String[] getPropertyNames() 
	{
		return null;
	}

	
	
	
	
	
	
	@Override
	public void setProperty(String name, String value) throws IllegalArgumentException 
	{
		
	}




	public Level getLevel() {
		return baseLevel;
	}




	public void setLevel(Level baseLevel) {
		this.baseLevel = baseLevel;
	}




	public void setFilePath(String filePath) {
		this.filePath = filePath;
	}

}














