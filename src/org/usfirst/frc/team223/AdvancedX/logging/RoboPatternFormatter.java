package org.usfirst.frc.team223.AdvancedX.logging;

import org.usfirst.frc.team223.AdvancedX.RoboLogManagerBase;

import edu.wpi.first.wpilibj.Timer;
import net.sf.microlog.core.Level;
import net.sf.microlog.core.format.PatternFormatter;


/**
 * A {@link RoboLogManagerBase} compatible class that adds a few extra pattern specifiers to the
 * {@link PatternFormatter} class that ships with Microlog. Currently, these extra formatters
 * include: <P/>
 * %M - match time <P/>
 * 
 * @see PatternFormatter
 * @author Brian Duemmer
 *
 */
public class RoboPatternFormatter extends PatternFormatter 
{
	
	private static final char MATCH_TIME_CHAR = 'M';
	
	
	
	
	public String format(String clientID, String name, long time, Level level, Object message, Throwable t)
	{
		String msg = super.format(clientID, name, time, level, message, t);
		return parseCustomArgs(msg);
	}
	
	
	
	public void setPattern(String pattern)
	{
		super.setPattern(preProcess(pattern));
	}
	
	
	/**
	 * Extracts any custom arguments added by {@link RoboPatternFormatter} and
	 * inlines them into the returned pattern
	 * @param pattern the raw pattern
	 * @return pattern, with all custom args replaced by their interpereted values
	 */
	private String parseCustomArgs(String str)
	{
		// add a new line for each character sequence
		str = str.replaceAll("%" + MATCH_TIME_CHAR, new Double(Timer.getMatchTime()).toString());
		
		return str;
	}
	
	
	
	/**
	 * Process the raw pattern string to compensate for the new conversion 
	 * characters. This is accomplished by adding a second percent symbol 
	 * before any new conversion characters, which causes it to be ignored 
	 * by the default PatternFormatter and processed by the RoboPatternFormatter
	 */
	private String preProcess(String raw)
	{
		raw = raw.replaceAll("%" + MATCH_TIME_CHAR, "%%" + MATCH_TIME_CHAR);
		return raw;
	}
}











