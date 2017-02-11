package org.usfirst.frc.team223.AdvancedX.utility;

/**
 * Class for performing simple numerical integration and differentiation
 * @author develoer
 *
 */
public class Differentiator extends Thread
{
	@SuppressWarnings("unused")
	private int level;
	private long lastFeedTime;
	private double lastFeedVal;
	
	private double lastRetVal;
	private boolean isintegral;
	
	/**
	 * creates a new differentiator, and starts it automatically
	 * @param level the amount of derivatives / integrals to perform. Eg. if level is 1, the first 
	 * derivative is done, 2 for second, and so on
	 * <P/>
	 * <b>NOTE:</b> as of now only level 1 is supported
	 * @param integrate if true, integrates instead of differentiates
	 */
	public Differentiator(int level, boolean integrate)
	{
		this.level = 1;
	}
	
	public double feed(double val)
	{
		double newVal;
		double dt = (double)(System.currentTimeMillis() - lastFeedTime) / 1000.0;
		
		if(isintegral)
		{
			// integrate using trapezoidal integration
			newVal = (0.5 * (val + lastFeedVal) * dt) + lastRetVal;
		} 
		// differentiate
		else
		{
			newVal = (val - lastFeedVal) / dt;
		}
		
		lastFeedTime = System.currentTimeMillis();
		lastRetVal = newVal;
		lastFeedVal = val;
		return newVal;
	}
	
	
	public void reset()
	{
		lastRetVal = 0;
		lastFeedVal = 0;
		lastFeedTime = System.currentTimeMillis();
	}
	
	
	public double get()
	{
		return lastRetVal;
	}
}





