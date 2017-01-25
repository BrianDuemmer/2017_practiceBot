package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * POD Type to store data for a PID loop, as parsed from {@link GXMLparser}
 * @author Brian Duemmer
 *
 */
public class PIDData 
{
	public double kp;
	public double ki;
	public double kd;
	public double kf;
	public double period;
	public double tolerance;
	public double min;
	public double max;
	public boolean continuous;
	
	/**
	 * Sets the parameters for the PIDData Object
	 */
	public PIDData(double kp, double ki, double kd, double kf, double period, double tolerance, double min, double max, boolean continuous)
	{
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kf = kf;
		this.period = period;
		this.tolerance = tolerance;
		this.min = min;
		this.max = max;
		this.continuous = continuous;
	}
	
	/**
	 * Default constructor
	 */
	public PIDData()
	{
		// Set the period to a safe value
		this.period = 0.02;
	}
	
	public String toString()
	{
		String ret = "";
		
		ret += "kp: " + kp + "\n";
		ret += "ki: " + ki + "\n";
		ret += "kd: " + kd + "\n";
		ret += "kf: " + kf + "\n";
		ret += "period: " + period + "\n";
		ret += "tolerance: " + tolerance + "\n";
		ret += "min: " + min + "\n";
		ret += "max: " + max + "\n";
		ret += "continuous: " + continuous + "\n";
		
		return ret;
	}

}
