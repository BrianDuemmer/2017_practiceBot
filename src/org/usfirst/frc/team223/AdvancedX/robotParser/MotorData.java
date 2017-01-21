package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * POD Type to store data for a motor, as parsed from {@link GXMLparser}
 * @author Brian Duemmer
 *
 */
public class MotorData 
{
	public EnumPair type;
	public int id;
	public boolean invert;
	public boolean brake;
	public double maxOut;
	
	/**
	 * Sets the parameters for the MotorData Object
	 */
	public MotorData(EnumPair type, int id, boolean invert, boolean brake, double maxOut)
	{
		this.type = type;
		this.id = id;
		this.brake = brake;
		this.invert = invert;
		this.maxOut = maxOut;
	}
	
	/**
	 * Default constructor
	 */
	public MotorData(){}
	
	
	public String toString()
	{
		String ret = "";
		
		ret += "type: " + type + "\n";
		ret += "id:" + id + "\n";
		ret += "brake: " + brake + "\n";
		ret += "invert: " + invert + "\n";
		ret += "maxOut: " + maxOut + "\n";
		
		return ret;
	}


}
