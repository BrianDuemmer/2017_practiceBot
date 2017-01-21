package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * POD Type to store data for a limit switch, as parsed from {@link GXMLparser}
 * @author Brian Duemmer
 *
 */
public class LimitData 
{
	public int id;
	public boolean normallyOpen;
	public EnumPair interruptEdge;
	public double debounceTime;
	
	/**
	 * Sets the parameters for the LimitData Object
	 */
	public LimitData(int id, boolean normallyOpen, EnumPair interruptEdge, double debounceTime)
	{
		this.interruptEdge = interruptEdge;
		this.id = id;
		this.normallyOpen = normallyOpen;
		this.debounceTime = debounceTime;
	}
	
	/**
	 * Default constructor
	 */
	public LimitData(){}
	
	
	public String toString()
	{
		String ret = "";
		
		ret += "interruptEdge: " + interruptEdge + "\n";
		ret += "id:" + id + "\n";
		ret += "normallyOpen: " + normallyOpen + "\n";
		ret += "debounceTime: " + debounceTime + "\n";
		
		return ret;
	}


}

