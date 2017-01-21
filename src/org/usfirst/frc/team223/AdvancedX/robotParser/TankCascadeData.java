package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * POD Type to store data for a Tank Cascade Controller, as parsed from {@link GXMLparser}
 * @author Brian Duemmer
 *
 */
public class TankCascadeData 
{
	public DriveSideData leftData;
	public DriveSideData rightData;
	public PIDData distancePID;
	public PIDData anglePID;
	
	public double wheelBaseWidth;
	
	/**
	 * Sets the parameters for the TankCascadeData Object
	 */
	public TankCascadeData(DriveSideData leftData, DriveSideData rightData, PIDData distancePID, PIDData anglePID, double wheelBaseWidth)
	{
		this.anglePID = anglePID;
		this.distancePID = distancePID;
		this.leftData = leftData;
		this.rightData = rightData;
		this.wheelBaseWidth = wheelBaseWidth;
	}
	
	/**
	 * Default constructor
	 */
	public TankCascadeData()
	{
		anglePID = new PIDData();
		distancePID = new PIDData();
		leftData = new DriveSideData();
		rightData = new DriveSideData();
	}
	
	
	public String toString()
	{
		String ret = "";
		
		ret += "anglePID: " + anglePID + "\n";
		ret += "distancePID:" + distancePID + "\n";
		ret += "wheelBaseWidth: " + wheelBaseWidth + "\n";
		ret += "leftData:" + leftData + "\n";
		ret += "rightData: " + rightData + "\n";
		
		return ret;
	}


}