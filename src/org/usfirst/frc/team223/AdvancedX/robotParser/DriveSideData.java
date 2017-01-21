package org.usfirst.frc.team223.AdvancedX.robotParser;

import java.util.ArrayList;
import java.util.List;

/**
 * POD Type to store data for a DriveSide, as parsed from {@link GXMLparser}
 * @author Brian Duemmer
 *
 */
public class DriveSideData 
{
	public List<MotorData> motors;
	public EncoderData encoder;
	public PIDData pid;
	
	/**
	 * Sets the parameters for the MotorData Object
	 */
	public DriveSideData(List<MotorData> motors, EncoderData encoder, PIDData pid)
	{
		this.motors = motors;
		this.encoder = encoder;
		this.pid = pid;
	}
	
	/**
	 * Default constructor
	 */
	public DriveSideData()
	{
		motors = new ArrayList<MotorData>();
		encoder = new EncoderData();
		pid = new PIDData();
	}
	
	
	public String toString()
	{
		String ret = "";
		
		ret += "motors: " + motors + "\n";
		ret += "encoder:" + encoder + "\n";
		ret += "pid: " + pid + "\n";
		
		return ret;
	}


}

