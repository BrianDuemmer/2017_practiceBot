package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * POD Type to store data for an encoder, as parsed from {@link GXMLparser}.
 * Note that this also includes encoders connected to CANTalons 
 * @author Brian Duemmer
 *
 */
public class EncoderData 
{
	public int Achannel;
	public int Bchannel;
	public int IDXchannel;
	public boolean invert;
	public double distPerCount;
	
	/**
	 * Sets the parameters for the MotorData Object
	 */
	public EncoderData(int Achannel, int Bchannel, int IDXchannel, boolean invert, double distPerCount)
	{
		this.Achannel = Achannel;
		this.Bchannel = Bchannel;
		this.IDXchannel = IDXchannel;
		this.invert = invert;
		this.distPerCount = distPerCount;
	}
	
	/**
	 * Default constructor
	 */
	public EncoderData(){}
	
	
	public String toString()
	{
		String ret = "";
		
		ret += "Achannel: " + Achannel + "\n";
		ret += "Bchannel:" + Bchannel + "\n";
		ret += "IDXchannel: " + IDXchannel + "\n";
		ret += "invert: " + invert + "\n";
		ret += "distPerCount: " + distPerCount + "\n";
		
		return ret;
	}


}

