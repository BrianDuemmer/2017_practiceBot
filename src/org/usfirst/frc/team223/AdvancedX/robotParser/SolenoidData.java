package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * POD class for parsing {@link edu.wpi.first.wpilibj.Solenoid} data
 * @author Brian Duemmer
 *
 */
public class SolenoidData 
{
	public int port;
	public int PCMid;
	public boolean invert;
	
	public SolenoidData(int port, int PCMid, boolean invert)
	{
		this.port = port;
		this.PCMid = PCMid;
		this.invert = invert;
	}
	
	public SolenoidData() {}
	
	
	public String toString()
	{
		String ret = "";
		ret += "PORT: " + this.port + '\n';
		ret += "PCM ID: " + this.PCMid + '\n';
		ret += "INVERT: " + this.invert + '\n';
		
		return ret;
	}
}
