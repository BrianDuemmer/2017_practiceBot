package org.usfirst.frc.team223.AdvancedX.robotParser;

/**
 * Acts as a simple POD type to pair an enum string with an integer ID
 * @author Duemmer
 *
 */
public class EnumPair 
{
	public String selection;
	public int value;
	
	public EnumPair(){}
	
	public EnumPair(String selection, int value)
	{
		this.selection = selection;
		this.value = value;
	}
	
	public String toString()
	{
		String ret = "";
		
		ret += "selection: " + selection + "\n";
		ret += "value: " + value + "\n";
		
		return ret;
	}
}
