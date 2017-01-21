package org.usfirst.frc.team223.AdvancedX.utility;

/**
 * Provides a convenient class fo doing some basic angle utilities
 * @author Brian Duemmer
 *
 */
public class AngleUtil {
	
	/**
	 * Takes in any angle and converts it to a range of [0 , 360)
	 * @param angle the angle to scale
	 * @return the scaled angle
	 */
	public static double norm360(double angle)
	{
		double newAngle = angle % 360;
		
		// if newAngle is negative, add 360
		newAngle += newAngle < 0  ?  360 : 0;
		return newAngle;
	}
	
	/**
	 * Takes in any angle and converts it to a range of [-180 , 180)
	 * @param angle the angle to scale
	 * @return the scaled angle
	 */
	public static double norm180(double angle)
	{
		double newAngle = norm360(angle);
		
		// if newAngle is > 180, subtract it from 180
		newAngle = newAngle > 180  ?  180 - newAngle : newAngle;
		return newAngle;
	}
}
