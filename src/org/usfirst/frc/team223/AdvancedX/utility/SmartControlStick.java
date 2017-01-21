package org.usfirst.frc.team223.AdvancedX.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This provides an intelligent wrapper for controlling analog sticks
 * on xbox / ps3 / etc. controllers. This provides several extra methods
 * for getting polar coordinates from the stick, automatically eliminating
 * deadband, and responding to the digital button built into the stick. After
 * calling the constructor, setParams should be called to initialize the rest
 * the data. This is done so as not to have excessive parameters 
 * in the constructor
 * 
 * @author Brian Duemmer
 */
public class SmartControlStick {
	
	JoystickButton s_btn;
	Joystick controller;
	int xAxisIdx;
	int yAxisIdx;
	double deadband;
	boolean invertX;
	boolean invertY;
	
	double prevAngle;
	double maxMagnitude;
	
	
	/**
	 *  Initializes the SmartControlStickObject
	 *  @param joyIn reference to the joystick that contains the stick
	 *  @param xAxisIdxIn the index of the x-axis stick
	 *  @param yAxisIdxIn the index of the y-axis stick
	 *  @param btnIdxIn the index of the stick button
	 */
	
	public SmartControlStick(Joystick joyIn, int xAxisIdxIn, int yAxisIdxIn, int btnIdxIn) {
		// initialize the controller and button
		controller = joyIn;
		s_btn = new JoystickButton(controller, btnIdxIn);
		
		// initialize the class members
		xAxisIdx = xAxisIdxIn;
		yAxisIdx = yAxisIdxIn;
		
		// initialize data variables
		prevAngle = 0;
		maxMagnitude = 0;
		
	}
	
	
	
	
	/** 
	 * Sets some of the control parameters for the stick. This is done here
	 * instead of the constructor in order to reduce the amount of arguments
	 * in the constructor method
	 * 
	 * @param invertXin if true, invert the x-axis
	 * @param invertYin if true, invert the y-axis
	 * @param deadbandIn deadband threshold for the controller
	 * @param maxMagnitudeIn maximum value allowed to be returned by the
	 * 		  getMagnitude() function. set to -1 in order to ignore this
	 * 		  parameter
	 */
	public void setParams(boolean invertXin, boolean invertYin, double deadbandIn, double maxMagnitudeIn) {
		invertX = invertXin;
		invertY = invertYin;
		deadband = deadbandIn;
		maxMagnitude = maxMagnitudeIn;
	}
	
	
	
	
	
	/** returns the value of the x-axis stick, and can optionally invert it
	 * 
	 * @return the value of the axis, on the domain of [-1,1]
	 */
	public double getX() {
		// read the axis, and remove deadband
		double ret = calcDeadband(controller.getRawAxis(xAxisIdx));
		
		// invert if necessary
		if (invertX)
			ret *= -1;
		
		// return ret
		return ret;
	}
	
	
	
	
	/** returns the value of the y-axis stick, and can optionally invert it
	 * 
	 * @return the value of the axis, on the domain of [-1,1]
	 */
	public double getY() {
		// read the axis, and remove deadband
		double ret = calcDeadband(controller.getRawAxis(yAxisIdx));
		
		// invert if necessary
		if (invertY)
			ret *= -1;
		
		// return ret
		return ret;
	}
	
	
	
	
	/**
	 * gets the polar angle of the stick, in navigator format.
	 * 
	 * @return the angle hat the stick is pointing in, or the angle returned
	 * by this function the last time it was called, if both axes are in range
	 * of deadband
	 */
	public double getAngle() {
		// read the two axes
		double xVal = getX();
		double yVal = getY();
		
		double currAngle;
		
		// if at least one of the sticks is not within deadband, proceed normally
		if(xVal != 0 || yVal != 0) {
			// get the angle
			currAngle = Math.atan2(yVal, xVal);
			
			// convert from radians to degrees
			currAngle *= 180;
			currAngle /= Math.PI;
			
			// convert from math-style to navigator style angles
			currAngle *= -1;
			currAngle -= 90;
			
			// make sure the angle is in range
			if(currAngle < -180)
				currAngle += 360;
			
			if(currAngle > 180)
				currAngle -= 360;
			
			//set the prevAngle variable to what we just calculated
			prevAngle = currAngle;
			
			// return
			return currAngle;
		}
		
		// if not, return the previous angle
		else
			return prevAngle;
	}
	
	
	
	/**
	 * gets the polar magnitude of the stick, or 0 if within deadband
	 * @return the magnitude of the stick, scaled to be within the range
	 * specified by maxMagnitude
	 */
	public double getMagnitude() {
		// read the two axes
		double xVal = getX();
		double yVal = getY();
		
		// calculate the magnitude - c^2 = a^2 + b^2
		double mag = Math.sqrt(xVal*xVal + yVal*yVal);
		
		// make sure mag is within the range allowed by maxMagnitude. If this
		// value is <= 0, ignore it
		if( maxMagnitude > 0) {
			if( mag > maxMagnitude)
				mag = maxMagnitude;
			else if( mag < maxMagnitude * -1)
				mag = maxMagnitude * -1;
		}
		
		// return
		return mag;
	}
	
	
	
	
	/**
	 * checks if the stick button is pressed
	 * @return the value of the button
	 */
	public boolean getButtonVal() {
		return s_btn.get();
	}
	
	
	
	
	
	/** Remove deadband from val
	 * 
	 * @param 	val the value to remove deadband from. this is assumed to be on the
	 * 		  	domain of [-1,1]
	 * 
	 * @return 	val with deadband removed, and scaled such that when full
	 * 			forward or reverse, it returns 1 or -1 respectively, and zero if
	 * 			equal or less than the max deadband threshold, or NaN
	 */
	private double calcDeadband(double val) {
		// do a NaN check. if NaN, return 0
		if (val != val)
			return 0;
			
		// see if val is small enough to be considered negligible
		boolean inRange = Math.abs(val) <= deadband;
		
		/* if so, return 0. if not, return a scaled val, such that when full
		 * forward or reverse, it returns 1 or -1 respectively, and zero if
		 * equal to the max deadband threshold
		 */
		
		if (inRange)
			return 0;
		
		else {
			val -= (deadband * Math.signum(val));
			val /= (1 - deadband);
		}
		// return
		return val;
	}
	
	
	
	
	/**
	 * returns a reference to the stick button
	 * @return stick button reference
	 */
	public JoystickButton getButton() {
		return s_btn;
	}
	
}















