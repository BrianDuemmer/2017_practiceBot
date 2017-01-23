package org.usfirst.frc.team223.AdvancedX.motionControl;

/**
 * This interface defines a standard for all OmniDirectional drive systems. This allows for 
 * more versatility and re-usability in the future
 * 
 * @author Brian Duemmer
 *
 */
public interface OmniDirectionalDrive 
{
	/**
	 * Sets the output of the driveTrain
	 */
	public void setRawOutput(double fwd, double strafe, double turn);
	
	/**
	 * Gets the angle of the robot relative to the field
	 */
	public double getAngle();
	
	/**
	 * Gets the rate of change of the angle of the robot relative to the field
	 */
	public double getAngleRate();
	
	
	/**
	 * Gets the y(fwd) distance of the robot relative to the field
	 */
	public double getY();
	
	/**
	 * Gets the y(fwd) velocity of the robot relative to the field
	 */
	public double getYRate();
	
	
	/**
	 * Gets the x(fwd) distance of the robot relative to the field
	 */
	public double getX();
	
	/**
	 * Gets the x(fwd) velocity of the robot relative to the field
	 */
	public double getXRate();
	
	
	/**
	 * Advanced feature that allows the robot to perform G1 (linear) interpolation between 
	 * one point and another. This feature doesn't have to be implemented, but it is an 
	 * extremely powerful tool for autonomous movement
	 * @param dy The distance that the robot has to travel forward
	 * @param dx The distance that the robot has to travel to the side
	 * @param dTheta the angular change that the robot has to make, or the final angle if 
	 * <code> isFieldCentric </code> is true
	 * @param feedRate the linear speed that the robot will move at
	 * @param isFieldCentric if true, dy will be the distance along the field's y axis, dx will be
	 * the distance along the field's x axis, and dTheta will be the terminal angle
	 */
	public default void G1(double dy, double dx, double dTheta, double feedRate, boolean isFieldCentric)
	{
		return;
	}
}
