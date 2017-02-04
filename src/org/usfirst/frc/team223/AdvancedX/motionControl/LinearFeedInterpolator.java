package org.usfirst.frc.team223.AdvancedX.motionControl;

import edu.wpi.first.wpilibj.Timer;
import net.sf.microlog.core.Logger;

/**
 * Class that manages the feedrate of a robot mechanism via linear interpolation. This is given a 
 * maximum acceleration and velocity, which it will make sure to never overstep during operation. 
 * this is primarily intended for drivetrains, but it could also prove useful under other circumstances.
 * @author Brian Duemmer
 *
 */
public class LinearFeedInterpolator 
{
	// constants
	private double kAmax;
	private double kVmax;
	
	// equal to kVmax, but with a corrected sign
	private double kVpk;
	
	// start time
	private double t0;
	
	// time we should be rising
	private double t1;
	
	// time we should maintain peak velocity
	private double t2;
	
	// time we should be falling
	private double t3;
	
	// total time that this should take
	private double tTotal;
	
	
	
	// total distance
	private double sTotal;
	
	private double s1;
	private double s2;
	private double s3;
	
	
	
	// initial velocity
	private double v0;
	
	// final velocity
	private double vf;
	
	
	// type of interpolation mode. It can be 1, 2, or 3. if 1, there is only a linear interpolation from v0 to vf. If type 
	// 2, it will ramp up and then back down to vf. If type 3, it will ramp up, remain constant for some time, and ramp back to vf.
	private int moveType;
	
	
	
	// utility
	private Logger log;
	
	private boolean isActive;
	
	
	
	
	
	
	
	
	
	public LinearFeedInterpolator(Logger log, double aMax, double vMax)
	{
		this.log = log;
		this.kAmax = aMax;
		this.kVmax = vMax;
		
		log.info("Created new LinearFeedInterpolator with max acceleration of " +aMax+ "   and max velocity of " +vMax);
	}
	
	
	
	/**
	 * Starts the {@link LinearFeedInterpolator}. Now calls to {@link LinearFeedInterpolator#getTargetVel() getTargetVel()} and 
	 * {@link LinearFeedInterpolator#getTargetPos() getTargetPos()} will return what the system should be doing at the current time. If 
	 * for whatever reason you want to abort an interpolation, call {@link LinearFeedInterpolator#cancel() cancel()}.
	 * @param currVel The current velocity of the system
	 * @param finalVel The target final velocity of the system
	 * @param dist the distance that the system should travel
	 * @return the amount of time that the interpolation will take to complete
	 */
	public double start(double currVel, double finalVel, double dist)
	{
		// update the instance variables
		this.t0 = Timer.getFPGATimestamp();
		
		// equal to kVmax, but with a corrected sign
		kVpk = Math.signum(dist) * this.kVmax;
		
		this.v0 = currVel;
		this.vf = finalVel;
		this.sTotal = dist;
		
		// say that we are now active
		this.isActive = true;
		
		
		
		// calculate the base subsection times
		this.t1 = Math.abs(kVpk - this.v0) / this.kAmax;
		this.t3 = Math.abs(kVpk - this.vf) / this.kAmax;
		
		
		// calculate the base subsection distances
		this.s1 = 0.5*this.t1*(this.v0 + kVpk);
		this.s3 = 0.5*this.t3*(this.vf + kVpk);
		
		
		// find the correct interpolation type
		if(Math.abs(s1 + s3) < Math.abs(sTotal))
		{
			this.moveType = 3;
			s2 = sTotal - (s1+s3);
			t2 = s2 / kVpk;
		}
		
		else
		{
			
		}
		
		
		return this.tTotal;
	}
	
	
	
	
	public double getTargetVel()
	{
		if(moveType == 3)
			return calcVType3();
		return 0; 
		
	}
	
	
	
	
	public double getTargetPos()
	{
		if(moveType == 3)
			return calcSType3();
		return 0;
		
	}
	
	
	/**
	 * Calculates the current position for a type 3 interpolation
	 * @return the current position setpoint
	 */
	private double calcSType3()
	{
		// current time, relative to cycle start
		double t = Timer.getFPGATimestamp() - t0;
		
		// if section 1
		if(t < t1)
			return 0.5 * kAmax * Math.signum(kVpk-v0) * t*t  +  v0*t;
		
		// if section 2
		else if(t < t1+t2)
			return s1 + t*kVpk;
		
		// if section 3
		else if(t < tTotal)
			return s1+s2 + Math.signum(vf-kVpk) * (0.5*t*t - t*tTotal);
		
		// if past the interpolation sequence
		else
			return sTotal;
	}
	
	
	
	
	/**
	 * Calculates the current velocity for a type 3 interpolation
	 * @return the current velocity setpoint
	 */
	private double calcVType3()
	{
		// current time, relative to cycle start
		double t = Timer.getFPGATimestamp() - t0;
		
		// if section 1
		if(t < t1)
			return v0 + (kAmax*Math.signum(kVpk) * t);
		
		// if section 2
		else if(t < t1+t2)
			return kVpk;
		
		// if section 3
		else if(t < tTotal)
			return (t - tTotal) * kAmax * Math.signum(vf-kVpk);
		
		// if past the interpolation sequence
		else
			return vf;
	}
	
	
	
	public void cancel()
	{
		
	}
	
}










