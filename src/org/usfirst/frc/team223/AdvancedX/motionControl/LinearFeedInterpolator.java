package org.usfirst.frc.team223.AdvancedX.motionControl;

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

	// acceleration for type 1 move
	private double aType1;


	// type of interpolation mode. It can be 1, 2, or 3. if 1, there is only a linear interpolation from v0 to vf. If type 
	// 2, it will ramp up and then back down to vf. If type 3, it will ramp up, remain constant for some time, and ramp back to vf.
	private int moveType;

	// if true, invert the output velocity and position setpoints
	private boolean isReverse;



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
	
	
	
	
	public void setConstraints(double aMax, double vMax)
	{
		this.kAmax = aMax;
		this.kVmax = vMax;
		
		log.info("LFI constraints are now:  aMax-" +aMax+ " vMax-" +vMax);
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
		this.t0 = (double)System.currentTimeMillis() / 1000.0;
		
		log.info("Starting LinearFeedInterpolation sequence...");

		//make sure all of the velocities and distance are in the same direction
		if(currVel >= 0 ^ dist >= 0)
		{
			log.info("current velocity and distance do not have matching sign! setting current velocity to 0");
			currVel = 0;
		}

		if(finalVel >= 0 ^ dist >= 0)
		{
			log.info("final velocity and distance do not have matching sign! setting final velocity to 0");
			finalVel = 0;
		}
		
		log.info("LFI specs: v0: " +currVel+ "   vf: " +finalVel+ "   aMax: " +kAmax+ "   vMax: " +kVmax);



		// setup direction stuff
		if(dist < 0 )
		{
			// invert sign of distance and velocities
			dist *= -1;
			currVel *= -1;
			finalVel *= -1;

			// set the reverse flag
			this.isReverse = true;
		} else
			this.isReverse = false;


		this.v0 = currVel;
		this.vf = finalVel;
		this.sTotal = dist;

		// say that we are now active
		this.isActive = true;
		
		// calculate the base move type
		double sa = ((2 * kVmax*kVmax) - (v0*v0 + vf*vf)) / (2*kAmax);
		moveType = sTotal >= sa  ?  3 : 2;

		// check for an illegal move
		if(sTotal < Math.abs(v0*v0 - vf*vf) / (2*kAmax) )
		{
			log.warn("Attempted movement would exceed preset limit for acceleration! Proceeding anyway...");
			moveType = 1;
		}

		// initialize the movement based on the proper move type
		if(moveType == 3)
			initType3();

		else if(moveType == 2)
			initType2();

		else if (moveType == 1)
			initType1();

		log.info("Movement type is: " + moveType);
		return this.tTotal;
	}








	/**
	 * Initializes a type 3 move, setting up all instance variables as necessary
	 */
	private void initType3()
	{
		// calc the subset distances
		s1 = (kVmax*kVmax - v0*v0) / (2*kAmax);
		s3 = (kVmax*kVmax - vf*vf) / (2*kAmax);
		s2 = sTotal - (s1+s3);

		// calc the subset times
		t1 = (kVmax-v0) / kAmax;
		t3 = (kVmax-vf) / kAmax;
		t2 = (s2 / kVmax);

		tTotal = t1+t2+t3;
	}






	/**
	 * Initializes a type 2 move, setting up all instance variables as necessary
	 */
	private void initType2()
	{
		double max = Math.max(v0, vf);
		double min = Math.min(v0, vf);

		s1 = (max*max - min*min) / (2*kAmax);
		s2 = sTotal - s1;

		t1 = (max-min) / (kAmax);
		t2 = (2/kAmax) * (Math.sqrt(kAmax*s2 + max*max) - max);

		tTotal = t1 + t2;

		log.info("s2: " +s2+ "  s1: " +s1+ "  t1: " +t1+ "  t2: " +t2);
	}





	/**
	 * Initializes a type 1 move, setting up all instance variables as necessary
	 */
	private void initType1()
	{
		aType1 = (vf*vf - v0*v0) / (2*sTotal);
		tTotal = (vf -v0) / aType1;
	}





	/**
	 * Gets the setpoint velocity at the current time
	 */
	public double getTargetVel()
	{
		double vel = 0;

		// calculate the proper motion type
		if(moveType == 3)
			vel =  calcVType3();

		else if(moveType == 2)
			vel =  calcVType2();

		else if(moveType == 1)
			vel =  calcVType1();

		return vel;

	}



	
	
	
	/**
	 * Gets the setpoint position at the current time
	 */
	public double getTargetPos()
	{
		double pos = 0;

		// calculate the proper motion type
		if(moveType == 3)
			pos =  calcSType3();

		else if(moveType == 2)
			pos =  calcSType2();

		else if(moveType == 1)
			pos =  calcSType1();

		return pos;

	}

	
	
	
	

	/**
	 * Calculates the current position for a type 3 interpolation
	 * @return the current position setpoint
	 */
	private double calcSType3()
	{
		// current time, relative to cycle start
		double t = ((double)System.currentTimeMillis() / 1000.0) - t0;

		double pos = 0;

		// area of the part 3 curve before part 3 starts
		double prevAreaS3 = (t1+t2) * (kVmax + 0.5*kAmax*(t1+t2));


		// if section 1
		if(t < t1)
			pos = v0*t + 0.5*kAmax*t*t;

		// if section 2
		else if(t < t1+t2)
			pos = s1 + kVmax * (t-t1);

		// if section 3
		else if(t < tTotal)
			pos = s1+s2 - prevAreaS3 + t*(kVmax - 0.5*kAmax*t + kAmax * (t1+t2));

		// if past the interpolation sequence
		else
			pos = sTotal;


		// invert if we are in reverse
		if(isReverse)
			pos *= -1;

		return pos;
	}





	
	
	/**
	 * Calculates the current position for a type 2 interpolation
	 * @return the current position setpoint
	 */
	private double calcSType2()
	{
		// current time, relative to cycle start
		double t = ((double)System.currentTimeMillis() / 1000.0) - t0;
		double pos = 0;

		
		
		// use the proper time bounds for section 1 depending on whether v0 of vf is larger
		if((v0>=vf && t < t2/2) || (v0<vf && t < t1 + t2/2))
			pos = v0*t + 0.5*kAmax*t*t;

		// if section 2, v0 is less than vf
		else if(t < tTotal && v0<vf)
			pos = vf*t - 0.5*kAmax * (t - t1 - 0.5*t2) * Math.abs(t - t1 - 0.5*t2) + 0.5*kAmax*t2*t - 0.5*kAmax*(t1 + 0.5*t2) * Math.abs(t1 + 0.5*t2);
		
		// if section 2, v0 is greater than vf
		else if(t < tTotal && v0>=vf)
			pos = v0*t - 0.5*kAmax * (t - 0.5*t2) * Math.abs(t - 0.5*t2) + 0.5*kAmax * t2*t - 0.125*kAmax * t2*t2;

		// if past the interpolation sequence
		else
			pos = sTotal;

		

		// invert if we are in reverse
		if(isReverse)
			pos *= -1;

		return pos;
	}
	
	
	
	
	
	
	
	/**
	 * Calculates the current position for a type 1 interpolation
	 * @return the current position setpoint
	 */
	private double calcSType1()
	{
		// current time, relative to cycle start
		double t = ((double)System.currentTimeMillis() / 1000.0) - t0;
		double pos = 0;

		if(t < tTotal)
			pos = v0*t + 0.5*aType1*t*t;
		else
			pos = sTotal;

		// invert if we are in reverse
		if(isReverse)
			pos *= -1;

		return pos;
	}
	
	
	




	/**
	 * Calculates the current velocity for a type 3 interpolation
	 * @return the current velocity setpoint
	 */
	private double calcVType3()
	{
		// current time, relative to cycle start
		double t = ((double)System.currentTimeMillis() / 1000.0) - t0;

		//velocity to be returned
		double vel;

		// if section 1
		if(t < t1)
			vel = v0 + t*kAmax;

		// if section 2
		else if(t < t1+t2)
			vel = kVmax;

		// if section 3
		else if(t < tTotal)
			vel = kVmax - kAmax * (t - (t1+t2));

		// if past the interpolation sequence
		else
			vel = vf;

		// invert the velocity if we are moving in reverse
		if(isReverse)
			vel *= -1;

		return vel;
	}






	/**
	 * Calculates the current velocity for a type 2 interpolation
	 * @return the current velocity setpoint
	 */
	private double calcVType2()
	{
		// current time, relative to cycle start
		double t = ((double)System.currentTimeMillis() / 1000.0) - t0;

		//velocity to be returned
		double vel;

		// change time bounds v0 is greater than vf
		if(v0 > vf)
		{
			// if section 1
			if(t < t2 / 2)
				vel = v0 + t*kAmax;

			// if section 2
			else if(t <  tTotal)
				vel = vf - kAmax * (t - (t1+t2));

			// if past the interpolation sequence
			else
				vel = vf;
		}

		// if vf is greater than v0
		else
		{
			// if section 1
			if(t < t1 + (t2 / 2))
				vel = v0 + t*kAmax;

			// if section 2
			else if(t < tTotal)
				vel = vf - kAmax * (t - (t1+t2));

			// if past the interpolation sequence
			else
				vel = vf;
		}



		// invert the velocity if we are moving in reverse
		if(isReverse)
			vel *= -1;

		return vel;
	}




	/**
	 * Calculates the current velocity for a type 1 interpolation
	 * @return the current velocity setpoint
	 */
	private double calcVType1()
	{
		// current time, relative to cycle start
		double t = ((double)System.currentTimeMillis() / 1000.0) - t0;

		//velocity to be returned
		double vel;

		// if in the interpolation sequence
		if(t < tTotal)
			vel = v0 + t*aType1;

		// if past the interpolation sequence
		else
			vel = vf;

		// invert the velocity if we are moving in reverse
		if(isReverse)
			vel *= -1;

		return vel;
	}




	/**
	 * Cancels the current interpolation sequence. Once this is called, {@link LinearFeedInterpolator#isActive() isActive()}
	 * will return false until {@link LinearFeedInterpolator#start() start()} is called again.
	 */
	public void cancel()
	{
		isActive = false;
	}



	/**
	 * if this returns true, we are in an active interpolation sequence. If false, we are idle.
	 * Call this periodically to find out when the target has been reached
	 */
	public boolean isActive() {
		isActive = ((double)System.currentTimeMillis() / 1000.0) <= t0 + tTotal;
		return isActive;
	}
}










