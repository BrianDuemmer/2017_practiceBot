package org.usfirst.frc.team223.AdvancedX.motionControl;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.DriveSideData;
import org.usfirst.frc.team223.AdvancedX.robotParser.Freeable;
import org.usfirst.frc.team223.AdvancedX.utility.Differentiator;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import net.sf.microlog.core.Logger;

/**
 * Implements a single side of a standard tank style drivetrain. This has an embedded
 * PID that controls the velocity of the wheels. This is usually used as a subset of a full
 * drivetrain class that implements two of these, to allow for finer, more independent control
 * @author Brian Duemmer
 *
 */
public class DriveSide extends PIDSubsystem implements Freeable
{
	// Encoder objects
	PIDSource velocityPidSrc;
	
	// All of the motors
	private List<SpeedController> motors;
	
	// Encoder data
	double distPerPulse;
	boolean invert;
	
	//max output
	double maxOut;
	
	// Object to log data about the driveside
	private Logger logger;
	
	AdvancedXManager manager;
	
	private Differentiator posToVel;
	
	
	/**
	 * Constructor for the DriveSide. Make sure to configure the motors, PID, 
	 * and PIDsource before use
	 * <P/>
	 * <b>WARNING: </b> Do not pass a non-positive value for the period!
	 * @param period the time, in seconds, between PID updates
	 */
	public DriveSide(double period, AdvancedXManager manager, String name)
	{
		super(0, 0, 0, period);
		
		this.logger = manager.getRoboLogger().getLogger(name);
		this.manager = manager;
		
		motors = new ArrayList<SpeedController>();
		
		// Set the maximum output to a safe default
		maxOut = 1;
		
		// setup the differentiator
		this.posToVel = new Differentiator(1, false);
		this.posToVel.start();
	}
	
	
	
	
	/**
	 * Constructor for the DriveSide. Make sure to configure the motors, PID, 
	 * and PIDsource before use
	 */
	public DriveSide(AdvancedXManager manager, String name)
	{
		super(0, 0, 0);
		
		this.logger = manager.getRoboLogger().getLogger(name);
		this.manager = manager;
		
		motors = new ArrayList<SpeedController>();
		
		// Set the maximum output to a safe default
		maxOut = 1;
		
		this.posToVel = new Differentiator(1, false);
		this.posToVel.start();
	}
	
	
	
	
	/**
	 * Sets the PID constants for the drive controller
	 * @param kp Proportional gain term
	 * @param ki Integral term
	 * @param kd Derivative term
	 * @param kf Feedforward term
	 */
	public void setPID(double kp, double ki, double kd, double kf)
	{
		logger.info("PID constants are now: "
				+ "  kp: "+ kp
				+ "  ki: "+ ki
				+ "  kd: "+ kd
				+ "  kf: "+ kf);
		this.getPIDController().setPID(kp, ki, kd, kf);
	}


	
	
	
	/**
	 * Sets the PID source for the driveSide. this is usually an encoder / CANTalon.
	 * @param src the PIDsource to use
	 */
	public void setVelocityPIDSource(PIDSource src)
	{   
		velocityPidSrc = src;   
		velocityPidSrc.setPIDSourceType(PIDSourceType.kRate);
	}
	
	
	/** 
	 * Sets the scalings for the PIDSource
	 * @param distPerPulse the value that is multiplied by the value returned by the PIDSource
	 * @param invert if true, multiplies the output of PIDSource by -1
	 * @deprecated Create a custom PIDSource object and do your scalings in that instead
	 */
	public void setPIDSrcScalings(double distPerPulse, boolean invert)
	{
		this.distPerPulse = distPerPulse;
		this.invert = invert;
	}
	
	
	/**
	 * Adds a motor to the driveSide
	 * @param mot a fully initialized motor. Don't use it anywhere else
	 */
	public void addMotor(SpeedController mot)
	{   
		motors.add(mot);  
		logger.info("Motor added to DriveSide. There are now " + motors.size());
		
	}

	
	
	/**
	 * Sets the raw output to the motors. Automatically disables
	 * PID control.
	 * @param out the output to send
	 */
	public void setRawOutput(double out)
	{
		// Turn off the PID and reset
		this.getPIDController().reset();
		
		// Set the output
		output(out);
	}
	
	
	
	
	/** Sets the target velocity. Automatically enables the PID
	 * 
	 * @param setpoint
	 */
	public void setSetpoint(double setpoint)
	{
		// Don't run if a velocityPID Source hasn't been set
		if(velocityPidSrc != null)
		{
			// set the setpoint
			super.setSetpoint(setpoint);
			
			// enable the PID
			this.enable();
		}
	}
	
	
	
	/**
	 * Directly send an output to the motors
	 * @param output the output to send
	 */
	protected void output(double output) {
		
		// Coerce the output to an allowable range specified by maxOut
		double newOut = output > 0  ?  Math.min(output, maxOut) : Math.max(output, maxOut * -1);
		
		// Iterate through all of the motors and set their outputs
		for(SpeedController i : motors)
			i.set(newOut);
		
	}

	@Override
	protected void initDefaultCommand() {}
	
	@Override
	protected double returnPIDInput() {   return getVel();   }
	
	@Override
	protected void usePIDOutput(double output) {
		
		// only output if the PID is enabled
		if(this.getPIDController().isEnabled())
			output(output);
		
	}
	
	
	
	/**
	 * Sets the amount of motors to set into brake mode.
	 * Only works for {@link CANTalon}s.
	 */
	public void setBrakeCount(int motCt)
	{
		
		logger.info("Setting brake count to " +motCt+ " motors");
		
		// iterate over the motor list
		for(SpeedController i : motors)
		{
			// if the motor is a CANTalon, and motCt > 0, turn on the brakes and decrement motCt
			if(i.getClass() == CANTalon.class && motCt > 0)
			{
				motCt--;
				((CANTalon)i).enableBrakeMode(true);
			}
			
			// if the motor is a CANTalon, and motCt <= 0, turn off the brakes
			else if(i.getClass() == CANTalon.class)
				((CANTalon)i).enableBrakeMode(false);
		}
	}
	
	
	
	
	
	/**
	 * Sets the maximum allowable output to send to the motors.
	 * this should be between [0,1]. Default is 1.
	 * @param output
	 */
	public void setMaxOutput(double output) {   maxOut = output;   }




	/**
	 * @return the motors
	 */
	public List<SpeedController> getMotors() {
		return motors;
	}




	/**
	 * @param motors the motors to set
	 */
	public void setMotors(List<SpeedController> motors) {
		this.motors = motors;
	}
	
	
	
	/**
	 * Frees all of the resources allocated by the DriveSide
	 */
	public void free()
	{
		logger.info("Attempting to free DriveSide...");
		
		logger.info("Attempting to free PIDSource...");
		this.manager.destroy(this.velocityPidSrc);
		logger.info("Finished freeing PIDSource");
		
		logger.info("Attempting to free PIDController...");
		this.manager.destroy(this.getPIDController());
		logger.info("Finished freeing PIDController");
		
		logger.info("Attempting to free motors...");
		for(SpeedController i: this.motors)
			this.manager.destroy(i);
		logger.info("Finished freeing motors");
	}
	
	
	
	
	/**
	 * Frees all of the resources allocated by the DriveSide
	 * @deprecated Use {@link DriveSide#free() free()} instead
	 */
	public void _free()
	{
		
		logger.info("Attempting to free DriveSide...");
		
		// iterate through the motors and deallocate them
		for(int i=0; i<motors.size(); i++)
		{
			// If it is a PWM controller, free() it
			if(motors.get(i) != null && motors.get(i).getClass().isAssignableFrom(PWM.class))
			{
				logger.info("Attempting to free a PWM speed controller...");
				((PWM) motors.get(i)).free();
				logger.info("Finished freeing PWM Speed controller");
			}
			
			// If it is a CANTalon, delete() it
			if(motors.get(i) != null && motors.get(i).getClass() == CANTalon.class)
			{
				logger.info("Attempting to free a CANTalon...");
				((CANTalon) motors.get(i)).delete();
				logger.info("Finished freeing CANTalon");
			}
			
			// If it equals the PIDSource, set that to null as well. This has to be done due to 
			// poor design of the CANTalon Class
			if(motors.get(i).equals(this.velocityPidSrc))
				this.velocityPidSrc = null;
			
			// Whatever it is, set it to null manually. This is because calling delete() a second 
			// time on a CANTalon causes a JVM crash (SIGSEGV error)
			motors.set(i, null);
		}
		
		
		// free() the PIDController
		logger.info("Attempting to free PID controller...");
		if(this.getPIDController() != null)
		{
			this.getPIDController().free();
			logger.info("Finished freeing PID Controller");
		} else
			logger.warn("PID Controller is null! Taking no action...");
		
		
		// free() the PIDSource. Try to cast it to a free()-able type. If we cannot, just ignore it
		logger.info("Attempting to free PID Source...");
		if(this.velocityPidSrc != null && this.velocityPidSrc.getClass() == CANTalon.class)
		{
			((CANTalon) this.velocityPidSrc).delete();
			logger.info("Finished freeing CANTalon PIDSource");
		}
		
		else if(this.velocityPidSrc != null && this.velocityPidSrc.getClass().isAssignableFrom(SensorBase.class))
		{
			((SensorBase) this.velocityPidSrc).free();
			logger.info("Finished freeing SensorBase PIDSource");
		}
		
		// Put another free() here if necessary
		else{
			logger.warn("Unable to free PIDSource! Possibly due to it being freed already");
		}
		
	}
	
	
//	private double prevPos = 0;
//	private double prevUpdateTime = Timer.getFPGATimestamp();
//	
//	public double getPos()
//	{
//		// get the current position
//		double currPos = velocityPidSrc.pidGet() * distPerPulse * (invert  ?  -1 : 1);
//		return currPos;
//	}
//	
//	
//	
//	public double getVel()
//	{
//		double currPos = getPos();
//		double currVel = (currPos - prevPos) / (Timer.getFPGATimestamp() - prevUpdateTime);
//		
//		prevPos = currPos;
//		prevUpdateTime = Timer.getFPGATimestamp();
//		
//		return currVel;
//		
//	}
	
	
	public double getVel()
	{
		return velocityPidSrc.pidGet();
	}
	
	
	/**
	 * Gets the current draws of each motor on the driveside
	 * @param pdp the power distribution panel
	 */
	public double[] getCurrent(PowerDistributionPanel pdp, DriveSideData data)
	{
		double[] ret = new double[data.motors.size()];
		
		for(int i=0; i<ret.length; i++)
			ret[i] = pdp.getCurrent(data.motors.get(i).pdpChannel);
		
		return ret;
	}
}













