package org.usfirst.frc.team223.AdvancedX.motionControl;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.utility.AngleUtil;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import net.sf.microlog.core.Logger;

/**
 * Acts as the master class of the Drive Train cascade control loop. This class holds 
 * all output control over the Drive Train. Wrap this class in the main drive subsystem.
 * @author Brian Duemmer
 *
 */
public class TankCascadeController
{
    
    // DriveSides
    private DriveSide leftSide;
    private DriveSide rightSide;
    
    //Position and Turn PID loops
    private PIDController posPID;
    private PIDController turnPID;
    
    // Gyroscope sensor
    private Gyro gyro;
    
    // Master loop outputs
    private double positionOutput;
    private double turnOutput;
    
    // Master loop PIDSources
    private PositionPIDSource posSrc;
    private TurnPIDSource turnSrc;
    
    // Setpoints
    private double targetAngle;
    private double targetDistance;
    private double targetVel;
    
    // True if the controller is enabled
    private boolean isEnabled;
    
    // object to log data output
    private Logger logger;
    
    
    
    
//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Dependent Classes ///////////////////////////////////////// 
//////////////////////////////////////////////////////////////////////////////////////////////////
    
    /**
     * Acts as the PIDSource for the position PID loop of the cascade controller
     * @author Brian Duemmer
     *
     */
    class PositionPIDSource implements PIDSource
    {
    	// The position value returned by the previous call to pidGet(). Defaults to 0. 
    	double prevPos;
    	
    	// The timestamp from the last time pidGet() was called
    	double prevTime;
    	
    	
    	/**
    	 * Initializes the PositionPIDSource object, with prevPos being set to 0
    	 * and prevTime being set to -1. This allows us to check if the previousTime
    	 * is valid, so the initial call to {@link pidGet()} does not calculate an
    	 * incorrect value.
    	 */
    	public PositionPIDSource()
    	{
    		prevPos = 0;
    		prevTime = -1;
    	}
    	
    	
    	
    	/**
    	 * Sets the current position of the drivetrain. Useful for resetting the PID.
    	 */
    	public void setPosition(double pos) {   prevPos = pos;   }
    	
    	
    	
    	
    	/**
    	 * {@inheritDoc} This method has no effect on the actual operation of the system,
    	 * as this source will only ever use displacement, never rate.
    	 */
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) 
		{
			// Don't do anything because the type here will always be displacement
		}

		
		
		
		/**
		 * {@inheritDoc} This will always return displacement.
		 */
		@Override
		public PIDSourceType getPIDSourceType() 
		{
			return PIDSourceType.kDisplacement;
		}

		
		
		
		/**
		 * {@inheritDoc} Calculates the distance traveled by the robot, by integrating the average of the 
		 * velocities from the left and right sides. Automatically keeps track of time. If prevTime is -1,
		 * meaning that this is the first call to pidGet, return 0 and update the prevTime variable. 
		 */
		@Override
		public double pidGet() 
		{
			// Current time
			double currTime = Timer.getFPGATimestamp();
			
			// Average the two velocities
			double avgVel = (getLeftSide().getPID() + rightSide.getPID()) / 2;
			
			
			// if prevTime is >= 0, it is valid, and we can proceed normally
			if(prevTime >= 0)
			{
				// Integrate the average velocity from Tprev to Tcurr
				double distTraveled = avgVel * (currTime - prevTime);
				
				// Add it to the previous position
				distTraveled += prevPos;
				
				// update the prevTime and prevPos variables
				prevTime = currTime;
				prevPos = distTraveled;
				
				// return the total distance traveled
				return distTraveled;
			}
			
			// If not, initialize prevTime and return 0
			else
			{
				prevTime = currTime;
				return 0;
			}
			
		}
		
		
		
		/**
		 * Resets the position integrator
		 */
		public void reset()
		{
			setPosition(0);
			prevTime = -1;
		}
    	
    }
    
    
    
    
    
    /**
     * Acts as the PIDSource for the turning PID controller in the driveTrain
     * cascade control loop
     * @author Brian Duemmer
     *
     */
    class TurnPIDSource implements PIDSource
    {
    	// Offset angle that is subtracted from the yaw value returned from the navX
    	double offsetAngle;
    	
    	
    	/**
    	 * {@inheritDoc} This method has no effect on the actual operation of the system,
    	 * as this source will only ever use displacement, never rate.
    	 */
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) 
		{
			// Don't do anything because the type here will always be displacement
		}

		
		
		
		/**
		 * {@inheritDoc} This will always return displacement.
		 */
		@Override
		public PIDSourceType getPIDSourceType() 
		{
			return PIDSourceType.kDisplacement;
		}

		
		
		/**
		 * Calculates the robot's yaw angle, after accounting for the offset angle. Automatically
		 * converts the angle to lie between [0, 360).
		 */
		@Override
		public double pidGet() {   return AngleUtil.norm360( gyro.getAngle() - offsetAngle);   }
		
		
		
		/**
		 * Sets the current angular position of the robot
		 */
		public void setCurrentAngle(double currAngle) { offsetAngle = gyro.getAngle() - currAngle;   }

    	
    }



    /** Acts as the PIDoutput for the position PID loop. All this does is update the
     * positionOutput variable in the main class, and updates the system.
     * 
     * @author Brian Duemmer
     *
     */
    class PositionPIDOutput implements PIDOutput
    {
    	/**
    	 * {@inheritDoc} Simply writes the output value to the positionOutput variable in the main class, and updates the system.
    	 */
		@Override
		public void pidWrite(double output) 
		{   
			positionOutput = output;
			update();
		}
    }
    
    
    
    
    
    
    /** Acts as the PIDoutput for the turn PID loop. All this does is update the
     * turnOutput variable in the main class, and updates the system.
     * 
     * @author Brian Duemmer
     *
     */
    class TurnPIDOutput implements PIDOutput
    {
    	/**
    	 * {@inheritDoc} Simply writes the output value to the turnOutput variable in the main class, and updates the system.
    	 */
		@Override
		public void pidWrite(double output) 
		{   
			turnOutput = output;
			update();
		}
    }

    
    
 
    
    
    
//////////////////////////////////////////////////////////////////////////////////////////////////   
//////////////////////////// Main Cascade Controller Implementation ////////////////////////////// 
//////////////////////////////////////////////////////////////////////////////////////////////////


    /**
     *  Initialize a new CascadeController. This will automatically allocate two PID loops, in
     *  addition two the two PIDs in <i>leftSide</i> and <i>rightSide</i>. This class should also
     *  be given full control of all drivetrain output (i.e. call {@link #setRawOut} or {@link #setRawVel} instead of 
     *  {@link leftSide#setRawOutput} and {@link rightSide#setRawOutput}. Be sure to set the PID
     *  constants for the two master loops, with {@link #setPosPID} and {@link #setTurnPID}
     *  
     * @param leftSide The left {@link DriveSide}. Make sure it is fully initialized first.
     * @param rightSide The right {@link DriveSide}. Make sure it is fully initialized first.
     * @param gyro The {@link Gyro} to be used for turn control
     * @param masterPidPeriod the period that the master(position and turn) PID loops run at. 
     * This should be ~3x slower than the slave response time. 0.3s is a good value for this.
     */
	public TankCascadeController(DriveSide leftSide, DriveSide rightSide, Gyro gyro, double distPidPeriod, double anglePidPeriod, AdvancedXManager manager)
	{
		this.logger = manager.getRoboLogger().getLogger("TankCascadeController");
		
		// Make sure the variables are not illegal values
		if(distPidPeriod <= 0)
		{
			logger.error("Illegal value " +distPidPeriod+ " for distance PID period. Setting to default instead");
			distPidPeriod = PIDController.kDefaultPeriod;
		}
		
		if(anglePidPeriod <= 0)
		{
			logger.error("Illegal value " +anglePidPeriod+ " for angle PID period. Setting to default instead");
			anglePidPeriod = PIDController.kDefaultPeriod;
		}
		
		// Update the instance variables
		this.leftSide = leftSide;
		this.rightSide = rightSide;
		this.gyro = gyro;
		
		// Initialize the PIDSources
		posSrc = new PositionPIDSource();
		turnSrc = new TurnPIDSource();
		
		// Initialize the PIDOutputs
		PositionPIDOutput posOut = new PositionPIDOutput();
		TurnPIDOutput turnOut = new TurnPIDOutput();
		
		// Initialize the master PID controllers. Set the all of the gains to 0 initially
		posPID = new PIDController(0, 0, 0, 0, posSrc, posOut, distPidPeriod);
		turnPID = new PIDController(0, 0, 0, 0, turnSrc, turnOut, anglePidPeriod);
		
		// Configure the turn PID Controller
		turnPID.setContinuous(true);
		turnPID.setInputRange(-180, 180);
		
	}
	
	
	
	
	
	/**
	 * Updates the Cascade Controller. This is called automatically by the two master
	 * PID loops whenever they update their outputs. Only update the setpoints if enabled.
	 */
	private synchronized void update()
	{
		// Only update everything if enabled
		if(isEnabled)
		{
			// Calculate the resultant velocity setpoints for the slave PID loops
			double leftVel = targetVel + positionOutput + turnOutput;
			double rightVel = targetVel + positionOutput - turnOutput;
			
			// Update the slave setpoints
			leftSide.setSetpoint(leftVel);
			rightSide.setSetpoint(rightVel);
		}
	}
	
	
	
	
	
	/**
	 * Enables the Cascade Controller. This enables all four PID loops and sets the internal
	 * isEnabled flag to true.
	 */
	public void enable()
	{
		// Enable all of the PIDs
		posPID.enable();
		turnPID.enable();
		leftSide.enable();
		rightSide.enable();
		
		// Say that we are enabled
		isEnabled = true;
		logger.info("Enabling PID Control");
	}
	
	
	
	
	
	/**
	 * disable the Cascade Controller. This disables all four PID loops and sets the internal
	 * isEnabled flag to false.
	 */
	public void disable()
	{
		// disable all of the PIDs
		posPID.disable();
		turnPID.disable();
		leftSide.disable();
		rightSide.disable();
		
		// Say that we are disabled
		isEnabled = false;
		logger.info("Disabling PID Control");
	}
	
	
	
	
	/**
	 * Resets the Cascade Controller. This resets all four PID loops, as well as the position PID integrator.
	 * @param reEnable if true, automatically re-enables the PID loops
	 */
	public void reset(boolean reEnable)
	{
		logger.info("Resseting controllers...");
		
		// Reset the PID loops
		posPID.reset();
		turnPID.reset();
		leftSide.getPIDController().reset();
		rightSide.getPIDController().reset();
		
		// reset the position integrator
		posSrc.reset();
		
		// if reEnable is true, re-enable the Cascade Controller
		if(reEnable)
			enable();
	}
	
	
	
	
	
	/**
	 * Sets the raw output to the motors. Automatically disables the Cascade Controller
	 * @param left the left output. This should be between [-1, 1]
	 * @param right the right output. This should be between [-1, 1]
	 */
	public void setRawOut(double left, double right)
	{
		// Disable if necessary
		if(isEnabled)
			disable();
		
		// Set the raw outputs of the driveSides
		leftSide.setRawOutput(left);
		rightSide.setRawOutput(right);
	}
	
	
	
	
	
	/**
	 * Sets the raw velocity for each side. Automatically disables the Cascade Controller
	 * @param left the left velocity setpoint
	 * @param right the right velocity setpoint
	 */
	public void setRawVel(double left, double right)
	{
		// Disable if necessary
		if(isEnabled)
			disable();
		
		// Set the velocity setpoints
		leftSide.setSetpoint(left);
		rightSide.setSetpoint(right);
	}
	
	
	
	
	
	/**
	 * Sets the PID constants for the position control PID.
	 * @param kp proportional gain coefficient
	 * @param ki integral gain coefficient
	 * @param kd derivative gain coefficient
	 * @param kf feedforward coefficient
	 */
	public void setPosPID(double kp, double ki, double kd, double kf) 
	{   
		posPID.setPID(kp, ki, kd, kf);   
		logger.info("PID constants are now: "
				+ "  kp: "+ kp
				+ "  ki: "+ ki
				+ "  kd: "+ kd
				+ "  kf: "+ kf);
	}
	
	
	
	/**
	 * Sets the PID constants for the turning control PID.
	 * @param kp proportional gain coefficient
	 * @param ki integral gain coefficient
	 * @param kd derivative gain coefficient
	 * @param kf feedforward coefficient
	 */
	public void setTurnPID(double kp, double ki, double kd, double kf) 
	{   
		turnPID.setPID(kp, ki, kd, kf);   
		logger.info("PID constants are now: "
				+ "  kp: "+ kp
				+ "  ki: "+ ki
				+ "  kd: "+ kd
				+ "  kf: "+ kf);
	}


	
	
	

	/** Gets the current target angle
	 * @return the targetAngle
	 */
	public double getTargetAngle() {   return targetAngle;   }



	/** Sets the current target angle. This should be called repeatedly in order to keep the loop up to date.
	 * @param targetAngle the targetAngle to set
	 */
	public void setTargetAngle(double targetAngle) 
	{   
		this.targetAngle = targetAngle;
		turnPID.setSetpoint(targetAngle);
	}



	/** Gets the current target distance
	 * @return the targetDistance
	 */
	public double getTargetDistance() {   return targetDistance;   }



	/** Sets the current target distance. This should be called repeatedly in order to keep the loop up to date.
	 * @param targetDistance the targetDistance to set
	 */
	public void setTargetDistance(double targetDistance) 
	{   
		this.targetDistance = targetDistance;   
		posPID.setSetpoint(targetDistance);
	}



	/** Gets the current target velocity
	 * @return the targetVel
	 */
	public double getTargetVel() {   return targetVel;   }



	/** Sets the current target velocity. This should be called repeatedly in order to keep the loop up to date.
	 * @param targetVel the targetVel to set
	 */
	public void setTargetVel(double targetVel) {   this.targetVel = targetVel;   }



	/** 
	 * @return the leftSide
	 */
	public DriveSide getLeftSide() {   return leftSide;   }
	
	
	
	/** 
	 * @return the rightSide 
	 */
	public DriveSide getRightSide() {   return rightSide;   }
	
	
	
	/**
	 * Sets the current yaw angle of the robot, and optionally disables the cascade controller
	 * @param currYaw
	 * @param disable
	 */
	public void setYaw(double currYaw, boolean disable)
	{
		turnSrc.setCurrentAngle(currYaw);
		
		// disable if necessary
		if(disable)
			disable();
	}
	
	
	
	
	/**
	 * Frees all of the resources held by this {@link TankCascadeController}. This
	 * prepares the physical resources used by this object to be reused
	 */
	public void free()
	{ 
		logger.info("Attempting to shut down TankCascadeController...");
		
    	// free the resources
		if(this.leftSide != null)
		{
			this.leftSide.free();
			logger.info("Finished shutting down left side");
		}
		
		if(this.rightSide != null)
		{
			this.rightSide.free();
			logger.info("Finished shutting down right side");
		}
		
		if(this.gyro != null)
		{
			this.gyro.free();
			logger.info("Finished shutting down Gyro");
		}
		
		if(this.posPID != null)
		{
			this.posPID.free();
			logger.info("Finished freeing position PID");
		}
		
		if(this.turnPID != null)
		{
			this.turnPID.free();
			logger.info("Finished freeing turn PID");
		}
	}

	
}









