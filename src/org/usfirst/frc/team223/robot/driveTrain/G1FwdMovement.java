package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.AdvancedX.motionControl.LinearFeedInterpolator;
import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot forward in a G1 (linear) fashion. This will automatically 
 * invoke the {@link LinearFeedInterpolator} to perform the movement
 */
public class G1FwdMovement extends Command 
{
	double dist;
	double finalVel;
	boolean useTraction;


	public G1FwdMovement(double dist, double finalVel, boolean useTraction, double timeout) 
	{
		this(dist, finalVel, useTraction);
		setTimeout(timeout);
	}
	
	
	
	public G1FwdMovement(double dist, double finalVel, boolean useTraction)
	{
		requires(Robot.drive);
		this.dist = dist;
		this.useTraction = useTraction;
	}


	protected void initialize() 
	{
		Robot.drive.log.info("Starting new G1Fwd sequence...");
		
		// set the drive type
		driveType mode = useTraction  ?  driveType.FULL_TRACTION : driveType.FULL_OMNI;
		Robot.drive.setDriveType(mode, true);

		// copy over the correct constraints
		double aMax = Robot.drive.maxTractionAcceleration;
		double vMax = Robot.drive.maxTractionVel;

		// setup the PIDs
		Robot.drive.resetPIDs();
		Robot.drive.fwdPosPID.enable();

		// setup and start the interpolator
		Robot.drive.distInterpolator.setConstraints(aMax, vMax);
		Robot.drive.distInterpolator.start(Robot.drive.getVelocity(), finalVel, dist);
	}
	
	
	
	

	// Called repeatedly when this Command is scheduled to run
	protected void execute() 
	{
		// feed the master PIDs
		Robot.drive.fwdPosPID.setSetpoint(Robot.drive.distInterpolator.getTargetPos());

		// base velocities calculated via the LFI
		double fwdRateAction = Robot.drive.distInterpolator.getTargetVel();
		Robot.drive.leftRateAction = fwdRateAction;
		Robot.drive.rightRateAction = fwdRateAction;
		
		// Set the other stuff to 0
		Robot.drive.centerDistAction = 0;
		Robot.drive.turnPosAction = 0;

		// feed the slave PIDs
		Robot.drive.feedLRSlavePIDs();
	}

	
	

	protected boolean isFinished() 
	{
		// if any exit conditions are met, log them and return true. Else, return false.
		
		if(isTimedOut())
		{
			Robot.drive.log.warn("G1Fwd command has timed out!");
			return true;
		} else if(!Robot.drive.distInterpolator.isActive())
		{
			Robot.drive.log.info("Finished G1Fwd interpolation sequence");
			return true;
		}
		
		
		return false;
	}

	
	protected void end() 
	{
		Robot.drive.log.info("Stopping G1Fwd command...");
		Robot.drive.resetPIDs();
		Robot.drive.setRawOutput(0, 0, 0);
	}
	
	
	

	protected void interrupted() {}
}
