package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.AdvancedX.motionControl.LinearFeedInterpolator;
import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot forward in a G1 (linear) fashion. This will automatically 
 * invoke the {@link LinearFeedInterpolator} to perform the movement
 */
public class G1StrafeMovement extends Command 
{
	double dist;
	double finalVel;


	public G1StrafeMovement(double dist, double finalVel, double timeout) 
	{
		this(dist, finalVel);
		setTimeout(timeout);
	}
	
	
	
	public G1StrafeMovement(double dist, double finalVel)
	{
		requires(Robot.drive);
		this.dist = dist;
		this.finalVel = finalVel;
	}


	protected void initialize() 
	{
		Robot.drive.log.info("Starting new G1Strafe sequence...");
		
		Robot.drive.setDriveType(driveType.FULL_OMNI, true);

		// copy over the correct constraints
		double aMax = Robot.drive.maxCenterAcceleration;
		double vMax = Robot.drive.maxCenterVel;

		// setup the PIDs
		Robot.drive.resetPIDs();
		Robot.drive.centerPosPID.enable();

		// setup and start the interpolator
		Robot.drive.distInterpolator.setConstraints(aMax, vMax);
		Robot.drive.distInterpolator.start(0, finalVel, dist);
	}
	
	
	
	

	// Called repeatedly when this Command is scheduled to run
	protected void execute() 
	{
		// feed the master PID - it will automatically set its output
		Robot.drive.centerPosPID.setSetpoint(Robot.drive.distInterpolator.getTargetPos());
		
		// Set the other stuff to 0
		Robot.drive.turnPosAction = 0;
		Robot.drive.leftRateAction = 0;
		Robot.drive.rightRateAction = 0;

	}

	
	

	protected boolean isFinished() 
	{
		// if any exit conditions are met, log them and return true. Else, return false.
		
		if(isTimedOut())
		{
			Robot.drive.log.warn("G1Strafe command has timed out!");
			return true;
//		} else if(!Robot.drive.distInterpolator.isActive())
		} else if(Robot.drive.centerPosPID.onTarget())
		{
			Robot.drive.log.info("Finished G1Strafe interpolation sequence");
			return true;
		}
		
		
		return false;
	}

	
	protected void end() 
	{
		Robot.drive.log.info("Stopping G1Strafe command...");
		Robot.drive.resetPIDs();
		Robot.drive.setRawOutput(0, 0, 0);
	}
	
	
	

	protected void interrupted() {}
}
