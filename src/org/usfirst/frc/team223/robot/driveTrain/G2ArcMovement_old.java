package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class G2ArcMovement_old extends Command 
{
	private double radius;
	private double angle;
	private driveType type;

	public G2ArcMovement_old(double radius, double angle, boolean useTraction) 
	{
		this.type = useTraction ? driveType.FULL_TRACTION : driveType.FULL_OMNI;
		this.radius = radius;
		this.angle = angle;
		
		this.setInterruptible(true);

		requires(Robot.drive);
	}


	protected void initialize() 
	{
		if(Robot.isDebug)
		{
			Robot.drive.setDriveType(type, true);
			Robot.drive.drive_G2Arc(radius, angle, 0);
		}
	}


	protected void execute() {
	}


	protected boolean isFinished() {
		return true;
	}


	protected void end() {
	}


	protected void interrupted() {
	}
}
