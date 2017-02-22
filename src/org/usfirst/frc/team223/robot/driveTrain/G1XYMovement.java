package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class G1XYMovement extends Command 
{
	private double xDist;
	private double yDist;
	private driveType type;

	public G1XYMovement(double xDist, double yDist, boolean useTraction) 
	{
		this.type = useTraction ? driveType.FULL_TRACTION : driveType.FULL_OMNI;
		this.yDist = yDist;
		this.xDist = xDist;

		requires(Robot.drive);
	}


	protected void initialize() 
	{
		if(Robot.isDebug)
		{
			Robot.drive.setDriveType(type);
			Robot.drive.drive_G1xyCartesian(xDist, yDist, 0);
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
