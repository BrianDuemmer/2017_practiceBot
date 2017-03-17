package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Old school method for G1XY interpolation. This one is blocking and can cause some nasty 
 * hangups and highly erratic behavior if not properly shut down
 * 
 * @deprecated Do not use under any circumstance!! use {@link G1FwdMovement} or 
 * {@link G1StrafeMovement} instead!
 */
public class G1XYMovement_old extends Command 
{
	private double xDist;
	private double yDist;
	private driveType type;

	public G1XYMovement_old(double xDist, double yDist, boolean useTraction) 
	{
		this.type = useTraction ? driveType.FULL_TRACTION : driveType.FULL_OMNI;
		this.yDist = yDist;
		this.xDist = xDist;
		
		this.setInterruptible(true);

		requires(Robot.drive);
	}


	protected void initialize() 
	{
		if(Robot.isDebug)
		{
			Robot.drive.setDriveType(type, true);
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
