package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveTrainAbort extends Command 
{
	
	public DriveTrainAbort()
	{
		requires(Robot.drive);
		
		this.setInterruptible(false);
	}
	
	protected void initialize() 
	{
		Robot.drive.log.error("Aborting drivetrain...");
	};
	
	@Override
	protected void execute() 
	{
		Robot.drive.resetPIDs();
		Robot.drive.distInterpolator.cancel();
	}
	
	@Override
	protected boolean isFinished() 
	{
		return false;
	}

}
