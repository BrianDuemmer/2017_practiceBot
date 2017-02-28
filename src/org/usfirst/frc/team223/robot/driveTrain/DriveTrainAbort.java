package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveTrainAbort extends Command 
{
	
	public DriveTrainAbort()
	{
		requires(Robot.drive);
	}

	@Override
	protected boolean isFinished() 
	{
		// TODO Auto-generated method stub
		return false;
	}

}
