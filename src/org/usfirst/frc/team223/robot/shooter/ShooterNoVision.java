package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShooterNoVision extends Command 
{
	// The last time the auger jammed, or -1 if it has not. When a jam is detected, the auger will run in reverse for a short 
	// time, to hopefully free up the jam, before returning to normal
	private static double lastJamTime = -1;

	private boolean upToSpeed;
	private boolean prevEnabled = false;

	public ShooterNoVision()
	{
		requires(Robot.shooter);
		//		requires(Robot.intake);
	}


	public void initialize()
	{
	}


	public void execute()
	{

		// Set the setpoint to the target RPMs and enable the PID
		if(Robot.oi.operatorController.getRawAxis(3) > 0.75)
		{
			if(!prevEnabled) // rising edge of button
			{
				Robot.shooter.log.info("Entering ShooterNoVisionCommand");
				Robot.shooter.bringUpToSpeed();
			}

			prevEnabled = true;
			Robot.intake.setOutput(1);

			// run the auger if we are up to speed
			if(Robot.shooter.isUpToSpeed())  { Robot.shooter.augerMotor.set(1); }
			else { Robot.shooter.augerMotor.set(0); }
			
			
		} else if(Robot.oi.operatorController.getRawAxis(3) < 0.75 && prevEnabled) // falling edge of button
		{
			end();
			prevEnabled = false;
		}

		else  { prevEnabled = false; }

	}


	protected void end() 
	{
		Robot.shooter.log.info("Exiting ShooterNoVision command");
		// turn off the PID
		Robot.shooter.spinDown();

		// turn off the auger and intake
		Robot.shooter.augerMotor.set(0);
	}


	@Override
	protected boolean isFinished() { return false;   }
}










