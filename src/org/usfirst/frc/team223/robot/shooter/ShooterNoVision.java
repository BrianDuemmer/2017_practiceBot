package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ShooterNoVision extends Command 
{
	// time that the blender last changed state
	private static double lastChgTime;

	private boolean upToSpeed;
	private boolean prevEnabled = false;
	private boolean force;

	public ShooterNoVision()   { this(false); }
	
	/**
	 * Runs the shooter without vision targeting of the boiler
	 * @param force if true, ignores user input state. Useful for auto.
	 */
	public ShooterNoVision(boolean force)
	{
		this.force = force;
		requires(Robot.shooter);
	}


	public void initialize() {}


	public void execute()
	{

		// Set the setpoint to the target RPMs and enable the PID
		if(Robot.oi.operatorController.getRawAxis(3) > 0.75 || force)
		{
			if(!prevEnabled) // rising edge of button
			{
				Robot.shooter.log.info("Entering ShooterNoVisionCommand");
				Robot.shooter.bringUpToSpeed();
				lastChgTime = Timer.getFPGATimestamp();
			}

			prevEnabled = true;
			Robot.intake.setOutput(1);

			// run the auger if we are up to speed
			if(Robot.shooter.isUpToSpeed())  { runAuger(); }
			else { lastChgTime = 0; } // reset the timer
			
			
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
		Robot.intake.setOutput(0);
	}


	@Override
	protected boolean isFinished() { return false;   }
	
	
	
	private void runAuger()
	{
		boolean isOn = Robot.shooter.augerMotor.get() != 0;
		double dt = Timer.getFPGATimestamp() - lastChgTime;
		
		double out = 0;
		
		if(isOn && dt < Robot.shooter.augerHighTime) // we were running and should still run
			out = Robot.shooter.augerData.maxOut;
		
		else if(!isOn && dt > Robot.shooter.augerLowTime )  // we weren't running, but now we should run
		{
			out = Robot.shooter.augerData.maxOut;
			lastChgTime = Timer.getFPGATimestamp();
		}
		
		else if (isOn && dt > Robot.shooter.augerHighTime) // we were running, but now we should stop
		{
			out = 0;
			lastChgTime = Timer.getFPGATimestamp();
		}
		
		else if(!isOn && dt < Robot.shooter.augerLowTime) // we were not running, and we still shouldn't
			out = 0;
		
		else // illegal state
			Robot.shooter.log.error("Illegal state in ShooterNoVision");
		
		Robot.shooter.augerMotor.set(out);
	}
}










