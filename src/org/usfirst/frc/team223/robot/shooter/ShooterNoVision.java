package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShooterNoVision extends Command 
{
	// The last time the auger jammed, or -1 if it has not. When a jam is detected, the auger will run in reverse for a short 
	// time, to hopefully free up the jam, before returning to normal
	private static double lastJamTime = -1;
	
	private boolean upToSpeed;
	
	public ShooterNoVision()
	{
		requires(Robot.shooter);
//		requires(Robot.intake);
	}
	
	
	public void initialize()
	{
		Robot.shooter.log.info("Entering ShooterNoVisionCommand");
		upToSpeed = false;
		Robot.shooter.getShooterPID().reset();
	}
	
	
	public void execute()
	{
		
		// Set the setpoint to the target RPMs and enable the PID
		Robot.shooter.getShooterPID().setSetpoint(Robot.shooter.shooterTargetRPM);
		Robot.shooter.getShooterPID().enable();
		Robot.intake.setOutput(1);
		
		// run the auger
		runAuger();
		
	}
	
	
	protected void end() 
	{
		Robot.shooter.log.info("Exiting ShooterNoVision command");
		// turn off the PID
		Robot.shooter.getShooterPID().setSetpoint(0);
		Robot.shooter.getShooterPID().disable();
		
		// turn off the auger and intake
		Robot.shooter.augerMotor.set(0);
		Robot.intake.setOutput(0);
	}

	
	@Override
	protected boolean isFinished() {
		return false;
	}
	
	
	
	/**
	 * Updates the auger output, and does the jam check
	 */
	private void runAuger()
	{
		double target = Robot.shooter.shooterTargetRPM;
		
		if(Math.abs(Robot.shooter.getShooterRPM() - target) / target < 0.1 || upToSpeed)
		{
			Robot.shooter.augerMotor.set(Robot.shooter.augerMotorData.maxOut);
			upToSpeed = true;
		}
		else
			Robot.shooter.augerMotor.set(0);
	}

}










