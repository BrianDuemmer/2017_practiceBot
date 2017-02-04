package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShooterNoVision extends Command {
	
	
	public void initialize()
	{
		requires(Robot.shooter);
		Robot.shooter.log.info("Entering ShooterNoVisionCommand");
	}
	
	
	public void execute()
	{
		
		// Set the setpoint to the target RPMs and enable the PID
		Robot.shooter.getShooterPID().setSetpoint(Robot.shooter.shooterTargetRPM);
		Robot.shooter.getShooterPID().enable();
	}
	
	
	protected void end() 
	{
		// turn off the PID
		Robot.shooter.getShooterPID().setSetpoint(0);
		Robot.shooter.getShooterPID().disable();
	}

	
	@Override
	protected boolean isFinished() {
		// We never finish automatically, so return false
		return false;
	}

}
