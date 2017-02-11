package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterDisabled extends Command {

    public ShooterDisabled() {
    	requires(Robot.shooter);
    }


    protected void execute() {
    	Robot.shooter.augerMotor.set(0);
    	Robot.shooter.augerMotor.set(0);
    }

    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

}
