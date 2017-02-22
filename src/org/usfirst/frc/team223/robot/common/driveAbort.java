package org.usfirst.frc.team223.robot.common;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Knocks other commands out of the scheduler as an emergency stop feature
 */
public class driveAbort extends Command {

    public driveAbort() {
        requires(Robot.drive);
    }

    protected void initialize() 
    {
    }

    protected void execute() 
    {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
