package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot from a controlle
 */
public class DriveFromController extends Command {

    public DriveFromController() 
    {
        requires(Robot.drive);
    }

    protected void initialize() {}


    protected void execute() 
    {
    	double fwd = Robot.oi.stick_dL.getY();
    	double strafe = Robot.oi.stick_dL.getX();
    	double turn = Robot.oi.stick_dR.getX();
    	
    	Robot.drive.setRawOutput(fwd, strafe, turn);
    }

    
    protected boolean isFinished() {
        return false;
    }

    
    protected void end() {
    }


    protected void interrupted() {
    }
}
