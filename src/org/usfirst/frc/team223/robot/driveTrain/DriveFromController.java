package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot from a controller
 */
public class DriveFromController extends Command {
	

    public DriveFromController() 
    {
        requires(Robot.drive);
    }

    protected void initialize() {}


    protected void execute() 
    {
    	// Set the traction mode
    	if(Robot.oi.button_dR.get())
    		Robot.drive.setDriveType(driveType.FULL_TRACTION);
    	
    	else if(Robot.oi.button_dL.get())
    		Robot.drive.setDriveType(driveType.FRONT_TRACTION);
    	
    	else
    		Robot.drive.setDriveType(driveType.FULL_OMNI);
    	
    	
    	// set the Drive motor output
    	double fwd = Robot.oi.stick_dL.getY() * -1;
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
