package org.usfirst.frc.team223.robot.intake;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeControl extends Command {

    public IntakeControl() {
    	requires(Robot.intake);
    	setInterruptible(true);
    }


    protected void initialize() {
    }


    protected void execute() 
    {
//    	double out = Robot.oi.operatorController.getRawAxis(3) - Robot.oi.operatorController.getRawAxis(2);
    	double out = 0;
    	if(Robot.oi.button_oR.get())
    		out = -1*Robot.intake.motorData.maxOut;
    	
    	else if(Robot.oi.button_oL.get())
    		out = Robot.intake.motorData.maxOut;
    	
    	Robot.intake.setOutput(out);
    }


    protected boolean isFinished() {
        return false;
    }


    protected void end() {
    }


    protected void interrupted() {
    }
}
