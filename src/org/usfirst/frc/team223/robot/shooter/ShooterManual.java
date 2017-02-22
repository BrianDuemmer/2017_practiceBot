package org.usfirst.frc.team223.robot.shooter;

import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterManual extends Command {

    public ShooterManual() {
    	requires(Robot.shooter);
    	this.setInterruptible(true);
    }


    protected void execute() {
    	Robot.shooter.augerMotor.set(Robot.oi.stick_oL.getX());
    	
    	if(Robot.oi.button_oA.get())
    		Robot.shooter.shooterMotor.set(0.25);
    	
    	else if(Robot.oi.button_oB.get())
    		Robot.shooter.shooterMotor.set(0.5);
    	
    	else if(Robot.oi.button_oX.get())
    		Robot.shooter.shooterMotor.set(0.75);
    	
    	else if(Robot.oi.button_oY.get())
    		Robot.shooter.shooterMotor.set(1);
    	
    	else
    		Robot.shooter.shooterMotor.set(Robot.oi.stick_oR.getX());
    }

    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

}
