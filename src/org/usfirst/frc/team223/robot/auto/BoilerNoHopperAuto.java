package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import org.usfirst.frc.team223.robot.driveTrain.G1StrafeMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.gear.DropFrontGear;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

import edu.wpi.first.wpilibj.command.CommandGroup;
import net.sf.microlog.core.Level;

/**
 *	throws 10 balls in the boiler, then crosses baseline
 */
public class BoilerNoHopperAuto extends CommandGroup 
{
	// utility copy
	private Autonomous a = Robot.auto;  
    
    
    public BoilerNoHopperAuto() 
    {
        addSequential(new LogMsg(a.log, Level.INFO, "retracting..."));
        addSequential(new G1FwdMovement(-1*a.bNoH_retract, 0, true));
        
        addSequential(new LogMsg(a.log, Level.INFO, "turning..."));
        addSequential(new G2ArcMovement(0.00001, a.bNoH_turn, 0, true));
        
        addSequential(new LogMsg(a.log, Level.INFO, "shooting..."));
        addSequential(new ShooterNoVision(true));
    }
}
