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
public class CenterGearBoiler extends CommandGroup 
{
	// utility copy
	private Autonomous a = Robot.auto;
	
    public CenterGearBoiler() 
    {
        addSequential(new LogMsg(a.log, Level.INFO, "turning and shooting"));
        addSequential(new G2ArcMovement(0.000001, a.centerHopper_turn1, 0, true));
        addSequential(new ShooterNoVision(true), a.centerHopper_dwell);
        
        addSequential(new LogMsg(a.log, Level.INFO, "retracting..."));
        addSequential(new G2ArcMovement(0.00001, a.centerHopper_turn2, 0, true));
        addSequential(new G1FwdMovement(a.centerHopper_retract, 0, true));
        
        addSequential(new DropFrontGear());
        addSequential(new G1FwdMovement(-2, 0, true));
        
    }
}
