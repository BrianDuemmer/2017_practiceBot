package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;

import edu.wpi.first.wpilibj.command.CommandGroup;
import net.sf.microlog.core.Level;

/**
 *	Does a simple boiler-only auto, with the robot starting near the 
 *	boiler, making a turn, and emptying its hopper
 */
public class BoilerAuto extends CommandGroup 
{
	// utility copy
	private Autonomous a = Robot.auto;
	
    public BoilerAuto() 
    {
        addSequential(new LogMsg(a.log, Level.INFO, "Driving forward..."));
//        addSequential(new G1FwdMovement(dist, finalVel, useTraction));
    }
}
