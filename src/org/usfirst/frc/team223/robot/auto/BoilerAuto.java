package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

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
        addSequential(new G1FwdMovement(-1*a.boilerFwdDist, 0, true));
        
        addSequential(new LogMsg(a.log, Level.INFO, "Turning to boiler..."));
        addSequential(new G2ArcMovement(0.00001, -1*a.boilerTurnAngle, 0, true, 3));
        
        // start the shooter
        Robot.shooter.bringUpToSpeed();
        
        addSequential(new LogMsg(a.log, Level.INFO, "Approaching boiler..."));
        addSequential(new G1FwdMovement(a.boilerApproachDist, 0, true));
        
        addSequential(new ShooterNoVision(true));
    }
}
