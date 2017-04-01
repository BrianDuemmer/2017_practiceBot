package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import org.usfirst.frc.team223.robot.driveTrain.G1StrafeMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

import edu.wpi.first.wpilibj.command.CommandGroup;
import net.sf.microlog.core.Level;

/**
 *	Goes to the hopper, gets some balls, and (hopefully) knocks ou the boiler
 */
public class BoilerHopperAuto extends CommandGroup 
{
	// utility copy
	private Autonomous a = Robot.auto;
	
    public BoilerHopperAuto() 
    {
        addSequential(new LogMsg(a.log, Level.INFO, "Initially backing up..."));
        addSequential(new G1FwdMovement(-1*a.boilerDistA, 0, true));
        
        addSequential(new LogMsg(a.log, Level.INFO, "Turning towards hopper..."));
        addSequential(new G2ArcMovement(0.00001, a.boilerTurnAngle, 0, true, 2));
        
        addSequential(new LogMsg(a.log, Level.INFO, "Initially approaching Hopper..."));
        addSequential(new G1FwdMovement(-1*a.boilerDistB, 0, true));
        
        addSequential(new LogMsg(a.log, Level.INFO, "Aligning with Hopper..."));
        addSequential(new G2ArcMovement(0.00001, -1*a.boilerTurnAngle, 0, true, 2));
        
        // start the shooter
        Robot.shooter.bringUpToSpeed();
        
        addSequential(new LogMsg(a.log, Level.INFO, "Approaching Hopper..."));
        addSequential(new G1StrafeMovement(-1*a.boilerApproachDist, 0));
        
        
        addSequential(new ShooterNoVision(true));
    }
}
