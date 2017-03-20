package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import org.usfirst.frc.team223.robot.driveTrain.G1StrafeMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.gear.DropGear;

import edu.wpi.first.wpilibj.command.CommandGroup;
import net.sf.microlog.core.Level;

/**	Places a gear on peg 3, optionally approaches the boiler
 *
 */
public class Gear3Auto extends CommandGroup 
{
	// utility copy to simplify code
	private Autonomous a = Robot.auto;
	
    public Gear3Auto() 
    {
    	addSequential(new LogMsg(a.log, Level.INFO, "Beginning Tower approach..."));
    	addSequential(new G1FwdMovement(a.distToGear3, 0, true)); // drive to tower
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Turning onto tower..."));
    	addSequential(new G2ArcMovement(0.000001, -1*a.towerAngle, 0, true, a.turnTimeout)); // turn to tower
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Approaching peg..."));
    	addSequential(new G1FwdMovement(a.gear3ApproachDist, 0, true)); // approach peg
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Dropping Gear..."));
    }
}
