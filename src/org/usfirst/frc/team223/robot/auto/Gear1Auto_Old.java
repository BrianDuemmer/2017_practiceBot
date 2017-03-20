package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import org.usfirst.frc.team223.robot.driveTrain.G1StrafeMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.gear.DropGear;

import edu.wpi.first.wpilibj.command.CommandGroup;
import net.sf.microlog.core.Level;

/**	Places a gear on peg 1, optionally approaches the boiler
 *
 */
public class Gear1Auto_Old extends CommandGroup 
{
	// utility copy to simplify code
	private Autonomous a = Robot.auto;
	
    public Gear1Auto_Old() 
    {
    	addSequential(new LogMsg(a.log, Level.INFO, "Beginning Tower approach..."));
    	addSequential(new G1FwdMovement(a.distToGear1, 0, true)); // drive to tower
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Turning onto tower..."));
    	addSequential(new G2ArcMovement(0.000001, -1*a.towerAngle, 0, true, a.turnTimeout)); // turn to tower
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Strafing to peg..."));
    	addSequential(new G1StrafeMovement(a.gear1ApproachDist, 0, 3)); // approach peg
    	
    	addSequential(new DropGear()); // Drop the gear
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "pulling out..."));
    	addSequential(new G1StrafeMovement(-1*a.gear1ApproachDist, 0, 3)); // pull away from peg
    }
}
