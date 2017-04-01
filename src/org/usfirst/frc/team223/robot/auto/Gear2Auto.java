package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.gear.DropFrontGear;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import net.sf.microlog.core.Level;

/**	Places a gear on peg 2, optionally approaches the boiler
 *
 */
public class Gear2Auto extends CommandGroup 
{
	// utility copy to simplify code
	private Autonomous a = Robot.auto;
	
    public Gear2Auto() 
    {
    	double fwdDist = a.towerDist - a.robotLength;
    	a.log.info("fwdDist: " +fwdDist);
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Beginning Tower approach..."));
    	addSequential(new G1FwdMovement(fwdDist, 0, true)); // drive to tower
    	
    	addSequential(new LogMsg(a.log, Level.INFO, "Dropping Gear..."));
    	addSequential(new DropFrontGear()); // Drop gear

    	addSequential(new LogMsg(a.log, Level.INFO, "Leaving peg..."));
    	addSequential(new G1FwdMovement(-1*a.centerGearBoilerRetract, 0, true)); // leave peg
    	
    	//shoot into the boiler if necessary
    	if(a.boilerAfter)
    	{
    		addParallel(new InstantCommand()
			{
    			protected void initialize()   { Robot.shooter.bringUpToSpeed(); }
			});
    		
    		addSequential(new G2ArcMovement(0.000001, a.centerGearBoilerTurn, 0, true));
    		addSequential(new ShooterNoVision(true));
    	}
    }
}
