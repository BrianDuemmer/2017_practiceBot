package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.AdvancedX.motionControl.LinearFeedInterpolator;
import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.driveTrain.ButterflyHDrive.driveType;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot in an arc formation via a G2 (circular) fashion. This will automatically 
 * invoke the {@link LinearFeedInterpolator} to perform the movement
 */
public class G2ArcMovement extends Command 
{
	private double radius;
	private double angleChange;
	private double finalVel;
	
	// converts raw LFI output to velocity outputs
	private double lfiScalar;
	private double nominalDist;

	private boolean useTraction;

	// if true, a fault condition has occured, so shutdown immediately
	private boolean faultCond = false;

	// gyro angle at the start of the interpolation
	private double startAngle;


	public G2ArcMovement(double radius, double angleChange, double finalVel, boolean useTraction, double timeout) 
	{
		this(radius, angleChange, finalVel, useTraction);
		setTimeout(timeout);
	}



	public G2ArcMovement(double radius, double angleChange, double finalVel, boolean useTraction)
	{
		requires(Robot.drive);
		this.radius = radius;
		this.angleChange = Math.toRadians(angleChange);
		this.useTraction = useTraction;
	}


	protected void initialize() 
	{
		Robot.drive.log.info("Starting new G2Arc sequence...");
		
		// set the drive type
		driveType mode = useTraction  ?  driveType.FULL_TRACTION : driveType.FULL_OMNI;
		Robot.drive.setDriveType(mode, true);
		
		// reset fault condition
		faultCond = false;

		// check for fault conditions

		if(angleChange == 0) // Illegal move
		{
			Robot.drive.log.error("Attempted G2Arc cycle with an angleChange of 0! Aborting move...");
			faultCond = true;
			return;
		} 		

		// calculate the left, right, and nominal distances
		double leftDist;
		double rightDist;

		if(angleChange > 0)
		{
			leftDist = angleChange * (radius + 0.5*Robot.drive.wheelbaseWidth);
			rightDist = angleChange * (radius - 0.5*Robot.drive.wheelbaseWidth);
		} else
		{
			leftDist = -angleChange * (radius - 0.5*Robot.drive.wheelbaseWidth);
			rightDist = -angleChange * (radius + 0.5*Robot.drive.wheelbaseWidth);
		}

		nominalDist = angleChange * radius;

		// calculate LFI scalar, as the nominal robot velocity will be different from the velocities of the left and right sides
		lfiScalar = Math.max(Math.abs(leftDist), Math.abs(rightDist)) / Math.abs(nominalDist);

		// get the master PIDs ready
		Robot.drive.resetPIDs();

		Robot.drive.fwdPosPID.enable();
		Robot.drive.turnPosPID.enable();


		// Setup the LFI, making sure to reduce everything by the lfiScalar value
		Robot.drive.distInterpolator.setConstraints(Robot.drive.maxTractionAcceleration / lfiScalar,  Robot.drive.maxTractionVel / lfiScalar);
		Robot.drive.distInterpolator.start(Robot.drive.getVelocity() / lfiScalar, finalVel / lfiScalar, radius * Math.abs(angleChange));



		// copy over the correct constraints
		double aMax = Robot.drive.maxCenterAcceleration;
		double vMax = Robot.drive.maxCenterVel;

		// set up any other math type stuff
		startAngle = Robot.drive.getAngle();

		// setup the PIDs
		Robot.drive.resetPIDs();
		Robot.drive.turnPosPID.enable();
		
		Robot.drive.log.info("angChg: " +Math.toDegrees(angleChange)+ "  stAngle: " +startAngle);

	}





	// Called repeatedly when this Command is scheduled to run
	protected void execute() 
	{
		// nominal velocity / position
		double vNominal = Robot.drive.distInterpolator.getTargetVel();
		double sNominal = Robot.drive.distInterpolator.getTargetPos();

		// feed master PIDs
		Robot.drive.fwdPosPID.setSetpoint(sNominal);
		Robot.drive.turnPosPID.setSetpoint(Math.toDegrees((sNominal / Math.abs(nominalDist)) * angleChange) + startAngle);

		// calculate adjusted forward distance
		double adjFwd = Robot.drive.fwdDistAction / lfiScalar;

		double leftVel;
		double rightVel;


		// calculate rate actions, factoring in for forward rate action
		leftVel = ((vNominal + adjFwd)/radius) * (radius + Math.signum(angleChange)*Robot.drive.wheelbaseWidth * 0.5);
		rightVel = ((vNominal + adjFwd)/radius) * (radius - Math.signum(angleChange)*Robot.drive.wheelbaseWidth * 0.5);

		Robot.drive.feedLRSlavePIDs(leftVel, rightVel, 0, Robot.drive.turnPosAction);
		
//		Robot.drive.log.info("finExec");
	}




	protected boolean isFinished() 
	{
//		Robot.drive.log.info("isfinished");
		boolean pastSet = false; // whether we have passed our setpoint and should stop
		
		if(angleChange > 0)
			pastSet = Robot.drive.getAngle() + Robot.drive.turnPosPIDData.tolerance > startAngle + Math.toDegrees(angleChange);
		else
			pastSet = Robot.drive.getAngle() - Robot.drive.turnPosPIDData.tolerance < startAngle + Math.toDegrees(angleChange);
		
		Robot.manager.getNt().putBoolean("pastSet", pastSet);
		
		// if any exit conditions are met, log them and return true. Else, return false.
		if(isTimedOut())
		{
			Robot.drive.log.warn("G1Strafe command has timed out!");
			return true;
		} else if(pastSet)
		{
			Robot.drive.log.info("Setpoint for G2Arc interpolation has been passed");
			return true;
		} else if(faultCond)
		{
			Robot.drive.log.error("Fault condition detected in G2Arc command!");
			return true;
		}


		return false;
	}


	protected void end() 
	{
		Robot.drive.log.info("Stopping G2Arc command...");
		Robot.drive.resetPIDs();
		Robot.drive.setRawOutput(0, 0, 0);
	}




	protected void interrupted() {}
}
