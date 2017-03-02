package org.usfirst.frc.team223.robot.driveTrain;

import org.usfirst.frc.team223.AdvancedX.vision.VisionData;
import org.usfirst.frc.team223.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Aligns the robot with the goal via the onboard camera. This internally querys the camera for 
 * the latest snapshot of positioning data, and uses that to adjust our position
 */
public class cameraAlign extends Command 
{
	private boolean enableFwd;
	private boolean enableAngle;

	private VisionData visionPacket;

	// number of consecutive failed transmissions
	private int failureCt;
	private int maxFailureCt;

	// indicates a failure to fetch vision data
	private boolean fetchFailure;

	/**
	 * Craetes a new {@link cameraAlign}
	 * @param enableFwd if true, enabled forward adjustment
	 * @param enableAngle if true, enables angular adjustment
	 */
	public cameraAlign(boolean enableFwd, boolean enableAngle, int maxFailureCt, double timeout) 
	{
		requires(Robot.drive);

		this.enableAngle = enableAngle;
		this.enableFwd = enableFwd;

		this.maxFailureCt = maxFailureCt;

		this.setTimeout(timeout);
		this.setInterruptible(true);
	}


	protected void initialize() 
	{
		// reset failure count
		failureCt = 0;
		fetchFailure = false;

		Robot.drive.log.info("Starting new CameraAlign Command...");
	}


	protected void execute() 
	{
		// fetch the newest packet of data, and return if there is a transmission error
		if(!fetchPacket())
			return;

		// if we reach here, there was not a transmission error
		// align the angle first
		if(enableAngle)
			Robot.drive.drive_G2Point(visionPacket.angleError * -1, true);

		// re-update, and again return on error
		if(!fetchPacket())
			return;

		// align the distance
		if(enableFwd)
			Robot.drive.drive_G1xyCartesian(0, visionPacket.distError * -1, 0);
	}


	/**
	 * returns true if:
	 * <ul>
	 * 		<li>the command timed out</li>
	 * 		<li>there was an overrun on transmission errors</li>
	 *		<li>we are clear to shoot</li>
	 *		<li>the goal is not detected</li>
	 * </ul>
	 * 
	 * <br></br>
	 * {@inheritDoc}
	 */
	protected boolean isFinished() 
	{
		return fetchFailure || isTimedOut() || visionPacket.clearToshoot || !visionPacket.seesGoal;
	}


	protected void end() 
	{
		// shut down the drive
		Robot.drive.setRawOutput(0, 0, 0);

		Robot.drive.log.info("Exiting CameraAlign command...");

		Robot.drive.log.info("Exit status:  "
				+ "FetchFailure-" +fetchFailure 
				+ "  timeout-" +isTimedOut()
				+ "  clearToShoot-" + visionPacket.clearToshoot
				+ "  goalNotFound-" +!visionPacket.seesGoal);

	}


	protected void interrupted() 
	{
		end();
	}



	/**
	 * Fetches the newest data packet from the server
	 * @return true if the transmission was successful, false otherwise
	 */
	private boolean fetchPacket()
	{
		// obtain the newest packet
		visionPacket = Robot.visionClient.getDataPacket();

		// check for validity, and exit the method on failure
		if(!visionPacket.valid)
		{
			failureCt++;

			// Check for an overrun on failures
			if(failureCt > maxFailureCt)
			{
				Robot.drive.log.error("transmission failure count exceeded maximum failure limit! Cancelling command...");
				fetchFailure  = true;
			}

			return false; // return false on failure
		}

		return true; // return true otherwise
	}
}
