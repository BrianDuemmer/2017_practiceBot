package org.usfirst.frc.team223.AdvancedX.vision;

import java.util.Arrays;

/**
 * POD type that contains all of the important data from the vision code. One object of this type 
 * will serve as a "packet" of vision data
 * @author Brian Duemmer
 *
 */
public class VisionData 
{
	/**
	 * The width of the goal's bounding rectangle
	 */
	public double boundWidth = 0;

	/**
	 * The height of the goal's bounding rectangle
	 */
	public double boundHeight = 0;

	/**
	 * The x-coordinate of the center of the goal's bounding rectangle
	 */
	public double boundX = 0;

	/**
	 * The y-coordinate of the center of the goal's bounding rectangle
	 */
	public double boundY = 0;

	/**
	 * the error of the distance from the goal
	 */
	public double distError = 0;

	/**
	 * The error of the angle to the goal
	 */
	public double angleError = 0;

	/**
	 * The measured distance between the robot and the goal
	 */
	public double measuredDist = 0;

	/**
	 * The measured angle between the robot and the goal
	 */
	public double measuredAngle = 0;

	/**
	 * If true, we are lined up to shoot
	 */
	public boolean clearToshoot = false;

	/**
	 * If <code>true</code>, then the data contained is valid from the camera. If false, 
	 * there was some error in parsing out the data
	 */
	public boolean valid = false;
	
	
	/**
	 * If true, the goal is being detected by the camera
	 */
	public boolean seesGoal = false;




	/**
	 * Parses out a raw data packet from the server TCP connection into the data itself
	 */
	public VisionData(String rawPacket)
	{
		try
		{
			// parse the string
			String[] rawDataElements = rawPacket.split("\n");
			System.out.println(Arrays.toString(rawDataElements));
			
			String[] processedDataElements = new String[rawDataElements.length];

			for(int i=0; i<rawDataElements.length; i++)
			{
				String str = rawDataElements[i];
				processedDataElements[i] = str.substring(str.indexOf(":") + 1, str.length());
			}
			
			System.out.println(Arrays.toString(processedDataElements));

			// populate the data elements
			boundWidth = new Double(processedDataElements[0]);
			boundHeight = new Double(processedDataElements[1]);
			boundX = new Double(processedDataElements[2]);
			boundY = new Double(processedDataElements[3]);
			distError = new Double(processedDataElements[4]);
			angleError = new Double(processedDataElements[5]);
			measuredDist = new Double(processedDataElements[6]);
			measuredAngle = new Double(processedDataElements[7]);
			clearToshoot = new Boolean(processedDataElements[8]);

			// set the valid flag to true if we reached this point
			valid = true;

		} catch(Exception e)
		{
			e.printStackTrace();
		}

	}



	/**
	 * Default constructor, sets all args as 0 or false
	 */
	public VisionData() {}
	
	
	
	public byte[] buildDataPacket()
	{
		StringBuilder sb = new StringBuilder();
		
		// add one line for each variable (in order)
		sb.append("BoundWidth:" + boundWidth + '\n');
		sb.append("BoundHeight:" + boundHeight + '\n');
		sb.append("BoundX:" + boundX + '\n');
		sb.append("BoundY:" + boundY + '\n');
		sb.append("DistanceError:" + distError + '\n');
		sb.append("AngleError:" + angleError + '\n');
		sb.append("MeasuredDistance:" + measuredDist + '\n');
		sb.append("MeasuredAngle:" + measuredAngle + '\n');
		sb.append("ClearToShoot:" + clearToshoot + '\n');
		sb.append("SeesGoal:" + seesGoal + '\n');
		
		// convert to byte array
		byte[] dataOut = sb.toString().getBytes();
		
		return dataOut;
	}



	@Override
	public String toString() {
		return "VisionData [boundWidth=" + boundWidth + ", boundHeight=" + boundHeight + ", boundX=" + boundX
				+ ", boundY=" + boundY + ", distError=" + distError + ", angleError=" + angleError + ", measuredDist="
				+ measuredDist + ", measuredAngle=" + measuredAngle + ", clearToshoot=" + clearToshoot + ", valid="
				+ valid + "]";
	}

}








