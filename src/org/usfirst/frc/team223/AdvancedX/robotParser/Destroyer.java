package org.usfirst.frc.team223.AdvancedX.robotParser;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SensorBase;
import net.sf.microlog.core.Logger;

/**
 * Utility Class for freeing the resources used by objects in code such as 
 * motors, sensors, etc. Each instance will automatically keep track of all of 
 * the resources that it destroys, and will not destroy objects that have already been 
 * destroyed, so you don't have to worry about errors and crashes due to trying to destroy
 * an object twice
 * @author Brian Duemmer
 *
 */
public class Destroyer 
{
	private Logger log;
	private List<Object> destroyedObjs;

	/**
	 * Creates a new instance of the {@link Destroyer} class. There should be one and only 
	 * one created throughout the lifetime of the program, and it should be owned by a 
	 * {@link AdvancedXManager Manager}.
	 * @param log the {@link Logger} that data will be logged to
	 */
	public Destroyer(Logger log)
	{
		this.log = log;
		log.info("Creating new instance of Destroyer");
		this.destroyedObjs = new ArrayList<Object>();
	}


	/**
	 * Clears the internal cache of destroyed objects
	 */
	public void clearDestroyed()
	{
		log.info("Resetting Destroyer. List of destroyed objects will be cleared");
		this.destroyedObjs = new ArrayList<Object>();
	}




	public boolean destroy(Object obj)
	{
		// Check if obj is null, and print a warning if it is
		if(obj == null)
		{
			log.warn("Attempted to destroy null object! Taking no action...");
			return false;
		}

		// Check for duplicate items
		if(this.destroyedObjs.contains(obj))
		{
			log.info("Item \"" +obj.toString()+ "\" has already been destroyed. Taking no action...");
			return false;
		}


		// If neither of those if tests were triggered, we can destroy the object

		// add the object to the list of objects we destroyed, so we don't try to destroy it again
		this.destroyedObjs.add(obj);

		// See what type of object this is, and call the proper destruction method
		if(obj instanceof SensorBase)
			((SensorBase) obj).free();

		else if(obj instanceof CANTalon)
			((CANTalon) obj).delete();
		
		else if(obj instanceof Freeable)
			((Freeable) obj).free();
		
		else if(obj instanceof PIDController)
			((PIDController) obj).free();
		
		else
		{
			log.error("No way to destroy object " +obj.toString());
			return false;
		}
		
		log.info("Destroyed object " +obj.toString());
		return true;
	}
}













