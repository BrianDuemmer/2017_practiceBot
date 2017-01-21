package org.usfirst.frc.team223.AdvancedX.motionControl;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;




/**
 * Serves as a wrapper class that allows the NavX to implement the {@link Gyro} interface.
 * @author Brian Duemmer
 *
 */
public class GyroNavX extends AHRS implements Gyro {

	public GyroNavX(Port serial_port_id) {
		super(serial_port_id);
	}

	
	/**
	 * {@inheritDoc}
	 * <P/>
	 * <b>NOTE: </b> This does nothing for the NavX. It will automatically calibrate itself.
	 */
	@Override
	public void calibrate() {}

}
