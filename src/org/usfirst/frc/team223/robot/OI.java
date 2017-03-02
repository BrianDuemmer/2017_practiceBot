package org.usfirst.frc.team223.robot;

import org.usfirst.frc.team223.AdvancedX.utility.SmartControlStick;
import org.usfirst.frc.team223.robot.driveTrain.DriveTrainAbort;
import org.usfirst.frc.team223.robot.driveTrain.G1XYMovement;
import org.usfirst.frc.team223.robot.driveTrain.G2ArcMovement;
import org.usfirst.frc.team223.robot.hangar.HangControl;
import org.usfirst.frc.team223.robot.shooter.ShooterNoVision;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


public class OI 
{
	public Joystick driverController;
	public Joystick operatorController;
	
	public SmartControlStick stick_dL;
	public SmartControlStick stick_dR;
	
	public SmartControlStick stick_oL;
	public SmartControlStick stick_oR;
	
	// Driver controller buttons
	public JoystickButton button_dA;
	public JoystickButton button_dB;
	public JoystickButton button_dX;
	public JoystickButton button_dY;
	public JoystickButton button_dL;
	public JoystickButton button_dR;
	public JoystickButton button_dStart;
	public JoystickButton button_dBack;
	
	
	// Operator controller buttons
	public JoystickButton button_oA;
	public JoystickButton button_oB;
	public JoystickButton button_oX;
	public JoystickButton button_oY;
	public JoystickButton button_oL;
	public JoystickButton button_oR;
	public JoystickButton button_oStart;
	public JoystickButton button_oBack;
	
	
	
	
	
public OI() {
		
		driverController = new Joystick(0);
		operatorController = new Joystick(1);
		
		// bind the buttons for the driver controller
		button_dA = new JoystickButton(driverController, 1);
		button_dB = new JoystickButton(driverController, 2);
		button_dX = new JoystickButton(driverController, 3);
		button_dY = new JoystickButton(driverController, 4);
		button_dL = new JoystickButton(driverController, 5);
		button_dR = new JoystickButton(driverController, 6);
		button_dStart = new JoystickButton(driverController, 7);
		button_dBack = new JoystickButton(driverController, 8);
		
		// bind the analog sticks for the driver controller
		stick_dL = new SmartControlStick(driverController, 0, 1, 9);
		stick_dL.setParams(false, true, 0.1, 1);
		stick_dR = new SmartControlStick(driverController, 4, 5, 10);
		stick_dR.setParams(false, true, 0.1, 1);
		
		
		// bind the buttons for the operator controller
		button_oA = new JoystickButton(operatorController, 1);
		button_oB = new JoystickButton(operatorController, 2);
		button_oX = new JoystickButton(operatorController, 3);
		button_oY = new JoystickButton(operatorController, 4);
		button_oL = new JoystickButton(operatorController, 5);
		button_oR = new JoystickButton(operatorController, 6);
		button_oStart = new JoystickButton(operatorController, 8);
		button_oBack = new JoystickButton(operatorController, 7);
		
		// bind the analog sticks to the operator controller
		stick_oL = new SmartControlStick(operatorController, 0, 1, 9);
		stick_oL.setParams(false, true, 0.1, 1);
		stick_oR = new SmartControlStick(operatorController, 4, 5, 10);
		stick_oR.setParams(false, true, 0.1, 1);
		
		/////////////////////////// Driver Buttons ////////////////////////////
		button_dA.whenActive(new G1XYMovement(10, 0, false));
		button_dB.whenActive(new G1XYMovement(0, 10, true));
//		button_dX.whenActive(new G2ArcMovement(0.00001, 90, true));
		button_dX.whenActive(new G2ArcMovement(3, 90, true));
		button_dY.whenActive(new G2ArcMovement(0.00001, -90, true));
		button_dBack.whileActive(new DriveTrainAbort());

		
		////////////////////////// Operator Buttons ///////////////////////////
		button_oR.whileActive(new ShooterNoVision());
		button_oL.whileActive(new HangControl());
		


	}
	
}
