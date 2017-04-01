package org.usfirst.frc.team223.robot.auto;

import org.usfirst.frc.team223.AdvancedX.AdvancedXManager;
import org.usfirst.frc.team223.AdvancedX.robotParser.EnumPair;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BasicType;
import org.usfirst.frc.team223.robot.Robot;
import org.usfirst.frc.team223.robot.common.LogMsg;
import org.usfirst.frc.team223.robot.driveTrain.G1FwdMovement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import net.sf.microlog.core.Level;
import net.sf.microlog.core.Logger;

/**
 * Contains the jumping off point for all auto routines, {@link Autonomous#runAuto() runAuto()}
 * 
 * @author Brian Duemmer
 */
public class Autonomous
{
	// length of auto period
	public static final double AUTO_TIME = 15;
	
	
	double towerDist = 10;
	double towerWidth = 3.918;
	double towerAngle = 30;
	double robotLength = 3;
	
	
	double distToGear1 = 8.702;
	double gear1ApproachDist = 2.55;
	double gear1CrossDist = 12;
	
	double distToGear3 = 7.316;
	double gear3ApproachDist = 2.55;
	double gear3CrossDist = 12;

	double gear2ApproachDist = 1.96;
	
	double lineCrossDist = 12;
	
	double boilerDistA = 1;
	double boilerDistB = 1;
	double boilerTurnAngle = 30;
	double boilerApproachDist = 1;
	
	double sideGearBoilerRetract;
	double sideGearBoilerTurn;
	
	double centerGearBoilerRetract;
	double centerGearBoilerTurn;
	
	double turnTimeout = 5;
	
	EnumPair autoSelect;
	boolean passAutoLine;
	boolean boilerAfter;
	
	double centerHopper_turn1;
	double centerHopper_dwell;
	double centerHopper_turn2;
	double centerHopper_retract;
	
	double bNoH_retract;
	double bNoH_turn;



	// utility
	Logger log;
	
	Alliance alliance;
	Alliance overrideAlliance = Alliance.Red;
	
	// auto that will run. Initialized to a safe default
	public Command autoCommand = new LogMsg(log, Level.FATAL, "This is the default auto. This should never run!");
	
	
	/**
	 * Loads all of the data and initialized autonomous
	 */
	public Autonomous(AdvancedXManager manager)
	{		
		log = manager.getRoboLogger().getLogger("Autonomous_old");
		
		GXMLparser parser = manager.obtainParser();
		
		// parse common keys
		autoSelect = (EnumPair)parser.getKeyByPath("auto/mode", BasicType.ENUM);
		passAutoLine = (boolean) parser.getKeyByPath("auto/crossLineAfter", BasicType.BOOL);
		lineCrossDist = (double) parser.getKeyByPath("auto/gear1/lineCrossDist", BasicType.DOUBLE);
		towerDist = (double) parser.getKeyByPath("auto/general/towerDist", BasicType.DOUBLE);
		towerAngle = (double) parser.getKeyByPath("auto/general/towerAngle", BasicType.DOUBLE);
		towerWidth = (double) parser.getKeyByPath("auto/general/towerWidth", BasicType.DOUBLE);
		boilerAfter = (boolean) parser.getKeyByPath("auto/general/boilerAfter", BasicType.BOOL);
		EnumPair alliance = (EnumPair) parser.getKeyByPath("auto/overrideAlliance", BasicType.ENUM);
		
		if(alliance.selection.equals("Red"))
			overrideAlliance = Alliance.Red;
		else
			overrideAlliance = Alliance.Blue;
		
		// gear 1 specific
		distToGear1 = (double) parser.getKeyByPath("auto/gear1/fwdDist", BasicType.DOUBLE);
		gear1ApproachDist = (double) parser.getKeyByPath("auto/gear1/approachDist", BasicType.DOUBLE);
		gear1CrossDist = (double) parser.getKeyByPath("auto/gear1/lineCrossDist", BasicType.DOUBLE);
		
		// gear 2 specific
		gear2ApproachDist = (double) parser.getKeyByPath("auto/gear2/approachDist", BasicType.DOUBLE);
		
		// gear 3 specific
		distToGear3 = (double) parser.getKeyByPath("auto/gear3/fwdDist", BasicType.DOUBLE);
		gear3ApproachDist = (double) parser.getKeyByPath("auto/gear3/approachDist", BasicType.DOUBLE);
		gear3CrossDist = (double) parser.getKeyByPath("auto/gear3/lineCrossDist", BasicType.DOUBLE);
		
		// boiler specific
		boilerDistA = (double) parser.getKeyByPath("auto/boiler/distA", BasicType.DOUBLE);
		boilerDistB = (double) parser.getKeyByPath("auto/boiler/distB", BasicType.DOUBLE);
		boilerApproachDist = (double) parser.getKeyByPath("auto/boiler/approachDist", BasicType.DOUBLE);
		boilerTurnAngle = (double) parser.getKeyByPath("auto/boiler/turnAngle", BasicType.DOUBLE);
		
		// boiler / gear keys
		sideGearBoilerRetract = (double) parser.getKeyByPath("auto/boiler/gearSide/retract", BasicType.DOUBLE);
		sideGearBoilerTurn = (double) parser.getKeyByPath("auto/boiler/gearSide/turn", BasicType.DOUBLE);
		
		centerGearBoilerRetract = (double) parser.getKeyByPath("auto/boiler/centerGear/retract", BasicType.DOUBLE);
		centerGearBoilerTurn = (double) parser.getKeyByPath("auto/boiler/centerGear/turn", BasicType.DOUBLE);
		
		// boiler no hopper
		centerHopper_turn1 = (double) parser.getKeyByPath("auto/centerGearBoiler/turn1", BasicType.DOUBLE);
		centerHopper_dwell = (double) parser.getKeyByPath("auto/centerGearBoiler/dwell", BasicType.DOUBLE);
		centerHopper_turn2 = (double) parser.getKeyByPath("auto/centerGearBoiler/turn2", BasicType.DOUBLE);
		centerHopper_retract = (double) parser.getKeyByPath("auto/centerGearBoiler/retract", BasicType.DOUBLE);
		
		bNoH_retract = (double) parser.getKeyByPath("auto/bNoH/retract", BasicType.DOUBLE);
		bNoH_turn = (double) parser.getKeyByPath("auto/bNoH/turn", BasicType.DOUBLE);

	}
	
	/**
	 * Non-blocking method that actually runs autonomous. Call this in {@link Robot#autonomousInit() autonomousInit()}
	 * and it will handle the rest from there.
	 */
	public void runAuto()
	{
		log.info("===============================================================================");
		log.info("============================== Starting Auto... ===============================");
		log.info("===============================================================================");
		
		log.info("Mode is:  \"" +autoSelect.selection+ "\"");
		
		alliance = DriverStation.getInstance().getAlliance();
		if(alliance == Alliance.Invalid)
			alliance = overrideAlliance;
		log.info("Alliance is: " + alliance.toString());	
		
		
		// Select the proper auto
		switch(autoSelect.selection)
		{
			case("gear1"):
				log.info("gear1");
				autoCommand = new Gear1Auto();
				break;
				
			case("gear2"):
				log.info("gear2");
				autoCommand = new Gear2Auto();
				break;
			
			case("gear3"):
				log.info("gear3");
				autoCommand = new Gear3Auto();
				break;
			
			case("crossLine"):
				log.info("crossLine");
				autoCommand = new G1FwdMovement(lineCrossDist, 0, true);
				break;
				
			case("boilerHopper"): 
				log.info("boilerHopper");
				autoCommand = new BoilerHopperAuto();
				break;
				
			case("centerGearBoiler"): 
				log.info("CenterGearBoiler");
				autoCommand = new CenterGearBoiler();
				break;

			
			case("doNothing"):
				log.info("doNothing");
				autoCommand = new LogMsg(log, Level.INFO, "Doing nothing for auto");
				break;
		}
		
		//set the auto going
		autoCommand.start();
	}
}
