package org.usfirst.frc.team223.AdvancedX.robotParser;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathFactory;

import org.usfirst.frc.team223.AdvancedX.RoboLogManagerBase;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import net.sf.microlog.core.Logger;


/**
 * This class serves as a parser for GXML configuration files saved by the LabVIEW dashboard.
 * 
 * @author Brian Duemmer
 *
 */
public class GXMLparser 
{
	// References for the active document and XPath engine
	private Document doc;
	private XPath xpath;
	
	// Types that can be parsed in GXML
	public enum BASIC_TYPE
	{
		DOUBLE,
		INT,
		STRING,
		BOOL,
		ENUM
	}
	
	// logger object to print messages
	Logger logger;
	
	
	
	/**
	 * Creates a new instance of of the parser.
	 * 
	 * <i> NOTE:</i> There should only be one GXMLparser for
	 * each XML file
	 * @param path the path to the target XML file
	 */
	public GXMLparser(String path, RoboLogManagerBase roboLogManagerBase)
	{
		this.logger = roboLogManagerBase.getLogger("GXMLParser");
		
		logger.info("Attempting to open XML configuration file at path \"" + path + "\"...");
		
		// Attempt to open the document
		try
		{
			// initialize the document
			DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
			DocumentBuilder db = dbf.newDocumentBuilder();
			doc = db.parse(new File(path));
			
			//initialize xpath
			XPathFactory xpf = XPathFactory.newInstance();
			xpath = xpf.newXPath();
			
			// Print a success message
			logger.info("Successfully opened configuration file at \"" + path + "\"");
			
		} catch(Exception e)
		{
			// Print an error message on error
			logger.fatal("Failed to open configuration file \"" + path +"\". DETAILS: ", e);
		}
	}

	
	
	
	
	/**
	 * Parses a single key out of the xml file, 
	 * @return an {@link Object} that contains the key data. Cast it to a double, int, etc. as needed.
	 */
	public Object getKeyByPath(String path, BASIC_TYPE type)
	{
		// Object that will be returned
		Object ret = null;
		
		// String representation of the data in ret
		String data = "";
		
		// Expression to evaluate in order to find the target node
		XPathExpression expr;
		
		// The target node specified by path
		Node key;
		
		// attempt to parse the key. If it fails, print a warning, and return null
		try 
		{
			expr = xpath.compile("GXML_Root/" + path);
			key = (Node)expr.evaluate(doc, XPathConstants.NODE);
			data = key.getTextContent();
			
			// Create a new object with type corresponding to the type argument, only if data isn't null
			if(type == BASIC_TYPE.BOOL)
				ret = new Boolean(data);
			
			if(type == BASIC_TYPE.INT)
				ret = new Integer(data);
			
			if(type == BASIC_TYPE.DOUBLE)
				ret = new Double(data);
			
			if(type == BASIC_TYPE.STRING)
				ret = data;
			
			if(type == BASIC_TYPE.ENUM)
			{
				int val = new Integer(data);
				ret = new EnumPair(key.getAttributes().getNamedItem("sel").getTextContent(), val);
			}
			
			// print a message saying that this was successful
			logger.info("Successfully parsed key \"" + path + "\" with value of: " +ret);
			
		} catch (Exception e)
		{
			logger.warn("Failed to parse key \"" + path + "\". DETAILS: ", e);
		}
		
		// if ret is null, return the default value
		if(ret == null)
		{
			if(type == BASIC_TYPE.BOOL)
				return false;
			
			else if(type == BASIC_TYPE.DOUBLE)
				return 0.0;
			
			else if(type == BASIC_TYPE.ENUM)
				return new EnumPair("", 0);
			
			else if(type == BASIC_TYPE.INT)
				return 0;
			
			else if(type == BASIC_TYPE.STRING)
				return "";
			
			else
				logger.error("Unrecognized type passed to parse key");
		}
		
		// Return the newly allocated object
		return ret;
	}
	
	
	
	
	/**
	 * Gets an atomic element value given the direct parent and the name of the element.

	 * @return An Object containing the data found by the search, or null if not found.
	 * Add a cast statement to cast it to whatever type that was parsed
	 * 
	 */
	public Object getElementByName(String name, Node parent, BASIC_TYPE type)
	{
		// if parent is null, print an error message and return null
		if(parent == null)
		{
			logger.warn("Attempted to get child \"" + name + "\" from parent node, but parent node is null. Returning null...");
			return null;
		}
		
		// Child Node
		Node child = parent.getFirstChild();
		
		// Type value
		String typeVal = null;
		
		// Data value
		String dataVal = null;
		
		// Return value
		Object ret = null;
		
		do
		{
			// if it is an element node, and the name is correct, proceed
			if(child.getNodeType() == Node.ELEMENT_NODE && name.equals(child.getNodeName()))
			{
				// Get the type and break out of the loop
				typeVal = child.getAttributes().getNamedItem("type").getTextContent();
				break;
			}
			
			child = child.getNextSibling();
		}   
		while(child != null);
		
		// proceed only if the type attribute was found on a matching element
		if(typeVal != null)
		{
			// get the data
			dataVal = child.getTextContent();
			
			// Parse it as the proper value
			if(typeVal.equals("DBL") || typeVal.equals("SGL") || typeVal.equals("EXT") && type == BASIC_TYPE.DOUBLE)
			{
				ret = new Double(dataVal);
			}
			
			
			if	(	typeVal.equals("U8") || 
					typeVal.equals("U16") || 
					typeVal.equals("U32") || 
					typeVal.equals("U64") || 
					typeVal.equals("I8") || 
					typeVal.equals("I16") || 
					typeVal.equals("I32") || 
					typeVal.equals("I64") &&
					type == BASIC_TYPE.INT
				)
			{
				ret = new Integer(dataVal);
			}
			
			
			if(typeVal.equals("String") || typeVal.equals("Path") && type == BASIC_TYPE.STRING)
			{
				ret = dataVal;
			}
			
			
			if(typeVal.equals("Bool") && type == BASIC_TYPE.BOOL)
			{
				ret = new Boolean(dataVal);
			}
			

			if(typeVal.equals("Enum U8") || typeVal.equals("Enum U16") || typeVal.equals("Enum U32") && type == BASIC_TYPE.ENUM)
			{
				String sel = child.getAttributes().getNamedItem("sel").getTextContent();
				ret = new EnumPair(sel, new Integer(dataVal));
			}
			
			// if ret is not null, we have successfully parsed a value. Print a according message and return
			if(ret != null)
			{
				logger.info("Successfully parsed child \"" + name + "\" from path \"" + this.getNodePath(parent) + "\" with value of: " +ret);
				return ret;
			}
		}
		
		// if nothing is found, print a warning
		logger.warn("Failed to find child \"" + name + "\" under parent \"" + parent.getBaseURI() + "\"");
		
		// return the default value for the specified type
		if(type == BASIC_TYPE.DOUBLE)
			return new Double(0);
		
		if(type == BASIC_TYPE.BOOL)
			return new Boolean(false);
		
		if(type == BASIC_TYPE.ENUM)
			return new EnumPair("", -1);
		
		if(type == BASIC_TYPE.INT)
			return new Integer(0);
		
		if(type == BASIC_TYPE.STRING)
			return "";
		
		//unreachable statement to satisfy compiler
		return null;
		
	}
	
	
	
	
	
	/**
	 * Parses a PID loop from the document, and returns a PIDData object containing the data
	 * @param path the path to the PID Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the PID data
	 */
	public PIDData parsePID(String path)
	{
		// PIDData element
		PIDData data = new PIDData();
		
		// Expression to find the PID node
		XPathExpression pidExp;
		
		//PID node
		Node pidNode;
		
		// print a message to say that we are parsing the PID
		logger.info("Attempting to parse PID at path \"" + path + "\"");
		
		// Attempt to parse the data. if there is an error, print an error message
		try 
		{
			pidExp = xpath.compile("GXML_Root/" + path);
			pidNode = (Node) pidExp.evaluate(doc, XPathConstants.NODE);
			
			// parse out the elements
			data.kp = (Double)getElementByName("kp", pidNode, BASIC_TYPE.DOUBLE);
			data.ki = (Double)getElementByName("ki", pidNode, BASIC_TYPE.DOUBLE);
			data.kd = (Double)getElementByName("kd", pidNode, BASIC_TYPE.DOUBLE);
			data.kf = (Double)getElementByName("kf", pidNode, BASIC_TYPE.DOUBLE);
			data.period = (Double)getElementByName("period", pidNode, BASIC_TYPE.DOUBLE);
			data.tolerance = (Double)getElementByName("tolerance", pidNode, BASIC_TYPE.DOUBLE);
			data.min = (Double)getElementByName("min", pidNode, BASIC_TYPE.DOUBLE);
			data.max = (Double)getElementByName("max", pidNode, BASIC_TYPE.DOUBLE);
			data.continuous = (Boolean)getElementByName("continuous", pidNode, BASIC_TYPE.BOOL);
			
			// Say that we have finished parsing the PID
			logger.info("Finished parsing PID at path \"" + path + "\"");
		} 
		catch(Exception e){
			logger.error("Failed to parse PID at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
	}
	
	
	
	
	/**
	 * Parses a motor from the document, and returns a MotorData object containing the data
	 * @param path the path to the motor Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the motor data
	 */
	public MotorData parseMotor(String path)
	{
		// MotorData element
		MotorData data = new MotorData();
		
		// Expression to find the motor node
		XPathExpression motorExp;
		
		//motor node
		Node motorNode;
		
		// print a message to say that we are parsing the motor
		logger.info("Attempting to parse motor at path \"" + path + "\"");
		
		// Attempt to parse the motor
		try
		{
			motorExp = xpath.compile("GXML_Root/" + path);
			motorNode = (Node) motorExp.evaluate(doc, XPathConstants.NODE);
	
			
			// parse out the elements
			data.brake = (Boolean)getElementByName("brake", motorNode, BASIC_TYPE.BOOL);
			data.id = (Integer)getElementByName("id", motorNode, BASIC_TYPE.INT);
			data.pdpChannel = (Integer)getElementByName("pdpChannel", motorNode, BASIC_TYPE.INT);
			data.invert = (Boolean)getElementByName("invert", motorNode, BASIC_TYPE.BOOL);
			data.maxOut = (Double)getElementByName("maxOut", motorNode, BASIC_TYPE.DOUBLE);
			data.type = (EnumPair)getElementByName("type", motorNode, BASIC_TYPE.ENUM);
			
			// Say that we have finished parsing the motor
			logger.info("Finished parsing motor at path \"" + path + "\"");
		}
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse motor at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
	}
	
	
	
	
	
	/**
	 * Parses an encoder from the document, and returns an EncoderData object containing the data
	 * @param path the path to the encoder Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the encoder data
	 */
	public EncoderData parseEncoder(String path)
	{
		// MotorData element
		EncoderData data = new EncoderData();
		
		// Expression to find the motor node
		XPathExpression encoderExp;
		
		//motor node
		Node encoderNode;
		
		// print a message to say that we are parsing the encoder
		logger.info("Attempting to parse encoder at path \"" + path + "\"");
		
		// Attempt to parse the encoder
		try
		{
			encoderExp = xpath.compile("GXML_Root/" + path);
			encoderNode = (Node) encoderExp.evaluate(doc, XPathConstants.NODE);
			
			
			// parse out the elements
			data.Achannel = (Integer)getElementByName("Achannel", encoderNode, BASIC_TYPE.INT);
			data.Bchannel = (Integer)getElementByName("Bchannel", encoderNode, BASIC_TYPE.INT);
			data.IDXchannel = (Integer)getElementByName("IDXchannel", encoderNode, BASIC_TYPE.INT);
			data.invert = (Boolean)getElementByName("invert", encoderNode, BASIC_TYPE.BOOL);
			data.distPerCount = (Double)getElementByName("distPerCount", encoderNode, BASIC_TYPE.DOUBLE);
			
			// Say that we have finished parsing the encoder
			logger.info("Finished parsing encoder at path \"" + path + "\"");
		}
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse encoder at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
	}
	
	
	
	
	/**
	 * Parses a Solenoid from the document, and returns a SolenoidData object containing the data
	 * @param path the path to the Solenoid Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the Solenoid data
	 */
	public SolenoidData parseSolenoid(String path)
	{
		// MotorData element
		SolenoidData data = new SolenoidData();
		
		// Expression to find the Solenoid node
		XPathExpression SolenoidExp;
		
		//Solenoid node
		Node SolenoidNode;
		
		// print a message to say that we are parsing the Solenoid
		logger.info("Attempting to parse Solenoid at path \"" + path + "\"");
		
		// Attempt to parse the Solenoid
		try
		{
			SolenoidExp = xpath.compile("GXML_Root/" + path);
			SolenoidNode = (Node) SolenoidExp.evaluate(doc, XPathConstants.NODE);
			
			// parse out the elements
			data.invert = (Boolean)getElementByName("invert", SolenoidNode, BASIC_TYPE.BOOL);
			data.port = (Integer)getElementByName("port", SolenoidNode, BASIC_TYPE.INT);
			data.PCMid = (Integer)getElementByName("PCMid", SolenoidNode, BASIC_TYPE.INT);
			
			// Say that we have finished parsing the Solenoid
			logger.info("Finished parsing Solenoid at path \"" + path + "\"");
		}
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse Solenoid at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
	}
	
	
	
	
	
	/**
	 * Parses a limit switch from the document, and returns a LimitData object containing the data
	 * @param path the path to the limit switch Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the limit data
	 */
	public LimitData parseLimit(String path)
	{
		// MotorData element
		LimitData data = new LimitData();
		
		// Expression to find the motor node
		XPathExpression limitExp;
		
		//motor node
		Node limitNode;
		
		// print a message to say that we are parsing the limit
		logger.info("Attempting to parse limit at path \"" + path + "\"");
		
		// Attempt to parse the limit
		try
		{
			limitExp = xpath.compile("GXML_Root/" + path);
			limitNode = (Node) limitExp.evaluate(doc, XPathConstants.NODE);
			
			// parse out the elements
			data.debounceTime = (Double)getElementByName("debounceTime", limitNode, BASIC_TYPE.DOUBLE);
			data.id= (Integer)getElementByName("id", limitNode, BASIC_TYPE.INT);
			data.interruptEdge = (EnumPair)getElementByName("interruptEdge", limitNode, BASIC_TYPE.ENUM);
			data.normallyOpen = (Boolean)getElementByName("normallyOpen", limitNode, BASIC_TYPE.BOOL);
			
			// Say that we have finished parsing the limit
			logger.info("Finished parsing limit at path \"" + path + "\"");
		}
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse limit at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
	}
	
	
	
	
	/**
	 * Parses a limit switch from the document, and returns a LimitData object containing the data
	 * @param path the path to the limit switch Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the limit data
	 */
	public DriveSideData parseDriveSide(String path)
	{
		// MotorData element
		DriveSideData data = new DriveSideData();
		
		// Expression to find the driveSide node
		XPathExpression driveSideExp;
		
		//motor driveSide. Not used currently, but may be in the future
		@SuppressWarnings("unused")
		Node driveSideNode;
		
		// print a message to say that we are parsing the DriveSide
		logger.info("Attempting to parse DriveSide at path \"" + path + "\"");
		
		
		// Attempt to parse the DriveSide
		try
		{
			driveSideExp = xpath.compile("GXML_Root/" + path);
			driveSideNode = (Node) driveSideExp.evaluate(doc, XPathConstants.NODE);
			
			// Create an expression to find all of the motors
			XPathExpression motorsExp;
			NodeList motors;
			
			motorsExp = xpath.compile("GXML_Root/" + path + "/motors/motor");
			motors = (NodeList)motorsExp.evaluate(doc, XPathConstants.NODESET);
			
			
			// only parse as an array if there is more than 1 element
			if(motors.getLength() > 1)
			{
				// iterate through the motors and add them to the driveSide
				for(int i = 1; i <= motors.getLength(); i++)
					data.motors.add(parseMotor(path + "/motors/motor[" + i + "]"));
			} 
			
			// if there is only one motor, parse it as an element
			else if(motors.getLength() == 1)
				data.motors.add(parseMotor(path + "/motors/motor"));
			
			// If we get here, then something is wrong
			else
				logger.error("Bad number of motors parsed from DriveSide \"" +path+ "\"");
			
			
			
			// populate the data variable
			data.pid = parsePID(path + "/PID");
			data.encoder = parseEncoder(path + "/encoder");
			
			// Say that we have finished parsing the DriveSide
			logger.info("Finished parsing DriveSide at path \"" + path + "\"");
		}
		
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse DriveSide at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
		
	}
	
	
	
	
	
	/**
	 * Parses a Tank Cascade Controller from the document, and returns a TankCascadeData object containing the data
	 * @param path the path to the Tank Cascade Controller Element in the document, without the root "GXML_Root" tag
	 * @return a data element containing the Tank Cascade Controller data
	 */
	public TankCascadeData parseTankCascade(String path)
	{
		// TankCascadeData element
		TankCascadeData data = new TankCascadeData();
		
		// Expression to find the TankCascadeData node
		XPathExpression tankCascadeExp;
		
		// Node for the TankCascadeController
		Node tankCascadeNode;
		
		// print a message to say that we are parsing the TankCascadeController
		logger.info("Attempting to parse TankCascadeController at path \"" + path + "\"");
		
		// Attempt to parse the TankCascadeController
		try
		{
			tankCascadeExp = xpath.compile("GXML_Root/" + path);
			tankCascadeNode = (Node) tankCascadeExp.evaluate(doc, XPathConstants.NODE);		
			
			// parse the individual elements
			data.leftData = parseDriveSide(path + "/leftSide");
			data.rightData = parseDriveSide(path + "/rightSide");
			data.anglePID = parsePID(path + "/anglePID");
			data.distancePID = parsePID(path + "/distancePID");
			data.wheelBaseWidth = (Double)getElementByName("wheelBaseWidth", tankCascadeNode, BASIC_TYPE.DOUBLE);
			
			// Say that we have finished parsing the TankCascadeController
			logger.info("Finished parsing TankCascadeController at path \"" + path + "\"");
		}
		
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse TankCascadeController at path \"" + path + "\". DETAILS: ", e);
		}
		
		// return the parsed values
		return data;
		
	}
	
	
	
	/**
	 * Parses a setpoint from a setpoint list specified by <code>path</code> with the name <code>name</code>
	 * @return the value of the specified setpoint
	 */
	public double parseSetpoint(String path, String name)
	{
		// Expression to get the setpoint
		XPathExpression expr;
		
		// Node that contains the value of a setpoint
		Node setpoint;
		
		// The value of the setpoint
		double ret = 0;
		
		// print a message to say that we are parsing the Setpoint
		logger.info("Attempting to parse Setpoint at path \"" + path + "\"");
		
		// Attempt to parse the setpoint
		try
		{
			// get the proper value node
			// expression: GXML_Root/<path>/Setpoint[name='<name>']/value
			expr = xpath.compile("GXML_Root/" + path + "/Setpoint[name=\'" + name + "\']/value");
			setpoint = (Node)expr.evaluate(doc, XPathConstants.NODE);
			
			// parse out the content of the value node, and return
			ret = new Double(setpoint.getTextContent());
			
			// Say that we have finished parsing the Setpoint
			logger.info("Finished parsing Setpoint at path \"" + path + "\"");
		}
		
		// Print a warning if an error occurs
		catch(Exception e){
			logger.error("Failed to parse Setpoint at path \"" + path + "\". DETAILS: ", e);
		}
		
		// Return the parsed setpoint value
		return ret;
	}
	
	
	
	
	private String getNodePath(Node key)
	{
		String path = "";
		
		// list to contain all nodes in path
		List<Node> nodes = new ArrayList<Node>();
		
		// populate nodes with all nodes in path to key
		while(key.getParentNode() != null) {
			nodes.add(key);
			key = key.getParentNode();
		}
		
		// Remove the GXML_Root element
		nodes.remove(nodes.size() -1);
		
		// produce the path
		for(int i=nodes.size()-1; i>=0; i--)
			path += "/" + nodes.get(i).getNodeName();
		
		return path;
	}
}

















