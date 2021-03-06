package org.usfirst.frc.team223.AdvancedX;


import org.usfirst.frc.team223.AdvancedX.robotParser.Destroyer;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLAllocator;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser;
import org.usfirst.frc.team223.AdvancedX.robotParser.GXMLparser.BASIC_TYPE;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import net.sf.microlog.core.Logger;

/**
 * Manages the connection between the robot using the GXML configuration
 * framework and a LabVIEW ConigXML plugin bound to this device, as well
 * as all of the logging controls. Instances
 * of this class will automatically create a daemon thread that runs in 
 * the background, which will automatically communicate and handshake
 * with the dashboard.
 * <P/>
 * 
 * An instance of this class must be created at the beginning of 
 * robot code. In addition, all instances of {@link GXMLparser} and
 * {@link GXMLAllocator} should be obtained using {@link #obtainParser}
 * and {@link #obtainAllocator}.
 * <P/>
 * 
 * Be sure to call {@link start()} in your initialization code!
 * 
 * @author Brian Duemmer
 *
 */
public abstract class AdvancedXManager implements Runnable
{
	// Active parser object, bound to the document
	private GXMLparser parserRef;

	// Active Allocator object
	private GXMLAllocator allocator;

	private Destroyer destroyer;

	// name of the NetworkTables key name that will be used for handshaking
	private String handshakeKey;

	// name of key that indicates a successful reload
	private String successKey;

	// name of the key that reports the filename back to the dashboard
	private String fileNameKey;

	// the object that this class will log information to
	private Logger logger;
	private RoboLogManagerBase roboLogManagerBase;

	// the table that will be used for the communication
	private NetworkTable nt;

	// Boolean that is true only on the first run of the ScheduledExecutor, and false
	// all other times
	private boolean firstRun = true;

	// target filename
	private String fileName;

	// target namespace
	private String namespace = "";

	// thread that runs the manager check
	private Thread manager;

	// ms between reload checks
	private int updateRate;






	/**
	 * Create a new instance of {@link AdvancedXManager} that is bound to the configuration
	 * file at <code>filePath</code>
	 * 
	 * @param fileName the path of the GXML config file from the dashboard
	 * @param roboLogManagerBase a {@link RoboLogManagerBase} that can be used by the {@link AdvancedXManager}
	 * @param nt the {@link NetworkTables} that can communicate with the dashboard. This is necessary for 
	 * handshaking between the robot and the dashboard
	 */
	public AdvancedXManager(String fileName, RoboLogManagerBase roboLogManagerBase)
	{
		// update instance variables
		this.fileName = fileName;

		// get the namespace
		this.namespace = getNamespace(fileName);

		// initialize the logger
		this.roboLogManagerBase = roboLogManagerBase;
		this.logger = roboLogManagerBase.getLogger("AdvancedXManager_" + namespace);
		this.logger.info("Starting AdvancedXManager...");

		// construct the keys
		this.handshakeKey = "CONFIGXML_" + this.namespace + "_RELOAD";
		this.fileNameKey = "CONFIGXML_" + this.namespace + "_FILENAME";
		this.successKey = "CONFIGXML_" + this.namespace + "_SUCCESS";

		this.destroyer = new Destroyer(this.roboLogManagerBase.getLogger("DESTROYER"));
	}
	
	
	
	/**
	 * Sets up the Network Tables server
	 */
	public NetworkTable initNT()
	{
		// attempt to initialize NetworkTables
		logger.info("Attempting to initialize NetworkTables...");
		try
		{
			NetworkTable.setServerMode();
			NetworkTable.setPort(1735);
			NetworkTable.initialize();
			nt = NetworkTable.getTable("SmartDashboard");
			logger.info("Successfully initialized NetworkTables");
		} 
		catch(Exception e){
			logger.fatal("Failed to initialize networkTables! DETAILS: ", e);
		}
		
		return nt;
	}






	/**
	 * Obtains the namespace of the configuration setup based on the path.
	 * This is the name of the file, without an extension.
	 * 
	 * @param filePath the path to the configuration file
	 * @return the namespace 
	 */
	protected String getNamespace(String filePath)
	{
		String namespace;

		// get the index of the last "/" and last "."
		int startIdx = filePath.lastIndexOf("/")+1;
		int endIdx = filePath.lastIndexOf(".");

		// get the filename based on that information
		if(startIdx > 0 && endIdx > 0)
			namespace = filePath.substring(startIdx, endIdx);
		else if(startIdx > 0)
			namespace = filePath.substring(startIdx);
		else
			namespace = filePath;

		return namespace;
	}






	/**
	 * Obtains the current instance of the {@link GXMLparser}. This will automatically
	 * attempt to allocate the document if it has not been already.
	 */
	public GXMLparser obtainParser()
	{
		// if the parserRef is null, allocate a new instance of it. If it is not,
		// use the existing parserRef
		if(this.parserRef == null)
			try
		{
				this.parserRef = new GXMLparser(fileName, roboLogManagerBase);
		} catch (Exception e)
		{
			logger.error("Error allocating parser! ", e);
		}
		
		logger.info("Parser is: " + parserRef);

		return this.parserRef;
	}




	/**
	 * Obtains the active instance of the {@link GXMLAllocator}, or creates one 
	 * if there isn't an active instance.
	 */
	public GXMLAllocator obtainAllocator()
	{
		if(this.allocator == null)
			this.allocator = new GXMLAllocator(this);
		return this.allocator;
	}




	/**
	 * Starts the {@link AdvancedXManager}. This will run {@link #reload()} all config data initially,
	 * and will subsequently run {@link #reload()} whenever the boolean NT entry specified for
	 * handshaking is set to true.
	 * @param updateRate The period, in ms, that the handshake key is checked for updates
	 */
	public void start(int updateRate)
	{
		this.logger.info("Starting AdvancedXManager Background thread with update rate of " + updateRate + " ms");

		// create a thread to run this
		this.manager = new Thread(this);

		// setup the thread
		this.manager.setDaemon(true);
		this.manager.setName(this.namespace + "_MANAGER_THREAD");
		this.updateRate = updateRate;

		// start it
		this.manager.start();
	}






	/**
	 * periodically check if the Dashboard has requested
	 * a config reload. If it did, call the {@link #reload()} method.
	 * Updates NT keys as necessary
	 */
	@Override
	public void run()
	{	
		// loop infinitely for the duration of robot code
		while(true)
		{
			boolean success = false;

			// see if the Dashboard has requested an update, or this is the first call. Reload if it did.
			if(nt.getBoolean(handshakeKey, false) || this.firstRun)
			{
				// Prevent access to the scheduler class everywhere else while we do this
				synchronized(Scheduler.class)
				{
					// kill any running commands
					Scheduler.getInstance().removeAll();

					// if not the first run, an update from the dashboard was requested
					if(!this.firstRun)
						logger.info("Reload request recieved from Dashboard. Attempting Startup cycle...");
					else
						logger.info("Attempting initial startup cycle...");

					// only free() if not a first call, as the data wasn't loaded yet
					if(!this.firstRun)
					{
						logger.info("Attempting to free data...");
						logger.info("\r\n\r\n\r\n\r\n======================================================="
								+"\r\n================= Shutting Down Robot ================="
								+"\r\n=======================================================");
						try 
						{
							// Reset the destroyer
							this.destroyer.clearDestroyed();

							success = free();
							logger.info("Shutdown cycle finished with success status: " + success);
						} catch (Throwable e) {
							logger.error("Error during shutdown cycle. DETAILS:", e);
							success = false;
						}
					} else 
					{
						logger.info("This is the first call, so free() was not called");

						// set to true so the following "success &= load()" will still work
						success = true;
					}

					// load() all of the data
					try
					{
						logger.info("\r\n\r\n\r\n\r\n======================================================="
								+"\r\n================= Initializing Robot =================="
								+"\r\n=======================================================");

						// Clear the parser so it is forced to re-open the file
						this.parserRef = null;

						// both load() and free() must be successful in order for the cycle to be considered successful
						success &= load();
					} catch(Throwable e) {
						logger.error("Exception encountered during load()! DETAILS: ", e);
						success = false;
					}

					// Set the success flag to the return value from reload()
					nt.putBoolean(successKey, success);
					logger.info("Reload cycle finished with success status: " +success);
				}

				// Set the handshake flag to false
				nt.putBoolean(handshakeKey, false);

				// update the filename key to the value of the FILE_NAME attribute of the document.
				nt.putString(fileNameKey, (String)this.obtainParser().getKeyByPath("FILE_NAME", BASIC_TYPE.STRING));

				// make sure firstRun is false
				this.firstRun = false;
			}

			// delay for the necessary time, converting from ms to seconds
			try {
				Thread.sleep(updateRate);
			} catch (InterruptedException e) {
				logger.info("Error in delay:   ", e);
			}



			//			logger.info("HandshakeKey: " + nt.getBoolean(handshakeKey, false));
			//			logger.info("ContainsReload: " + nt.containsKey(handshakeKey));
			//			logger.info(nt.getString("myString", "not found"));
		}
	}





	/**
	 * This method must be defined by the user. It specifies what to do in order
	 * to load all of the robot's data freshly from the configuration file.
	 * This will be called automatically by the {@link AdvancedXManager}.
	 * 
	 * @return true if the data was loaded successfully
	 */
	public abstract boolean load();




	/**
	 * This method must be defined by the user. It specifies what to do in order
	 * to free all of the robot's data freshly from the configuration file.
	 * This will be called automatically by the {@link AdvancedXManager}.
	 * 
	 * @return true if the data was freed successfully
	 */
	public abstract boolean free();






	public RoboLogManagerBase getRoboLogger() {
		return roboLogManagerBase;
	}







	public NetworkTable getNt() {
		return nt;
	}




	/**
	 * @See {@link Destroyer}
	 */
	public boolean destroy(Object obj)
	{
		return this.destroyer.destroy(obj);
	}



	public void setNt(NetworkTable nt) {
		this.nt = nt;
	}

}
























