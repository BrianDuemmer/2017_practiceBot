package org.usfirst.frc.team223.AdvancedX.vision;

import java.io.IOException;
import java.net.Socket;
import java.nio.ByteBuffer;

import net.sf.microlog.core.Logger;

/**
 * Serves as a client to recieve information from a remote vision server
 * @author develoer
 *
 */
public class PiVisionClient implements Runnable
{
	private String serverAddress;
	private int serverPort;
	
	private Socket cnxToServer;
	
	private static final byte PING_CODE = 0;
	private static final byte DATA_CODE = 3;
	
	private Logger log;
	
	private Thread serverThread;
	
	// States wheteher the current connection to the server is valid
	private boolean validCnx = false;
	
	
	/**
	 * Creates an instance of the {@link PiVisionClient}, and starts it automatically
	 * @param address the DNS or IP address of the vision server
	 * @param port the port that the vision server is listening on
	 * @param log the {@link Logger} that information and errors will be logged to
	 */
	public PiVisionClient(String address, int port, Logger log)
	{
		this.serverAddress = address;
		this.serverPort = port;
		this.log = log;
		
		// initialize the server
		log.info("Creating new VisionClient...");
		
		serverThread = new Thread(this);
		serverThread.setDaemon(true);
		
		// start it up
		serverThread.start();
	}



	@Override
	public void run() 
	{
		// loop infinitely
		while(true)
		{
			// attempt to connect to the server
			log.info("Attempting to connect to server...");
			
			try
			{
				cnxToServer = new Socket(serverAddress, serverPort);
				
				validCnx = true;
				
				log.info("Successfully connected to vision server");
				
				
				while(validCnx)
				{
					// Ping the server, and shutdown if it failed
					if(ping() == -1)
					{
						validCnx = false;
						
						//try to shut down gracefully
						cnxToServer.close();
						log.info("Ping to server failed, but shut down connection gracefully");
					}
					
					// Delay for a few seconds
					Thread.sleep(5000);
				}
				
				
			} catch(Exception e)
			{
				log.error("Encountered error in Vision Client:  ", e);
				log.error("Delaying for 10 seconds before restarting...");
				
				 validCnx = false;
				
				try { Thread.sleep(10000);   } 
				catch(InterruptedException e1) { log.error("Exception encountered during sleep  ", e1);}
				
				
			}
		}
	}
	
	
	
	
	/**
	 * Attempts to ping the vision server
	 * @return the time (in milliseconds) that the ping took, or -1 if an error 
	 * occurs
	 */
	public long ping()
	{
		// return a failure if not a valid connection or the connection to the server is null
		if(!validCnx || cnxToServer == null)
			return -1;
		
		long startTime = System.currentTimeMillis();
		
		try {
			cnxToServer.getOutputStream().write(PING_CODE);
			
			// if the read returns -1, then the stream is bad, so return -1
			if(cnxToServer.getInputStream().read() == -1)
				return -1;
			
			else // if it doesn't return 0, then the ping went through successfully, so returhn the time it took
				return System.currentTimeMillis() - startTime;
			
		} catch (IOException e) {
			log.error("unable to ping server,  ", e);
			return -1;
		}
	}
	
	
	
	/**
	 * Queries the vision server for the newest revision of the data from the camera
	 * @return a {@link VisionData} object containing all of the information from the 
	 * camera 
	 */
	public VisionData getDataPacket()
	{
		log.info("Querying Vision server for camera data packet...");
		
		try
		{
			// request a data packet
			cnxToServer.getOutputStream().write(DATA_CODE);
			
			// read the first 4 bytes, which contain the length information
			byte[] lenArr = new byte[4];
			cnxToServer.getInputStream().read(lenArr);
			int datalen = ByteBuffer.wrap(lenArr).getInt();
			
			// create a buffer array to store the data from the server
			byte[] dataArr = new byte[datalen];
			
			// read the data and convert to string
			cnxToServer.getInputStream().read(dataArr);
			String dataPacket = new String(dataArr);
		
			// parse and return
			return new VisionData(dataPacket);
		} catch(Exception e)
		{
			log.error("Exception encountered when trying to request data form vision server:  ", e);
			return new VisionData();
		}
	}
	
	
	
}









