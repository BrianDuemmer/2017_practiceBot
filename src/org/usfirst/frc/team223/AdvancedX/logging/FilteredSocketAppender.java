package org.usfirst.frc.team223.AdvancedX.logging;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import net.sf.microlog.core.Appender;
import net.sf.microlog.core.Formatter;
import net.sf.microlog.core.Level;



/**
 * Implements an appender that will log its data to a client via a TCP
 * socket
 * @author Brian Duemmer
 *
 */
public class FilteredSocketAppender implements Appender, Runnable
{
	// Thread that the server will run in
	private Thread serverThread;
	
	// Server reference
	ServerSocket server;
	
	// List of ClientHandlers to handle the client connection(s)
	private List<ClientHandler> clients = new ArrayList<ClientHandler>();	
	
	// Port that the server will run on
	private int port;
	
	// Formatter that contains the formatting for the appender
	private Formatter formatter;
	
	// Min logging level that will be reported
	private Level minLevel;
	
	// Message that will be printed
	private String message;
	
	
	
	
	
	/**
	 * Member class that sends output to a single client
	 * @author Brian Duemmer
	 *
	 */
	private class ClientHandler extends Thread
	{
		private Socket socket;
		
		@SuppressWarnings("unused")
		private boolean stop;
		
		
		public ClientHandler(Socket socket)
		{
			super();
			this.socket = socket;
		}
		

		@Override
		public void run() 
		{
			PrintWriter writer = null;
			
			// Attempt to set the PwintWriter to the Socket's OutputStream
			try {   writer = new PrintWriter(this.socket.getOutputStream());   }
			catch (Exception e) 
			{   
				System.out.println("Error attaching client to server. DETAILS: " + e.getMessage());          
				return;
			}
			
			// enter an infinite loop to handle messages
			while(true)
			{
				
				// Run everything inside of a synchronized block to allow for the use of wait() and notify()
				synchronized(this)
				{
					try 
					{
						// wait until we are called upon to print messages
						this.wait();
						
						// Print the message to the socket
						writer.println(message);
						writer.flush();
					} 
					catch (Exception e)  
					{   
						System.out.println("Error writing log in client thread " + e.getMessage());   
						return;
					}
					
				}
				
			}
			
		}
		
		
		
		/**
		 * Stops the thread and disconnects from the client
		 */
		public void stopThread()
		{
			this.stop = true;
			this.interrupt();
			try {
				this.socket.close();
			} catch (IOException e) {
				System.out.println("Error shutting down socket " + e.getMessage());
			}
		}
	}
	
	

	
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// Main Class ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	
	
	/**
	 * Creates a new {@link FilteredSocketAppender}
	 * @param port the network port that the server will use
	 */
	public FilteredSocketAppender(int port)
	{
		this.port = port;
	}
	
	
	
	
	/**
	 * Start the server.
	 */
	public void open() 
	{
		if (serverThread == null) {
			serverThread = new Thread(this);
		}

		if (!serverThread.isAlive()) {
			serverThread.start();
		}
	}
	
	
	
	

	@Override
	public void run()
	{
		
		// Attempt to open the server
		try
		{
			System.out.println("Creating new Server...");
			this.server = new ServerSocket(this.port);
			System.out.println("Successfully created new Server...");
		} catch(Exception e)
		{
			System.err.println("Failed to create Server. DETAILS: " + e.getMessage());
			return;
		}
		
		// Accept incoming connections
		while(this.server != null)
		{
			// attempt to accept connections
			try 
			{  
				Socket clientSoc = this.server.accept(); 
				
				// Create a new thread to listen to the connection
				ClientHandler clientHdl = new ClientHandler(clientSoc);
				
				// start it and add it to the client array
				clientHdl.start();
				this.clients.add(clientHdl);
				
			} 
			catch (IOException e) {   System.out.println("Error accepting client connection: " + e.getMessage());   }
		

			
		}
	}
	

	
	
	
	
	
	@Override
	public void doLog(String clientID, String name, long time, Level level, Object message, Throwable t) 
	{
		// make sure the formatter was set
		if(this.formatter == null)
		{
			System.out.println("formatter was not set");
			return;
		}
		
		
		// make sure the level is correct
		if(this.minLevel == null || this.minLevel.toInt() <= level.toInt())
		{
			// format the message
			this.message = formatter.format(clientID, name, time, level, message, t);
			
			// tell each of the client threads to send the message to their client
			for(ClientHandler i : this.clients)
			{
				// if the socket is closed, shut down the thread and remove it from the system
				if(!i.isAlive()) 
				{   
					this.clients.remove(i);   
					i.stopThread();
				}
				
				// If not, proceed as normal
				else
				{
					// notify each inside of the synchronize block. this will cause each to print this.message to their client
					synchronized(i) {   i.notifyAll();   }
				}
			}
			
		}
	}

	
	
	
	
	@Override
	public void clear() {}

	
	
	
	@Override
	public void close() throws IOException 
	{
		// close all of the clients
		for(ClientHandler i : this.clients)
			i.stopThread();
		
		// close the server
		this.server.close();
	}


	

	/**
	 * Tells if the appender is ready to log
	 * @return true if there is at least client connected 
	 */
	@Override
	public boolean isLogOpen() 
	{
		return true;
	}

	
	
	
	
	
	@Override
	public long getLogSize() {   return Appender.SIZE_UNDEFINED;   }

	@Override
	public void setFormatter(Formatter formatter) {   this.formatter = formatter;   }

	@Override
	public Formatter getFormatter() {   return null; }

	@Override
	public String[] getPropertyNames() {   return null;   }

	@Override
	public void setProperty(String name, String value) throws IllegalArgumentException {}

	public void setMinLevel(Level minLevel) {   this.minLevel = minLevel;   }


}
