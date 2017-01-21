package org.usfirst.frc.team223.AdvancedX.utility;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class that allows for a specific command to be called when a state change
 * occurs on a digital input. Useful for avoiding polling loops. When an interrupt
 * fires, a new instance of Command type "handler" is automatically created and run
 * @author Brian Duemmer
 *
 */
public class InterruptableLimit extends DigitalInput {
	
	private boolean normallyOpen;
	
	
	
	
	/**
	 * Provides a simple, easy POD class to pass multiple arguments into an
	 * InterruptableLimit ISR
	 * @author Brian Duemmer
	 *
	 */
	private class InterruptableLimitArg {
		public Runnable handler;
		public DigitalInput input;
		public boolean onRising;
		public boolean onFalling;
		public boolean originalState;
		public double debounceTime;
		
		
		/**
		 * Provides a simple, easy POD class to pass multiple arguments into an
		 * InterruptableLimit ISR
		 * @param handler the {@link Runnable} object that will be called on an interrupt
		 * @param input reference to the digitalInput
		 * @param onRising Whether to fire on the rising edge
		 * @param onFalling Whether to fire on the falling edge
		 * @param originalState The original pin state
		 * @param debounceTime the time (in seconds) that a pin state must hold in order to be counted as an interrupt
		 */
		InterruptableLimitArg(Runnable handler, DigitalInput input, boolean onRising, boolean onFalling, boolean originalState, double debounceTime)
		{
			this.handler = handler;
			this.input = input;
			this.onRising = onRising;
			this.onFalling = onFalling;
			this.originalState = originalState;
			this.debounceTime = debounceTime;
		}
	}
	
	
	
	
	
	
	/**Initializes the InterruptableLimit object.
	 * 
	 * @param handler the {@link Runnable} object that will be called on an interrupt
	 * 
	 * @param index DigitalInput channel that the limit is connected to
	 * 
	 * @param handler Command to be run upon an interrupt. Note that the constructor for
	 * this command cannot have any parameters.
	 *  
	 * @param normallyOpen If true, the limit switch is normally open
	 * 
	 * @param fireOnHit If true, enables interrupts upon the switch being hit
	 * 
	 * @param fireOnRelease If true, enables interrupts upon the switch being released
	 * 
	 * @param debounceTime time (in seconds) to debounce the digital signal coming in
	 */	
	public InterruptableLimit(Runnable handler, int index, boolean normallyOpen, boolean fireOnHit, boolean fireOnRelease, double debounceTime)
	{
		// initialize the input channel
		super(index);
		
		/*
		 * If normally open is false, then the rising edge on the input would
		 * corresponding to the switch being hit and vice versa. If normally
		 * open is true, then the opposite is true: a rising edge would mean
		 * the switch is being released, and so forth. 
		 */
		boolean fireOnRising = normallyOpen ? fireOnRelease : fireOnHit;
		boolean fireOnFalling = normallyOpen ? fireOnHit : fireOnRelease;
		
		// Original pin state
		boolean origState = super.get();
		
		// Create the arg object to pass to the ISR
		InterruptableLimitArg arg = new InterruptableLimitArg(handler, this, fireOnRising, fireOnFalling, origState, debounceTime);
		
		
		/**
		 * Allocates a new instance of the InterrupHandlerFunction class. Overrides
		 * the overridableParameter and interruptFired methods in order to allow
		 * running of a command with type of handlerType.
		 */
		InterruptHandlerFunction<InterruptableLimitArg> ISR = new InterruptHandlerFunction<InterruptableLimitArg>()
		{
			
			// Override this method to give us a parameter to pass to the interruptFired routine
			@Override
			public InterruptableLimitArg overridableParameter(){   return arg;   }
			
			
			// Override this method for the new InterruptHandlerFunction to do what we want
			@Override
			public void interruptFired(int interruptAssertedMask, InterruptableLimitArg param) 
			{
				// Timestamps
				double riseTime = param.input.readRisingTimestamp();
				double fallTime = param.input.readFallingTimestamp();
				double currTime = Timer.getFPGATimestamp();
				
				// Edge type
				boolean isRising;
				
				// Tells us if the edge is correct
				boolean correctEdge;
				
				// Tells if enough time has elapsed
				boolean enoughTime;
				
				
				
				// If both are 0, the no interrupts have occurred yet.
				// Use the original State to tell what edge just occurred
				if(riseTime == 0 && fallTime == 0)
					isRising = !param.originalState;
				
				// If they aren't zero, then we have tracked interrupts.
				// Whichever edge has the newer (bigger) timestamp just occurred
				else {   isRising = fallTime < riseTime;   }
				
				// See if enough time has elapsed since the last interrupt
				enoughTime = currTime - Math.min(riseTime, fallTime) > param.debounceTime;
				
				// See if the edge is correct
				correctEdge = (param.onRising && isRising) || (param.onFalling && !isRising);
				
				
				// If the edge is right and enough time has elapsed, run the handler
				if(enoughTime && correctEdge) {   param.handler.run();   }
			}
		};
				
		// set up the handler
		this.requestInterrupts(ISR);
		
		// We want to catch both edges, in order to debounce properly
		this.setUpSourceEdge(true, true);
	}
	
	
	/**
	 * Returns true if the limit is hit. Accounts for whether or not the limit is
	 * normally open or closed
	 * @return true if the limit is pressed, false otherwise
	 */
	public boolean get()
	{
		return super.get() ^ normallyOpen;
	}
	
	
	/**
	 * {@inheritDoc}. Also frees the interrupt.
	 */
	public void free()
	{
		this.cancelInterrupts();
		super.free();
	}

}


















