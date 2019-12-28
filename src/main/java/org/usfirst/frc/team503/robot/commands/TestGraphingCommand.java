package org.usfirst.frc.team503.robot.commands;


import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.UniversalGrapher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestGraphingCommand extends Command {
	public static double x = 1.0; 
	public static double y = 2.0;
	DataEntry xValue = new DataEntry("Time");
	DataEntry yValue = new DataEntry("Position");
	
	public double initialTime;
	public TestGraphingCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {	
		initialTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		xValue.addValue(Timer.getFPGATimestamp() - initialTime);
		yValue.addValue(y);
	
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (Timer.getFPGATimestamp() - initialTime > 3) {
			return true;
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		// System.out.println("program ended");
		UniversalGrapher.addEntries(xValue);
		UniversalGrapher.addEntries(yValue);
		UniversalGrapher.sendValues();
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}