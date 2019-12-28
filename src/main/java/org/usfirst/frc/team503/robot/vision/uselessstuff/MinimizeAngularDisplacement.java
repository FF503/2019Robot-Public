/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class uses radians to measure angles
 */
public class MinimizeAngularDisplacement extends Command {
	private double basePower = Robot.bot.getADBasePower();
	// private double basePower = 0.2; // 0.2
	private double[] outputRange = Robot.bot.getADOutputRange();
	private SynchronousPID pid = new SynchronousPID();

	public MinimizeAngularDisplacement() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("Angular displacement started");

		double[] pidValues = Robot.bot.getADPIDV();
		pid.setPID(pidValues[0], pidValues[1], pidValues[2]);
		pid.setSetpoint(0.0);
		pid.setInputRange(-27.0, 27.0);
		pid.setOutputRange(outputRange[0], outputRange[1]);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		// double[] position =
		// VisionLocalizer.getInstance().translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		// double angleToCenter =
		// VisionLocalizer.getInstance().translatedAngleToTarget();
		// double turnPower = pid.calculate(angleToCenter);

		// double left = basePower - turnPower, right = basePower + turnPower;
		// Drive.getInstance().tankDrive(left, right);
		// System.out.println("At tx of " + angleToCenter + ", left power of " + left +
		// " and right power of " + right);

		double angleToCenter = -RobotState.getInstance().getTargetData().getAdjustedAngle();
		// double angleToCenter =
		// VisionLocalizer.getInstance().translatedAngleToTarget();

		double turnPower = pid.calculate(angleToCenter);

		double left = basePower - turnPower, right = basePower + turnPower;
		Drive.getInstance().tankDrive(left, right);
		System.out.println("At tx of " + angleToCenter + ", left power of " + left + " and right power of " + right);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// return pid.onTarget(tolerance);
		return VisionLocalizer.getInstance().areaOnTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		SmartDashboard.putBoolean("ended", true);
		System.out.println("Ended angular displacer");
		Drive.getInstance().tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}

	public boolean commandFinished() {
		return isCompleted();
	}

	public boolean isRunning() {
		return timeSinceInitialized() > 0 && !isCompleted();
	}
}
