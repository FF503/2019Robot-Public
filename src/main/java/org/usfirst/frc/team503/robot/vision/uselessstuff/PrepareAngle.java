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

public class PrepareAngle extends Command {
	private boolean stop;
	// private double kP = 0.02;
	// private double pivotP = 0.025;
	// private double kI = 0.0;
	// private double kD = 0.0;
	private double setpoint = 0;

	private SynchronousPID pid = new SynchronousPID();

	public PrepareAngle(boolean stop) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Drive.getInstance());
		this.stop = stop;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		VisionLocalizer.getInstance().setIndividual();
		double[] pidVals = Robot.bot.getPAPID();
		if (Robot.bot.getPABasePower() == 0) {

			pid.setPID(pidVals[1], pidVals[2], pidVals[3]);

			System.out.println("Prepare to angle is in pivot mode");

		} else {
			pidVals = Robot.bot.getPAPID();
			pid.setPID(pidVals[0], pidVals[2], pidVals[3]);

			System.out.println("Prepare to angle is in curve mode");
		}

		setpoint = Robot.bot.getPACheese() * VisionLocalizer.getInstance().translatedPrepareAngleTarget();

		pid.setSetpoint(setpoint);
		pid.setOutputRange(-0.3, 0.3);
		System.out.println("Robot will attempt to turn to " + setpoint + " degrees relative to the target");
		System.out.println("Initial angle: " + RobotState.getInstance().getTargetData().getAdjustedAngle());
		System.out.println("Begin PrepareAngle loop");

		if (RobotState.getInstance().isVisionDebug()) {
			SmartDashboard.putNumber("PAPivotKP", pidVals[1]);
			SmartDashboard.putNumber("PAPivotKI", pidVals[2]);
			SmartDashboard.putNumber("PAPivotKD", pidVals[3]);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();
		double turnPower = pid.calculate(angle) * Robot.bot.getPAPID()[4];
		double basePower = Robot.bot.getPABasePower();
		double left = basePower - turnPower, right = basePower + turnPower;

		left *= RobotState.getInstance().getCameraDirection().getMultiplier();
		right *= RobotState.getInstance().getCameraDirection().getMultiplier();

		Drive.getInstance().tankDrive(left, right);

		System.out.println("At angle " + angle + ", executed PrepareAngle with left power of " + left
				+ ", right power of " + right);
		// setpoint =
		// Robot.bot.getPACheese()*VisionLocalizer.getInstance().calculateTranslatedPrepareAngleTarget();
		// pid.setSetpoint(setpoint);
		// System.out.println("Setpoint adjusted to: " + setpoint);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return pid.onTarget(Robot.bot.getPATolerance());
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Ended PrepareAngle with a final angle of: "
				+ RobotState.getInstance().getTargetData().getAdjustedAngle());
		if (stop) {
			Drive.getInstance().zeroPowerMotors();
		}
		System.out.println("finished");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Drive.getInstance().zeroPowerMotors();
	}
}
