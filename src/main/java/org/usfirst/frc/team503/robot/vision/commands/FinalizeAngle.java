/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.State;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FinalizeAngle extends Command {
	private boolean stop;
	private double setpoint = 0;
	private boolean isFinished = false;
	// double[] position;

	private SynchronousPID pid = new SynchronousPID();

	public FinalizeAngle(boolean stop) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.stop = stop;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// double forwardFA_kP = 0.015;
		double forwardFA_pivotkP = 0.01;
		double forwardFA_kI = 0.0;
		double forwardFA_kD = 0;
		// VisionLocalizer.getInstance().setIndividual();
		// position = VisionLocalizer.getInstance()
		// .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		// RobotState.getInstance().setVisionStartPosition(position);
		// double[] pidVals = Robot.bot.getFAPID();

		pid.setPID(forwardFA_pivotkP, forwardFA_kI, forwardFA_kD);
		pid.setOutputRange(-0.4, 0.4);

		// setpoint = 45 * Math.signum(position[0]);

		pid.setSetpoint(0);
		// System.out.println("Position: " + position[0] + " " + position[1]);
		System.out.println("Robot will attempt to turn to " + setpoint + " degrees relative to the target");

		System.out.println("Initial angle: " + RobotState.getInstance().getTargetData().getAdjustedAngle());
		System.out.println("Begin PrepareAngle loop");

		// if (RobotState.getInstance().isVisionDebug()) {
		// SmartDashboard.putNumber("FAPivotKP", pidVals[1]);
		// SmartDashboard.putNumber("FAPivotKI", pidVals[2]);
		// SmartDashboard.putNumber("FAPivotKD", pidVals[3]);
		// }

		do {
			double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();
			SmartDashboard.putNumber("Adjusted angle", angle);
			double turnPower = pid.calculate(angle);

			turnPower *= -1;

			if (RobotState.getInstance().getTargetData().getTargetFieldAngle() < 0) {
				turnPower *= -1;
			}

			double left = -turnPower, right = turnPower;
			double multiplier = RobotState.getInstance().getCameraDirection().getMultiplier();
			left *= multiplier;
			right *= multiplier;

			// left *= Math.signum(RobotState.getInstance().getVisionStartPosition()[0]);
			// right*= Math.signum(RobotState.getInstance().getVisionStartPosition()[0]);

			Drive.getInstance().tankDrive(-left, -right);

			System.out.println("At angle " + angle + " and raw angle of " + Gyro.getInstance().getAngle()
					+ ", executed PrepareAngle with left power of " + left + ", right power of " + right);
			System.out.println("GyroPID Error " + pid.getError());
			System.out.println("GyroPID Setpoint: " + pid.getSetpoint());
			// setpoint =
			// Robot.bot.getFACheese()*VisionLocalizer.getInstance().calculateTranslatedPrepareAngleTarget();
			// pid.setSetpoint(setpoint);
			// System.out.println("Setpoint adjusted to: " + setpoint);

			System.out.println("State: " + RobotState.getInstance().getState().toString());
			isFinished = RobotState.getInstance().getState().equals(State.DISABLED)
					|| pid.onTarget(Robot.bot.getFATolerance()); // Math.abs(angle - pid.getSetpoint()) <
																	// Robot.bot.getFATolerance();

		} while (!isFinished);

		isFinished = true;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// return pid.onTarget(Robot.bot.getFATolerance());
		return isFinished;
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
