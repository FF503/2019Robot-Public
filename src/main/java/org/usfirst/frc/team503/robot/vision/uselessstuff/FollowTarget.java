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
import org.usfirst.frc.team503.robot.vision.PositionEstimator;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Deprecated
public class FollowTarget extends Command {

	private SynchronousPID pid;
	double basePower = Robot.bot.getFTBasePower();

	public FollowTarget() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		RobotState.getInstance().setTargetData(PositionEstimator.getInstance().guessPosition());
		SmartDashboard.putBoolean("xZero", false);
		SmartDashboard.putBoolean("area on target", false);
		pid = new SynchronousPID();
		double[] pidValues = Robot.bot.getFTPIDV();
		double[] outputRange = Robot.bot.getFTOutputRange();

		pid.setPID(pidValues[0], pidValues[1], pidValues[2]);
		pid.setOutputRange(outputRange[0], outputRange[1]);

		double setpoint = VisionLocalizer.getInstance()
				.translatedAngleToCenter(VisionLocalizer.getInstance().getPosition());
		pid.setSetpoint(setpoint);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// double angle = VisionLocalizer.getInstance().translatedAngleToCenter();
		double[] estimate = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();
		double setpoint = VisionLocalizer.getInstance()
				.translatedAngleToCenter(VisionLocalizer.getInstance().getPosition());

		// Basepower calculations
		double curDistance = estimate[0];
		// double power = pidControlller.calculate(curDistance);
		double error = VisionLocalizer.getInstance().getError(0.0, curDistance);
		System.out.println("Horizontal Error: " + error);
		SmartDashboard.putNumber("Horizontal Error", error);

		double power;
		if (Math.abs(error) > 10) {
			power = basePower;
		} else {
			power = 0.05 * Math.abs(error);
		}

		System.out.println("Base power: " + power);

		pid.setSetpoint(setpoint);
		System.out.println("setpoint: " + setpoint);
		SmartDashboard.putNumber("setpoint", setpoint);
		System.out.println("angle: " + angle);
		SmartDashboard.putNumber("Angle", angle);
		double turn = pid.calculate(angle);

		turn *= 2;
		if (Robot.bot.getName().equals("ProgrammingBot")) {
			turn *= -1;
		}

		if (RobotState.getInstance().getTargetData().getTargetFieldAngle() < 0) {
			SmartDashboard.putBoolean("Field angle less than 0", true);
			turn *= -1;
		}

		basePower = power;
		turn *= Math.abs(basePower);

		double left = basePower + turn, right = basePower - turn;
		System.out.println("Left: " + left + ", Right: " + right);
		Drive.getInstance().tankDrive(left, right);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		double[] positionEstimate = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		boolean areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();
		// boolean xZero =
		// pid.onTarget(Robot.bot.getFTTolerance());//(Math.abs(positionEstimate[0] -
		// Math.signum(positionEstimate[0]) * 0.5) < Robot.bot
		boolean xZero = Math.abs(positionEstimate[0]) < Robot.bot.getFTTolerance();

		// .getFTTolerance());

		double lidarDistance = RobotState.getInstance().getLidarDistance(0);

		if (xZero) {
			System.out.println("xZero");
			SmartDashboard.putBoolean("xZero", true);
		}
		if (areaOnTarget) {
			System.out.println("Area on target");
			// SmartDashboard.putBoolean("area on target", true);
		}
		// if (RobotState.getInstance().getState().equals(RobotState.State.TELEOP)) {
		// System.out.println("In teleop");
		// SmartDashboard.putBoolean("Target button", OI.getTargetButton());

		// if (RobotState.getInstance().getIsManual()) {
		// return true;
		// }
		// }

		return xZero || areaOnTarget || (Math.abs(lidarDistance) < 12);
		// return (Math.abs(positionEstimate[0]) < 1);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Drive.getInstance().zeroPowerMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
