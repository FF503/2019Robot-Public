/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.CameraDirection;
import org.usfirst.frc.team503.robot.RobotState.State;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowTarget extends Command {

	// private SynchronousPID pid;
	// double basePower = Robot.bot.getFTBasePower(), basePowerP =
	// Robot.bot.getFTBasePowerP();
	double[] pidValues = Robot.bot.getFTPIDV(), outputRange = Robot.bot.getFTOutputRange();
	double totalDistance;
	double setpoint;
	double validSetpoint;

	double distance, lidarDistance;

	double startDistance = 0;

	double startLeft, startRight;

	boolean isReversed;

	public FollowTarget() {

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("*****************FollowTarget Starting***************************");
		RobotState.getInstance().setVisionFollowerRunning(true);
		double[] estimate = VisionLocalizer.getInstance().getPosition();
		SmartDashboard.putBoolean("xZero", false);
		SmartDashboard.putBoolean("area on target", false);

		isReversed = RobotState.getInstance().getCameraDirection().equals(CameraDirection.BACK);
		FFDashboard.getInstance().putBoolean("IS VISION REVERSED", isReversed);
		System.out.println("Camera reversed? " + isReversed);
		// pid = new SynchronousPID();

		// pid.setPID(pidValues[0], pidValues[1], pidValues[2]);
		// pid.setOutputRange(outputRange[0], outputRange[1]);

		setpoint = VisionLocalizer.getInstance().translatedAngleToCenter(VisionLocalizer.getInstance().getPosition());

		// setpoint = RobotState.getInstance().getTargetAngles()[1];

		// pid.setSetpoint(setpoint);
		System.out.println("FollowTarg: Initial Setpoint:" + (setpoint));
		totalDistance = Math.hypot(estimate[0], estimate[1]);
		System.out.println("Total distance: " + totalDistance);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// double angle = VisionLocalizer.getInstance().translatedAngleToCenter();
		double[] estimate = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();

		System.out.println("x: " + estimate[0] + ", y: " + estimate[1]);
		double setpoint = VisionLocalizer.getInstance()
				.translatedAngleToCenter(VisionLocalizer.getInstance().getPosition());

		distance = Math.hypot(estimate[0], estimate[1]);
		// Basepower calculations
		double curDistance = estimate[0];
		// double power = pidControlller.calculate(curDistance);
		double error = VisionLocalizer.getInstance().getError(0.0, curDistance);
		// System.out.println("Horizontal Error: " + error);
		SmartDashboard.putNumber("Horizontal Error", error);

		if (!RobotState.getInstance().lostTarget()) {
			System.out.println("Did not lose target");
			setpoint = VisionLocalizer.getInstance()
					.translatedAngleToCenter(VisionLocalizer.getInstance().getPosition());
			// setpoint = RobotState.getInstance().getTargetAngles()[1];

			validSetpoint = setpoint;
		} else {
			System.out.println("Lost target");
			setpoint = validSetpoint;
			if (startDistance == 0) {
				System.out.println("<<<<<<<<<<<<<<<<<<<< Lost target >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
				startDistance = distance;
				startLeft = Drive.getInstance().getLeftDistanceInches();
				startRight = Drive.getInstance().getRightDistanceInches();

			}

		}

		// System.out.println("Base power: " + power);

		// pid.setSetpoint(setpoint);
		System.out.println("Angle setpoint: " + setpoint);

		// SmartDashboard.putNumber("setpoint", setpoint);
		// System.out.println("angle: " + angle);
		SmartDashboard.putNumber("Angle", angle);
		double turn = (setpoint - angle) * 0.025;

		// Was working on prog bot
		// turn *= 2;
		// turn *= pidValues[3];

		if (RobotState.getInstance().getState().equals(State.TELEOP)) {
			turn *= -1;
		}

		// if(Robot.bot.getName().equals("PracticeBot")) {
		// turn *= -1;
		// }

		// if (RobotState.getInstance().getTargetData().getTargetFieldAngle() < 0) {
		// SmartDashboard.putBoolean("Field angle less than 0", true);
		// turn *= -1;
		// }

		// TODO: Comment this out later

		double basePower = distance / totalDistance * 0.6;
		// double turnSignum = Math.signum(turn);
		// turn = turnSignum*Math.pow(turn, 2);
		// turn *= Math.abs(basePower);

		// System.out.println("turn: " + turn);
		// System.out.println("basePower: " + basePower);

		// turn = 0;

		// turn *= -Math.signum(angle);

		// turn = Math.pow(turn, 2);
		double left = basePower - turn, right = basePower + turn;
		if (!isReversed) {
			System.out.println("Tank Drive Forward: " + left + ", Right: " + right);
			Drive.getInstance().tankDrive(left, right);
		} else {
			System.out.println("Tank Drive Reverse: " + left + ", Right: " + right);
			Drive.getInstance().tankDrive(-right, -left);
		}

		// Log what is happening...
		// System.out.println("FollowTarg: Herr:" + df.format(error) + " ang SP:" +
		// df.format(setpoint) + " angle: "
		// + df.format(angle) + " turn: " + df.format(turn) + " left: " +
		// df.format(left) + " right: "
		// + df.format(right) + " distance: " + df.format(distance) + " base power: " +
		// df.format(basePower));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// double[] positionEstimate = VisionLocalizer.getInstance()
		// .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		boolean areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();
		// boolean xZero = pid.onTarget(0.5);// (Math.abs(positionEstimate[0] -
		// Math.signum(positionEstimate[0]) * 0.5) <
		// Robot.bot
		// boolean xZero = Math.abs(positionEstimate[0]) < Robot.bot.getFTTolerance();

		// .getFTTolerance());

		// boolean xZero = pid.onTarget(Robot.bot.getFTTolerance());

		int lidarAngle = 270;
		if (isReversed) {
			lidarAngle = 90;
		}
		if (Robot.bot.hasLidar()) {
			lidarDistance = RobotState.getInstance().getLidarDistance(lidarAngle);
			System.out.println("Lidar distance: " + lidarDistance);
		}
		// if (xZero) {
		// System.out.println("xZero");
		// SmartDashboard.putBoolean("xZero", true);
		// }
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
		// double distance = (Drive.getInstance().getLeftDistanceInches() +
		// Drive.getInstance().getRightDistanceInches())/2;
		// boolean distanceStop = (distance > startDistance) && (startDistance != 0);
		double pctremaining = distance / totalDistance;
		System.out.println("Distance Completed: " + pctremaining);
		if (Robot.bot.hasLidar()) {
			return (lidarDistance < 40);
		} else {
			// System.out.println("IsFinshed?" + (distance < stopdist));
			return (pctremaining < .05);
		}
		// return (Math.abs(positionEstimate[0]) < 1);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Follow target ended. Stop motors...");
		Drive.getInstance().zeroPowerMotors();
		RobotState.getInstance().setVisionFollowerRunning(false);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
