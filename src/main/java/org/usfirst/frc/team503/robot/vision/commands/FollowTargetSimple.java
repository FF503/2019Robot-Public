/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.CameraDirection;
import org.usfirst.frc.team503.robot.motionProfiling.PathFollowerPure;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Deprecated
public class FollowTargetSimple extends Command {

	// private SynchronousPID pid;
	// double basePower = Robot.bot.getFTBasePower(), basePowerP =
	// Robot.bot.getFTBasePowerP();

	double totalDistance;
	double setpoint;
	double validSetpoint;

	double distance, lidarDistance;

	double startTime;
	double tx;
	double[] previousTx = new double[10];
	int index = 0;
	double kp = 1.0, kd = 0.0;
	double autonTolerance;
	boolean isReversed;
	boolean valueInitialized = false;
	double kDriveP = 0.4;
	double initialCamera, initialGyro, desiredGyro;
	ProgramState state = ProgramState.FOLLOWING;
	double loadingStartTime;
	double backUpStartTime;
	double pivotStartTime;

	public static enum ProgramState {
		FOLLOWING, PIVOT, LOADING, BACKUP, FINISHED;
	}

	public FollowTargetSimple() {
		RobotState.getInstance().setAutonToleranceAdjustment(0.0);
	}

	public FollowTargetSimple(double autonToleranceAdjustment) {
		RobotState.getInstance().setAutonToleranceAdjustment(autonToleranceAdjustment);
	}

	public FollowTargetSimple(double autonToleranceAdjustment, double kp) {
		this.kp = kp;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("*****************FollowTarget Starting***************************");
		VisionLocalizer.getInstance().setPipeline(2);
		RobotState.getInstance().setVisionFollowerRunning(true);
		SmartDashboard.putBoolean("xZero", false);
		SmartDashboard.putBoolean("area on target", false);
		isReversed = RobotState.getInstance().getCameraDirection().equals(CameraDirection.BACK);
		if (isReversed) {
			kp = 0.01;
		} else {
			kp = 0.01;
		}
		FFDashboard.getInstance().putBoolean("IS VISION REVERSED", isReversed);
		System.out.println("Camera reversed? " + isReversed);
		setpoint = VisionLocalizer.getInstance().getTX();
		VisionLocalizer.getInstance().setIndividual();
		System.out.println("FollowTarg: Initial Setpoint:" + (setpoint));

		System.out.println("Total distance: " + totalDistance);
		startTime = Timer.getFPGATimestamp();
		state = ProgramState.FOLLOWING;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		VisionLocalizer.getInstance().setPipeline(2);
		switch (state) {
		case FOLLOWING:
			double basePower = 0, turn = 0;
			tx = VisionLocalizer.getInstance().getTX();
			if (index == 10) {
				index = 0;
			}
			previousTx[index] = tx;
			index++;
			double setpoint = tx;
			if (isReversed) {
				// setpoint += 11;
				basePower = kDriveP / VisionLocalizer.getInstance().getTA();
			} else {
				basePower = kDriveP / VisionLocalizer.getInstance().getTA();
				// setpoint -= 18;
			}
			basePower = Math.min(0.8, basePower);
			basePower = Math.max(0.15, basePower);
			System.out.println("tx" + tx);
			System.out.println("Angle setpoint: " + setpoint);
			// System.out.println("LOST TARGET: " +
			// VisionLocalizer.getInstance().lostTarget());
			// if (VisionLocalizer.getInstance().lostTarget() && !valueInitialized) {
			// System.out.println("Lost target, flag set");
			// initialCamera = setpoint;
			// initialGyro = Gyro.getInstance().getAngle();
			// desiredGyro = initialGyro + setpoint;
			// valueInitialized = true;
			// }
			if (valueInitialized) { // if the target's lost
				// basePower and turn need to be set
				double remainingTurn = desiredGyro - Gyro.getInstance().getAngle();// initialCamera -
																					// Gyro.getInstance().getAngle() -
																					// initialGyro;
				turn = remainingTurn * kp;
				// turn = setpoint * kp;
				// System.out.println("lost target turn calculation");
			} else {
				turn = setpoint * kp;
				// System.out.println("normal turn calculation");
			}
			if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
				turn *= -1;
			}
			// System.out.println("Turn: " + turn);

			// System.out.println("running simple");
			double left = basePower + turn, right = basePower - turn;
			if (!isReversed) {
				System.out.println("Tank Drive Forward: " + left + ", Right: " + right);
				Drive.getInstance().tankDrive(left, right);
			} else {
				System.out.println("Tank Drive Reverse: " + left + ", Right: " + right);
				Drive.getInstance().tankDrive(-right, -left);
			}
			if (followerEnd()) {
				// System.out.println("finished");
				state = ProgramState.PIVOT;
				pivotStartTime = Timer.getFPGATimestamp();
				// loadingStartTime = Timer.getFPGATimestamp();
			}
			break;

		case PIVOT:
			System.out.println("check:" + PathFollowerPure.boundHalfDegrees(Gyro.getInstance().getTranslatedAngle()));
			System.out.println("compare 1: " + boundHalfDegrees(-80));
			System.out.println("compare 2: " + boundHalfDegrees(-100));
			double power = -0.2;
			if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
				power *= -1;
			}
			if (PathFollowerPure.boundHalfDegrees(Gyro.getInstance().getTranslatedAngle()) >= boundHalfDegrees(-80)) {
				System.out.println("if 1");
				Drive.getInstance().tankDrive(-power, 0);
			} else if (PathFollowerPure
					.boundHalfDegrees(Gyro.getInstance().getTranslatedAngle()) <= boundHalfDegrees(-100)) {
				System.out.println("if 2");
				Drive.getInstance().tankDrive(0, -power);
			} else {
				System.out.println("else");
				state = ProgramState.LOADING;
				loadingStartTime = Timer.getFPGATimestamp();
			}
			if (Timer.getFPGATimestamp() - pivotStartTime > 1.0) {

			}
			break;
		case LOADING:
			power = 0.15;
			if (Timer.getFPGATimestamp() - loadingStartTime < 0.5) {
				Drive.getInstance().tankDrive(power, power);
			} else if (Timer.getFPGATimestamp() - loadingStartTime < 1.3) {
				Drive.getInstance().zeroPowerMotors();
			} else {
				state = ProgramState.BACKUP;
				backUpStartTime = Timer.getFPGATimestamp();
			}
			break;
		case BACKUP:
			power = -0.5;
			double gyroOff = boundHalfDegrees(-90)
					- PathFollowerPure.boundHalfDegrees(Gyro.getInstance().getTranslatedAngle());
			double turnAdjust = -kp * gyroOff;
			if (Timer.getFPGATimestamp() - backUpStartTime < 0.8) {
				Drive.getInstance().tankDrive(power + turnAdjust, power - turnAdjust);
			} else {
				state = ProgramState.FINISHED;
			}
			break;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {

		// boolean areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();

		// int lidarAngle = 270;
		// if (isReversed) {
		// lidarAngle = 90;
		// }
		// if (Robot.bot.hasLidar()) {
		// lidarDistance = RobotState.getInstance().getLidarDistance(lidarAngle);
		// System.out.println("Lidar distance: " + lidarDistance);
		// }

		// if (areaOnTarget) {
		// System.out.println("Area on target");
		// return true;
		// }

		// double sum = 0;
		// for (int i = 0; i < previousTx.length; i++) {
		// sum += previousTx[i];
		// }
		// if (sum <= 0.01) {
		// return true;
		// }

		return state == ProgramState.FINISHED
				|| (RobotState.getInstance().getState() == RobotState.State.TELEOP);

	}

	private boolean followerEnd() {
		boolean areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();
		if (areaOnTarget) {
			System.out.println("Area on target");
			return true;
		}

		// double sum = 0;
		// for (int i = 0; i < previousTx.length; i++) {
		// sum += previousTx[i];
		// }
		// if (sum <= 0.01) {
		// return true;
		// }
		return false;
	}

	private double boundHalfDegrees(double deg) {
		return PathFollowerPure.boundHalfDegrees(deg);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Follow target ended. Stop motors...");
		Drive.getInstance().zeroPowerMotors();
		valueInitialized = false;
		RobotState.getInstance().setVisionFollowerRunning(false);
		// if (RobotState.getInstance().getState() != State.AUTON) {
		VisionLocalizer.getInstance().setDrive();
		// }
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
