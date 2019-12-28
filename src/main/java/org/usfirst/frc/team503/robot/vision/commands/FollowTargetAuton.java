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
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.auton.FroggyAuton;
import org.usfirst.frc.team503.robot.commands.LedSetCommand;
import org.usfirst.frc.team503.robot.motionProfiling.PathFollowerPure;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Intake;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowTargetAuton extends Command {

	// private SynchronousPID pid;
	// double basePower = Robot.bot.getFTBasePower(), basePowerP =
	// Robot.bot.getFTBasePowerP();

	double totalDistance;
	double setpoint;
	double validSetpoint;

	double distance, lidarDistance;

	double startTime;
	double tx;
	double[] lastFiveTx = new double[5];
	double[] previousTx = new double[10];
	int index = 0;
	private double kp = 1.0, kd = 0.0;
	double autonTolerance;
	boolean isReversed;
	boolean valueInitialized = false;
	double kDriveP = 0.4;
	double initialCamera, initialGyro, desiredGyro;
	ProgramState state = ProgramState.FOLLOWING;
	double loadingStartTime;
	double backUpStartTime;
	double pivotStartTime;
	double maxTurn = 0.015;
	double minTurn = 0.01;
	double maxArea = Robot.bot.getAreaThreshold();
	double minArea = 0.0;
	double slope = (maxTurn - minTurn) / (maxArea - minArea);
	boolean turnHarder = false;
	boolean folowingRightReload = false;
	boolean folowingLeftReload = false;
	double toleranceAdjustment = 0.0;

	public static enum ProgramState {
		FOLLOWING, LOADING, BACKUP, FINISHED;
	}

	public FollowTargetAuton() {
		toleranceAdjustment = 0.0;
		//RobotState.getInstance().setAutonToleranceAdjustment(0.0);
	}

	public FollowTargetAuton(boolean leftReload, boolean rightReload) {
		folowingLeftReload = leftReload;
		folowingRightReload = rightReload;
		toleranceAdjustment = 0.0;
		//RobotState.getInstance().setAutonToleranceAdjustment(0.0);
	}


	public FollowTargetAuton(double autonToleranceAdjustment) {
		toleranceAdjustment = autonToleranceAdjustment;
		//RobotState.getInstance().setAutonToleranceAdjustment(autonToleranceAdjustment);
	}


	public FollowTargetAuton(double autonToleranceAdjustment, boolean turnHarder) {
		toleranceAdjustment = 0.0;
	//	RobotState.getInstance().setAutonToleranceAdjustment(autonToleranceAdjustment);
		this.turnHarder = turnHarder;
	}

	public FollowTargetAuton(double autonToleranceAdjustment, double kp) {
		toleranceAdjustment = 0.0;
		this.kDriveP = kp;
	}

	
	public FollowTargetAuton(double autonToleranceAdjustment, double kp, boolean turnHarder) {
		toleranceAdjustment = 0.0;
		this.kDriveP = kp;
		this.turnHarder = turnHarder;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("*****************FollowTarget Starting***************************");
		VisionLocalizer.getInstance().setPipeline(RobotState.getInstance().getPipelineSelector().getPipeline());
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
		if (!Intake.getInstance().hasHatch()) {
			new LedSetCommand(LedColors.GREEN).start();
		}
		startTime = Timer.getFPGATimestamp();
		state = ProgramState.FOLLOWING;
		RobotState.getInstance().setAutonToleranceAdjustment(toleranceAdjustment);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		VisionLocalizer.getInstance().setPipeline(3);
		switch (state) {
		case FOLLOWING:
			double areaOfTarget = VisionLocalizer.getInstance().getTA();
			double basePower = 0, turn = 0;
			tx = VisionLocalizer.getInstance().getTX();

			if (tx == 0.0 && folowingRightReload) {
				double targetX = FroggyAuton.FieldLocations.RightHatchReload.getLocation().getX();
				double targetY = FroggyAuton.FieldLocations.RightHatchReload.getLocation().getY();
				Pose pose = RobotState.getInstance().getPoseOdometry();
				double deltaX = targetX - pose.getX();
				double deltaY = targetY - pose.getY();
				double targetAngle = Math.toDegrees(Math.atan(deltaX / deltaY)) - 90;
				tx = boundHalfDegrees(targetAngle -pose.getTheta());
			}

			if (index == 10) {
				index = 0;
			}
			previousTx[index] = tx;
			index++;
			double setpoint = tx;
			if (isReversed) {
				// setpoint += 11;
				basePower = kDriveP / areaOfTarget;
			} else {
				basePower = kDriveP / areaOfTarget;
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
			// if (valueInitialized) { // if the target's lost
			// // basePower and turn need to be set
			// double remainingTurn = desiredGyro - Gyro.getInstance().getAngle();//
			// initialCamera -
			// // Gyro.getInstance().getAngle() -
			// kp = slope * areaOfTarget + minTurn; // initialGyro;
			// turn = remainingTurn * kp;
			// // turn = setpoint * kp;
			// // System.out.println("lost target turn calculation");
			// } else {
			kp = slope * areaOfTarget + minTurn;
			if (turnHarder) {
				kp = kp * 1.5;
			}
			turn = setpoint * kp;
			// System.out.println("normal turn calculation");
			// }
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
				state = ProgramState.LOADING;
				// pivotStartTime = Timer.getFPGATimestamp();
				loadingStartTime = Timer.getFPGATimestamp();
			}
			break;
		case LOADING:
			double power = 0.15;
			if (Timer.getFPGATimestamp() - loadingStartTime < 0.5) {
				Drive.getInstance().tankDrive(power, power);
			} else if (Timer.getFPGATimestamp() - loadingStartTime < 0.6/* 1.3 */) {
				Drive.getInstance().zeroPowerMotors();
			} else {
				state = ProgramState.FINISHED;
				// backUpStartTime = Timer.getFPGATimestamp();
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

		// if (DriverStation.getInstance().isOperatorControl() &&
		// !OI.getIntakeFollow()){
		// return true;
		// }
		return state == ProgramState.FINISHED;

	}

	private boolean followerEnd() {
		boolean areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();
		double sum = 0.0;
		for (int i = 0; i < previousTx.length; i ++){
			sum += Math.abs(previousTx[i]);
		}
		double avg = sum / previousTx.length;
		if ( (avg == 0.0 && Timer.getFPGATimestamp() - startTime > 1.2 && !(folowingRightReload || folowingLeftReload))){
			RobotState.getInstance().setLostTarget(true);
			return true;
		}
		if (areaOnTarget) {
			System.out.println("Area on target");
			return true;
		}
		// if (Intake.getInstance().hasHatch()){
		// 	return true;
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
		new LedSetCommand(LedColors.BLACK).start();

		// }
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
