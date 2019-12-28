/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.CameraDirection;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.commands.LedSetCommand;
import org.usfirst.frc.team503.robot.commands.RumbleDriveJoystick;
import org.usfirst.frc.team503.robot.motionProfiling.PathFollowerPure;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopFollowTarget extends Command {

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
	double kp = 0.01, kd = 0.0;
	double autonTolerance;
	boolean isReversed;
	boolean valueInitialized = false;
	double kDriveP = 0.4;
	double initialCamera, initialGyro, desiredGyro;
	ProgramState state = ProgramState.FOLLOWING;
	FollowingType type;
	double loadingStartTime;
	double backUpStartTime;
	double pivotStartTime;
	double maxTurn = 0.015;
	double minTurn = 0.01;
	double maxArea = Robot.bot.getAreaThreshold();
	double minArea = 0.0;
	double slope = (maxTurn - minTurn) / (maxArea - minArea);

	public static enum ProgramState {
		FOLLOWING, LOADING, BACKUP, FINISHED;
	}

	public static enum FollowingType {
		INTAKE, PLACE;
	}

	public TeleopFollowTarget() {
		RobotState.getInstance().setAutonToleranceAdjustment(2.0);
	}

	public TeleopFollowTarget(FollowingType type) {
		RobotState.getInstance().setAutonToleranceAdjustment(0.0);
		this.type = type;
	}

	public TeleopFollowTarget(double autonToleranceAdjustment) {
		RobotState.getInstance().setAutonToleranceAdjustment(autonToleranceAdjustment);
		this.type = FollowingType.PLACE;
	}

	public TeleopFollowTarget(double autonToleranceAdjustment, double kp) {
		this.kp = kp;
		this.type = FollowingType.PLACE;
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
		if (RobotState.getInstance().hasHatch()) {
			this.type = FollowingType.PLACE;
		} else {
			this.type = FollowingType.INTAKE;
		}
		System.out.println("Total distance: " + totalDistance);
		startTime = Timer.getFPGATimestamp();
		state = ProgramState.FOLLOWING;
		if (type == FollowingType.INTAKE) {
			(new LedSetCommand(LedColors.GREEN)).start();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		VisionLocalizer.getInstance().setPipeline(3);
		SmartDashboard.putString("FOllowing TYpe", type.name());
		switch (state) {
		case FOLLOWING:
			double areaOfTarget = VisionLocalizer.getInstance().getTA();
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
			if (valueInitialized) { // if the target's lost
				// basePower and turn need to be set
				double remainingTurn = desiredGyro - Gyro.getInstance().getAngle();// initialCamera -
				// Gyro.getInstance().getAngle() -
				kp = slope * areaOfTarget + minTurn; // initialGyro;
				turn = remainingTurn * kp;
				// turn = setpoint * kp;
				// System.out.println("lost target turn calculation");
			} else {
				kp = slope * areaOfTarget + minTurn;
				System.out.println();
				System.out.println();
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
				state = ProgramState.LOADING;
				// pivotStartTime = Timer.getFPGATimestamp();
				loadingStartTime = Timer.getFPGATimestamp();
			}
			break;
		case LOADING:
			double power = 0.05;
			if (isReversed) {
				power *= -1;
			}
			Drive.getInstance().tankDrive(power, power);

			if (RobotState.getInstance().hasHatch() && type == FollowingType.INTAKE) {
				backUpStartTime = Timer.getFPGATimestamp();
				(new LedSetCommand(LedColors.BLACK)).start();
				(new RumbleDriveJoystick()).start();
				state = ProgramState.BACKUP;
			}
			break;
		case BACKUP:
			power = -0.5;
			// double gyroOff = boundHalfDegrees(-90)
			// - PathFollowerPure.boundHalfDegrees(Gyro.getInstance().getTranslatedAngle());
			double turnAdjust = 0;// -kp * gyroOff;
			// if (Timer.getFPGATimestamp() - backUpStartTime < 0.7) {
			// Drive.getInstance().tankDrive(power + turnAdjust, power - turnAdjust);
			// } else {
			// state = ProgramState.FINISHED;
			// }
			state = ProgramState.FINISHED;
			break;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (DriverStation.getInstance().isEnabled() && !OI.getFollowButton()) {
			return true;
		}
		return state == ProgramState.FINISHED;

	}

	private boolean followerEnd() {
		boolean areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();
		if (areaOnTarget) {
			System.out.println("Area on target");
			return true;
		}
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
		(new LedSetCommand(LedColors.BLACK)).start();
		// }
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
