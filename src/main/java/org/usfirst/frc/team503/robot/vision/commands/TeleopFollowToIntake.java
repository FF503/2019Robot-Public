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
import org.usfirst.frc.team503.robot.RobotState.PipelineSelector;
import org.usfirst.frc.team503.robot.RobotState.State;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Deprecated
public class TeleopFollowToIntake extends Command {

	// private SynchronousPID pid;
	// double basePower = Robot.bot.getFTBasePower(), basePowerP =
	// Robot.bot.getFTBasePowerP();

	double totalDistance;
	double setpoint;
	double validSetpoint;

	double distance, lidarDistance;

	double startTime;
	double tx;
	double kp, kd;
	double autonTolerance;
	boolean isReversed;
	boolean areaOnTarget;

	public TeleopFollowToIntake() {
		RobotState.getInstance().setAutonToleranceAdjustment(0.0);
	}

	public TeleopFollowToIntake(double autonToleranceAdjustment){
		RobotState.getInstance().setAutonToleranceAdjustment(autonToleranceAdjustment);
	}

	public TeleopFollowToIntake(PipelineSelector pipelineMode, double autonToleranceAdjustment) {
		RobotState.getInstance().setAutonToleranceAdjustment(autonToleranceAdjustment);
		RobotState.getInstance().setPipelineSelector(pipelineMode);
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
		kp = 0.01; // 0.02
		FFDashboard.getInstance().putBoolean("IS VISION REVERSED", isReversed);
		System.out.println("Camera reversed? " + isReversed);

		setpoint = VisionLocalizer.getInstance().getTX();
		System.out.println("FollowTarg: Initial Setpoint:" + (setpoint));

		System.out.println("Total distance: " + totalDistance);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		tx = VisionLocalizer.getInstance().getTX();
		double setpoint = tx - Robot.bot.getLimelightAbsoluteOffset();
		double basePower = 0.3;// 0.4;
		if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
			basePower = 0.3 - 0.03 * VisionLocalizer.getInstance().getTA();
		} else {
			basePower = 0.4 - 0.04 * VisionLocalizer.getInstance().getTA();
		}
		System.out.println("Angle setpoint: " + setpoint);
		double turn = setpoint * kp;
		if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
			turn *= -1;
		}
		System.out.println("running simple");
		double left = basePower + turn, right = basePower - turn;
		if (!isReversed) {
			System.out.println("Tank Drive Forward: " + left + ", Right: " + right);
			Drive.getInstance().tankDrive(left, right);
		} else {
			System.out.println("Tank Drive Reverse: " + left + ", Right: " + right);
			Drive.getInstance().tankDrive(-right, -left);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		areaOnTarget = VisionLocalizer.getInstance().areaOnTarget();
		System.out.println("area: " + VisionLocalizer.getInstance().getTA());
		int lidarAngle = 270;
		if (isReversed) {
			lidarAngle = 90;
		}
		if (Robot.bot.hasLidar()) {
			lidarDistance = RobotState.getInstance().getLidarDistance(lidarAngle);
			System.out.println("Lidar distance: " + lidarDistance);
		}
		if ((tx == 0.0 || areaOnTarget) && Timer.getFPGATimestamp() - startTime > 0.5) {
			return true;
		}
		// if(!VisionLocalizer.getInstance().isTargetSeen()) {
		// return true;
		// }
		double pctremaining = distance / totalDistance;
		System.out.println("Distance Completed: " + pctremaining);
		boolean lidarAngleGood = lidarDistance > 0 && Robot.bot.hasLidar();
		if (Timer.getFPGATimestamp() - startTime > 0.5) {
			if (lidarAngleGood) {
				return (Math.abs(lidarDistance) < 40);
			} else {
				return (pctremaining < .05);
			}
		}
		// return !OI.getIntakeFollow();
		return true;

	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Follow target ended. Stop motors...");
		Drive.getInstance().zeroPowerMotors();
		RobotState.getInstance().setVisionFollowerRunning(false);
		if (RobotState.getInstance().getState() != State.AUTON) {
			VisionLocalizer.getInstance().setDrive();
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
