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
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;

public class FollowCube extends Command {
	private boolean pointingToTarget = false;
	private double desiredGyro;
	private double basePowerMax, basePowerP;

	private double[] ftPIDVals = Robot.bot.getFTPIDV(), ftOutputRange = Robot.bot.getFTOutputRange();
	private SynchronousPID ftPID = new SynchronousPID();

	private double[] dsPIDVals = Robot.bot.getDSPID(), dsOutputRange = Robot.bot.getDSOutputRange();
	private SynchronousPID dsPID = new SynchronousPID();

	private double[] startingPosition, startingEncoders;

	private double[] goalPoint = { 0, 0 };

	private boolean driveStraightInitialized = false;

	double distanceTraveled;
	boolean completed;

	public FollowCube() {
		// Use requires() here to declare subsystem dependencies
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		basePowerMax = Robot.bot.getFTBasePower();
		basePowerP = Robot.bot.getFTBasePowerP();

		ftPID = new SynchronousPID(ftPIDVals[0], ftPIDVals[1], ftPIDVals[2]);
		ftPID.setOutputRange(ftOutputRange[0], ftOutputRange[1]);

		dsPID = new SynchronousPID(dsPIDVals[0], dsPIDVals[1], dsPIDVals[2]);
		ftPID.setOutputRange(dsOutputRange[0], dsOutputRange[1]);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double[] positionEstimate = VisionLocalizer.getInstance().getPosition();
		double left, right, base = 0;

		double gyroAngle = RobotState.getInstance().getTargetData().getAdjustedAngle();

		if (!pointingToTarget) {
			double angleToTarget = VisionLocalizer.getInstance().translatedAngleToCenter(positionEstimate);

			if ((Math.abs(gyroAngle - angleToTarget) < 2)) { // TODO: un-hardcode this
				desiredGyro = angleToTarget;
				pointingToTarget = true;
				startingPosition = positionEstimate;
				startingEncoders = new double[] { Drive.getInstance().getLeftDistanceInches(),
						Drive.getInstance().getRightDistanceInches() };
			} else {

				// this specific segment assumes that it is able to see the target at all times
				System.out.println("Following target");
				// Calculating basePower
				double curDistance = positionEstimate[0];
				double error = VisionLocalizer.getInstance().getError(0.0, curDistance);
				System.out.println("Horizontal Error: " + error);

				if (Math.abs(error) > 10) { // TODO: Redo this constant "10" in robot hardware
					base = basePowerMax;
				} else {
					base = basePowerP * Math.abs(error);
				}

				System.out.println("Base power: " + base);

				// Calculating turn
				ftPID.setSetpoint(angleToTarget);
				System.out.println("Angular setpoint: " + angleToTarget);

				System.out.println("angle: " + gyroAngle);
				System.out.println("error: " + ftPID.getError());

				double turn = ftPID.calculate(gyroAngle);
				turn *= ftPIDVals[3]; // multiply turn by F

				if (RobotState.getInstance().getState().equals(State.TELEOP)) {
					turn *= -1;
				}

				turn *= Math.abs(base);

				left = base - turn;
				right = base + turn;
				System.out.println("Left: " + left + ", Right: " + right);
				Drive.getInstance().tankDrive(left, right);

			}
		} else if (pointingToTarget)

		{

			if (!driveStraightInitialized) {

				System.out.println("Driving straight to target");
				// Calculating basePower
				double leftDist = Drive.getInstance().getLeftDistanceInches() - startingEncoders[0];
				double rightDist = Drive.getInstance().getLeftDistanceInches() - startingEncoders[1];

				distanceTraveled = (leftDist + rightDist) / 2;

				double setpoint = 1 / Math.sin(gyroAngle) * startingPosition[0] - 20;

				// Math.hypot((goalPoint[0] - startingPosition[0]),
				// (goalPoint[1] - startingPosition[1]));
				dsPID.setSetpoint(setpoint);
				driveStraightInitialized = true;

			}
			System.out.println("Drive straight traveled: " + distanceTraveled);
			if (!dsPID.onTarget(1)) { // TODO: un-hardcode this
				base = dsPID.calculate(distanceTraveled);
				System.out.println("Drive Straight Power: " + base);
			} else {
				Drive.getInstance().zeroPowerMotors();
			}

			System.out.println("Base power: " + base);

			// Calculating turn
			ftPID.setSetpoint(desiredGyro);
			System.out.println("Angular setpoint: " + desiredGyro);

			System.out.println("angle: " + gyroAngle);

			double turn = ftPID.calculate(gyroAngle);
			turn *= ftPIDVals[3]; // multiply turn by F

			if (RobotState.getInstance().getState().equals(State.TELEOP)) {
				turn *= -1;
			}

			turn *= Math.abs(base);

			left = base - turn;
			right = base + turn;
			System.out.println("Left: " + left + ", Right: " + right);
			// Drive.getInstance().tankDrive(left, right);
		}

		if(dsPID.onTarget(10) && pointingToTarget) {
			completed = true;
		}
		
		// TODO: Drive code needs to be added
		// Basepower calculated based on how far the robot has left to travel the
		// distance that has been calculated



		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		double distance = RobotState.getInstance().getLidarDistance(0);
		System.out.println("Lidar distance: " + distance);
		// return (distance < 60);
		return completed;
		
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