/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.motionProfiling.PurePursuit;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Drive.DriveMotorOutput;
import org.usfirst.frc.team503.robot.utils.Vector2D;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;

public class PurePursuitVisionTarget extends Command {
	PurePursuit follower;
	double vel;
	Vector2D lookAhead;
	double forwardDist = 20;

	public PurePursuitVisionTarget() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		double[] lookAheadArray = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		double targetAngle = Math.toRadians(RobotState.getInstance().getTargetData().getTargetFieldAngle());
		Vector2D translationVector = new Vector2D(-Math.sin(targetAngle) * forwardDist,
				-Math.cos(targetAngle) * forwardDist);
		lookAhead = (new Vector2D(lookAheadArray)).add(translationVector);
		lookAhead = lookAhead.add(RobotState.getInstance().getPoseOdometry().toVector());
		follower = new PurePursuit(lookAhead, Robot.bot.TRACK_WIDTH);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		vel = 40;
		DriveMotorOutput output = follower.calculateVelocitesFixedLookAhead(RobotState.getInstance().getPoseOdometry(),
				vel);
		double lp = output.getLeftVelocity();
		double rp = output.getRightVelocity();
		output.setPercentPowers(lp, rp);
		output.setDriveDirection(false);
		Drive.getInstance().tankDrive(output);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (RobotState.getInstance().getPoseOdometry().toVector().distanceTo(lookAhead) <= 3);
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
		end();
	}
}
