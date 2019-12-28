/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowBall extends Command {

	private double tolerance = 0.25;
	private double kP = 0.02;
	private double kI = 0.0;
	private double kD = 0.0;
	private double basePower = 0.6; // 0.9
	private double areaPercentThreshold = 5.5;
	private double[] outputRange = { -0.3, 0.3 }; // 0.2
	private SynchronousPID pid = new SynchronousPID();

	private double endTime;
	private boolean isFinished = false;

	public FollowBall() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// VisionLocalizer.getInstance().setPipeline(6); Add this back in if the
		// follower's strangely not working
		System.out.println("Ball following started");
		pid.setPID(kP, kI, kD);
		pid.setSetpoint(0.0);
		pid.setInputRange(-27.0, 27.0);
		pid.setOutputRange(outputRange[0], outputRange[1]);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		double angleToCenter = VisionLocalizer.getInstance().getGroupedTargetAngle();

		double turnPower = pid.calculate(angleToCenter);
		Drive.getInstance().tankDrive(basePower - turnPower, basePower + turnPower);
		SmartDashboard.putNumber("Error (degrees)", angleToCenter);
		SmartDashboard.putNumber("turn power", turnPower);
		SmartDashboard.putNumber("left power", basePower - turnPower);
		SmartDashboard.putNumber("Right power", basePower + turnPower);

		if ((VisionLocalizer.getInstance().getBallArea() == 0) && !isFinished) {

			endTime = Timer.getFPGATimestamp();
			isFinished = true;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (isFinished) {
			return (Timer.getFPGATimestamp() - endTime > 0.2);
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Drive.getInstance().tankDrive(0.0, 0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
