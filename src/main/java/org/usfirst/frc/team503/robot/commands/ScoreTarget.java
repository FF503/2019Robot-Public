/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoreTarget extends Command {
	double initLeft, initRight;
	double[] initPos;

	public ScoreTarget() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		initLeft = Drive.getInstance().getLeftDistanceInches();
		initRight = Drive.getInstance().getRightDistanceInches();
		initPos = VisionLocalizer.getInstance()
				.positionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());

		// System.out.println("Init y: ");
		SmartDashboard.putNumber("init y", initPos[1]);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Drive.getInstance().tankDrive(0.1, 0.1);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// double distance = ((Drive.getInstance().getLeftDistanceInches() - initLeft)
		// + (Drive.getInstance().getRightDistanceInches() - initRight)) / 2;

		// System.out.println("Distance: " + distance);
		// SmartDashboard.putNumber("Distance", distance);

		double area = (VisionLocalizer.getInstance().getIndividualTargetAreaStraight());
		SmartDashboard.putNumber("area", area);

		return area > 2.6;


		

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
