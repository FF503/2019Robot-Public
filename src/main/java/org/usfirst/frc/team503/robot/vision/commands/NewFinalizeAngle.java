/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.State;
import org.usfirst.frc.team503.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;

public class NewFinalizeAngle extends Command {
	private boolean stop;
	private double setpoint = 0;
	// private boolean isFinished = false;
	// double[] position;

	public NewFinalizeAngle(boolean stop) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.stop = stop;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();
		double turnPower = 0.1;

		if (RobotState.getInstance().getState().equals(State.TELEOP)) {
			turnPower *= -1;
		}
		// if (RobotState.getInstance().getTargetData().getTargetFieldAngle() < 0) {
		// turnPower *= -1;
		// }

		// if(Robot.bot.getName().equals("PracticeBot")) {
		// 	turnPower *= -1;
		// }

		turnPower *= Math.signum(angle);
		System.out.println("angle");

		double left = turnPower, right = -turnPower;

		Drive.getInstance().tankDrive(left, right);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// return pid.onTarget(Robot.bot.getFATolerance());
		return (Math.abs(RobotState.getInstance().getTargetData().getAdjustedAngle()) < 2);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Ended PrepareAngle with a final angle of: "
				+ RobotState.getInstance().getTargetData().getAdjustedAngle());
		if (stop) {
			Drive.getInstance().zeroPowerMotors();
		}
		System.out.println("finished");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Drive.getInstance().zeroPowerMotors();
	}
}
