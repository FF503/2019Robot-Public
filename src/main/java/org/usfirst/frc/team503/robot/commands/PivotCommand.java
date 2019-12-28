/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class PivotCommand extends Command {
	double kp, ki, kd;
	double setpoint;
	double lastError;
	double sum;
	double lastTime;
	double startTIme;
	double minPower = 0.0;
	double tolerance = Robot.bot.PIVOT_TOLERANCE;

	public PivotCommand(double setpoint) {
		kp = Robot.bot.PIVOT_P;
		ki = Robot.bot.PIVOT_I;
		kd = Robot.bot.PIVOT_D;
		this.setpoint = setpoint;
	}

	public PivotCommand(double setpoint, double kp, double tolerance) {
		this.kp = kp;
		ki = Robot.bot.PIVOT_I;
		kd = Robot.bot.PIVOT_D;
		this.setpoint = setpoint;
		this.tolerance = tolerance;
	}

	public PivotCommand(double setpoint, double tolerance) {
		this(setpoint);
		this.tolerance = tolerance;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		sum = 0;
		lastError = 0;
		startTIme = Timer.getFPGATimestamp();
	}


	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double curError = setpoint - Gyro.getInstance().getTranslatedAngle();
		curError = boundHalfDegrees(curError);
		sum += curError;
		double power = minPower + (curError * kp + ki * sum + kd * (curError - lastError) / (Timer.getFPGATimestamp() - lastTime));
		Drive.getInstance().tankDrive(power, -power);
		lastTime = Timer.getFPGATimestamp();
		lastError = curError;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return Gyro.getInstance().getAngle() >= angle;
		return (Math.abs(Gyro.getInstance().getTranslatedAngle() - setpoint) < tolerance || Timer.getFPGATimestamp() - startTIme > 1.5);
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private double boundHalfDegrees(double angle_degrees) {
		while (angle_degrees >= 180.0)
			angle_degrees -= 360.0;
		while (angle_degrees < -180.0)
			angle_degrees += 360.0;
		return angle_degrees;
	}
}