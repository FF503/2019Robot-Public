/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class PrepareAngleNew extends Command {
	private boolean stop;
	private double setpoint = 0;
	private double[] position;

	private SynchronousPID pid = new SynchronousPID();

	public PrepareAngleNew(boolean stop) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Drive.getInstance());
		this.stop = stop;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		double initTime = Timer.getFPGATimestamp();
		VisionLocalizer.getInstance().setIndividual();

		position = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());

		RobotState.getInstance().setVisionStartPosition(position);
		// double[] pidVals = Robot.bot.getPAPID();

		// pid.setPID(pidVals[1], pidVals[2], pidVals[3]);
		// pid.setOutputRange(-1, 1); // 1.45 on programming bot

		setpoint = 30 * Math.signum(position[0]);

		// pid.setSetpoint(setpoint);
		System.out.println("Position: " + position[0] + " " + position[1]);
		// System.out.println("Robot will attempt to turn to " + setpoint + " degrees
		// relative to the target");
		// System.out.println("Initial angle: " +
		// RobotState.getInstance().getTargetData().getAdjustedAngle());
		// System.out.println("Begin PrepareAngle loop");

		// if (RobotState.getInstance().isVisionDebug()) {
		// SmartDashboard.putNumber("PAPivotKP", pidVals[1]);
		// SmartDashboard.putNumber("PAPivotKI", pidVals[2]);
		// SmartDashboard.putNumber("PAPivotKD", pidVals[3]);
		// }
		System.out.println("Time: " + (Timer.getFPGATimestamp() - initTime));

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {/*
								 * double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();
								 * double turnPower = pid.calculate(angle); double basePower = 0; double left =
								 * basePower + turnPower, right = basePower - turnPower;
								 * 
								 * // TODO: this might need to be fixed later
								 * 
								 * if (Robot.bot.getName().equals("ProgrammingBot")) { left *= -1; right *= -1;
								 * }
								 * 
								 * left *= -RobotState.getInstance().getCameraDirection().getMultiplier(); right
								 * *= -RobotState.getInstance().getCameraDirection().getMultiplier();
								 * 
								 * Drive.getInstance().tankDrive(left, right);
								 * 
								 * System.out.println("At angle " + angle + " and raw angle of " +
								 * Gyro.getInstance().getAngle() + ", executed PrepareAngle with left power of "
								 * + left + ", right power of " + right); // setpoint = //
								 * Robot.bot.getPACheese()*VisionLocalizer.getInstance().
								 * calculateTranslatedPrepareAngleTarget(); // pid.setSetpoint(setpoint); //
								 * System.out.println("Setpoint adjusted to: " + setpoint);
								 */
		Drive.getInstance().tankDrive(-0.2 * Math.signum(position[0]), 0.2 * position[0]);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		/*
		 * return pid.onTarget(Robot.bot.getPATolerance());
		 */
		return Math.abs(RobotState.getInstance().getTargetData().getAdjustedAngle() - setpoint) <= 12;

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
