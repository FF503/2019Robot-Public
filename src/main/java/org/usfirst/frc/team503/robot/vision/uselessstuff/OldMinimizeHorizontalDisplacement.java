/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;

public class OldMinimizeHorizontalDisplacement extends Command {

	private boolean stop;
	private double setpoint = 0;

	private SynchronousPID pid = new SynchronousPID();

	public OldMinimizeHorizontalDisplacement(boolean stop) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Drive.getInstance());
		this.stop = stop;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("Horizontal displacement started");
		double[] outputRange = Robot.bot.getHDOutputRange();
		pid.setOutputRange(outputRange[0], outputRange[1]); // perhaps could change this?
		VisionLocalizer.getInstance().setIndividual();
		double position = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0];
		setpoint = Robot.bot.getHDSetpoint(position);
		pid.setSetpoint(setpoint);
		pid.setPID(Robot.bot.getHDPIDV()[0], Robot.bot.getHDPIDV()[1], Robot.bot.getHDPIDV()[2]);
		if (RobotState.getInstance().isVisionDebug()) {
			System.out.println("kP: " + pid.getP());
			System.out.println("kI: " + pid.getI());
			System.out.println("kD: " + pid.getD());
			System.out.println("kV: " + Robot.bot.getHDPIDV()[3]);

			System.out.println(
					"Output range: " + Robot.bot.getHDOutputRange()[0] + ", " + Robot.bot.getHDOutputRange()[1]);
			System.out.println("Setpoint: " + pid.getSetpoint());
		}

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		double estimate = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0];

		double input = estimate * Math.signum(RobotState.getInstance().getTargetData().getAdjustedAngle());
		double turn = 0;

		double basePower = Robot.bot.getHDPIDV()[3] - pid.calculate(input);

		if (Math.abs(estimate) > Robot.bot.getHDDeadband()) {
			turn = Robot.bot.getHDTurnConstant(estimate) * basePower;
		} else {
			input = VisionLocalizer.getInstance()
					.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0];

		}
		if (RobotState.getInstance().isVisionDebug()) {
			System.out.println(
					"At x distance of " + input + ", supplied base power of " + basePower + " and turn of " + turn);
		}

		basePower *= RobotState.getInstance().getCameraDirection().getMultiplier();
		turn *= RobotState.getInstance().getCameraDirection().getMultiplier();

		Drive.getInstance().tankDrive(basePower - turn * Math.signum(estimate),
				basePower + turn * Math.signum(estimate));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// if (VisionLocalizer.getInstance().areaOnTarget()) {
		// return true;
		// }
		double[] estimate = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());

		System.out.println(estimate[0] - setpoint);

		System.out.println(estimate[0]);
		System.out.println(setpoint);
		return (Math.abs(estimate[0] - setpoint) < Robot.bot.getHDTolerance())
				|| VisionLocalizer.getInstance().areaOnTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Program ended with horizontal distance of " + VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0]);

		System.out.println("Horizontal Displacement ended");
		if (stop) {
			Drive.getInstance().zeroPowerMotors();
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Drive.getInstance().zeroPowerMotors();
	}

	public boolean isRunning() {
		return timeSinceInitialized() > 0 && !isCompleted();
	}
}