/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.utils.UniversalGrapher;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Deprecated
public class FusedTargetDrive extends Command {

	private static final double distanceV = 0.2;
	private static final double distanceP = 0.02;
	private static final double distanceI = 0.0;
	private static final double distanceD = 0.01;
	private static final double distanceTol = 1.2;

	private DataEntry time = new DataEntry("time");
	private DataEntry angleOffset = new DataEntry("angleError");

	private static final double angleP = 0.075;
	private static final double angleI = 0;
	private static final double angleD = 0.05;
	private static final double areaPercentThreshold = 5.5;
	private SynchronousPID anglePID = new SynchronousPID();
	private SynchronousPID distancePID = new SynchronousPID();

	public FusedTargetDrive() {
		anglePID.setPID(angleP, angleI, angleD);
		anglePID.setSetpoint(0.0);
		anglePID.setInputRange(-28.3, 28.3);
		anglePID.setOutputRange(-0.2, 0.2);

		distancePID.setPID(distanceP, distanceI, distanceD);
		distancePID.setSetpoint(0.0);
		distancePID.setInputRange(-30, 30);
		distancePID.setOutputRange(-0.3, 0.3);

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		double angleError = VisionLocalizer.getInstance().getGroupedTargetAngle();
		double anglePower = anglePID.calculate(angleError);

		SmartDashboard.putNumber("Angle power", anglePower);
		SmartDashboard.putNumber("Angle error", angleError);

		double distanceError = VisionLocalizer.getInstance()
				.positionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0]
				* Math.signum(RobotState.getInstance().getTargetData().getAdjustedAngle());
		double distancePower = distanceV - distancePID.calculate(distanceError);
		SmartDashboard.putNumber("distance error", distanceError);
		SmartDashboard.putNumber("distance power", distancePower);
		Drive.getInstance().tankDrive(distancePower - anglePower, distancePower + anglePower);

		time.addValue(Timer.getFPGATimestamp());
		angleOffset.addValue(angleError);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return VisionLocalizer.getInstance().getGroupedTargetArea() > areaPercentThreshold;

		// return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Program has ended");
		Drive.getInstance().zeroPowerMotors();
		UniversalGrapher.addEntries(angleOffset);
		UniversalGrapher.addEntries(time);
		UniversalGrapher.sendValues();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
