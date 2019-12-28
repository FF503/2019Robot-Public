package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.vision.PositionEstimator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArcadeDriveCommand extends Command {

	public ArcadeDriveCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Drive.getInstance());

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// System.out.println("init");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!RobotState.getInstance().isVisionFollowerRunning()) {
			// VisionLocalizer.getInstance().setDrive();
			PositionEstimator.getInstance().driverSetTarget();
			if (RobotState.getInstance().getDriveReversed()) {
				Drive.getInstance().arcadeDrive(OI.getDriverLeftYValue(), -OI.getDriverLeftXValue());
			} else {
				Drive.getInstance().arcadeDrive(-OI.getDriverLeftYValue(), -OI.getDriverLeftXValue());
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return !DriverStation.getInstance().isOperatorControl();
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
}
