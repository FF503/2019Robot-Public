package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.FroggyDriveHelper;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AssistedArcadeDriveCommand extends Command {

	public AssistedArcadeDriveCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Drive.getInstance());

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// System.out.println("init arcade drive");
		VisionLocalizer.getInstance().setDrive();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!RobotState.getInstance().isVisionFollowerRunning() && !OI.getFollowButton()) {
			boolean reversed = RobotState.getInstance().getDriveReversed();
			double yJoystickValue = -OI.getDriverLeftYValue() * (reversed ? -1 : 1);
			// PositionEstimator.getInstance().driverSetTarget();
			FroggyDriveHelper.getInstance().froggyDrive(yJoystickValue, OI.getDriverLeftXValue(),
					OI.getDriverRightTrigger());
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return DriverStation.getInstance().isDisabled() || (DriverStation.getInstance().isAutonomous() && OI.getDriverDPadUp());
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().zeroPowerMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	} 
}
