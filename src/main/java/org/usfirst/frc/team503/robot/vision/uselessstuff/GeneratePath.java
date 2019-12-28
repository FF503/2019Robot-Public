package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.commands.PurePursuitDrive;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import motionProfiling.Trajectory;

public class GeneratePath extends Command {
	private PurePursuitDrive purePursuitDriveCommand;
	private boolean done = false;

	public GeneratePath() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		double visionX = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0];
		double visionY = VisionLocalizer.getInstance()
				.translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[1];
		double distanceBelowTarget = 15; // to steer in you would want a point below the setpoint so the curve is smooth
		double gyroAngle = RobotState.getInstance().getTargetData().getAdjustedAngle(); // 0 is straight and positive in
																						// clockwise direction
		// Magnitude of the vector facing the direction of the gyro heading to decide
		// the 2nd point which makes our turns smoother
		visionY -= Robot.bot.endCoordinateSubtract;
		double secondX, secondY; // Coordinates for the second point
		if (gyroAngle <= 0) {
			secondX = -Math.cos(Math.toRadians(90 + gyroAngle)) * Robot.bot.magnitude;
			secondY = Math.sin(Math.toRadians(90 + gyroAngle)) * Robot.bot.magnitude;
		} else {
			secondX = Math.cos(Math.toRadians(90 - gyroAngle)) * Robot.bot.magnitude;
			secondY = Math.sin(Math.toRadians(90 - gyroAngle)) * Robot.bot.magnitude;
		}
		// double[][] positions = { { 0, 0 }, { secondX, secondY }, { visionX, visionY -
		// distanceBelowTarget },
		// { visionX, visionY } };

		// double[][] positions = { { 0, 0 }, { secondX, secondY }, { -25, 25 -
		// distanceBelowTarget },
		// { 5, 130 } };

		double[][] positions = { { 0, 0 }, { 0, 60 } };

		VisionPath vp = new VisionPath();
		double initTime = System.currentTimeMillis();
		double[][] smoothPath = vp.smoothVisionTrajectory(positions, Robot.bot.numTimesToSmooth);
		double[] targetVelocities = vp.targetVelocities(smoothPath);

		vp.printDoubleArray(smoothPath);
		// System.out.println("Printing all x coords");

		// for (int i = 0; i < smoothPath.length; i++) {
		// System.out.println(smoothPath[i][0]);
		// }

		// System.out.println("\n Printing all y coords");
		// for (int i = 0; i < smoothPath.length; i++) {
		// System.out.println(smoothPath[i][1]);
		// }

		NetworkTableInstance.getDefault().getTable("vision spline").getEntry("x coordinates")
				.setDoubleArray(smoothPath[0]);
		NetworkTableInstance.getDefault().getTable("vision spline").getEntry("x coordinates")
				.setDoubleArray(smoothPath[1]);

		System.out.println("Begin output");
		Trajectory traj = vp.toTrajectory(smoothPath, targetVelocities);

		// purePursuitDriveCommand = new PurePursuitDrive(traj, false, false);

		System.out.println(traj.toString());

		double finalTime = System.currentTimeMillis();
		vp.sendNetworkTablesData(smoothPath, targetVelocities);

		System.out.println("Trajectory Generation Run Time: " + (finalTime - initTime));
		// purePursuitDriveCommand.start();
		done = true;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return done;
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
	}

	public boolean commandFinished() {
		return isCompleted();
	}

	public boolean isRunning() {
		return timeSinceInitialized() > 0 && !isCompleted();
	}
}
