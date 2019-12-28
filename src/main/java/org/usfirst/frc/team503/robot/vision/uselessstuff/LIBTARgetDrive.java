/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision.uselessstuff;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.commands.PurePursuitDrive;
import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.UniversalGrapher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import motionProfiling.Config;
import motionProfiling.PathGenerator;
import motionProfiling.Points;
import motionProfiling.Trajectory;
import motionProfiling.Waypoint;

public class LIBTARgetDrive extends Command {
	public LIBTARgetDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// double[] position = VisionLocalizer.getInstance()
		// .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());

		double[] position = { 10, 60 };
		Points points = new Points();
		points.addWaypoint(new Waypoint(0, 0, 90, true));
		points.addWaypoint(new Waypoint(position[0], position[1] - Robot.bot.endCoordinateSubtract, 90, true));
		double initTime = Timer.getFPGATimestamp();

		System.out.println("Lib path generating");
		Trajectory traj = PathGenerator.generateCentralTrajectory(points, (new Config(0.05, 130, 100, 8000)));
		// (new GeneratePath()).start();
		DataEntry x = new DataEntry("x");
		DataEntry y = new DataEntry("y");
		DataEntry v = new DataEntry("v");

		for (int i = 0; i < traj.getNumSegments(); i++) {

			x.addValue(traj.getSegment(i).x);
			y.addValue(traj.getSegment(i).y);
			v.addValue(traj.getSegment(i).vel);

		}

		UniversalGrapher.addEntries(x);
		UniversalGrapher.addEntries(y);
		UniversalGrapher.addEntries(v);

		System.out.println("Lib path generated");
		SmartDashboard.putNumber("Generation time", Timer.getFPGATimestamp() - initTime);
		UniversalGrapher.sendValues();

		new PurePursuitDrive(traj).start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
