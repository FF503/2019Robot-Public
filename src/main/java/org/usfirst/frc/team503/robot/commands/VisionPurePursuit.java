/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import java.io.BufferedReader;
import java.io.FileReader;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.motionProfiling.PurePursuit;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Drive.DriveMotorOutput;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;
import motionProfiling.Trajectory;
import motionProfiling.Trajectory.Segment;

public class VisionPurePursuit extends Command {
  private Trajectory trajectory;
  private double xOffset;
  private double yOffset;
  private PurePursuit engine;
  private final int segTolerance = 10;
  private FFDashboard visionTable = new FFDashboard("Vision");

  public VisionPurePursuit(Trajectory trajectory) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.trajectory = trajectory;

  }

  public VisionPurePursuit(String file) {
    this.trajectory = readTrajectoryFromFile(file);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double[] visionTarget = VisionLocalizer.getInstance()
        .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());

    visionTable.putNumber("Starting Vision X", visionTarget[0]);
    visionTable.putNumber("Starting Vision Y", visionTarget[1]);
    Pose gpsPose = RobotState.getInstance().getPoseOdometry();
    Segment lastSeg = trajectory.getSegment(trajectory.getNumSegments() - 1);
    xOffset = lastSeg.x - visionTarget[0] - gpsPose.getX();
    yOffset = lastSeg.y - visionTarget[1] - gpsPose.getY();
    double lookAheadDist = Math.abs(visionTarget[0]);
    if (lookAheadDist > Robot.bot.lookAheadDistance) {
      lookAheadDist = Robot.bot.lookAheadDistance;
    }
    engine = new PurePursuit(trajectory, lookAheadDist, Robot.bot.TRACK_WIDTH);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Pose currentPose = RobotState.getInstance().getPoseOdometry().copy();
    currentPose.increment(xOffset, yOffset, 0.0);
    visionTable.putNumber("Current Pose X", currentPose.getX());
    visionTable.putNumber("Current Pose Y", currentPose.getY());
    DriveMotorOutput motorOutput = engine.calculateVelocities(currentPose);
    double leftPower = motorOutput.getLeftVelocity();
    double rightPower = motorOutput.getRightVelocity();
    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftPower), Math.abs(rightPower));
    if (maxMagnitude > 1.0) {
      leftPower /= maxMagnitude;
      rightPower /= maxMagnitude;
    }
    motorOutput.setPercentPowers(leftPower, rightPower);
    motorOutput.setDriveDirection(false);
    Drive.getInstance().tankDrive(motorOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (trajectory.getNumSegments() - (engine.getSegmentIndex() + 1)) <= segTolerance;
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

  private Trajectory readTrajectoryFromFile(String file) {
    if (!file.endsWith(".txt")) {
      file = file + ".txt";
    }
    if (!file.startsWith(Robot.bot.motionProfilingRioFolder)) {
      file = Robot.bot.motionProfilingRioFolder + file;
    }

    Trajectory trajectory = null;
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      int length = Integer.parseInt(in.readLine());
      trajectory = new Trajectory(length);
      String[] data;
      for (int i = 0; i < length; i++) {
        String line = in.readLine();
        data = line.split(",");
        Trajectory.Segment seg = new Trajectory.Segment();
        seg.dt = Double.parseDouble(data[0]);
        seg.x = Double.parseDouble(data[1]);
        seg.y = Double.parseDouble(data[2]);
        seg.pos = Double.parseDouble(data[3]);
        seg.vel = Double.parseDouble(data[4]);
        seg.acc = Double.parseDouble(data[5]);
        seg.jerk = Double.parseDouble(data[6]);
        seg.heading = Double.parseDouble(data[7]);
        seg.curvature = Double.parseDouble(data[8]);
        trajectory.setSegment(i, seg);
      }
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("FILE NOT FOUND");
    }

    return trajectory;
  }

}
