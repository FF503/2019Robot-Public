/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.auton.FroggyAuton.FieldLocations;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.utils.Pose.priority;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class SetPositionCommand extends InstantCommand {
  /**
   * Add your docs here.
   */
  private Pose pose;
  private double y = 0;

  public SetPositionCommand(FieldLocations location) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pose = location.getLocation();
  }

  public SetPositionCommand(Pose pose) {
    this.pose = pose;
  }

  public SetPositionCommand(double y) {
    this.y = y;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (y != 0.0) {
      Pose robotPose = RobotState.getInstance().getPoseOdometry();
      robotPose.updateY(y);
      pose = robotPose;
    }
    RobotState.getInstance().setPose(pose, priority.HIGH);
  }

}
