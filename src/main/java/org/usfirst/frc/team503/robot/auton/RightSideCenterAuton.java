/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState.PipelineSelector;
import org.usfirst.frc.team503.robot.commands.ReleaseHatchCommand;
import org.usfirst.frc.team503.robot.commands.SetPipelineEnum;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetAuton;

public class RightSideCenterAuton extends FroggyAuton {
  private String[] profiles = new String[] { "RightToCenter" };

  @Override
  protected void initAuton() {
    addSequential(new SetPipelineEnum(PipelineSelector.RIGHT));
    // addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 2.0));
    // addParallel(new MoveArmCommand(ArmDirection.BACK, GameElement.HATCH,
    // TargetHeight.LOW, 1.5));
    froggySequentialDrive("RightToCenter");
    addSequential(new FollowTargetAuton());
    addSequential(new ReleaseHatchCommand(0.5));

  }

  @Override
  protected AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.Right;
  }
}
