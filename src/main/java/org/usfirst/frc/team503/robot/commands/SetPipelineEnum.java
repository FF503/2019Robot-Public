/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.PipelineSelector;

import edu.wpi.first.wpilibj.command.Command;

public class SetPipelineEnum extends Command {
  PipelineSelector pipelineSelector;

  public SetPipelineEnum(PipelineSelector pipelineSelector) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pipelineSelector = pipelineSelector;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotState.getInstance().setPipelineSelector(pipelineSelector);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
