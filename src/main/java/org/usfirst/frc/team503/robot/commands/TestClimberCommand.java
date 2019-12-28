/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.State;
import org.usfirst.frc.team503.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.command.Command;

public class TestClimberCommand extends Command {
  public TestClimberCommand() {
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
    Climber.getInstance().runHandMotors(-OI.getOperatorLeftYValue());
    Climber.getInstance().runLegMotor(-OI.getOperatorRightYValue());
    // System.out.println("RUUUUUUUUUUUNN "+ -OI.getOperatorLeftYValue());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotState.getInstance().getState() == State.DISABLED;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Climber.getInstance().stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
