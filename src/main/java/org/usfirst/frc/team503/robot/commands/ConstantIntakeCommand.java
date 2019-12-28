/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class ConstantIntakeCommand extends Command {

  public ConstantIntakeCommand() {
    requires(Intake.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Intake.getInstance().intakeCargo();
    new LedSetCommand().start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RobotState.getInstance().setLedColor(Intake.getInstance().hasCargo() ? LedColors.BLUE : LedColors.BLACK);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.getInstance().stopIntake();
    RobotState.getInstance().setLedColor(Intake.getInstance().hasCargo() ? LedColors.BLUE : LedColors.BLACK);
    new LedCargoListenerCommand().start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
