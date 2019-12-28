/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.subsystems.Extension;

public class TestExtensionCommand extends Command {
  public TestExtensionCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Extension.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Extension.getInstance().setMotorPower(-0.8*OI.getOperatorRightYValue());
    // Extension.getInstance().setTargetPosition(0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !DriverStation.getInstance().isOperatorControl();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Extension.getInstance().setMotorPower(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
