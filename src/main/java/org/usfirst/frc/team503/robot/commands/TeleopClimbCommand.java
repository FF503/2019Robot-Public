/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;



import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class TeleopClimbCommand extends Command {
  public TeleopClimbCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Climber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!OI.getClimbOldButton()) {
      Climber.getInstance().runHandMotors(-OI.getClimberRightYValue());
      Climber.getInstance().runLegMotor(-OI.getClimberLeftYValue());
    } else {
      Climber.getInstance().runHandMotors(Robot.bot.climbBasePower*Robot.bot.climbPowerRatio);
      Climber.getInstance().runLegMotor(Robot.bot.climbBasePower);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !DriverStation.getInstance().isOperatorControl();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Climber.getInstance().runHandMotors(0.);
    Climber.getInstance().runLegMotor(0.);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
