/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Arm;
import org.usfirst.frc.team503.robot.subsystems.Extension;

import edu.wpi.first.wpilibj.command.Command;

public class TestExtLimit extends Command {
  private double eTgt = 0.0;;

  public TestExtLimit() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    eTgt = 0.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // eLim = 0.0;
    if (Arm.getInstance().getEncoderDeg() > 0.0 && Arm.getInstance().getEncoderDeg() < 180.0) {
      eTgt = Robot.bot.gArmExtLength * (Math.abs(1 / Math.cos(Math.toRadians(Arm.getInstance().getEncoderDeg()))) - 1);
    } else {
      eTgt = Robot.bot.gExtMinLim;
    }
    // System.out.println("eTgt = " + eTgt);
    Extension.getInstance().setTargetPosition(eTgt);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Extension.getInstance().setMotorOutput(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
