/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Climber;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.command.Command;

public class DriveOntoHabTwoCommand extends Command {

  private double pitch = 0;

  public DriveOntoHabTwoCommand(double target) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Drive.getInstance());
    requires(Climber.getInstance());
    this.pitch = target;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Drive.getInstance().setMotorOutputs(-0.3, -0.3);
    Climber.getInstance().runHandMotors(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Gyro.getInstance().getPitch() >= pitch;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Drive.getInstance().setMotorOutputs(0.0, 0.0);
    Climber.getInstance().runHandMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
