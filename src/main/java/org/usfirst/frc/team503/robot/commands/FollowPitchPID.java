/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Climber;
import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.command.Command;

public class FollowPitchPID extends Command {

  private double target, pidGain = 0.0;

  public FollowPitchPID(double pitch) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Climber.getInstance());
    this.target = pitch;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pidGain = Climber.getInstance().getCalculatedPitchGain(target);
    Climber.getInstance().runHandMotors(pidGain);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(pidGain) < 0.05 || Math.abs(Gyro.getInstance().getPitch() - target) < 2.0;
    // return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Climber.getInstance().runHandMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // new AssistedArcadeDriveCommand().start();
    // new TeleopClimbCommand().start();
    end();
  }
}
