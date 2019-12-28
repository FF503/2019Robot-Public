/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;



import org.usfirst.frc.team503.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class RaiseHandsCommand extends Command {
  double startTime;

  public RaiseHandsCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Climber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Climber.getInstance().runHandMotors(0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Timer.getFPGATimestamp()-startTime > 1.0);
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
    // new AssistedArcadeDriveCommand().start();
    // new TeleopClimbCommand().start();
    end();
  }
}
