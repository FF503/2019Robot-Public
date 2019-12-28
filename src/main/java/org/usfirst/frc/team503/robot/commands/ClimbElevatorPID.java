/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Climber;
import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbElevatorPID extends Command {

  private double target, pidGain = 0.0;

  public ClimbElevatorPID(double rot) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Climber.getInstance());
    this.target = rot;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pidGain = Climber.getInstance().getCalculatedElevGain(target);
    Climber.getInstance().runLegMotor(pidGain);
    Climber.getInstance().runHandMotors(Climber.getInstance().getLegCounts()*Robot.bot.climbPowerRatio);
    // System.out.println("STILL RUNNING CLimb");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(pidGain) < 0.05 || Math.abs(Gyro.getInstance().getPitch() - target) < 2.0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Climber.getInstance().runHandMotors(0.0);
    Climber.getInstance().runLegMotor(0.0);
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
