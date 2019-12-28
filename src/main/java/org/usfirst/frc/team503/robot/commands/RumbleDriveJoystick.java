/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class RumbleDriveJoystick extends Command {
  private double startTime;
  private double timeout;
  private double rumblePower;

  public RumbleDriveJoystick() {
    this(1.0);
  }

  public RumbleDriveJoystick(double timeout) {
    this(0.5, timeout);
  }

  public RumbleDriveJoystick(double rumblePower, double timeout) {
    this.timeout = timeout;
    this.rumblePower = rumblePower;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    OI.setDriveRumble(rumblePower);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) > timeout;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    OI.setDriveRumble(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
