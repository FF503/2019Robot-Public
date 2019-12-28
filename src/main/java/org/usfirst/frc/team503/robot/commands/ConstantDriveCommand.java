/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;

public class ConstantDriveCommand extends Command {

  double power = 0.0;

  public ConstantDriveCommand(double power) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Drive.getInstance());
    this.power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotState.getInstance().setBrakeInDisable(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("FINISHED CLIMB SEG");
    Drive.getInstance().setMotorOutputs(power, power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Drive.getInstance().setMotorOutputs(0.0, 0.0);

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
