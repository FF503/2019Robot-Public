/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class VacuumPowerCommand extends Command {
  public VacuumPowerCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (RobotState.getInstance().isVisionFollowerRunning() && !Intake.getInstance().hasHatch()) { // Case for primary intaking
      Intake.getInstance().setVacuumPower(Robot.bot.intakeVaccHighPower);
    }
    else if (RobotState.getInstance().getReleasing()){
      Intake.getInstance().stopVacuum();
    } 
    else if (RobotState.getInstance().hasCargo()) { // Case for cargo handling
      Intake.getInstance().setVacuumPower(Robot.bot.intakeVaccLowPower);
    } else { // Case for holding hatch power
      Intake.getInstance().setVacuumPower(Robot.bot.intakeVaccPower);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return DriverStation.getInstance().isDisabled();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.getInstance().setVacuumPower(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
