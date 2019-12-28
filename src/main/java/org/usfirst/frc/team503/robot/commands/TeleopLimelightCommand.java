/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Arm;
import org.usfirst.frc.team503.robot.subsystems.LimelightTurret;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;

public class TeleopLimelightCommand extends Command {
  public TeleopLimelightCommand() {
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
  //  System.out.println("running limelight command");
    if (Arm.getInstance().getEncoderDeg() >= 90) {
     // System.out.println("Limelight pointed to back");
      LimelightTurret.getInstance().turnToBack();

    } else {
     // System.out.println("Limelight pointed to front");
      LimelightTurret.getInstance().turnToFront();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // this command was looping just setting the turrent to front
    // i chaned commange to end it -- KRM

    // it needs to be false the way we have it structured because it is constantly
    // running in the background checking the position of the arm.
    // This was easier to integrate in the code so that it would work no matter how
    // the arm was moved -- Ankith
    //
    return RobotState.isDisabled();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
