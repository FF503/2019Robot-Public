/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoHabTwoCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoHabTwoCommand() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // set constant pwr to drive motors while climber is running
    addParallel(new ConstantDriveCommand(-0.15), 1.3);

    // waits a bit before starting hand pid
    addSequential(new ClimbWaitCommand(0.25));

    // start hand pid relative to robot pitch
    addSequential(new FollowPitchPID(Robot.bot.targetPitch2));

    addSequential(new DriveOntoHabTwoCommand(0.0));

    // NO HANDS!
    addParallel(new RaiseHandsCommand());
    
    // drive for 1 seconds up onto hab
    addSequential(new ClimbWaitCommand(1.0));
  }
}
