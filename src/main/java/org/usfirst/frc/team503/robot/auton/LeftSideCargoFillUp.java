/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.MoveArmCommand;
import org.usfirst.frc.team503.robot.commands.PivotCommand;
import org.usfirst.frc.team503.robot.commands.ReleaseHatchCommand;
import org.usfirst.frc.team503.robot.commands.SetPositionCommand;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetAuton;

public class LeftSideCargoFillUp extends FroggyAuton {

  @Override
  protected void initAuton() {
    addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 1.5));
    addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME, 3));
    froggySequentialDrive("LeftToFirstCargo2", 3.0);
    addSequential(new PivotCommand(0), 0.75);
    addSequential(new FollowTargetAuton(0.0, 0.6, true), 4.0);
    addSequential(new SetPositionCommand(FieldLocations.LeftFirstBay));
    addSequential(new AssistedArcadeDriveCommand());
    addParallel(new ReleaseHatchCommand(3.0));

    // Backup and go Reload
    froggySequentialDrive("leftFirstCargoBackup", 1.5);
    addSequential(new PivotCommand(-90), 0.6);
    froggySequentialDrive("leftFirstCargoReload", 1.5);
    // addSequential(new AssistedArcadeDriveCommand());
    addSequential(new FollowTargetAuton(1.0, false), 1.8);
    addSequential(new SetPositionCommand(15));

    froggySequentialDrive("reloadToLeftCargo", 2.9);
    addSequential(new PivotCommand(0, 2), 1.5);
    addSequential(new FollowTargetAuton(0.75, 0.55, true));
    // addParallel(new ReleaseHatchCommand(3.0));
    addSequential(new AssistedArcadeDriveCommand());
  }

  @Override
  protected AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.Left;
  }
}
