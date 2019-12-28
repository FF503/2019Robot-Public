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

import edu.wpi.first.wpilibj.command.WaitCommand;

public class RightSideCargoFillUp extends FroggyAuton {
  private String[] profiles = new String[] { "" };

  @Override
  protected void initAuton() {
    // Deliver First Hatch Panel @ First Bay
    addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 2.0));
    addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME, 3.5));
    froggySequentialDrive("RightToFirstCargo2", 3.0);
    addSequential(new PivotCommand(180), 0.75);
    addSequential(new FollowTargetAuton(),4.0);
    addSequential(new SetPositionCommand(FieldLocations.RightFirstBay));
    addParallel(new ReleaseHatchCommand(3.0));
    addSequential(new WaitCommand(0.15));

    // Backup and go Reload
    froggySequentialDrive("rightFirstCargoBackup", 1.5);
    addSequential(new PivotCommand(270), 0.6);
    froggySequentialDrive("rightFirstCargoReload", 1.5);
    // addSequential(new AssistedArcadeDriveCommand());
    addSequential(new FollowTargetAuton(), 3.4);

    // froggySequentialDrive("rightFirstCargoBackup", 1.5);
    // froggySequentialDrive("rightFirstCargoReload", 2.5);
    // addSequential(new FollowTargetAuton());
    // addSequential(new SetPositionCommand(FieldLocations.RightHatchReload));
    froggySequentialDrive("reloadToRightCargo", 2.8);
    addSequential(new PivotCommand(180,2.0), 1.0);
    addSequential(new FollowTargetAuton(0.0));
    // addParallel(new ReleaseHatchCommand(3.0));
    addSequential(new AssistedArcadeDriveCommand());
  }

  @Override
  protected AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.Right;
  }
}
