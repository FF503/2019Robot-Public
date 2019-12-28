/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.PipelineSelector;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.MoveArmCommand;
import org.usfirst.frc.team503.robot.commands.SetPipelineEnum;

public class RightSideCargoFillUpLevel2 extends FroggyAuton {
  private String[] profiles = new String[] { "RightToFirstCargo2Lvl2" };

  @Override
  protected void initAuton() {
    addSequential(new SetPipelineEnum(PipelineSelector.LEFT));
    addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 2.0));

    froggySequentialDrive("RightToFirstCargo2Lvl2");
    // // addSequential(new WaitCommand(0.5));
    addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME));
    // froggySequentialDrive("rightFirstCargoBackup");
    // addSequential(new WaitCommand(0.5));

    // addSequential(new FollowTargetSimple(1, 0.015));

    // addSequential(new WaitCommand(0.5));
    // addSequential(new DriveSlowAuton());
    addSequential(new AssistedArcadeDriveCommand());
    // addSequential(new PlaceSequence());
    // addSequential(new ReleaseHatchCommand(0.5));
    // addSequential(new
    // SetPositionCommand(FieldLocations.RightThirdBay.getLocation()));
    // froggySequentialDrive("rightThirdCargoBackup");
    // addSequential(new PivotCommand(270));
    // froggySequentialDrive("thirdCargoToReload");
    // addSequential(new FollowTargetSimple(1));
    // addSequential(new ReleaseHatchCommand(0.5));
    // froggySequentialDrive("rightFirstCargoToReload");
  }

  @Override
  protected AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.RightLevel2;
  }
}
