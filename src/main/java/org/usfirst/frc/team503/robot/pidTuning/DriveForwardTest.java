/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.pidTuning;

import org.usfirst.frc.team503.robot.auton.FroggyAuton;
import org.usfirst.frc.team503.robot.commands.PurePursuitDrive;

public class DriveForwardTest extends FroggyAuton {
  private String[] profiles = new String[] { "DriveForwardTest" };


  @Override
  public void initAuton() {
    // froggySequentialDrive("DriveForwardTest");
    addSequential(new PurePursuitDrive("DriveForwardTest"));
    // froggySequentialDrive("testBackwards");
  }

  @Override
  public AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.Origin;
  }
}
