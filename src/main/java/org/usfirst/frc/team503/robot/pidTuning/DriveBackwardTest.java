/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.pidTuning;

import org.usfirst.frc.team503.robot.auton.FroggyAuton;

public class DriveBackwardTest extends FroggyAuton {
  private String[] profiles = new String[] { "DriveBackwardTest" };


  @Override
  public void initAuton() {
    froggySequentialDrive("DriveBackwardTest");
  }

  @Override
  public AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.tenFeetForward;
  }

}
