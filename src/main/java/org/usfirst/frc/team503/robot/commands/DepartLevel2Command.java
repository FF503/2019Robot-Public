/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.auton.FroggyAuton;

public class DepartLevel2Command extends FroggyAuton {
  private AutonStartingLocation actualStartingLocation;

  public DepartLevel2Command(AutonStartingLocation startingLocation) {
    this.actualStartingLocation = startingLocation;
    initAuton();
  }

  @Override
  protected void initAuton() {
    addSequential(new SetPositionCommand(AutonStartingLocation.Origin.getStartingPose()));
    froggySequentialDrive("departLevel2", 0.5);
    addSequential(new BoonkFullBlast(0.0, true, false), 0.5);
    addSequential(new SetPositionCommand(actualStartingLocation.getStartingPose()));
  }

  @Override
  protected AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.Origin;
  }
}
