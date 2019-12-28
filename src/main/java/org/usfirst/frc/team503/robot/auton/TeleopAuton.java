/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;

public class TeleopAuton extends FroggyAuton {

	@Override
	public void initAuton() {
		// addParallel(new RumbleDriveJoystick(1.0));
		addSequential(new AssistedArcadeDriveCommand());
	}

	@Override
	public AutonStartingLocation getStartingLocation() {
		return AutonStartingLocation.Origin;
	}
}
