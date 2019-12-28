/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.vision.DriveToTarget1;

public class CenterAuton extends FroggyAuton {

	@Override
	public void initAuton() {
		// startingLocation = AutonStartingLocation.Center;
		// VisionLocalizer.getInstance().setLeftPipeline();
		// froggySequentialDrive("LeftToFarSideRocket1");
		addSequential(new DriveToTarget1());// 
		// froggySequentialDrive("leftRocketBackToReload");
	}

	@Override
	public AutonStartingLocation getStartingLocation() {
		return AutonStartingLocation.Origin;
	}
}
