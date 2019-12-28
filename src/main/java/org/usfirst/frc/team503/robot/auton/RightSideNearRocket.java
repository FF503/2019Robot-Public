/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.StartingDirection;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.BoonkFullBlast;
import org.usfirst.frc.team503.robot.commands.MoveArmCommand;
import org.usfirst.frc.team503.robot.commands.PivotCommand;
import org.usfirst.frc.team503.robot.commands.ReleaseHatchCommand;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetAuton;

public class RightSideNearRocket extends FroggyAuton {
	private String[] profiles = new String[] {};

	@Override
	protected void initAuton() {
		addSequential(new BoonkFullBlast(0.05,false,false),0.4);
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 2.0));
		// froggySequentialDrive("rightToNearRocket");
		addSequential(new PivotCommand(50));
		addSequential(new BoonkFullBlast(0.1,false,false),0.5);
		addSequential(new FollowTargetAuton(-4.0));
		addParallel(new ReleaseHatchCommand(3.0));
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 1.5));
		addSequential(new BoonkFullBlast(0.3, true, false), 0.3);
		addSequential(new PivotCommand(-90),2.0);
		addSequential(new FollowTargetAuton());
		addSequential(new AssistedArcadeDriveCommand());

	}

	@Override
	protected StartingDirection getStartingDirection() {
		return StartingDirection.FORWARD;
	}

	@Override
	protected AutonStartingLocation getStartingLocation() {
		return AutonStartingLocation.Right;
	}
}
